#include "raven_control/config/motor_config.hpp"
#include "raven_control/hal/can_interface.hpp"
#include "raven_control/hal/motor_driver.hpp"
#include "raven_control/safety/joint_limiter.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr std::chrono::milliseconds TRANSITION_DURATION{4000};
constexpr std::chrono::milliseconds POSE_HOLD_DURATION{750};

struct JointSpec {
    const char* name;
};

constexpr std::array<JointSpec, 3> JOINTS{{
    {"shoulder_Joint"},
    {"upperArm_Joint"},
    {"foreArm_Joint"},
}};

// upperArm and foreArm deliberately use opposite signs in both poses.
constexpr std::array<double, 3> POSE_A_OFFSET_DEG{
    5.0,
    -4.0,
    6.0,
};
constexpr std::array<double, 3> POSE_B_OFFSET_DEG{
    -5.0,
    4.0,
    -6.0,
};
constexpr std::array<double, 3> HOME_OFFSET_DEG{
    0.0,
    0.0,
    0.0,
};

static_assert(
    POSE_A_OFFSET_DEG[1] * POSE_A_OFFSET_DEG[2] < 0.0,
    "upperArm and foreArm must move in opposite directions");
static_assert(
    POSE_B_OFFSET_DEG[1] * POSE_B_OFFSET_DEG[2] < 0.0,
    "upperArm and foreArm must move in opposite directions");

using JointBindings = std::array<
    const raven_control::config::JointMotorRuntimeConfig*,
    JOINTS.size()>;
using JointPositions = std::array<double, JOINTS.size()>;

struct PlannedPose {
    const char* name;
    JointPositions target_rad;
};

volatile std::sig_atomic_t stop_requested = 0;

void handleSignal(int)
{
    stop_requested = 1;
}

double degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
}

double radiansToDegrees(double radians)
{
    return radians * 180.0 / PI;
}

double quinticSmoothstep(double u)
{
    const double clamped = std::clamp(u, 0.0, 1.0);
    const double u2 = clamped * clamped;
    const double u3 = u2 * clamped;
    return u3 * (10.0 + clamped * (-15.0 + 6.0 * clamped));
}

class TerminalMode {
public:
    TerminalMode()
    {
        if (::tcgetattr(STDIN_FILENO, &original_) != 0) {
            throw std::runtime_error(
                "Cannot read terminal settings: " +
                std::string(std::strerror(errno)));
        }

        termios raw = original_;
        ::cfmakeraw(&raw);
        raw.c_lflag |= ISIG;
        raw.c_oflag |= OPOST;
        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            throw std::runtime_error(
                "Cannot enable terminal raw mode: " +
                std::string(std::strerror(errno)));
        }
        active_ = true;
    }

    ~TerminalMode()
    {
        if (active_)
            (void)::tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }

    TerminalMode(const TerminalMode&) = delete;
    TerminalMode& operator=(const TerminalMode&) = delete;

private:
    termios original_{};
    bool active_ = false;
};

class StopGuard {
public:
    explicit StopGuard(raven_control::hal::MotorDriver& driver)
        : driver_(driver)
    {
    }

    ~StopGuard()
    {
        if (active_)
            (void)driver_.stopAll();
    }

    StopGuard(const StopGuard&) = delete;
    StopGuard& operator=(const StopGuard&) = delete;

    void release() noexcept
    {
        active_ = false;
    }

private:
    raven_control::hal::MotorDriver& driver_;
    bool active_ = true;
};

bool keyAvailable()
{
    timeval timeout{};
    fd_set descriptors;
    FD_ZERO(&descriptors);
    FD_SET(STDIN_FILENO, &descriptors);
    return ::select(
               STDIN_FILENO + 1,
               &descriptors,
               nullptr,
               nullptr,
               &timeout) > 0;
}

std::optional<char> readKey()
{
    char key = '\0';
    if (::read(STDIN_FILENO, &key, sizeof(key)) == 1)
        return key;
    return std::nullopt;
}

bool abortKeyPressed()
{
    if (!keyAvailable())
        return false;
    const std::optional<char> key = readKey();
    return key &&
           (*key == ' ' || *key == 'q' || *key == 'Q');
}

JointBindings bindConfiguredJoints(
    const raven_control::config::MotorRuntimeConfig& config)
{
    if (config.joints.size() != JOINTS.size()) {
        throw std::runtime_error(
            "coordinated_motion_demo requires exactly three joints");
    }

    JointBindings bindings{};
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        bindings[index] = config.findJoint(JOINTS[index].name);
        if (bindings[index] == nullptr) {
            throw std::runtime_error(
                "Motor config is missing joint '" +
                std::string(JOINTS[index].name) + "'");
        }
    }
    return bindings;
}

std::vector<raven_control::hal::JointMotorConfig> motorMap(
    const JointBindings& bindings)
{
    std::vector<raven_control::hal::JointMotorConfig> result;
    result.reserve(JOINTS.size());
    for (const auto* joint : bindings) {
        result.push_back({
            joint->joint_name,
            joint->motor_id,
            joint->position_sign,
            joint->joint_zero_at_motor_rad,
            joint->joint_to_motor_ratio,
        });
    }
    return result;
}

JointPositions readJointPositions(
    raven_control::hal::MotorDriver& driver)
{
    if (!driver.allFeedbackValid())
        throw std::runtime_error("Fresh feedback is unavailable");

    JointPositions positions{};
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto feedback = driver.feedback(JOINTS[index].name);
        if (!feedback || !feedback->valid) {
            throw std::runtime_error(
                "Missing feedback for '" +
                std::string(JOINTS[index].name) + "'");
        }
        positions[index] = feedback->position_rad;
    }
    return positions;
}

JointPositions offsetPose(
    const JointPositions& home,
    const std::array<double, JOINTS.size()>& offset_deg)
{
    JointPositions target{};
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        target[index] =
            home[index] + degreesToRadians(offset_deg[index]);
    }
    return target;
}

std::array<PlannedPose, 3> makePlan(const JointPositions& home)
{
    return {{
        {"Pose A", offsetPose(home, POSE_A_OFFSET_DEG)},
        {"Pose B", offsetPose(home, POSE_B_OFFSET_DEG)},
        {"Home", offsetPose(home, HOME_OFFSET_DEG)},
    }};
}

void validatePlan(
    const JointPositions& home,
    const std::array<PlannedPose, 3>& plan,
    const JointBindings& bindings,
    const raven_control::safety::JointLimiterMap& limiters)
{
    JointPositions segment_start = home;
    const double duration_seconds =
        std::chrono::duration<double>(TRANSITION_DURATION).count();

    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto limiter_entry = limiters.find(JOINTS[index].name);
        if (limiter_entry == limiters.end()) {
            throw std::runtime_error(
                "Joint limits are missing '" +
                std::string(JOINTS[index].name) + "'");
        }

        const auto& limiter = limiter_entry->second;
        if (!limiter.isConfirmed()) {
            throw std::runtime_error(
                "Joint limits are unconfirmed for '" +
                std::string(JOINTS[index].name) + "'");
        }
        if (limiter.isHardViolation(home[index]) ||
            home[index] < limiter.softMinRad() ||
            home[index] > limiter.softMaxRad()) {
            throw std::runtime_error(
                "Start position is outside the soft range for '" +
                std::string(JOINTS[index].name) + "'");
        }
    }

    for (const PlannedPose& pose : plan) {
        for (std::size_t index = 0; index < JOINTS.size(); ++index) {
            const auto& limiter = limiters.at(JOINTS[index].name);
            const double target = pose.target_rad[index];
            if (target < limiter.softMinRad() ||
                target > limiter.softMaxRad()) {
                throw std::runtime_error(
                    std::string(pose.name) +
                    " is outside the soft range for '" +
                    JOINTS[index].name + "'");
            }

            // Quintic smoothstep has a peak slope of 1.875.
            const double peak_velocity =
                1.875 *
                std::abs(target - segment_start[index]) /
                duration_seconds;
            if (peak_velocity >
                bindings[index]->
                    position_control.max_slew_rate_rad_s) {
                throw std::runtime_error(
                    std::string(pose.name) +
                    " exceeds max_slew_rate_rad_s for '" +
                    JOINTS[index].name + "'");
            }
        }
        segment_start = pose.target_rad;
    }
}

void printPlan(
    const JointPositions& home,
    const std::array<PlannedPose, 3>& plan)
{
    std::cout
        << "\nCoordinated demo plan (joint coordinates)\n"
        << "UpperArm and ForeArm move with opposite signs.\n"
        << std::fixed << std::setprecision(1);

    auto printPose = [](const char* name, const JointPositions& pose) {
        std::cout << std::setw(8) << name << " : ";
        for (std::size_t index = 0; index < JOINTS.size(); ++index) {
            std::cout
                << JOINTS[index].name << '='
                << std::setw(6)
                << radiansToDegrees(pose[index])
                << " deg";
            if (index + 1 < JOINTS.size())
                std::cout << " | ";
        }
        std::cout << '\n';
    };

    printPose("Start", home);
    for (const PlannedPose& pose : plan)
        printPose(pose.name, pose.target_rad);
}

void requestFeedback(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config)
{
    if (!driver.requestMechanicalPositions())
        throw std::runtime_error("Failed to request position feedback");

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::max(
            std::chrono::milliseconds(500),
            config.feedback_timeout * 2);
    while (!driver.allFeedbackValid() &&
           std::chrono::steady_clock::now() < deadline) {
        (void)driver.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!driver.allFeedbackValid())
        throw std::runtime_error("Timed out waiting for fresh feedback");
}

bool waitForStart(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config)
{
    std::cout
        << "\nSPACE: capture the current pose and start once\n"
        << "Q: stop and quit\n"
        << std::flush;

    auto next_request = std::chrono::steady_clock::now();
    while (!stop_requested) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_request) {
            if (!driver.requestMechanicalPositions()) {
                throw std::runtime_error(
                    "Failed to request position feedback");
            }
            next_request = now + config.position_request_period;
        }
        (void)driver.poll();
        if (driver.faultLatched())
            throw std::runtime_error(driver.faultReason());

        if (keyAvailable()) {
            const std::optional<char> key = readKey();
            if (key && *key == ' ')
                return true;
            if (key && (*key == 'q' || *key == 'Q'))
                return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

void printStatus(
    const char* phase_name,
    raven_control::hal::MotorDriver& driver,
    const JointPositions& target)
{
    std::cout
        << "\r\033[K" << std::setw(8) << phase_name << " | "
        << std::fixed << std::setprecision(1);
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto feedback = driver.feedback(JOINTS[index].name);
        const double actual =
            feedback && feedback->valid
            ? radiansToDegrees(feedback->position_rad)
            : 0.0;
        std::cout
            << "T:" << std::setw(6)
            << radiansToDegrees(target[index])
            << " A:" << std::setw(6) << actual;
        if (index + 1 < JOINTS.size())
            std::cout << " | ";
    }
    std::cout << std::flush;
}

void sendTargets(
    raven_control::hal::MotorDriver& driver,
    const JointBindings& bindings,
    const JointPositions& target)
{
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto result = driver.sendPositionCommand(
            JOINTS[index].name,
            target[index],
            bindings[index]->position_control.kp,
            bindings[index]->position_control.kd);
        if (result !=
            raven_control::hal::MotorCommandResult::Sent) {
            throw std::runtime_error(
                "Position command failed on '" +
                std::string(JOINTS[index].name) +
                "': " + raven_control::hal::toString(result));
        }
    }
}

bool runPhase(
    const char* phase_name,
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    const JointPositions& start,
    const JointPositions& goal,
    std::chrono::milliseconds duration)
{
    const auto phase_start = std::chrono::steady_clock::now();
    const auto phase_end = phase_start + duration;
    auto next_cycle = phase_start;
    auto next_request = phase_start;
    auto next_status = phase_start;

    while (!stop_requested) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= phase_end)
            break;
        if (abortKeyPressed())
            return false;

        if (now >= next_request) {
            if (!driver.requestMechanicalPositions()) {
                throw std::runtime_error(
                    "Failed to request position feedback");
            }
            do {
                next_request += config.position_request_period;
            } while (next_request <= now);
        }
        (void)driver.poll();
        if (driver.faultLatched())
            throw std::runtime_error(driver.faultReason());

        const double u =
            std::chrono::duration<double>(now - phase_start).count() /
            std::chrono::duration<double>(duration).count();
        const double blend = quinticSmoothstep(u);
        JointPositions target{};
        for (std::size_t index = 0; index < JOINTS.size(); ++index) {
            target[index] =
                start[index] +
                blend * (goal[index] - start[index]);
        }
        sendTargets(driver, bindings, target);

        if (now >= next_status) {
            printStatus(phase_name, driver, target);
            next_status = now + std::chrono::milliseconds(100);
        }

        next_cycle += config.control_period;
        std::this_thread::sleep_until(next_cycle);
    }
    return !stop_requested;
}

bool holdPose(
    const char* phase_name,
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    const JointPositions& pose)
{
    return runPhase(
        phase_name,
        driver,
        config,
        bindings,
        pose,
        pose,
        POSE_HOLD_DURATION);
}

}  // namespace

int main(int argc, char* argv[])
{
    const std::string interface_name =
        argc > 1 ? argv[1] : "can0";
    const std::string limits_path =
        argc > 2 ? argv[2] : "config/joint_limits.yaml";
    const std::string motor_config_path =
        argc > 3 ? argv[3] : "config/motor_config.yaml";

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    try {
        auto limiters =
            raven_control::safety::loadJointLimiters(limits_path);
        const auto motor_config =
            raven_control::config::loadMotorRuntimeConfig(
                motor_config_path);
        const JointBindings bindings =
            bindConfiguredJoints(motor_config);
        raven_control::hal::CanInterface can(interface_name);
        raven_control::hal::MotorDriver driver(
            can,
            motorMap(bindings),
            limiters,
            0xFD,
            motor_config.feedback_timeout);
        StopGuard stop_guard(driver);

        if (!driver.stopAll())
            throw std::runtime_error("Failed to send startup stop");
        requestFeedback(driver, motor_config);

        std::cout
            << "RAVEN coordinated three-joint demo\n"
            << "Joint limits: " << limits_path << '\n'
            << "Motor config: " << motor_config_path << '\n'
            << "Motion: Shoulder independent, UpperArm/ForeArm opposite\n";

        TerminalMode terminal;
        if (!waitForStart(driver, motor_config)) {
            std::cout << "\nDemo cancelled\n";
            return 0;
        }

        const JointPositions home = readJointPositions(driver);
        const auto plan = makePlan(home);
        validatePlan(home, plan, bindings, limiters);
        printPlan(home, plan);

        const auto enable_result = driver.enableAll();
        if (enable_result !=
            raven_control::hal::MotorCommandResult::Sent) {
            throw std::runtime_error(
                "Enable rejected: " +
                std::string(
                    raven_control::hal::toString(enable_result)));
        }

        bool completed = true;
        JointPositions segment_start = home;
        for (const PlannedPose& pose : plan) {
            if (!runPhase(
                    pose.name,
                    driver,
                    motor_config,
                    bindings,
                    segment_start,
                    pose.target_rad,
                    TRANSITION_DURATION) ||
                !holdPose(
                    pose.name,
                    driver,
                    motor_config,
                    bindings,
                    pose.target_rad)) {
                completed = false;
                break;
            }
            segment_start = pose.target_rad;
        }

        const bool stopped = driver.stopAll();
        stop_guard.release();
        std::cout << "\n";

        if (!stopped) {
            std::cerr << "Failed to send final stop\n";
            return 1;
        }
        if (!completed || stop_requested) {
            std::cout << "Demo stopped by user\n";
            return 0;
        }

        std::cout << "Demo complete: returned to the start pose and stopped\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "\nFatal error: " << error.what() << '\n';
        return 1;
    }
}
