#include "raven_control/config/motor_config.hpp"
#include "raven_control/dynamics/gravity_compensator.hpp"
#include "raven_control/hal/can_interface.hpp"
#include "raven_control/hal/motor_driver.hpp"
#include "raven_control/logging/motion_logger.hpp"
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
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr std::chrono::milliseconds TRANSITION_DURATION{4000};
constexpr std::chrono::milliseconds POSE_HOLD_DURATION{750};
constexpr std::chrono::milliseconds
    GRAVITY_COMPENSATION_RAMP_DURATION{1000};
constexpr std::size_t DIAGNOSTIC_RESERVE_SAMPLES = 30000;

struct JointSpec {
    const char* name;
};

constexpr std::array<JointSpec, 3> JOINTS{{
    {"shoulder_Joint"},
    {"upperArm_Joint"},
    {"foreArm_Joint"},
}};

// Edit these values in degrees. upperArm and foreArm deliberately use
// opposite signs in both poses.
constexpr std::array<double, 3> POSE_A_OFFSET_DEG{
    50.0,
    -40.0,
    30.0,
};
constexpr std::array<double, 3> POSE_B_OFFSET_DEG{
    -10.0,
    8.0,
    -12.0,
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
using JointTorques = std::array<double, JOINTS.size()>;

constexpr std::array<double, JOINTS.size()>

    GRAVITY_COMPENSATION_SCALE{
        1.0,   // base
        -1.0,  // UpperArm
        -1.0   // ForeArm
    };

struct GravityControlState {
    explicit GravityControlState(bool initially_enabled)
        : compensator(
              raven_control::dynamics::makeRavenUrdfGravityModel()),
          enabled(initially_enabled)
    {
    }

    raven_control::dynamics::GravityCompensator compensator;
    bool enabled = true;
    double blend = 0.0;
    std::chrono::steady_clock::time_point last_update{};
    JointTorques last_applied_torque_nm{};
};

struct CommandDispatch {
    bool feedback_hold = false;
    JointTorques feedforward_torque_nm{};
};

struct PlannedPose {
    const char* name;
    JointPositions target_rad;
};

struct DiagnosticState {
    std::uint64_t cycle_index = 0;
    JointPositions previous_feedback{};
    std::array<std::chrono::steady_clock::time_point, JOINTS.size()>
        previous_feedback_at{};
    std::array<bool, JOINTS.size()> feedback_initialized{};
    JointPositions estimated_actual_velocity{};
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

double quinticSmoothstepDerivative(double u)
{
    const double clamped = std::clamp(u, 0.0, 1.0);
    const double one_minus_u = 1.0 - clamped;
    return 30.0 * clamped * clamped *
           one_minus_u * one_minus_u;
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

bool abortKeyPressed(GravityControlState& gravity)
{
    if (!keyAvailable())
        return false;
    const std::optional<char> key = readKey();
    if (!key)
        return false;
    if (*key == 'g' || *key == 'G') {
        gravity.enabled = !gravity.enabled;
        std::cout
            << "\nGravity compensation "
            << (gravity.enabled ? "ON" : "OFF")
            << " (torque ramps over 1 second)\n"
            << std::flush;
        return false;
    }
    return *key == ' ' || *key == 'q' || *key == 'Q';
}

void resetGravityRamp(GravityControlState& gravity)
{
    gravity.blend = 0.0;
    gravity.last_update = std::chrono::steady_clock::now();
    gravity.last_applied_torque_nm = {};
}

JointTorques gravityCompensationTorque(
    raven_control::hal::MotorDriver& driver,
    GravityControlState& gravity)
{
    const auto now = std::chrono::steady_clock::now();
    if (gravity.last_update ==
        std::chrono::steady_clock::time_point{}) {
        gravity.last_update = now;
    }
    const double elapsed_seconds = std::clamp(
        std::chrono::duration<double>(
            now - gravity.last_update).count(),
        0.0,
        0.1);
    gravity.last_update = now;

    const double target_blend = gravity.enabled ? 1.0 : 0.0;
    const double max_blend_step =
        elapsed_seconds /
        std::chrono::duration<double>(
            GRAVITY_COMPENSATION_RAMP_DURATION).count();
    gravity.blend += std::clamp(
        target_blend - gravity.blend,
        -max_blend_step,
        max_blend_step);

    JointTorques compensation{};
    if (gravity.blend <= 0.0)
        return compensation;

    raven_control::dynamics::JointVector joint_positions{};
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto feedback = driver.feedback(JOINTS[index].name);
        if (!feedback || !feedback->valid)
            return compensation;
        joint_positions[index] = feedback->position_rad;
    }

    const auto full_compensation =
        gravity.compensator.compute(joint_positions);
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        compensation[index] =
            gravity.blend 
            * GRAVITY_COMPENSATION_SCALE[index]
            * full_compensation[index];
    }
    return compensation;
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

std::array<std::string, JOINTS.size()> diagnosticJointNames()
{
    std::array<std::string, JOINTS.size()> names{};
    for (std::size_t index = 0; index < JOINTS.size(); ++index)
        names[index] = JOINTS[index].name;
    return names;
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
    const JointPositions& target,
    const GravityControlState& gravity)
{
    std::cout
        << "\r\033[K" << std::setw(8) << phase_name << " | "
        << "G:" << (gravity.enabled ? "ON " : "OFF")
        << '(' << std::setw(3)
        << static_cast<int>(std::round(gravity.blend * 100.0))
        << "%) | "
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

CommandDispatch sendTargets(
    raven_control::hal::MotorDriver& driver,
    const JointBindings& bindings,
    const JointPositions& target,
    const JointPositions& target_velocity,
    GravityControlState& gravity)
{
    CommandDispatch dispatch;
    const JointTorques requested_feedforward =
        gravityCompensationTorque(driver, gravity);
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto result = driver.sendMitCommand(
            JOINTS[index].name,
            target[index],
            target_velocity[index],
            bindings[index]->position_control.kp,
            bindings[index]->position_control.kd,
            requested_feedforward[index]);
        if (result ==
            raven_control::hal::MotorCommandResult::FeedbackHold) {
            dispatch.feedback_hold = true;
        } else if (result !=
                   raven_control::hal::MotorCommandResult::Sent) {
            throw std::runtime_error(
                "MIT command failed on '" +
                std::string(JOINTS[index].name) +
                "': " + raven_control::hal::toString(result));
        }
    }
    if (!dispatch.feedback_hold)
        gravity.last_applied_torque_nm = requested_feedforward;
    dispatch.feedforward_torque_nm =
        gravity.last_applied_torque_nm;
    return dispatch;
}

void recordDiagnosticSample(
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& state,
    const char* phase_name,
    std::chrono::steady_clock::time_point scheduled_at,
    std::chrono::steady_clock::time_point cycle_started_at,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    raven_control::hal::MotorDriver& driver,
    const JointPositions& target,
    const JointPositions& target_velocity,
    const JointTorques& feedforward_torque_nm)
{
    const auto observed_at = std::chrono::steady_clock::now();

    raven_control::logging::MotionSample sample;
    sample.cycle_index = state.cycle_index++;
    sample.phase = phase_name;
    sample.scheduled_at = scheduled_at;
    sample.actual_at = cycle_started_at;
    sample.control_period =
        std::chrono::duration_cast<std::chrono::microseconds>(
            config.control_period);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        raven_control::logging::JointMotionSample joint_sample;
        joint_sample.command_position_rad = target[index];
        joint_sample.trajectory_velocity_rad_s =
            target_velocity[index];
        joint_sample.sent_velocity_rad_s = target_velocity[index];
        joint_sample.sent_feedforward_torque_nm =
            feedforward_torque_nm[index];
        joint_sample.kp = bindings[index]->position_control.kp;
        joint_sample.kd = bindings[index]->position_control.kd;

        const auto feedback = driver.feedback(JOINTS[index].name);
        if (feedback && feedback->valid) {
            joint_sample.feedback_valid = true;
            joint_sample.actual_position_rad = feedback->position_rad;
            joint_sample.feedback_age_ms =
                std::chrono::duration<double, std::milli>(
                    observed_at - feedback->received_at).count();

            const bool operation_feedback_fresh =
                feedback->operation_feedback_valid &&
                observed_at - feedback->operation_received_at <=
                    config.feedback_timeout;
            if (operation_feedback_fresh) {
                joint_sample.operation_feedback_valid = true;
                joint_sample.operation_feedback_age_ms =
                    std::chrono::duration<double, std::milli>(
                        observed_at -
                        feedback->operation_received_at).count();
                joint_sample.measured_torque_nm = feedback->torque_nm;
                joint_sample.motor_temperature_celsius =
                    feedback->temperature_celsius;
                joint_sample.motor_fault_flags = feedback->fault_flags;
                joint_sample.motor_mode_state = feedback->mode_state;
            } else {
                joint_sample.operation_feedback_age_ms = nan;
                joint_sample.measured_torque_nm = nan;
                joint_sample.motor_temperature_celsius = nan;
            }

            if (state.feedback_initialized[index] &&
                feedback->received_at >
                    state.previous_feedback_at[index]) {
                const double feedback_dt_seconds =
                    std::chrono::duration<double>(
                        feedback->received_at -
                        state.previous_feedback_at[index]).count();
                if (feedback_dt_seconds > 0.0) {
                    state.estimated_actual_velocity[index] =
                        (feedback->position_rad -
                         state.previous_feedback[index]) /
                        feedback_dt_seconds;
                }
            }

            if (!state.feedback_initialized[index] ||
                feedback->received_at >
                    state.previous_feedback_at[index]) {
                state.previous_feedback[index] =
                    feedback->position_rad;
                state.previous_feedback_at[index] =
                    feedback->received_at;
                state.feedback_initialized[index] = true;
            }

            joint_sample.actual_velocity_rad_s =
                operation_feedback_fresh
                ? feedback->velocity_rad_s
                : state.estimated_actual_velocity[index];
            joint_sample.position_error_rad =
                target[index] - feedback->position_rad;
            joint_sample.estimated_p_torque_nm =
                joint_sample.kp * joint_sample.position_error_rad;
            joint_sample.estimated_d_torque_nm =
                joint_sample.kd *
                (joint_sample.sent_velocity_rad_s -
                 joint_sample.actual_velocity_rad_s);
            joint_sample.estimated_control_torque_nm =
                joint_sample.estimated_p_torque_nm +
                joint_sample.estimated_d_torque_nm +
                joint_sample.sent_feedforward_torque_nm;
        } else {
            joint_sample.actual_position_rad = nan;
            joint_sample.actual_velocity_rad_s = nan;
            joint_sample.position_error_rad = nan;
            joint_sample.estimated_p_torque_nm = nan;
            joint_sample.estimated_d_torque_nm = nan;
            joint_sample.estimated_control_torque_nm = nan;
            joint_sample.feedback_age_ms = nan;
            joint_sample.operation_feedback_age_ms = nan;
            joint_sample.measured_torque_nm = nan;
            joint_sample.motor_temperature_celsius = nan;
        }

        sample.joints[index] = joint_sample;
    }

    logger.record(std::move(sample));
}

void holdAfterFeedbackLossUntilDisabled(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    const JointPositions& last_target,
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic_state,
    GravityControlState& gravity)
{
    std::cout
        << "\nType 2 feedback lost: "
        << driver.feedbackHoldReason() << '\n'
        << "Motion is frozen at the last safe target.\n"
        << "The motors remain enabled in MIT position hold.\n"
        << "SPACE: disable all motors and exit\n"
        << "Q: emergency stop and exit\n"
        << std::flush;

    auto next_cycle = std::chrono::steady_clock::now();
    auto next_status = next_cycle;
    const JointPositions zero_velocity{};

    while (!stop_requested) {
        const auto now = std::chrono::steady_clock::now();
        if (abortKeyPressed(gravity))
            return;

        (void)driver.poll();
        if (driver.faultLatched())
            throw std::runtime_error(driver.faultReason());

        const CommandDispatch dispatch = sendTargets(
            driver,
            bindings,
            last_target,
            zero_velocity,
            gravity);
        recordDiagnosticSample(
            logger,
            diagnostic_state,
            "Feedback Hold",
            next_cycle,
            now,
            config,
            bindings,
            driver,
            last_target,
            zero_velocity,
            dispatch.feedforward_torque_nm);

        if (now >= next_status) {
            printStatus("FB Hold", driver, last_target, gravity);
            next_status = now + std::chrono::milliseconds(200);
        }

        next_cycle += config.control_period;
        std::this_thread::sleep_until(next_cycle);
    }
}

bool runPhase(
    const char* phase_name,
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    const JointPositions& start,
    const JointPositions& goal,
    std::chrono::milliseconds duration,
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic_state,
    GravityControlState& gravity)
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
        if (abortKeyPressed(gravity))
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
        const double blend_rate =
            quinticSmoothstepDerivative(u) /
            std::chrono::duration<double>(duration).count();
        JointPositions target{};
        JointPositions target_velocity{};
        for (std::size_t index = 0; index < JOINTS.size(); ++index) {
            const double displacement = goal[index] - start[index];
            target[index] =
                start[index] + blend * displacement;
            target_velocity[index] = blend_rate * displacement;
        }
        const CommandDispatch dispatch = sendTargets(
            driver,
            bindings,
            target,
            target_velocity,
            gravity);
        recordDiagnosticSample(
            logger,
            diagnostic_state,
            phase_name,
            next_cycle,
            now,
            config,
            bindings,
            driver,
            target,
            target_velocity,
            dispatch.feedforward_torque_nm);

        if (dispatch.feedback_hold) {
            holdAfterFeedbackLossUntilDisabled(
                driver,
                config,
                bindings,
                target,
                logger,
                diagnostic_state,
                gravity);
            return false;
        }

        if (now >= next_status) {
            printStatus(phase_name, driver, target, gravity);
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
    const JointPositions& pose,
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic_state,
    GravityControlState& gravity)
{
    return runPhase(
        phase_name,
        driver,
        config,
        bindings,
        pose,
        pose,
        POSE_HOLD_DURATION,
        logger,
        diagnostic_state,
        gravity);
}

bool holdHomeUntilDisabled(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    const JointPositions& home,
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic_state,
    GravityControlState& gravity)
{
    std::cout
        << "\nReturned to the start pose.\n"
        << "The motors remain enabled and hold this pose.\n"
        << "SPACE: disable all motors and exit\n"
        << "Q: emergency stop and exit\n"
        << std::flush;

    auto next_cycle = std::chrono::steady_clock::now();
    auto next_request = next_cycle;
    auto next_status = next_cycle;
    const JointPositions zero_velocity{};

    while (!stop_requested) {
        const auto now = std::chrono::steady_clock::now();
        if (abortKeyPressed(gravity))
            return true;

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

        const CommandDispatch dispatch = sendTargets(
            driver,
            bindings,
            home,
            zero_velocity,
            gravity);
        recordDiagnosticSample(
            logger,
            diagnostic_state,
            "Home Hold",
            next_cycle,
            now,
            config,
            bindings,
            driver,
            home,
            zero_velocity,
            dispatch.feedforward_torque_nm);
        if (now >= next_status) {
            printStatus("Home Hold", driver, home, gravity);
            next_status = now + std::chrono::milliseconds(200);
        }

        next_cycle += config.control_period;
        std::this_thread::sleep_until(next_cycle);
    }
    return false;
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
    const std::string log_path =
        argc > 4
        ? argv[4]
        : raven_control::logging::makeTimestampedLogPath(
              "logs",
              "coordinated_motion");

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::optional<raven_control::logging::MotionLogger> logger;
    bool log_saved = false;

    try {
        auto limiters =
            raven_control::safety::loadJointLimiters(limits_path);
        const auto motor_config =
            raven_control::config::loadMotorRuntimeConfig(
                motor_config_path);
        const JointBindings bindings =
            bindConfiguredJoints(motor_config);
        GravityControlState gravity(
            motor_config.gravity_compensation_enabled);
        raven_control::hal::CanInterface can(interface_name);
        raven_control::hal::MotorDriver driver(
            can,
            motorMap(bindings),
            limiters,
            0xFD,
            motor_config.feedback_timeout);
        StopGuard stop_guard(driver);
        logger.emplace(
            diagnosticJointNames(),
            DIAGNOSTIC_RESERVE_SAMPLES);
        DiagnosticState diagnostic_state;

        if (!driver.stopAll())
            throw std::runtime_error("Failed to send startup stop");
        requestFeedback(driver, motor_config);

        std::cout
            << "RAVEN coordinated three-joint demo\n"
            << "Joint limits: " << limits_path << '\n'
            << "Motor config: " << motor_config_path << '\n'
            << "Motion log: " << log_path << '\n'
            << "Gravity compensation: "
            << (gravity.enabled ? "ON" : "OFF")
            << " (G toggles during the demo)\n"
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
        resetGravityRamp(gravity);

        bool completed = !gravity.enabled || runPhase(
            "G Ramp",
            driver,
            motor_config,
            bindings,
            home,
            home,
            GRAVITY_COMPENSATION_RAMP_DURATION,
            *logger,
            diagnostic_state,
            gravity);
        JointPositions segment_start = home;
        for (const PlannedPose& pose : plan) {
            if (!completed || !runPhase(
                    pose.name,
                    driver,
                    motor_config,
                    bindings,
                    segment_start,
                    pose.target_rad,
                    TRANSITION_DURATION,
                    *logger,
                    diagnostic_state,
                    gravity) ||
                !holdPose(
                    pose.name,
                    driver,
                    motor_config,
                    bindings,
                    pose.target_rad,
                    *logger,
                    diagnostic_state,
                    gravity)) {
                completed = false;
                break;
            }
            segment_start = pose.target_rad;
        }
        if (completed) {
            const JointPositions zero_velocity{};
            (void)sendTargets(
                driver,
                bindings,
                home,
                zero_velocity,
                gravity);
            (void)holdHomeUntilDisabled(
                driver,
                motor_config,
                bindings,
                home,
                *logger,
                diagnostic_state,
                gravity);
        }

        const bool stopped = driver.stopAll();
        if (!stopped) {
            throw std::runtime_error(
                "Failed to send final stop");
        }
        stop_guard.release();
        std::cout << "\n";

        if (logger->sampleCount() > 0) {
            logger->saveCsv(log_path);
            log_saved = true;
            std::cout
                << "Motion log saved: " << log_path
                << " (" << logger->sampleCount() << " samples)\n";
        }

        if (!completed || stop_requested) {
            std::cout << "Demo stopped by user\n";
            return 0;
        }

        std::cout << "Motors disabled by user\n";
        return 0;
    } catch (const std::exception& error) {
        if (logger && !log_saved && logger->sampleCount() > 0) {
            try {
                logger->saveCsv(log_path);
                std::cerr
                    << "Motion log saved after failure: "
                    << log_path << '\n';
            } catch (const std::exception& log_error) {
                std::cerr
                    << "\nFailed to save motion log: "
                    << log_error.what() << '\n';
            }
        }
        std::cerr << "\nFatal error: " << error.what() << '\n';
        return 1;
    }
}
