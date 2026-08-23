#include "raven_control/config/motor_config.hpp"
#include "raven_control/control/gravity_feedforward_controller.hpp"
#include "raven_control/dynamics/pinocchio_gravity_model.hpp"
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
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
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
constexpr const char* CAN_INTERFACE = "can0";
constexpr const char* JOINT_LIMITS_PATH = "config/joint_limits.yaml";
constexpr const char* MOTOR_CONFIG_PATH = "config/motor_config.yaml";
constexpr double DEMO_CLEARANCE_RAD = 2.0 * PI / 180.0;
constexpr std::size_t DIAGNOSTIC_RESERVE_SAMPLES = 30000;

constexpr std::array<const char*, 3> JOINT_NAMES{{
    "shoulder_Joint",
    "upperArm_Joint",
    "foreArm_Joint",
}};

using JointVector = std::array<double, JOINT_NAMES.size()>;
using JointBindings = std::array<
    const raven_control::config::JointMotorRuntimeConfig*,
    JOINT_NAMES.size()>;

struct TaughtPose {
    std::string name;
    JointVector position_rad{};
    double transition_seconds = 3.0;
    double hold_seconds = 0.7;
};

struct TeachSession {
    std::string name = "one_shot";
    std::uint64_t repeat_count = 1;
    double default_transition_seconds = 3.0;
    double default_hold_seconds = 0.7;
    double return_transition_seconds = 3.0;
    double controlled_stop_seconds = 1.0;
    std::vector<TaughtPose> poses;
};

struct TrajectorySample {
    JointVector position{};
    JointVector velocity{};
    JointVector acceleration{};
};

struct QuinticPolynomial {
    std::array<double, 6> coefficient{};

    [[nodiscard]] double position(double seconds) const
    {
        const auto& c = coefficient;
        return c[0] + seconds * (c[1] + seconds *
               (c[2] + seconds * (c[3] + seconds *
               (c[4] + seconds * c[5]))));
    }

    [[nodiscard]] double velocity(double seconds) const
    {
        const auto& c = coefficient;
        return c[1] + seconds * (2.0 * c[2] + seconds *
               (3.0 * c[3] + seconds *
               (4.0 * c[4] + seconds * 5.0 * c[5])));
    }

    [[nodiscard]] double acceleration(double seconds) const
    {
        const auto& c = coefficient;
        return 2.0 * c[2] + seconds *
               (6.0 * c[3] + seconds *
               (12.0 * c[4] + seconds * 20.0 * c[5]));
    }
};

struct QuinticSegment {
    std::array<QuinticPolynomial, JOINT_NAMES.size()> joints{};
    double duration_seconds = 0.0;

    [[nodiscard]] TrajectorySample sample(double elapsed_seconds) const
    {
        const double seconds = std::clamp(
            elapsed_seconds, 0.0, duration_seconds);
        TrajectorySample result;
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            result.position[index] = joints[index].position(seconds);
            result.velocity[index] = joints[index].velocity(seconds);
            result.acceleration[index] =
                joints[index].acceleration(seconds);
        }
        return result;
    }
};

struct PlannedSegment {
    std::string name;
    QuinticSegment trajectory;
    double hold_seconds = 0.0;
};

struct GravityState {
    explicit GravityState(
        const raven_control::config::MotorRuntimeConfig& config)
        : controller(
              std::make_unique<
                  raven_control::dynamics::PinocchioGravityModel>(
                  config.gravity_compensation.urdf_path),
              config.gravity_compensation),
          feedback_timeout(config.feedback_timeout)
    {
    }

    raven_control::control::GravityFeedforwardController controller;
    std::chrono::milliseconds feedback_timeout;
    raven_control::control::GravityFeedforwardResult last_result{};
    JointVector last_applied_torque_nm{};
};

struct CommandDispatch {
    bool feedback_hold = false;
    JointVector feedforward_torque_nm{};
    raven_control::control::GravityFeedforwardResult gravity;
};

enum class RunResult {
    Completed,
    StopRequested,
    FeedbackHold,
};

volatile std::sig_atomic_t stop_requested = 0;

void handleSignal(int)
{
    stop_requested = 1;
}

double radiansToDegrees(double radians)
{
    return radians * 180.0 / PI;
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

QuinticPolynomial makeQuintic(
    double start_position,
    double start_velocity,
    double start_acceleration,
    double goal_position,
    double goal_velocity,
    double goal_acceleration,
    double duration_seconds)
{
    if (!std::isfinite(duration_seconds) || duration_seconds <= 0.0)
        throw std::runtime_error("Quintic duration must be positive");
    const double t = duration_seconds;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    const double delta_position = goal_position -
        (start_position + start_velocity * t +
         0.5 * start_acceleration * t2);
    const double delta_velocity =
        goal_velocity - (start_velocity + start_acceleration * t);
    const double delta_acceleration =
        goal_acceleration - start_acceleration;

    QuinticPolynomial polynomial;
    polynomial.coefficient = {{
        start_position,
        start_velocity,
        0.5 * start_acceleration,
        (10.0 * delta_position - 4.0 * delta_velocity * t +
         0.5 * delta_acceleration * t2) / t3,
        (-15.0 * delta_position + 7.0 * delta_velocity * t -
         delta_acceleration * t2) / t4,
        (6.0 * delta_position - 3.0 * delta_velocity * t +
         0.5 * delta_acceleration * t2) / t5,
    }};
    return polynomial;
}

QuinticSegment makeRestToRestSegment(
    const JointVector& start,
    const JointVector& goal,
    double duration_seconds)
{
    QuinticSegment segment;
    segment.duration_seconds = duration_seconds;
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        segment.joints[index] = makeQuintic(
            start[index], 0.0, 0.0,
            goal[index], 0.0, 0.0,
            duration_seconds);
    }
    return segment;
}

QuinticSegment makeControlledStopSegment(
    const TrajectorySample& start,
    double duration_seconds)
{
    QuinticSegment segment;
    segment.duration_seconds = duration_seconds;
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const double goal_position =
            start.position[index] +
            0.5 * start.velocity[index] * duration_seconds;
        segment.joints[index] = makeQuintic(
            start.position[index],
            start.velocity[index],
            start.acceleration[index],
            goal_position,
            0.0,
            0.0,
            duration_seconds);
    }
    return segment;
}

JointBindings bindConfiguredJoints(
    const raven_control::config::MotorRuntimeConfig& config)
{
    if (config.joints.size() != JOINT_NAMES.size()) {
        throw std::runtime_error(
            "teach_and_play_demo requires exactly three joints");
    }
    JointBindings bindings{};
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        bindings[index] = config.findJoint(JOINT_NAMES[index]);
        if (bindings[index] == nullptr) {
            throw std::runtime_error(
                "Motor config is missing joint '" +
                std::string(JOINT_NAMES[index]) + "'");
        }
    }
    return bindings;
}

std::vector<raven_control::hal::JointMotorConfig> motorMap(
    const JointBindings& bindings)
{
    std::vector<raven_control::hal::JointMotorConfig> result;
    result.reserve(JOINT_NAMES.size());
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

std::array<std::string, JOINT_NAMES.size()> diagnosticJointNames()
{
    std::array<std::string, JOINT_NAMES.size()> result{};
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index)
        result[index] = JOINT_NAMES[index];
    return result;
}

const raven_control::safety::JointLimiter& limiterFor(
    const raven_control::safety::JointLimiterMap& limiters,
    std::size_t index)
{
    const auto entry = limiters.find(JOINT_NAMES[index]);
    if (entry == limiters.end()) {
        throw std::runtime_error(
            "Joint limits are missing '" +
            std::string(JOINT_NAMES[index]) + "'");
    }
    return entry->second;
}

void validatePose(
    const JointVector& pose,
    const std::string& pose_name,
    const raven_control::safety::JointLimiterMap& limiters,
    double clearance_rad)
{
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto& limiter = limiterFor(limiters, index);
        if (!limiter.isConfirmed()) {
            throw std::runtime_error(
                "Joint limits are unconfirmed for '" +
                std::string(JOINT_NAMES[index]) + "'");
        }
        const double minimum = limiter.softMinRad() + clearance_rad;
        const double maximum = limiter.softMaxRad() - clearance_rad;
        if (minimum > maximum || pose[index] < minimum ||
            pose[index] > maximum) {
            throw std::runtime_error(
                pose_name + " is outside the required soft-limit "
                "clearance for '" + JOINT_NAMES[index] + "'");
        }
    }
}

bool segmentIsSafe(
    const QuinticSegment& segment,
    const JointBindings& bindings,
    const raven_control::safety::JointLimiterMap& limiters,
    std::string* reason)
{
    constexpr std::size_t SAMPLE_COUNT = 1000;
    for (std::size_t sample_index = 0;
         sample_index <= SAMPLE_COUNT;
         ++sample_index) {
        const double seconds = segment.duration_seconds *
            static_cast<double>(sample_index) /
            static_cast<double>(SAMPLE_COUNT);
        const TrajectorySample sample = segment.sample(seconds);
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            const auto& limiter = limiterFor(limiters, index);
            if (sample.position[index] < limiter.softMinRad() ||
                sample.position[index] > limiter.softMaxRad()) {
                if (reason != nullptr) {
                    *reason = std::string(JOINT_NAMES[index]) +
                        " position leaves its soft range";
                }
                return false;
            }
            if (std::abs(sample.velocity[index]) >
                bindings[index]->position_control.max_slew_rate_rad_s +
                    1e-9) {
                if (reason != nullptr) {
                    *reason = std::string(JOINT_NAMES[index]) +
                        " velocity exceeds max_slew_rate_rad_s";
                }
                return false;
            }
        }
    }
    return true;
}

void validateSegment(
    const QuinticSegment& segment,
    const std::string& name,
    const JointBindings& bindings,
    const raven_control::safety::JointLimiterMap& limiters)
{
    std::string reason;
    if (!segmentIsSafe(segment, bindings, limiters, &reason)) {
        throw std::runtime_error(
            "Unsafe trajectory segment '" + name + "': " + reason);
    }
}

void validateStopEnvelope(
    const QuinticSegment& motion,
    double stop_duration_seconds,
    const std::string& name,
    const JointBindings& bindings,
    const raven_control::safety::JointLimiterMap& limiters)
{
    constexpr std::size_t TRIGGER_SAMPLES = 50;
    for (std::size_t index = 0; index <= TRIGGER_SAMPLES; ++index) {
        const double seconds = motion.duration_seconds *
            static_cast<double>(index) /
            static_cast<double>(TRIGGER_SAMPLES);
        const QuinticSegment stop = makeControlledStopSegment(
            motion.sample(seconds), stop_duration_seconds);
        std::string reason;
        if (!segmentIsSafe(stop, bindings, limiters, &reason)) {
            throw std::runtime_error(
                "Unsafe controlled-stop envelope in '" + name +
                "': " + reason);
        }
    }
}

double safeDuration(
    const JointVector& start,
    const JointVector& goal,
    double requested_seconds,
    const JointBindings& bindings)
{
    double duration = requested_seconds;
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const double required =
            1.875 * std::abs(goal[index] - start[index]) /
            bindings[index]->position_control.max_slew_rate_rad_s;
        duration = std::max(duration, required * 1.05);
    }
    return duration;
}

std::vector<PlannedSegment> buildSafePlan(
    const TeachSession& session,
    const JointVector& playback_start,
    const JointBindings& bindings,
    const raven_control::safety::JointLimiterMap& limiters)
{
    if (session.poses.empty())
        throw std::runtime_error("At least one taught pose is required");
    validatePose(playback_start, "Playback start pose", limiters, 0.0);
    for (const TaughtPose& pose : session.poses) {
        validatePose(
            pose.position_rad,
            "Taught pose '" + pose.name + "'",
            limiters,
            DEMO_CLEARANCE_RAD);
    }

    std::vector<PlannedSegment> plan;
    plan.reserve(
        static_cast<std::size_t>(session.repeat_count) *
            session.poses.size() + 1);
    JointVector current = playback_start;
    for (std::uint64_t cycle = 0; cycle < session.repeat_count; ++cycle) {
        for (const TaughtPose& pose : session.poses) {
            const double duration = safeDuration(
                current,
                pose.position_rad,
                pose.transition_seconds,
                bindings);
            PlannedSegment segment;
            segment.name = session.repeat_count > 1
                ? "Cycle " + std::to_string(cycle + 1) + " / " + pose.name
                : pose.name;
            segment.trajectory = makeRestToRestSegment(
                current, pose.position_rad, duration);
            segment.hold_seconds = pose.hold_seconds;
            validateSegment(
                segment.trajectory, segment.name, bindings, limiters);
            validateStopEnvelope(
                segment.trajectory,
                session.controlled_stop_seconds,
                segment.name,
                bindings,
                limiters);
            plan.push_back(std::move(segment));
            current = pose.position_rad;
        }
    }

    const double return_duration = safeDuration(
        current,
        playback_start,
        session.return_transition_seconds,
        bindings);
    PlannedSegment return_segment;
    return_segment.name = "Return To Start";
    return_segment.trajectory = makeRestToRestSegment(
        current, playback_start, return_duration);
    validateSegment(
        return_segment.trajectory,
        return_segment.name,
        bindings,
        limiters);
    validateStopEnvelope(
        return_segment.trajectory,
        session.controlled_stop_seconds,
        return_segment.name,
        bindings,
        limiters);
    plan.push_back(std::move(return_segment));
    return plan;
}

JointVector readJointPositions(raven_control::hal::MotorDriver& driver)
{
    if (!driver.allFeedbackValid())
        throw std::runtime_error("Fresh Type 17 feedback is unavailable");
    JointVector positions{};
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto feedback = driver.feedback(JOINT_NAMES[index]);
        if (!feedback || !feedback->valid ||
            feedback->source !=
                raven_control::hal::MotorFeedbackSource::MechanicalPosition) {
            throw std::runtime_error(
                "Missing Type 17 feedback for '" +
                std::string(JOINT_NAMES[index]) + "'");
        }
        positions[index] = feedback->position_rad;
    }
    return positions;
}

void serviceDisabledFeedback(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    std::chrono::steady_clock::time_point& next_request)
{
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_request) {
        if (!driver.requestMechanicalPositions()) {
            throw std::runtime_error(
                "Failed to request Type 17 mechanical positions");
        }
        next_request = now + config.position_request_period;
    }
    (void)driver.poll();
    if (driver.faultLatched())
        throw std::runtime_error(driver.faultReason());
}

void requestFreshMechanicalPositions(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::max(std::chrono::milliseconds(1000),
                 config.feedback_timeout * 4);
    auto next_request = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline) {
        serviceDisabledFeedback(driver, config, next_request);
        if (driver.allFeedbackValid())
            return;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error(
        "Timed out waiting for fresh Type 17 feedback from all joints");
}

void printDisabledStatus(
    const std::string& phase,
    raven_control::hal::MotorDriver& driver,
    std::size_t pose_count)
{
    std::cout << "\r\033[K" << std::setw(12) << phase
              << " | saved:" << std::setw(2) << pose_count << " | "
              << std::fixed << std::setprecision(1);
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto feedback = driver.feedback(JOINT_NAMES[index]);
        if (feedback && feedback->valid) {
            std::cout << JOINT_NAMES[index] << ':' << std::setw(6)
                      << radiansToDegrees(feedback->position_rad);
        } else {
            std::cout << JOINT_NAMES[index] << ":  ---";
        }
        if (index + 1 < JOINT_NAMES.size())
            std::cout << " | ";
    }
    std::cout << std::flush;
}

bool teachPoses(
    TeachSession& session,
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const raven_control::safety::JointLimiterMap& limiters)
{
    session.poses.clear();
    std::cout
        << "\nTeach mode: motors remain disabled. Support the robot while "
        << "moving it by hand.\n"
        << "SPACE: capture the current calibrated joint pose\n"
        << "X or Backspace: delete the last pose\n"
        << "C: clear every captured pose\n"
        << "ENTER: finish this in-memory teaching session\n"
        << "Q: cancel and discard every captured pose\n"
        << std::flush;

    auto next_request = std::chrono::steady_clock::now();
    auto next_status = next_request;
    while (true) {
        if (stop_requested != 0)
            return false;
        serviceDisabledFeedback(driver, config, next_request);
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_status) {
            printDisabledStatus("Teach", driver, session.poses.size());
            next_status = now + std::chrono::milliseconds(100);
        }
        if (!keyAvailable()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        const auto key = readKey();
        if (!key)
            continue;
        if (*key == 'q' || *key == 'Q')
            return false;
        if (*key == 'c' || *key == 'C') {
            session.poses.clear();
            std::cout << "\nCleared every taught pose\n";
            continue;
        }
        if (*key == 'x' || *key == 'X' || *key == '\b' ||
            static_cast<unsigned char>(*key) == 127U) {
            if (!session.poses.empty()) {
                std::cout << "\nDeleted " << session.poses.back().name
                          << '\n';
                session.poses.pop_back();
            } else {
                std::cout << "\nNo taught pose to delete\n";
            }
            continue;
        }
        if (*key == '\r' || *key == '\n') {
            if (session.poses.empty()) {
                std::cout << "\nCapture at least one pose before ENTER.\n";
                continue;
            }
            return true;
        }
        if (*key == ' ') {
            try {
                const JointVector position = readJointPositions(driver);
                validatePose(
                    position,
                    "Captured taught pose",
                    limiters,
                    DEMO_CLEARANCE_RAD);
                TaughtPose pose;
                std::ostringstream name;
                name << "pose_" << std::setfill('0') << std::setw(2)
                     << session.poses.size() + 1;
                pose.name = name.str();
                pose.position_rad = position;
                pose.transition_seconds =
                    session.default_transition_seconds;
                pose.hold_seconds = session.default_hold_seconds;
                session.poses.push_back(std::move(pose));
                std::cout << "\nCaptured " << session.poses.back().name
                          << '\n';
            } catch (const std::exception& error) {
                std::cout << "\nCapture rejected: " << error.what() << '\n';
            }
        }
    }
}

void printSession(const TeachSession& session)
{
    std::cout
        << "\nIn-memory taught poses: " << session.poses.size() << '\n'
        << "Playback: one cycle\n"
        << "Natural completion: return to execution-time start pose\n"
        << std::fixed << std::setprecision(1);
    for (std::size_t pose_index = 0;
         pose_index < session.poses.size();
         ++pose_index) {
        const TaughtPose& pose = session.poses[pose_index];
        std::cout << "  " << (pose_index + 1) << ". " << pose.name
                  << " | ";
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            std::cout << JOINT_NAMES[index] << '='
                      << radiansToDegrees(pose.position_rad[index])
                      << " deg";
            if (index + 1 < JOINT_NAMES.size())
                std::cout << " | ";
        }
        std::cout << " | move=" << pose.transition_seconds
                  << " s hold=" << pose.hold_seconds << " s\n";
    }
}

std::optional<JointVector> waitForPlaybackStart(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    std::size_t pose_count)
{
    std::cout
        << "\nSPACE: capture the execution-time start pose, enable, and play\n"
        << "Q: cancel while motors remain disabled\n"
        << std::flush;
    auto next_request = std::chrono::steady_clock::now();
    auto next_status = next_request;
    while (true) {
        if (stop_requested != 0)
            return std::nullopt;
        serviceDisabledFeedback(driver, config, next_request);
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_status) {
            printDisabledStatus("Preflight", driver, pose_count);
            next_status = now + std::chrono::milliseconds(200);
        }
        if (keyAvailable()) {
            const auto key = readKey();
            if (key && *key == ' ')
                return readJointPositions(driver);
            if (key && (*key == 'q' || *key == 'Q'))
                return std::nullopt;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

class PlaybackRuntime {
public:
    PlaybackRuntime(
        raven_control::hal::MotorDriver& driver,
        const raven_control::config::MotorRuntimeConfig& config,
        const JointBindings& bindings,
        const raven_control::safety::JointLimiterMap& limiters,
        raven_control::logging::MotionLogger& logger)
        : driver_(driver),
          config_(config),
          bindings_(bindings),
          limiters_(limiters),
          logger_(logger),
          gravity_(config),
          next_bus_voltage_request_(std::chrono::steady_clock::now())
    {
    }

    void resetGravityRamp()
    {
        gravity_.controller.reset(std::chrono::steady_clock::now());
        gravity_.last_result = {};
        gravity_.last_applied_torque_nm = {};
    }

    RunResult driveSegment(
        const std::string& phase_name,
        const QuinticSegment& segment,
        bool accept_stop_request,
        double controlled_stop_seconds,
        JointVector& final_pose)
    {
        const auto started_at = std::chrono::steady_clock::now();
        auto next_cycle = started_at;
        auto next_status = started_at;
        bool stop_at_segment_end = false;

        while (true) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed =
                std::chrono::duration<double>(now - started_at).count();
            if (elapsed >= segment.duration_seconds)
                break;

            if (pollMotionInput() && accept_stop_request) {
                stop_requested = 0;
                const TrajectorySample current = segment.sample(elapsed);
                const QuinticSegment braking = makeControlledStopSegment(
                    current, controlled_stop_seconds);
                std::string reason;
                if (segmentIsSafe(braking, bindings_, limiters_, &reason)) {
                    std::cout
                        << "\nStop requested. Controlled deceleration "
                        << "started.\n";
                    const RunResult braking_result = driveSegment(
                        "Controlled Stop",
                        braking,
                        false,
                        controlled_stop_seconds,
                        final_pose);
                    return braking_result == RunResult::FeedbackHold
                        ? RunResult::FeedbackHold
                        : RunResult::StopRequested;
                }
                std::cout
                    << "\nThe controlled-stop envelope is unsafe ("
                    << reason << "). Finishing this quintic segment before "
                    << "holding.\n";
                stop_at_segment_end = true;
            }

            pollDriver(now);
            const TrajectorySample target = segment.sample(elapsed);
            const CommandDispatch dispatch = sendTargets(
                target.position, target.velocity);
            recordSample(
                phase_name,
                next_cycle,
                now,
                target.position,
                target.velocity,
                dispatch.feedforward_torque_nm,
                dispatch.gravity);
            final_pose = target.position;
            if (dispatch.feedback_hold)
                return RunResult::FeedbackHold;
            if (now >= next_status) {
                printStatus(phase_name, target.position);
                next_status = now + std::chrono::milliseconds(100);
            }
            next_cycle += config_.control_period;
            std::this_thread::sleep_until(next_cycle);
        }

        const TrajectorySample goal = segment.sample(segment.duration_seconds);
        const CommandDispatch dispatch = sendTargets(
            goal.position, goal.velocity);
        final_pose = goal.position;
        if (dispatch.feedback_hold)
            return RunResult::FeedbackHold;
        return stop_at_segment_end
            ? RunResult::StopRequested
            : RunResult::Completed;
    }

    RunResult holdPoseFor(
        const std::string& phase_name,
        const JointVector& pose,
        double hold_seconds,
        double controlled_stop_seconds,
        JointVector& final_pose)
    {
        if (hold_seconds <= 0.0) {
            final_pose = pose;
            return RunResult::Completed;
        }
        return driveSegment(
            phase_name,
            makeRestToRestSegment(pose, pose, hold_seconds),
            true,
            controlled_stop_seconds,
            final_pose);
    }

    void holdFinalPoseUntilSpace(const JointVector& final_pose)
    {
        std::cout
            << "\nMotion stopped. The current commanded pose is held.\n"
            << "SPACE: disable all motors, save the log, and exit\n"
            << std::flush;
        stop_requested = 0;
        auto next_cycle = std::chrono::steady_clock::now();
        auto next_status = next_cycle;
        const JointVector zero_velocity{};
        while (true) {
            const auto now = std::chrono::steady_clock::now();
            if (keyAvailable()) {
                const auto key = readKey();
                if (key && *key == ' ')
                    return;
                if (key && (*key == 'g' || *key == 'G'))
                    toggleGravity();
            }
            if (stop_requested != 0) {
                stop_requested = 0;
                std::cout << "\nFinal Hold requires SPACE to disable.\n";
            }
            pollDriver(now);
            const CommandDispatch dispatch = sendTargets(
                final_pose, zero_velocity);
            recordSample(
                "Final Hold",
                next_cycle,
                now,
                final_pose,
                zero_velocity,
                dispatch.feedforward_torque_nm,
                dispatch.gravity);
            if (dispatch.feedback_hold) {
                holdAfterFeedbackLossUntilSpace(final_pose);
                return;
            }
            if (now >= next_status) {
                printStatus("Final Hold", final_pose);
                next_status = now + std::chrono::milliseconds(200);
            }
            next_cycle += config_.control_period;
            std::this_thread::sleep_until(next_cycle);
        }
    }

    void holdAfterFeedbackLossUntilSpace(const JointVector& final_pose)
    {
        std::cout
            << "\nType 2 feedback lost: "
            << driver_.feedbackHoldReason() << '\n'
            << "The existing latched Feedback Hold remains active.\n"
            << "SPACE: disable all motors and exit\n"
            << std::flush;
        stop_requested = 0;
        auto next_cycle = std::chrono::steady_clock::now();
        auto next_status = next_cycle;
        const JointVector zero_velocity{};
        while (true) {
            const auto now = std::chrono::steady_clock::now();
            if (keyAvailable()) {
                const auto key = readKey();
                if (key && *key == ' ')
                    return;
            }
            if (stop_requested != 0) {
                stop_requested = 0;
                std::cout << "\nFeedback Hold requires SPACE to disable.\n";
            }
            pollDriver(now);
            const CommandDispatch dispatch = sendTargets(
                final_pose, zero_velocity);
            recordSample(
                "Feedback Hold",
                next_cycle,
                now,
                final_pose,
                zero_velocity,
                dispatch.feedforward_torque_nm,
                dispatch.gravity);
            if (now >= next_status) {
                printStatus("Feedback Hold", final_pose);
                next_status = now + std::chrono::milliseconds(200);
            }
            next_cycle += config_.control_period;
            std::this_thread::sleep_until(next_cycle);
        }
    }

private:
    void pollDriver(std::chrono::steady_clock::time_point now)
    {
        if (now >= next_bus_voltage_request_) {
            if (!driver_.requestBusVoltages())
                throw std::runtime_error("Failed to request bus voltages");
            next_bus_voltage_request_ = now + std::chrono::seconds(1);
        }
        (void)driver_.poll();
        if (driver_.faultLatched())
            throw std::runtime_error(driver_.faultReason());
    }

    bool pollMotionInput()
    {
        if (stop_requested != 0)
            return true;
        if (!keyAvailable())
            return false;
        const auto key = readKey();
        if (!key)
            return false;
        if (*key == 'q' || *key == 'Q')
            return true;
        if (*key == 'g' || *key == 'G') {
            toggleGravity();
        } else if (*key == ' ') {
            std::cout
                << "\nMotion is active. Press Q to stop smoothly first.\n";
        }
        return false;
    }

    void toggleGravity()
    {
        const bool enabled = !gravity_.controller.enabled();
        gravity_.controller.setEnabled(
            enabled, std::chrono::steady_clock::now());
        std::cout << "\nGravity compensation "
                  << (enabled ? "ON" : "OFF")
                  << (gravity_.controller.config().dry_run
                          ? " [DRY-RUN]" : "")
                  << " (configured ramp and limits apply)\n";
    }

    raven_control::control::GravityFeedforwardResult gravityTorque()
    {
        const auto now = std::chrono::steady_clock::now();
        raven_control::dynamics::JointVector positions{};
        bool feedback_fresh = true;
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            const auto feedback = driver_.feedback(JOINT_NAMES[index]);
            if (!feedback || !feedback->valid ||
                !feedback->operation_feedback_valid ||
                now < feedback->operation_received_at ||
                now - feedback->operation_received_at >
                    gravity_.feedback_timeout) {
                feedback_fresh = false;
                break;
            }
            positions[index] = feedback->position_rad;
        }
        gravity_.last_result = gravity_.controller.compute(
            positions, feedback_fresh, now);
        return gravity_.last_result;
    }

    CommandDispatch sendTargets(
        const JointVector& target,
        const JointVector& target_velocity)
    {
        CommandDispatch dispatch;
        dispatch.gravity = gravityTorque();
        const JointVector& requested_torque =
            dispatch.gravity.commanded_torque_nm;
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            const auto result = driver_.sendMitCommand(
                JOINT_NAMES[index],
                target[index],
                target_velocity[index],
                bindings_[index]->position_control.kp,
                bindings_[index]->position_control.kd,
                requested_torque[index]);
            if (result ==
                raven_control::hal::MotorCommandResult::FeedbackHold) {
                dispatch.feedback_hold = true;
            } else if (result !=
                       raven_control::hal::MotorCommandResult::Sent) {
                throw std::runtime_error(
                    "MIT command failed on '" +
                    std::string(JOINT_NAMES[index]) + "': " +
                    raven_control::hal::toString(result));
            }
        }
        if (!dispatch.feedback_hold)
            gravity_.last_applied_torque_nm = requested_torque;
        dispatch.feedforward_torque_nm =
            gravity_.last_applied_torque_nm;
        return dispatch;
    }

    void recordSample(
        const std::string& phase,
        std::chrono::steady_clock::time_point scheduled_at,
        std::chrono::steady_clock::time_point actual_at,
        const JointVector& target,
        const JointVector& target_velocity,
        const JointVector& feedforward_torque,
        const raven_control::control::GravityFeedforwardResult& gravity)
    {
        const auto now = std::chrono::steady_clock::now();
        const double nan = std::numeric_limits<double>::quiet_NaN();
        raven_control::logging::MotionSample sample;
        sample.cycle_index = cycle_index_++;
        sample.phase = phase;
        sample.scheduled_at = scheduled_at;
        sample.actual_at = actual_at;
        sample.control_period =
            std::chrono::duration_cast<std::chrono::microseconds>(
                config_.control_period);
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            auto& joint = sample.joints[index];
            joint.command_position_rad = target[index];
            joint.trajectory_velocity_rad_s = target_velocity[index];
            joint.sent_velocity_rad_s = target_velocity[index];
            joint.sent_feedforward_torque_nm = feedforward_torque[index];
            joint.raw_gravity_torque_nm =
                gravity.raw_gravity_torque_nm[index];
            joint.ramped_gravity_torque_nm =
                gravity.ramped_gravity_torque_nm[index];
            joint.limited_gravity_torque_nm =
                gravity.limited_gravity_torque_nm[index];
            joint.gravity_scale = gravity.scale;
            joint.gravity_ramp_factor = gravity.ramp_factor;
            joint.gravity_enabled = gravity.enabled;
            joint.gravity_dry_run = gravity.dry_run;
            joint.gravity_input_valid = gravity.input_valid;
            joint.gravity_torque_clamped = gravity.torque_clamped[index];
            joint.kp = bindings_[index]->position_control.kp;
            joint.kd = bindings_[index]->position_control.kd;
            const auto feedback = driver_.feedback(JOINT_NAMES[index]);
            if (!feedback || !feedback->valid) {
                joint.actual_position_rad = nan;
                joint.actual_velocity_rad_s = nan;
                joint.position_error_rad = nan;
                joint.measured_torque_nm = nan;
                joint.motor_temperature_celsius = nan;
                joint.bus_voltage_v = nan;
                joint.feedback_age_ms = nan;
                joint.operation_feedback_age_ms = nan;
                joint.bus_voltage_age_ms = nan;
                continue;
            }
            joint.feedback_valid = true;
            joint.actual_position_rad = feedback->position_rad;
            joint.position_error_rad =
                target[index] - feedback->position_rad;
            joint.feedback_age_ms =
                std::chrono::duration<double, std::milli>(
                    now - feedback->received_at).count();
            const bool operation_fresh =
                feedback->operation_feedback_valid &&
                now - feedback->operation_received_at <=
                    config_.feedback_timeout;
            joint.operation_feedback_valid = operation_fresh;
            if (operation_fresh) {
                joint.actual_velocity_rad_s = feedback->velocity_rad_s;
                joint.measured_torque_nm = feedback->torque_nm;
                joint.motor_temperature_celsius =
                    feedback->temperature_celsius;
                joint.motor_fault_flags = feedback->fault_flags;
                joint.motor_mode_state = feedback->mode_state;
                joint.operation_feedback_age_ms =
                    std::chrono::duration<double, std::milli>(
                        now - feedback->operation_received_at).count();
                joint.estimated_p_torque_nm =
                    joint.kp * joint.position_error_rad;
                joint.estimated_d_torque_nm =
                    joint.kd *
                    (target_velocity[index] - feedback->velocity_rad_s);
                joint.estimated_control_torque_nm =
                    joint.estimated_p_torque_nm +
                    joint.estimated_d_torque_nm +
                    feedforward_torque[index];
            } else {
                joint.actual_velocity_rad_s = nan;
                joint.measured_torque_nm = nan;
                joint.motor_temperature_celsius = nan;
                joint.operation_feedback_age_ms = nan;
                joint.estimated_p_torque_nm = nan;
                joint.estimated_d_torque_nm = nan;
                joint.estimated_control_torque_nm = nan;
            }
            joint.bus_voltage_valid = feedback->bus_voltage_valid;
            if (feedback->bus_voltage_valid) {
                joint.bus_voltage_v = feedback->bus_voltage_v;
                joint.bus_voltage_age_ms =
                    std::chrono::duration<double, std::milli>(
                        now - feedback->bus_voltage_received_at).count();
            } else {
                joint.bus_voltage_v = nan;
                joint.bus_voltage_age_ms = nan;
            }
        }
        logger_.record(std::move(sample));
    }

    void printStatus(
        const std::string& phase,
        const JointVector& target)
    {
        std::cout << "\r\033[K" << std::setw(18) << phase
                  << " | G:"
                  << (gravity_.controller.enabled() ? "ON " : "OFF")
                  << (gravity_.controller.config().dry_run
                          ? "DRY " : "LIVE ")
                  << '(' << std::setw(3)
                  << static_cast<int>(
                         std::round(
                             gravity_.controller.rampFactor() * 100.0))
                  << "%) | " << std::fixed << std::setprecision(1);
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            const auto feedback = driver_.feedback(JOINT_NAMES[index]);
            const double actual = feedback && feedback->valid
                ? radiansToDegrees(feedback->position_rad)
                : 0.0;
            std::cout << "T:" << std::setw(6)
                      << radiansToDegrees(target[index])
                      << " A:" << std::setw(6) << actual;
            if (index + 1 < JOINT_NAMES.size())
                std::cout << " | ";
        }
        std::cout << std::flush;
    }

    raven_control::hal::MotorDriver& driver_;
    const raven_control::config::MotorRuntimeConfig& config_;
    const JointBindings& bindings_;
    const raven_control::safety::JointLimiterMap& limiters_;
    raven_control::logging::MotionLogger& logger_;
    GravityState gravity_;
    std::uint64_t cycle_index_ = 0;
    std::chrono::steady_clock::time_point next_bus_voltage_request_{};
};

std::string logPrefix(const std::string& session_name)
{
    std::string result = "teach_and_play_";
    for (const char character : session_name) {
        if ((character >= 'a' && character <= 'z') ||
            (character >= 'A' && character <= 'Z') ||
            (character >= '0' && character <= '9') ||
            character == '_' || character == '-') {
            result.push_back(character);
        } else {
            result.push_back('_');
        }
    }
    return result;
}

}  // namespace

int main(int argc, char*[])
{
    if (argc != 1) {
        std::cerr << "Usage: teach_and_play_demo\n";
        return 2;
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::optional<raven_control::logging::MotionLogger> logger;
    std::string log_path;
    bool log_saved = false;

    try {
        TeachSession session;
        auto limiters = raven_control::safety::loadJointLimiters(
            JOINT_LIMITS_PATH);
        const auto motor_config =
            raven_control::config::loadMotorRuntimeConfig(
                MOTOR_CONFIG_PATH);
        const JointBindings bindings = bindConfiguredJoints(motor_config);

        raven_control::hal::CanInterface can(CAN_INTERFACE);
        raven_control::hal::MotorDriver driver(
            can,
            motorMap(bindings),
            limiters,
            0xFD,
            motor_config.feedback_timeout);
        StopGuard stop_guard(driver);
        if (!driver.stopAll())
            throw std::runtime_error("Failed to send startup stop");
        requestFreshMechanicalPositions(driver, motor_config);

        std::cout
            << "\nRAVEN teach and play demo\n"
            << "Session storage: in memory only; every run starts empty\n"
            << "CAN interface: " << CAN_INTERFACE << '\n'
            << "Joint limits: " << JOINT_LIMITS_PATH << '\n'
            << "Motor config: " << MOTOR_CONFIG_PATH << '\n'
            << "Gravity compensation: "
            << (motor_config.gravity_compensation.enabled ? "ON" : "OFF")
            << (motor_config.gravity_compensation.dry_run
                    ? " [DRY-RUN]" : " [LIVE]")
            << " scale=" << motor_config.gravity_compensation.scale
            << " URDF=" << motor_config.gravity_compensation.urdf_path
            << '\n'
            << "ROS 2 visualization remains available through the separate "
            << "read-only raven_joint_state_bridge.\n";

        TerminalMode terminal;
        if (!teachPoses(
                session,
                driver,
                motor_config,
                limiters)) {
            std::cout
                << "\nTeaching cancelled; captured poses were discarded\n";
            return 0;
        }

        printSession(session);
        const std::optional<JointVector> playback_start =
            waitForPlaybackStart(
                driver, motor_config, session.poses.size());
        if (!playback_start) {
            std::cout << "\nPlayback cancelled\n";
            return 0;
        }

        requestFreshMechanicalPositions(driver, motor_config);
        const JointVector fresh_start = readJointPositions(driver);
        const std::vector<PlannedSegment> plan = buildSafePlan(
            session, fresh_start, bindings, limiters);
        std::cout << "\nPreflight passed for " << plan.size()
                  << " quintic segment(s):\n";
        for (const PlannedSegment& segment : plan) {
            std::cout << "  " << segment.name << " | move="
                      << std::fixed << std::setprecision(2)
                      << segment.trajectory.duration_seconds
                      << " s hold=" << segment.hold_seconds << " s\n";
        }

        log_path = raven_control::logging::makeTimestampedLogPath(
            "logs", logPrefix(session.name));
        logger.emplace(
            diagnosticJointNames(), DIAGNOSTIC_RESERVE_SAMPLES);
        std::cout << "Motion log: " << log_path << '\n';

        const auto enable_result = driver.enableAll();
        if (enable_result !=
            raven_control::hal::MotorCommandResult::Sent) {
            throw std::runtime_error(
                "Enable rejected: " + std::string(
                    raven_control::hal::toString(enable_result)));
        }

        PlaybackRuntime runtime(
            driver, motor_config, bindings, limiters, *logger);
        runtime.resetGravityRamp();
        JointVector current_pose = fresh_start;
        RunResult result = RunResult::Completed;
        if (motor_config.gravity_compensation.enabled) {
            result = runtime.holdPoseFor(
                "Gravity Ramp",
                fresh_start,
                std::chrono::duration<double>(
                    motor_config.gravity_compensation.ramp_duration).count(),
                session.controlled_stop_seconds,
                current_pose);
        }
        for (const PlannedSegment& segment : plan) {
            if (result != RunResult::Completed)
                break;
            result = runtime.driveSegment(
                segment.name,
                segment.trajectory,
                true,
                session.controlled_stop_seconds,
                current_pose);
            if (result != RunResult::Completed)
                break;
            result = runtime.holdPoseFor(
                segment.name + " Hold",
                current_pose,
                segment.hold_seconds,
                session.controlled_stop_seconds,
                current_pose);
        }

        if (result == RunResult::FeedbackHold)
            runtime.holdAfterFeedbackLossUntilSpace(current_pose);
        else
            runtime.holdFinalPoseUntilSpace(current_pose);

        if (!driver.stopAll())
            throw std::runtime_error("Failed to send final stop");
        stop_guard.release();
        std::cout << "\nMotors disabled by user\n";

        if (logger->sampleCount() > 0) {
            logger->saveCsv(log_path);
            log_saved = true;
            std::cout << "Motion log saved: " << log_path
                      << " (" << logger->sampleCount() << " samples)\n";
        }
        return 0;
    } catch (const std::exception& error) {
        if (logger && !log_saved && logger->sampleCount() > 0 &&
            !log_path.empty()) {
            try {
                logger->saveCsv(log_path);
                std::cerr << "\nMotion log saved after failure: "
                          << log_path << '\n';
            } catch (const std::exception& log_error) {
                std::cerr << "\nFailed to save motion log: "
                          << log_error.what() << '\n';
            }
        }
        std::cerr << "\nFatal error: " << error.what() << '\n';
        return 1;
    }
}
