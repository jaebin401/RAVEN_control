#include "raven_control/config/motor_config.hpp"
#include "raven_control/control/gravity_feedforward_controller.hpp"
#include "raven_control/dynamics/pinocchio_gravity_model.hpp"
#include "raven_control/hal/can_interface.hpp"
#include "raven_control/hal/motor_driver.hpp"
#include "raven_control/logging/motion_logger.hpp"
#include "raven_control/safety/joint_limiter.hpp"

#include <yaml-cpp/yaml.h>

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

enum class RepeatMode {
    Finite,
    Infinite,
};

struct MotionPhase {
    std::string name;
    JointVector target_rad{};
    double transition_seconds = 0.0;
    double hold_seconds = 0.0;
};

struct MotionScript {
    std::string name;
    std::size_t declared_phase_count = 0;
    double startup_transition_seconds = 0.0;
    double startup_hold_seconds = 0.0;
    RepeatMode repeat_mode = RepeatMode::Finite;
    std::uint64_t repeat_count = 1;
    double controlled_stop_seconds = 1.0;
    std::vector<MotionPhase> phases;
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

struct DiagnosticState {
    std::uint64_t cycle_index = 0;
};

enum class RunResult {
    Completed,
    StopRequested,
    FeedbackHold,
};

enum class MotionInput {
    None,
    Stop,
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

template <typename T>
T requiredValue(
    const YAML::Node& node,
    const char* field,
    const std::string& context)
{
    const YAML::Node value = node[field];
    if (!value) {
        throw std::runtime_error(
            context + " is missing '" + field + "'");
    }
    try {
        return value.as<T>();
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            context + " has invalid '" + field + "': " +
            error.what());
    }
}

void requireMap(const YAML::Node& node, const std::string& context)
{
    if (!node || !node.IsMap())
        throw std::runtime_error(context + " must be a map");
}

void requireFiniteNonnegative(
    double value,
    const std::string& field,
    bool allow_zero)
{
    if (!std::isfinite(value) || value < 0.0 ||
        (!allow_zero && value == 0.0)) {
        throw std::runtime_error(
            field + (allow_zero
                ? " must be finite and nonnegative"
                : " must be finite and positive"));
    }
}

MotionScript loadMotionScript(const std::string& path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            "Cannot load motion script '" + path + "': " +
            error.what());
    }
    requireMap(root, "Motion script root");

    const int schema_version =
        requiredValue<int>(root, "schema_version", "Motion script");
    if (schema_version != 1) {
        throw std::runtime_error(
            "Unsupported motion script schema_version: " +
            std::to_string(schema_version));
    }

    MotionScript script;
    const YAML::Node demo = root["demo"];
    requireMap(demo, "demo");
    script.name = requiredValue<std::string>(demo, "name", "demo");
    if (script.name.empty())
        throw std::runtime_error("demo.name must not be empty");
    script.declared_phase_count =
        requiredValue<std::size_t>(demo, "phase_count", "demo");

    const YAML::Node startup = root["startup"];
    requireMap(startup, "startup");
    const std::string startup_target =
        requiredValue<std::string>(startup, "target", "startup");
    if (startup_target != "joint_zero") {
        throw std::runtime_error(
            "startup.target must be 'joint_zero'");
    }
    script.startup_transition_seconds =
        requiredValue<double>(startup, "transition_s", "startup");
    script.startup_hold_seconds =
        requiredValue<double>(startup, "hold_s", "startup");
    requireFiniteNonnegative(
        script.startup_transition_seconds,
        "startup.transition_s",
        false);
    requireFiniteNonnegative(
        script.startup_hold_seconds,
        "startup.hold_s",
        true);

    const YAML::Node repeat = root["repeat"];
    requireMap(repeat, "repeat");
    const std::string repeat_mode =
        requiredValue<std::string>(repeat, "mode", "repeat");
    if (repeat_mode == "finite") {
        script.repeat_mode = RepeatMode::Finite;
        script.repeat_count =
            requiredValue<std::uint64_t>(repeat, "count", "repeat");
        if (script.repeat_count == 0)
            throw std::runtime_error("repeat.count must be at least 1");
    } else if (repeat_mode == "infinite") {
        script.repeat_mode = RepeatMode::Infinite;
    } else {
        throw std::runtime_error(
            "repeat.mode must be 'finite' or 'infinite'");
    }

    const YAML::Node controlled_stop = root["controlled_stop"];
    requireMap(controlled_stop, "controlled_stop");
    script.controlled_stop_seconds = requiredValue<double>(
        controlled_stop, "duration_s", "controlled_stop");
    requireFiniteNonnegative(
        script.controlled_stop_seconds,
        "controlled_stop.duration_s",
        false);

    const YAML::Node phases = root["phases"];
    if (!phases || !phases.IsSequence())
        throw std::runtime_error("phases must be a sequence");
    if (phases.size() == 0)
        throw std::runtime_error("phases must not be empty");
    if (phases.size() != script.declared_phase_count) {
        throw std::runtime_error(
            "demo.phase_count does not match phases sequence size");
    }

    script.phases.reserve(phases.size());
    for (std::size_t phase_index = 0;
         phase_index < phases.size();
         ++phase_index) {
        const YAML::Node phase_node = phases[phase_index];
        const std::string context =
            "phases[" + std::to_string(phase_index) + "]";
        requireMap(phase_node, context);

        MotionPhase phase;
        phase.name = requiredValue<std::string>(
            phase_node, "name", context);
        if (phase.name.empty())
            throw std::runtime_error(context + ".name must not be empty");
        phase.transition_seconds = requiredValue<double>(
            phase_node, "transition_s", context);
        phase.hold_seconds = requiredValue<double>(
            phase_node, "hold_s", context);
        requireFiniteNonnegative(
            phase.transition_seconds,
            context + ".transition_s",
            false);
        requireFiniteNonnegative(
            phase.hold_seconds,
            context + ".hold_s",
            true);

        const YAML::Node targets = phase_node["target_from_zero_deg"];
        requireMap(targets, context + ".target_from_zero_deg");
        if (targets.size() != JOINT_NAMES.size()) {
            throw std::runtime_error(
                context + ".target_from_zero_deg must contain "
                "exactly three joints");
        }
        for (const auto& entry : targets) {
            const std::string joint_name =
                entry.first.as<std::string>();
            if (std::find(
                    JOINT_NAMES.begin(),
                    JOINT_NAMES.end(),
                    joint_name) == JOINT_NAMES.end()) {
                throw std::runtime_error(
                    context + " contains unknown joint '" +
                    joint_name + "'");
            }
        }
        for (std::size_t joint_index = 0;
             joint_index < JOINT_NAMES.size();
             ++joint_index) {
            const double degrees = requiredValue<double>(
                targets,
                JOINT_NAMES[joint_index],
                context + ".target_from_zero_deg");
            if (!std::isfinite(degrees)) {
                throw std::runtime_error(
                    context + " has a non-finite target for '" +
                    JOINT_NAMES[joint_index] + "'");
            }
            phase.target_rad[joint_index] =
                degreesToRadians(degrees);
        }
        script.phases.push_back(std::move(phase));
    }
    return script;
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
    const double delta_position =
        goal_position -
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
            "scripted_motion_demo requires exactly three joints");
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
        for (std::size_t joint_index = 0;
             joint_index < JOINT_NAMES.size();
             ++joint_index) {
            const auto& limiter = limiterFor(limiters, joint_index);
            if (sample.position[joint_index] < limiter.softMinRad() ||
                sample.position[joint_index] > limiter.softMaxRad()) {
                if (reason != nullptr) {
                    *reason = std::string(JOINT_NAMES[joint_index]) +
                        " position leaves its soft range";
                }
                return false;
            }
            if (std::abs(sample.velocity[joint_index]) >
                bindings[joint_index]->
                    position_control.max_slew_rate_rad_s + 1e-9) {
                if (reason != nullptr) {
                    *reason = std::string(JOINT_NAMES[joint_index]) +
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

void validateFixedPlan(
    const MotionScript& script,
    const JointBindings& bindings,
    const raven_control::safety::JointLimiterMap& limiters)
{
    const JointVector zero_pose{};
    validatePose(
        zero_pose, "Joint-zero startup pose", limiters,
        DEMO_CLEARANCE_RAD);
    for (const MotionPhase& phase : script.phases) {
        validatePose(
            phase.target_rad,
            "Phase '" + phase.name + "'",
            limiters,
            DEMO_CLEARANCE_RAD);
    }

    JointVector start = zero_pose;
    for (const MotionPhase& phase : script.phases) {
        const QuinticSegment segment = makeRestToRestSegment(
            start, phase.target_rad, phase.transition_seconds);
        validateSegment(segment, phase.name, bindings, limiters);
        validateStopEnvelope(
            segment,
            script.controlled_stop_seconds,
            phase.name,
            bindings,
            limiters);
        start = phase.target_rad;
    }

    const bool repeats =
        script.repeat_mode == RepeatMode::Infinite ||
        script.repeat_count > 1;
    if (repeats) {
        const MotionPhase& first = script.phases.front();
        const QuinticSegment wrap = makeRestToRestSegment(
            start, first.target_rad, first.transition_seconds);
        validateSegment(wrap, "loop wrap", bindings, limiters);
        validateStopEnvelope(
            wrap,
            script.controlled_stop_seconds,
            "loop wrap",
            bindings,
            limiters);
    }
}

JointVector readJointPositions(raven_control::hal::MotorDriver& driver)
{
    if (!driver.allFeedbackValid())
        throw std::runtime_error("Fresh feedback is unavailable");
    JointVector positions{};
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto feedback = driver.feedback(JOINT_NAMES[index]);
        if (!feedback || !feedback->valid) {
            throw std::runtime_error(
                "Missing feedback for '" +
                std::string(JOINT_NAMES[index]) + "'");
        }
        positions[index] = feedback->position_rad;
    }
    return positions;
}

void requestInitialFeedback(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config)
{
    if (!driver.requestMechanicalPositions())
        throw std::runtime_error("Failed to request Type 17 feedback");
    const auto deadline = std::chrono::steady_clock::now() +
        std::max(std::chrono::milliseconds(500),
                 config.feedback_timeout * 2);
    while (!driver.allFeedbackValid() &&
           std::chrono::steady_clock::now() < deadline) {
        (void)driver.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!driver.allFeedbackValid()) {
        throw std::runtime_error(
            "Timed out waiting for fresh Type 17 feedback");
    }
}

double safeStartupDuration(
    const JointVector& current,
    double requested_seconds,
    const JointBindings& bindings)
{
    double duration = requested_seconds;
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const double required =
            1.875 * std::abs(current[index]) /
            bindings[index]->position_control.max_slew_rate_rad_s;
        duration = std::max(duration, required * 1.05);
    }
    return duration;
}

void printScript(const MotionScript& script)
{
    std::cout
        << "\nMotion script: " << script.name << '\n'
        << "Startup target: calibrated joint zero\n"
        << "Repeat: ";
    if (script.repeat_mode == RepeatMode::Infinite)
        std::cout << "infinite\n";
    else
        std::cout << script.repeat_count << " cycle(s)\n";
    std::cout << std::fixed << std::setprecision(1);
    for (std::size_t phase_index = 0;
         phase_index < script.phases.size();
         ++phase_index) {
        const MotionPhase& phase = script.phases[phase_index];
        std::cout << "  " << (phase_index + 1) << ". "
                  << phase.name << " | ";
        for (std::size_t joint_index = 0;
             joint_index < JOINT_NAMES.size();
             ++joint_index) {
            std::cout << JOINT_NAMES[joint_index] << '='
                      << radiansToDegrees(phase.target_rad[joint_index])
                      << " deg";
            if (joint_index + 1 < JOINT_NAMES.size())
                std::cout << " | ";
        }
        std::cout << " | move=" << phase.transition_seconds
                  << " s hold=" << phase.hold_seconds << " s\n";
    }
}

void resetGravityRamp(GravityState& gravity)
{
    gravity.controller.reset(std::chrono::steady_clock::now());
    gravity.last_result = {};
    gravity.last_applied_torque_nm = {};
}

raven_control::control::GravityFeedforwardResult gravityTorque(
    raven_control::hal::MotorDriver& driver,
    GravityState& gravity)
{
    const auto now = std::chrono::steady_clock::now();
    raven_control::dynamics::JointVector positions{};
    bool feedback_fresh = true;
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto feedback = driver.feedback(JOINT_NAMES[index]);
        if (!feedback || !feedback->valid ||
            !feedback->operation_feedback_valid ||
            now < feedback->operation_received_at ||
            now - feedback->operation_received_at >
                gravity.feedback_timeout) {
            feedback_fresh = false;
            break;
        }
        positions[index] = feedback->position_rad;
    }
    gravity.last_result = gravity.controller.compute(
        positions, feedback_fresh, now);
    return gravity.last_result;
}

MotionInput pollMotionInput(GravityState& gravity)
{
    if (stop_requested != 0)
        return MotionInput::Stop;
    if (!keyAvailable())
        return MotionInput::None;
    const auto key = readKey();
    if (!key)
        return MotionInput::None;
    if (*key == 'q' || *key == 'Q')
        return MotionInput::Stop;
    if (*key == 'g' || *key == 'G') {
        const bool enabled = !gravity.controller.enabled();
        gravity.controller.setEnabled(
            enabled, std::chrono::steady_clock::now());
        std::cout << "\nGravity compensation "
                  << (enabled ? "ON" : "OFF")
                  << (gravity.controller.config().dry_run
                          ? " [DRY-RUN]" : "")
                  << " (configured ramp and limits apply)\n";
    } else if (*key == ' ') {
        std::cout
            << "\nMotion is active. Press Q to stop smoothly first.\n";
    }
    return MotionInput::None;
}

CommandDispatch sendTargets(
    raven_control::hal::MotorDriver& driver,
    const JointBindings& bindings,
    const JointVector& target,
    const JointVector& target_velocity,
    GravityState& gravity)
{
    CommandDispatch dispatch;
    dispatch.gravity = gravityTorque(driver, gravity);
    const JointVector& requested_torque =
        dispatch.gravity.commanded_torque_nm;
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto result = driver.sendMitCommand(
            JOINT_NAMES[index],
            target[index],
            target_velocity[index],
            bindings[index]->position_control.kp,
            bindings[index]->position_control.kd,
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
        gravity.last_applied_torque_nm = requested_torque;
    dispatch.feedforward_torque_nm = gravity.last_applied_torque_nm;
    return dispatch;
}

void recordSample(
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic,
    const std::string& phase,
    std::chrono::steady_clock::time_point scheduled_at,
    std::chrono::steady_clock::time_point actual_at,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    raven_control::hal::MotorDriver& driver,
    const JointVector& target,
    const JointVector& target_velocity,
    const JointVector& feedforward_torque,
    const raven_control::control::GravityFeedforwardResult& gravity)
{
    const auto now = std::chrono::steady_clock::now();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    raven_control::logging::MotionSample sample;
    sample.cycle_index = diagnostic.cycle_index++;
    sample.phase = phase;
    sample.scheduled_at = scheduled_at;
    sample.actual_at = actual_at;
    sample.control_period =
        std::chrono::duration_cast<std::chrono::microseconds>(
            config.control_period);
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
        joint.kp = bindings[index]->position_control.kp;
        joint.kd = bindings[index]->position_control.kd;
        const auto feedback = driver.feedback(JOINT_NAMES[index]);
        if (!feedback || !feedback->valid) {
            joint.actual_position_rad = nan;
            joint.actual_velocity_rad_s = nan;
            joint.position_error_rad = nan;
            joint.measured_torque_nm = nan;
            joint.motor_temperature_celsius = nan;
            joint.feedback_age_ms = nan;
            joint.operation_feedback_age_ms = nan;
            continue;
        }
        joint.feedback_valid = true;
        joint.actual_position_rad = feedback->position_rad;
        joint.position_error_rad = target[index] - feedback->position_rad;
        joint.feedback_age_ms =
            std::chrono::duration<double, std::milli>(
                now - feedback->received_at).count();
        const bool operation_fresh =
            feedback->operation_feedback_valid &&
            now - feedback->operation_received_at <=
                config.feedback_timeout;
        joint.operation_feedback_valid = operation_fresh;
        if (operation_fresh) {
            joint.actual_velocity_rad_s = feedback->velocity_rad_s;
            joint.measured_torque_nm = feedback->torque_nm;
            joint.motor_temperature_celsius = feedback->temperature_celsius;
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
    }
    logger.record(std::move(sample));
}

void printStatus(
    const std::string& phase,
    raven_control::hal::MotorDriver& driver,
    const JointVector& target,
    const GravityState& gravity)
{
    std::cout << "\r\033[K" << std::setw(18) << phase
              << " | G:"
              << (gravity.controller.enabled() ? "ON " : "OFF")
              << (gravity.controller.config().dry_run ? "DRY " : "LIVE ")
              << '(' << std::setw(3)
              << static_cast<int>(std::round(
                     gravity.controller.rampFactor() * 100.0))
              << "%) | " << std::fixed << std::setprecision(1);
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto feedback = driver.feedback(JOINT_NAMES[index]);
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

RunResult driveSegment(
    const std::string& phase_name,
    const QuinticSegment& segment,
    bool accept_stop_request,
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    const raven_control::safety::JointLimiterMap& limiters,
    double controlled_stop_seconds,
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic,
    GravityState& gravity,
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

        if (pollMotionInput(gravity) == MotionInput::Stop) {
            stop_requested = 0;
            if (accept_stop_request) {
                const TrajectorySample current = segment.sample(elapsed);
                const QuinticSegment braking = makeControlledStopSegment(
                    current, controlled_stop_seconds);
                std::string reason;
                if (segmentIsSafe(braking, bindings, limiters, &reason)) {
                    std::cout
                        << "\nStop requested. Controlled deceleration "
                        << "started.\n";
                    return driveSegment(
                        "Controlled Stop",
                        braking,
                        false,
                        driver,
                        config,
                        bindings,
                        limiters,
                        controlled_stop_seconds,
                        logger,
                        diagnostic,
                        gravity,
                        final_pose) == RunResult::FeedbackHold
                        ? RunResult::FeedbackHold
                        : RunResult::StopRequested;
                }
                std::cout
                    << "\nA mid-segment stop would violate the safe "
                    << "envelope (" << reason << "). Finishing the "
                    << "current quintic segment before holding.\n";
                stop_at_segment_end = true;
            }
        }

        (void)driver.poll();
        if (driver.faultLatched())
            throw std::runtime_error(driver.faultReason());
        const TrajectorySample target = segment.sample(elapsed);
        const CommandDispatch dispatch = sendTargets(
            driver, bindings, target.position, target.velocity, gravity);
        recordSample(
            logger,
            diagnostic,
            phase_name,
            next_cycle,
            now,
            config,
            bindings,
            driver,
            target.position,
            target.velocity,
            dispatch.feedforward_torque_nm,
            dispatch.gravity);
        final_pose = target.position;
        if (dispatch.feedback_hold)
            return RunResult::FeedbackHold;
        if (now >= next_status) {
            printStatus(phase_name, driver, target.position, gravity);
            next_status = now + std::chrono::milliseconds(100);
        }
        next_cycle += config.control_period;
        std::this_thread::sleep_until(next_cycle);
    }

    const TrajectorySample goal = segment.sample(segment.duration_seconds);
    const CommandDispatch dispatch = sendTargets(
        driver, bindings, goal.position, goal.velocity, gravity);
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
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    const raven_control::safety::JointLimiterMap& limiters,
    double controlled_stop_seconds,
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic,
    GravityState& gravity,
    JointVector& final_pose)
{
    if (hold_seconds <= 0.0) {
        final_pose = pose;
        return RunResult::Completed;
    }
    const QuinticSegment hold = makeRestToRestSegment(
        pose, pose, hold_seconds);
    return driveSegment(
        phase_name,
        hold,
        true,
        driver,
        config,
        bindings,
        limiters,
        controlled_stop_seconds,
        logger,
        diagnostic,
        gravity,
        final_pose);
}

bool waitForStart(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config)
{
    std::cout
        << "\nSPACE: capture the current pose and start\n"
        << "Q: cancel while motors remain disabled\n"
        << std::flush;
    auto next_request = std::chrono::steady_clock::now();
    while (true) {
        if (stop_requested != 0)
            return false;
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_request) {
            if (!driver.requestMechanicalPositions()) {
                throw std::runtime_error(
                    "Failed to request Type 17 feedback");
            }
            next_request = now + config.position_request_period;
        }
        (void)driver.poll();
        if (driver.faultLatched())
            throw std::runtime_error(driver.faultReason());
        if (keyAvailable()) {
            const auto key = readKey();
            if (key && *key == ' ')
                return true;
            if (key && (*key == 'q' || *key == 'Q'))
                return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void holdAfterFeedbackLossUntilSpace(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    const JointVector& last_target,
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic,
    GravityState& gravity)
{
    std::cout
        << "\nType 2 feedback lost: "
        << driver.feedbackHoldReason() << '\n'
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
        (void)driver.poll();
        if (driver.faultLatched())
            throw std::runtime_error(driver.faultReason());
        const CommandDispatch dispatch = sendTargets(
            driver, bindings, last_target, zero_velocity, gravity);
        recordSample(
            logger,
            diagnostic,
            "Feedback Hold",
            next_cycle,
            now,
            config,
            bindings,
            driver,
            last_target,
            zero_velocity,
            dispatch.feedforward_torque_nm,
            dispatch.gravity);
        if (now >= next_status) {
            printStatus("Feedback Hold", driver, last_target, gravity);
            next_status = now + std::chrono::milliseconds(200);
        }
        next_cycle += config.control_period;
        std::this_thread::sleep_until(next_cycle);
    }
}

void holdFinalPoseUntilSpace(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    const JointVector& final_pose,
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic,
    GravityState& gravity)
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
            if (key && (*key == 'g' || *key == 'G')) {
                const bool enabled = !gravity.controller.enabled();
                gravity.controller.setEnabled(
                    enabled, std::chrono::steady_clock::now());
                std::cout << "\nGravity compensation "
                          << (enabled ? "ON" : "OFF")
                          << (gravity.controller.config().dry_run
                                  ? " [DRY-RUN]" : "")
                          << " (configured ramp and limits apply)\n";
            }
        }
        if (stop_requested != 0) {
            stop_requested = 0;
            std::cout << "\nFinal Hold requires SPACE to disable.\n";
        }
        (void)driver.poll();
        if (driver.faultLatched())
            throw std::runtime_error(driver.faultReason());
        const CommandDispatch dispatch = sendTargets(
            driver, bindings, final_pose, zero_velocity, gravity);
        recordSample(
            logger,
            diagnostic,
            "Final Hold",
            next_cycle,
            now,
            config,
            bindings,
            driver,
            final_pose,
            zero_velocity,
            dispatch.feedforward_torque_nm,
            dispatch.gravity);
        if (dispatch.feedback_hold) {
            holdAfterFeedbackLossUntilSpace(
                driver,
                config,
                bindings,
                final_pose,
                logger,
                diagnostic,
                gravity);
            return;
        }
        if (now >= next_status) {
            printStatus("Final Hold", driver, final_pose, gravity);
            next_status = now + std::chrono::milliseconds(200);
        }
        next_cycle += config.control_period;
        std::this_thread::sleep_until(next_cycle);
    }
}

std::string logPrefix(const std::string& demo_name)
{
    std::string result = "scripted_motion_";
    for (const char character : demo_name) {
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

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr
            << "Usage: scripted_motion_demo <motion_script.yaml>\n";
        return 2;
    }

    const std::string motion_script_path = argv[1];
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::optional<raven_control::logging::MotionLogger> logger;
    std::string log_path;
    bool log_saved = false;

    try {
        const MotionScript script = loadMotionScript(motion_script_path);
        auto limiters = raven_control::safety::loadJointLimiters(
            JOINT_LIMITS_PATH);
        const auto motor_config =
            raven_control::config::loadMotorRuntimeConfig(
                MOTOR_CONFIG_PATH);
        const JointBindings bindings = bindConfiguredJoints(motor_config);
        validateFixedPlan(script, bindings, limiters);
        printScript(script);

        log_path = raven_control::logging::makeTimestampedLogPath(
            "logs", logPrefix(script.name));
        logger.emplace(
            diagnosticJointNames(), DIAGNOSTIC_RESERVE_SAMPLES);
        DiagnosticState diagnostic;
        GravityState gravity(motor_config);

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
        requestInitialFeedback(driver, motor_config);

        std::cout
            << "\nRAVEN scripted motion demo\n"
            << "Motion script: " << motion_script_path << '\n'
            << "CAN interface: " << CAN_INTERFACE << '\n'
            << "Joint limits: " << JOINT_LIMITS_PATH << '\n'
            << "Motor config: " << MOTOR_CONFIG_PATH << '\n'
            << "Gravity compensation: "
            << (gravity.controller.enabled() ? "ON" : "OFF")
            << (gravity.controller.config().dry_run
                    ? " [DRY-RUN]" : " [LIVE]")
            << " scale=" << gravity.controller.config().scale
            << " URDF=" << gravity.controller.config().urdf_path << '\n'
            << "Motion log: " << log_path << '\n'
            << "ROS 2 joint-state output remains available through "
            << "the separate read-only raven_joint_state_bridge.\n";

        TerminalMode terminal;
        if (!waitForStart(driver, motor_config)) {
            std::cout << "\nDemo cancelled\n";
            return 0;
        }

        requestInitialFeedback(driver, motor_config);
        const JointVector captured_start = readJointPositions(driver);
        validatePose(
            captured_start,
            "Captured start pose",
            limiters,
            0.0);
        const double startup_duration = safeStartupDuration(
            captured_start,
            script.startup_transition_seconds,
            bindings);
        if (startup_duration > script.startup_transition_seconds + 1e-9) {
            std::cout
                << "\nStartup transition extended from "
                << script.startup_transition_seconds << " s to "
                << startup_duration
                << " s to respect max_slew_rate_rad_s.\n";
        }
        const JointVector zero_pose{};
        const QuinticSegment startup = makeRestToRestSegment(
            captured_start, zero_pose, startup_duration);
        validateSegment(startup, "Move To Zero", bindings, limiters);
        validateStopEnvelope(
            startup,
            script.controlled_stop_seconds,
            "Move To Zero",
            bindings,
            limiters);

        const auto enable_result = driver.enableAll();
        if (enable_result !=
            raven_control::hal::MotorCommandResult::Sent) {
            throw std::runtime_error(
                "Enable rejected: " + std::string(
                    raven_control::hal::toString(enable_result)));
        }
        resetGravityRamp(gravity);

        JointVector current_pose = captured_start;
        RunResult result = RunResult::Completed;
        if (gravity.controller.enabled()) {
            result = holdPoseFor(
                "Gravity Ramp",
                captured_start,
                std::chrono::duration<double>(
                    motor_config.gravity_compensation.ramp_duration).count(),
                driver,
                motor_config,
                bindings,
                limiters,
                script.controlled_stop_seconds,
                *logger,
                diagnostic,
                gravity,
                current_pose);
        }
        if (result == RunResult::Completed) {
            result = driveSegment(
                "Move To Zero",
                startup,
                true,
                driver,
                motor_config,
                bindings,
                limiters,
                script.controlled_stop_seconds,
                *logger,
                diagnostic,
                gravity,
                current_pose);
        }
        if (result == RunResult::Completed) {
            result = holdPoseFor(
                "Zero Hold",
                zero_pose,
                script.startup_hold_seconds,
                driver,
                motor_config,
                bindings,
                limiters,
                script.controlled_stop_seconds,
                *logger,
                diagnostic,
                gravity,
                current_pose);
        }

        std::uint64_t cycle = 0;
        while (result == RunResult::Completed &&
               (script.repeat_mode == RepeatMode::Infinite ||
                cycle < script.repeat_count)) {
            for (std::size_t phase_index = 0;
                 phase_index < script.phases.size();
                 ++phase_index) {
                const MotionPhase& phase = script.phases[phase_index];
                const QuinticSegment segment = makeRestToRestSegment(
                    current_pose,
                    phase.target_rad,
                    phase.transition_seconds);
                result = driveSegment(
                    phase.name,
                    segment,
                    true,
                    driver,
                    motor_config,
                    bindings,
                    limiters,
                    script.controlled_stop_seconds,
                    *logger,
                    diagnostic,
                    gravity,
                    current_pose);
                if (result != RunResult::Completed)
                    break;
                result = holdPoseFor(
                    phase.name + " Hold",
                    current_pose,
                    phase.hold_seconds,
                    driver,
                    motor_config,
                    bindings,
                    limiters,
                    script.controlled_stop_seconds,
                    *logger,
                    diagnostic,
                    gravity,
                    current_pose);
                if (result != RunResult::Completed)
                    break;
            }
            ++cycle;
        }

        if (result == RunResult::FeedbackHold) {
            holdAfterFeedbackLossUntilSpace(
                driver,
                motor_config,
                bindings,
                current_pose,
                *logger,
                diagnostic,
                gravity);
        } else {
            holdFinalPoseUntilSpace(
                driver,
                motor_config,
                bindings,
                current_pose,
                *logger,
                diagnostic,
                gravity);
        }

        if (!driver.stopAll())
            throw std::runtime_error("Failed to send final stop");
        stop_guard.release();
        std::cout << "\nMotors disabled by user\n";

        if (logger->sampleCount() > 0) {
            logger->saveCsv(log_path);
            log_saved = true;
            std::cout
                << "Motion log saved: " << log_path
                << " (" << logger->sampleCount() << " samples)\n";
        }
        return 0;
    } catch (const std::exception& error) {
        if (logger && !log_saved && logger->sampleCount() > 0 &&
            !log_path.empty()) {
            try {
                logger->saveCsv(log_path);
                std::cerr
                    << "\nMotion log saved after failure: "
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
