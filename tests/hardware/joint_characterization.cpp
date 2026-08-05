#include "raven_control/config/motor_config.hpp"
#include "raven_control/dynamics/gravity_compensator.hpp"
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
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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
constexpr std::array<const char*, 3> JOINT_NAMES{{
    "shoulder_Joint",
    "upperArm_Joint",
    "foreArm_Joint",
}};
constexpr std::array<double, 3> GRAVITY_DIRECTION_SCALE{{
    1.0,
    -1.0,
    -1.0,
}};

using JointVector = std::array<double, JOINT_NAMES.size()>;
using JointBindings = std::array<
    const raven_control::config::JointMotorRuntimeConfig*,
    JOINT_NAMES.size()>;

enum class MotionKind {
    Quintic,
    ConstantVelocity,
    Step,
    Hold,
};

struct Phase {
    std::string name;
    MotionKind motion = MotionKind::Hold;
    JointVector target_rad{};
    JointVector kp{};
    JointVector kd{};
    double duration_seconds = 0.0;
    double hold_seconds = 0.0;
    double velocity_rad_s = 0.0;
    double acceleration_seconds = 0.0;
    double gravity_scale = 0.0;
    bool analysis_window = false;
};

struct TestPlan {
    std::string name;
    std::string type;
    std::string description;
    std::string can_interface = "can0";
    std::string joint_limits_path = "config/joint_limits.yaml";
    std::string motor_config_path = "config/motor_config.yaml";
    std::string output_root = "logs/characterization";
    std::chrono::milliseconds vbus_request_period{100};
    double soft_limit_clearance_rad = 2.0 * PI / 180.0;
    double max_step_rad = 5.0 * PI / 180.0;
    double startup_transition_seconds = 3.0;
    double final_hold_seconds = 1.0;
    double startup_gravity_scale = 1.0;
    std::size_t repeat_count = 1;
    JointVector startup_target_rad{};
    std::vector<Phase> phases;
};

struct TrajectorySample {
    JointVector position{};
    JointVector velocity{};
    std::string subphase;
};

struct PhaseTrajectory {
    Phase phase;
    JointVector start{};
    double total_seconds = 0.0;
    std::optional<std::size_t> moving_joint;
    double direction = 0.0;
    double acceleration = 0.0;
    double acceleration_seconds = 0.0;
    double cruise_seconds = 0.0;
    double peak_velocity = 0.0;

    [[nodiscard]] TrajectorySample sample(double elapsed_seconds) const;
};

struct DiagnosticState {
    std::uint64_t cycle_index = 0;
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
    if (!value)
        throw std::runtime_error(context + " is missing '" + field + "'");
    try {
        return value.as<T>();
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            context + " has invalid '" + field + "': " + error.what());
    }
}

void requireMap(const YAML::Node& node, const std::string& context)
{
    if (!node || !node.IsMap())
        throw std::runtime_error(context + " must be a map");
}

void requireFinite(double value, const std::string& context)
{
    if (!std::isfinite(value))
        throw std::runtime_error(context + " must be finite");
}

JointVector readDegreesMap(
    const YAML::Node& node,
    const std::string& context)
{
    requireMap(node, context);
    if (node.size() != JOINT_NAMES.size()) {
        throw std::runtime_error(
            context + " must contain exactly three joints");
    }
    JointVector result{};
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const double degrees = requiredValue<double>(
            node, JOINT_NAMES[index], context);
        requireFinite(degrees, context + "." + JOINT_NAMES[index]);
        result[index] = degreesToRadians(degrees);
    }
    return result;
}

MotionKind parseMotionKind(const std::string& value)
{
    if (value == "quintic")
        return MotionKind::Quintic;
    if (value == "constant_velocity")
        return MotionKind::ConstantVelocity;
    if (value == "step")
        return MotionKind::Step;
    if (value == "hold")
        return MotionKind::Hold;
    throw std::runtime_error(
        "motion must be quintic, constant_velocity, step, or hold");
}

JointBindings bindConfiguredJoints(
    const raven_control::config::MotorRuntimeConfig& config)
{
    if (config.joints.size() != JOINT_NAMES.size()) {
        throw std::runtime_error(
            "joint_characterization requires exactly three joints");
    }
    JointBindings result{};
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        result[index] = config.findJoint(JOINT_NAMES[index]);
        if (result[index] == nullptr) {
            throw std::runtime_error(
                "Motor config is missing joint '" +
                std::string(JOINT_NAMES[index]) + "'");
        }
    }
    return result;
}

JointVector defaultKp(const JointBindings& bindings)
{
    JointVector result{};
    for (std::size_t index = 0; index < result.size(); ++index)
        result[index] = bindings[index]->position_control.kp;
    return result;
}

JointVector defaultKd(const JointBindings& bindings)
{
    JointVector result{};
    for (std::size_t index = 0; index < result.size(); ++index)
        result[index] = bindings[index]->position_control.kd;
    return result;
}

void applyGainOverrides(
    const YAML::Node& gains,
    const std::string& context,
    Phase& phase)
{
    if (!gains)
        return;
    requireMap(gains, context);
    for (const auto& entry : gains) {
        const std::string joint_name = entry.first.as<std::string>();
        const auto found = std::find(
            JOINT_NAMES.begin(), JOINT_NAMES.end(), joint_name);
        if (found == JOINT_NAMES.end()) {
            throw std::runtime_error(
                context + " contains unknown joint '" + joint_name + "'");
        }
        const std::size_t index = static_cast<std::size_t>(
            std::distance(JOINT_NAMES.begin(), found));
        requireMap(entry.second, context + "." + joint_name);
        phase.kp[index] = requiredValue<double>(
            entry.second, "kp", context + "." + joint_name);
        phase.kd[index] = requiredValue<double>(
            entry.second, "kd", context + "." + joint_name);
    }
}

TestPlan loadTestPlan(
    const std::string& path,
    const JointBindings& bindings)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            "Cannot load test plan '" + path + "': " + error.what());
    }
    requireMap(root, "Test plan root");
    if (requiredValue<int>(root, "schema_version", "Test plan") != 1)
        throw std::runtime_error("Unsupported test plan schema_version");

    TestPlan plan;
    const YAML::Node test = root["test"];
    requireMap(test, "test");
    plan.name = requiredValue<std::string>(test, "name", "test");
    plan.type = requiredValue<std::string>(test, "type", "test");
    if (const YAML::Node description = test["description"])
        plan.description = description.as<std::string>();
    if (const YAML::Node repeat = test["repeat_count"])
        plan.repeat_count = repeat.as<std::size_t>();
    if (plan.name.empty() || plan.type.empty() || plan.repeat_count == 0)
        throw std::runtime_error("test name/type and repeat_count are required");

    const YAML::Node runtime = root["runtime"];
    requireMap(runtime, "runtime");
    plan.can_interface = requiredValue<std::string>(
        runtime, "can_interface", "runtime");
    plan.joint_limits_path = requiredValue<std::string>(
        runtime, "joint_limits_path", "runtime");
    plan.motor_config_path = requiredValue<std::string>(
        runtime, "motor_config_path", "runtime");
    plan.output_root = requiredValue<std::string>(
        runtime, "output_root", "runtime");
    plan.vbus_request_period = std::chrono::milliseconds(
        requiredValue<int>(runtime, "vbus_request_period_ms", "runtime"));
    plan.soft_limit_clearance_rad = degreesToRadians(
        requiredValue<double>(runtime, "soft_limit_clearance_deg", "runtime"));
    plan.max_step_rad = degreesToRadians(
        requiredValue<double>(runtime, "max_step_deg", "runtime"));
    plan.startup_transition_seconds = requiredValue<double>(
        runtime, "startup_transition_s", "runtime");
    plan.final_hold_seconds = requiredValue<double>(
        runtime, "final_hold_s", "runtime");
    plan.startup_gravity_scale = requiredValue<double>(
        runtime, "startup_gravity_scale", "runtime");
    if (plan.vbus_request_period.count() <= 0 ||
        plan.soft_limit_clearance_rad < 0.0 ||
        plan.max_step_rad <= 0.0 ||
        plan.startup_transition_seconds <= 0.0 ||
        plan.final_hold_seconds < 0.0 ||
        plan.startup_gravity_scale < 0.0 ||
        plan.startup_gravity_scale > 1.0) {
        throw std::runtime_error("runtime contains an invalid safety value");
    }

    plan.startup_target_rad = readDegreesMap(
        root["startup_target_deg"], "startup_target_deg");

    const YAML::Node phases = root["phases"];
    if (!phases || !phases.IsSequence() || phases.size() == 0)
        throw std::runtime_error("phases must be a non-empty sequence");
    for (std::size_t phase_index = 0;
         phase_index < phases.size(); ++phase_index) {
        const YAML::Node node = phases[phase_index];
        const std::string context =
            "phases[" + std::to_string(phase_index) + "]";
        requireMap(node, context);
        Phase phase;
        phase.name = requiredValue<std::string>(node, "name", context);
        phase.motion = parseMotionKind(requiredValue<std::string>(
            node, "motion", context));
        phase.target_rad = readDegreesMap(
            node["target_deg"], context + ".target_deg");
        phase.kp = defaultKp(bindings);
        phase.kd = defaultKd(bindings);
        applyGainOverrides(node["gains"], context + ".gains", phase);
        phase.gravity_scale = requiredValue<double>(
            node, "gravity_scale", context);
        if (const YAML::Node analysis = node["analysis_window"])
            phase.analysis_window = analysis.as<bool>();
        if (const YAML::Node hold = node["hold_s"])
            phase.hold_seconds = hold.as<double>();

        if (phase.motion == MotionKind::ConstantVelocity) {
            phase.velocity_rad_s = degreesToRadians(
                requiredValue<double>(node, "velocity_deg_s", context));
            phase.acceleration_seconds = requiredValue<double>(
                node, "acceleration_s", context);
        } else {
            phase.duration_seconds = requiredValue<double>(
                node, "duration_s", context);
        }
        if (phase.name.empty() || phase.gravity_scale < 0.0 ||
            phase.gravity_scale > 1.0 || phase.hold_seconds < 0.0 ||
            (phase.motion == MotionKind::ConstantVelocity &&
             (phase.velocity_rad_s <= 0.0 ||
              phase.acceleration_seconds <= 0.0)) ||
            (phase.motion != MotionKind::ConstantVelocity &&
             phase.duration_seconds <= 0.0)) {
            throw std::runtime_error(context + " contains invalid values");
        }
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            requireFinite(phase.kp[index], context + ".kp");
            requireFinite(phase.kd[index], context + ".kd");
            if (phase.kp[index] < 0.0 ||
                phase.kp[index] >
                    raven_control::hal::RS02_OPERATION_MAX_KP ||
                phase.kd[index] < 0.0 ||
                phase.kd[index] >
                    raven_control::hal::RS02_OPERATION_MAX_KD) {
                throw std::runtime_error(
                    context + " contains an out-of-range gain");
            }
        }
        plan.phases.push_back(std::move(phase));
    }
    return plan;
}

std::string motorConfigPathFromPlan(const std::string& path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            "Cannot load test plan '" + path + "': " + error.what());
    }
    requireMap(root, "Test plan root");
    const YAML::Node runtime = root["runtime"];
    requireMap(runtime, "runtime");
    return requiredValue<std::string>(
        runtime, "motor_config_path", "runtime");
}

const raven_control::safety::JointLimiter& limiterFor(
    const raven_control::safety::JointLimiterMap& limiters,
    std::size_t index)
{
    const auto found = limiters.find(JOINT_NAMES[index]);
    if (found == limiters.end()) {
        throw std::runtime_error(
            "Joint limits are missing '" +
            std::string(JOINT_NAMES[index]) + "'");
    }
    return found->second;
}

void validatePose(
    const JointVector& pose,
    const std::string& name,
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
                name + " is outside the configured soft-limit clearance "
                "for '" + JOINT_NAMES[index] + "'");
        }
    }
}

double quinticBlend(double normalized)
{
    const double u = std::clamp(normalized, 0.0, 1.0);
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

double quinticBlendDerivative(double normalized)
{
    const double u = std::clamp(normalized, 0.0, 1.0);
    return 30.0 * u * u * (1.0 - u) * (1.0 - u);
}

PhaseTrajectory makeTrajectory(const Phase& phase, const JointVector& start)
{
    PhaseTrajectory trajectory;
    trajectory.phase = phase;
    trajectory.start = start;
    trajectory.total_seconds = phase.duration_seconds;

    if (phase.motion != MotionKind::ConstantVelocity)
        return trajectory;

    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        if (std::abs(phase.target_rad[index] - start[index]) > 1e-9) {
            if (trajectory.moving_joint) {
                throw std::runtime_error(
                    "constant_velocity phase '" + phase.name +
                    "' may move only one joint");
            }
            trajectory.moving_joint = index;
        }
    }
    if (!trajectory.moving_joint) {
        throw std::runtime_error(
            "constant_velocity phase '" + phase.name +
            "' has no moving joint");
    }

    const std::size_t index = *trajectory.moving_joint;
    const double delta = phase.target_rad[index] - start[index];
    const double distance = std::abs(delta);
    trajectory.direction = delta > 0.0 ? 1.0 : -1.0;
    trajectory.acceleration =
        phase.velocity_rad_s / phase.acceleration_seconds;
    const double nominal_ramp_distance =
        phase.velocity_rad_s * phase.acceleration_seconds;
    if (distance >= nominal_ramp_distance) {
        trajectory.peak_velocity = phase.velocity_rad_s;
        trajectory.acceleration_seconds = phase.acceleration_seconds;
        trajectory.cruise_seconds =
            distance / phase.velocity_rad_s - phase.acceleration_seconds;
    } else {
        trajectory.peak_velocity = std::sqrt(
            distance * trajectory.acceleration);
        trajectory.acceleration_seconds =
            trajectory.peak_velocity / trajectory.acceleration;
        trajectory.cruise_seconds = 0.0;
    }
    trajectory.total_seconds =
        2.0 * trajectory.acceleration_seconds +
        trajectory.cruise_seconds;
    return trajectory;
}

TrajectorySample PhaseTrajectory::sample(double elapsed_seconds) const
{
    const double elapsed = std::clamp(
        elapsed_seconds, 0.0, total_seconds);
    TrajectorySample result;
    result.position = start;

    if (phase.motion == MotionKind::Step ||
        phase.motion == MotionKind::Hold) {
        result.position = phase.target_rad;
        result.subphase = "hold";
        return result;
    }
    if (phase.motion == MotionKind::Quintic) {
        const double normalized = elapsed / total_seconds;
        const double blend = quinticBlend(normalized);
        const double derivative =
            quinticBlendDerivative(normalized) / total_seconds;
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            const double delta = phase.target_rad[index] - start[index];
            result.position[index] = start[index] + blend * delta;
            result.velocity[index] = derivative * delta;
        }
        result.subphase = "transition";
        return result;
    }

    const std::size_t index = *moving_joint;
    double distance = 0.0;
    double speed = 0.0;
    if (elapsed < acceleration_seconds) {
        distance = 0.5 * acceleration * elapsed * elapsed;
        speed = acceleration * elapsed;
        result.subphase = "accelerate";
    } else if (elapsed < acceleration_seconds + cruise_seconds) {
        const double cruise_elapsed = elapsed - acceleration_seconds;
        distance = 0.5 * peak_velocity * acceleration_seconds +
                   peak_velocity * cruise_elapsed;
        speed = peak_velocity;
        result.subphase = "cruise";
    } else {
        const double deceleration_elapsed = std::min(
            elapsed - acceleration_seconds - cruise_seconds,
            acceleration_seconds);
        const double before_deceleration =
            0.5 * peak_velocity * acceleration_seconds +
            peak_velocity * cruise_seconds;
        distance = before_deceleration +
                   peak_velocity * deceleration_elapsed -
                   0.5 * acceleration * deceleration_elapsed *
                       deceleration_elapsed;
        speed = std::max(
            0.0,
            peak_velocity - acceleration * deceleration_elapsed);
        result.subphase = "decelerate";
    }
    result.position[index] =
        start[index] + direction * distance;
    result.velocity[index] = direction * speed;
    if (elapsed >= total_seconds) {
        result.position = phase.target_rad;
        result.velocity = {};
    }
    return result;
}

void validatePlan(
    const TestPlan& plan,
    const JointBindings& bindings,
    const raven_control::safety::JointLimiterMap& limiters)
{
    validatePose(
        plan.startup_target_rad,
        "startup_target_deg",
        limiters,
        plan.soft_limit_clearance_rad);
    JointVector previous = plan.startup_target_rad;
    for (std::size_t repeat = 0; repeat < plan.repeat_count; ++repeat) {
        for (const Phase& phase : plan.phases) {
            validatePose(
                phase.target_rad,
                "Phase '" + phase.name + "'",
                limiters,
                plan.soft_limit_clearance_rad);
            if (phase.motion == MotionKind::Step) {
                std::size_t moving_joints = 0;
                for (std::size_t index = 0;
                     index < JOINT_NAMES.size(); ++index) {
                    const double step = std::abs(
                        phase.target_rad[index] - previous[index]);
                    if (step > 1e-9)
                        ++moving_joints;
                    if (step > plan.max_step_rad + 1e-9) {
                        throw std::runtime_error(
                            "Step phase '" + phase.name +
                            "' exceeds runtime.max_step_deg");
                    }
                }
                if (moving_joints != 1) {
                    throw std::runtime_error(
                        "Step phase '" + phase.name +
                        "' must move exactly one joint");
                }
            }
            const PhaseTrajectory trajectory =
                makeTrajectory(phase, previous);
            for (std::size_t index = 0;
                 index < JOINT_NAMES.size(); ++index) {
                double peak_velocity = 0.0;
                if (phase.motion == MotionKind::Quintic) {
                    peak_velocity = 1.875 * std::abs(
                        phase.target_rad[index] - previous[index]) /
                        phase.duration_seconds;
                } else if (phase.motion == MotionKind::ConstantVelocity &&
                           trajectory.moving_joint == index) {
                    peak_velocity = trajectory.peak_velocity;
                }
                if (peak_velocity >
                    bindings[index]->position_control.
                        max_slew_rate_rad_s + 1e-9) {
                    throw std::runtime_error(
                        "Phase '" + phase.name +
                        "' exceeds max_slew_rate_rad_s for '" +
                        JOINT_NAMES[index] + "'");
                }
            }
            previous = phase.target_rad;
        }
    }
}

std::vector<raven_control::hal::JointMotorConfig> motorMap(
    const JointBindings& bindings)
{
    std::vector<raven_control::hal::JointMotorConfig> result;
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

    void release() noexcept { active_ = false; }

private:
    raven_control::hal::MotorDriver& driver_;
    bool active_ = true;
};

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

private:
    termios original_{};
    bool active_ = false;
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
    if (::read(STDIN_FILENO, &key, 1) == 1)
        return key;
    return std::nullopt;
}

void requestInitialFeedback(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config)
{
    if (!driver.requestMechanicalPositions())
        throw std::runtime_error("Failed to request Type 17 positions");
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
            "Timed out waiting for fresh Type 17 positions");
    }
}

JointVector readJointPositions(raven_control::hal::MotorDriver& driver)
{
    JointVector result{};
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto feedback = driver.feedback(JOINT_NAMES[index]);
        if (!feedback || !feedback->valid) {
            throw std::runtime_error(
                "Missing feedback for '" +
                std::string(JOINT_NAMES[index]) + "'");
        }
        result[index] = feedback->position_rad;
    }
    return result;
}

JointVector gravityTorque(
    raven_control::hal::MotorDriver& driver,
    const raven_control::dynamics::GravityCompensator& compensator,
    double scale)
{
    raven_control::dynamics::JointVector positions{};
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto feedback = driver.feedback(JOINT_NAMES[index]);
        if (!feedback || !feedback->valid)
            return {};
        positions[index] = feedback->position_rad;
    }
    const JointVector raw = compensator.compute(positions);
    JointVector result{};
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        result[index] =
            scale * GRAVITY_DIRECTION_SCALE[index] * raw[index];
    }
    return result;
}

void recordSample(
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic,
    const std::string& phase,
    std::chrono::steady_clock::time_point scheduled_at,
    std::chrono::steady_clock::time_point actual_at,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointVector& target,
    const JointVector& target_velocity,
    const JointVector& kp,
    const JointVector& kd,
    const JointVector& feedforward_torque,
    raven_control::hal::MotorDriver& driver)
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
        joint.kp = kp[index];
        joint.kd = kd[index];
        const auto feedback = driver.feedback(JOINT_NAMES[index]);
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
        joint.position_error_rad = target[index] - feedback->position_rad;
        joint.feedback_age_ms =
            std::chrono::duration<double, std::milli>(
                now - feedback->received_at).count();
        joint.operation_feedback_valid =
            feedback->operation_feedback_valid;
        if (feedback->operation_feedback_valid) {
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
                kp[index] * joint.position_error_rad;
            joint.estimated_d_torque_nm =
                kd[index] *
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
    logger.record(std::move(sample));
}

void sendTargets(
    raven_control::hal::MotorDriver& driver,
    const JointVector& target,
    const JointVector& velocity,
    const JointVector& kp,
    const JointVector& kd,
    const JointVector& torque)
{
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto result = driver.sendMitCommand(
            JOINT_NAMES[index],
            target[index],
            velocity[index],
            kp[index],
            kd[index],
            torque[index]);
        if (result != raven_control::hal::MotorCommandResult::Sent) {
            throw std::runtime_error(
                "MIT command failed on '" +
                std::string(JOINT_NAMES[index]) + "': " +
                raven_control::hal::toString(result));
        }
    }
}

bool stopKeyPressed()
{
    if (stop_requested != 0)
        return true;
    if (!keyAvailable())
        return false;
    const auto key = readKey();
    return key && (*key == 'q' || *key == 'Q');
}

void runTrajectory(
    const PhaseTrajectory& trajectory,
    const std::string& phase_prefix,
    double gravity_scale,
    const JointVector& kp,
    const JointVector& kd,
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config,
    const raven_control::dynamics::GravityCompensator& compensator,
    std::chrono::milliseconds vbus_period,
    raven_control::logging::MotionLogger& logger,
    DiagnosticState& diagnostic)
{
    const auto started_at = std::chrono::steady_clock::now();
    auto next_cycle = started_at;
    auto next_vbus = started_at;
    auto next_status = started_at;
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - started_at).count();
        if (elapsed >= trajectory.total_seconds)
            break;
        if (stopKeyPressed())
            throw std::runtime_error("Operator stop requested");
        (void)driver.poll();
        if (driver.faultLatched())
            throw std::runtime_error(driver.faultReason());
        if (now >= next_vbus) {
            if (!driver.requestBusVoltages())
                throw std::runtime_error("Failed to request VBUS");
            next_vbus = now + vbus_period;
        }
        const TrajectorySample target = trajectory.sample(elapsed);
        const JointVector torque = gravityTorque(
            driver, compensator, gravity_scale);
        sendTargets(
            driver,
            target.position,
            target.velocity,
            kp,
            kd,
            torque);
        recordSample(
            logger,
            diagnostic,
            phase_prefix + "/" + target.subphase,
            next_cycle,
            now,
            config,
            target.position,
            target.velocity,
            kp,
            kd,
            torque,
            driver);
        if (now >= next_status) {
            std::cout << "\r\033[K" << std::setw(30)
                      << phase_prefix << " | ";
            for (std::size_t index = 0;
                 index < JOINT_NAMES.size(); ++index) {
                const auto feedback = driver.feedback(JOINT_NAMES[index]);
                std::cout << JOINT_NAMES[index] << '=' << std::fixed
                          << std::setprecision(1)
                          << (feedback && feedback->valid
                                  ? radiansToDegrees(
                                        feedback->position_rad)
                                  : 0.0)
                          << "deg";
                if (index + 1 < JOINT_NAMES.size())
                    std::cout << " | ";
            }
            std::cout << std::flush;
            next_status = now + std::chrono::milliseconds(200);
        }
        next_cycle += config.control_period;
        std::this_thread::sleep_until(next_cycle);
    }
    const TrajectorySample target = trajectory.sample(
        trajectory.total_seconds);
    const JointVector torque = gravityTorque(
        driver, compensator, gravity_scale);
    sendTargets(
        driver,
        target.position,
        target.velocity,
        kp,
        kd,
        torque);
}

Phase makeHoldPhase(
    std::string name,
    const JointVector& target,
    const JointVector& kp,
    const JointVector& kd,
    double seconds,
    double gravity_scale)
{
    Phase phase;
    phase.name = std::move(name);
    phase.motion = MotionKind::Hold;
    phase.target_rad = target;
    phase.kp = kp;
    phase.kd = kd;
    phase.duration_seconds = seconds;
    phase.gravity_scale = gravity_scale;
    return phase;
}

std::string timestampString()
{
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::ostringstream output;
    output << std::put_time(&local, "%Y%m%d_%H%M%S");
    return output.str();
}

std::string sanitize(std::string value)
{
    for (char& character : value) {
        const bool valid =
            (character >= 'a' && character <= 'z') ||
            (character >= 'A' && character <= 'Z') ||
            (character >= '0' && character <= '9') ||
            character == '-' || character == '_';
        if (!valid)
            character = '_';
    }
    return value;
}

std::filesystem::path createRunDirectory(
    const std::string& root,
    const std::string& name)
{
    const std::filesystem::path directory =
        std::filesystem::path(root) /
        (timestampString() + "_" + sanitize(name));
    std::filesystem::create_directories(directory);
    return directory;
}

void copySnapshot(
    const std::string& source,
    const std::filesystem::path& directory,
    const std::string& destination)
{
    std::filesystem::copy_file(
        source,
        directory / destination,
        std::filesystem::copy_options::overwrite_existing);
}

void writeMetadata(
    const std::filesystem::path& directory,
    const TestPlan& plan,
    const std::string& plan_path)
{
    char hostname[256]{};
    if (::gethostname(hostname, sizeof(hostname) - 1) != 0)
        std::strncpy(hostname, "unknown", sizeof(hostname) - 1);
    const char* git_commit = std::getenv("RAVEN_GIT_COMMIT");
    std::ofstream output(directory / "metadata.yaml");
    if (!output)
        throw std::runtime_error("Cannot create metadata.yaml");
    output << "schema_version: 1\n"
           << "test_name: " << plan.name << "\n"
           << "test_type: " << plan.type << "\n"
           << "created_at_local: " << timestampString() << "\n"
           << "hostname: " << hostname << "\n"
           << "git_commit: "
           << (git_commit != nullptr ? git_commit : "unknown") << "\n"
           << "can_interface: " << plan.can_interface << "\n"
           << "plan_source: " << plan_path << "\n"
           << "repeat_count: " << plan.repeat_count << "\n"
           << "vbus_parameter: 0x701C\n";
}

void writeRunStatus(
    const std::filesystem::path& directory,
    const std::string& status,
    const std::string& message,
    std::size_t sample_count)
{
    std::ofstream output(directory / "run_status.yaml");
    if (!output)
        return;
    output << "status: " << status << "\n"
           << "samples: " << sample_count << "\n"
           << "message: \"";
    for (const char character : message) {
        if (character == '"' || character == '\\')
            output << '\\';
        if (character == '\n' || character == '\r')
            output << ' ';
        else
            output << character;
    }
    output << "\"\n";
}

void createNotesTemplate(const std::filesystem::path& directory)
{
    std::ofstream output(directory / "notes.md");
    output
        << "# Test notes\n\n"
        << "- Operator:\n"
        << "- Power supply / voltage setting:\n"
        << "- Payload:\n"
        << "- Ambient conditions:\n"
        << "- Noise, vibration, cable interference:\n"
        << "- Reason for interruption, if any:\n";
}

void printPlan(const TestPlan& plan)
{
    std::cout << "\nRAVEN joint characterization\n"
              << "Test: " << plan.name << " (" << plan.type << ")\n"
              << "CAN: " << plan.can_interface << "\n"
              << "Repeats: " << plan.repeat_count << "\n"
              << "Phases per repeat: " << plan.phases.size() << "\n"
              << "Output root: " << plan.output_root << "\n"
              << "Q or Ctrl-C: stop and disable motors\n";
}

bool waitForStart()
{
    std::cout << "\nSPACE: start the validated plan\n"
              << "Q: cancel while motors remain disabled\n"
              << std::flush;
    while (true) {
        if (stop_requested != 0)
            return false;
        if (keyAvailable()) {
            const auto key = readKey();
            if (key && *key == ' ')
                return true;
            if (key && (*key == 'q' || *key == 'Q'))
                return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc < 2 || argc > 3) {
        std::cerr
            << "Usage: joint_characterization <test_plan.yaml> "
               "[output_root]\n";
        return 2;
    }

    const std::string plan_path = argv[1];
    std::optional<std::filesystem::path> run_directory;
    std::optional<raven_control::logging::MotionLogger> logger;
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    try {
        const auto motor_config =
            raven_control::config::loadMotorRuntimeConfig(
                motorConfigPathFromPlan(plan_path));
        const JointBindings initial_bindings =
            bindConfiguredJoints(motor_config);
        TestPlan plan = loadTestPlan(plan_path, initial_bindings);
        if (argc == 3)
            plan.output_root = argv[2];

        const auto runtime_config =
            raven_control::config::loadMotorRuntimeConfig(
                plan.motor_config_path);
        const JointBindings bindings =
            bindConfiguredJoints(runtime_config);
        // Reload the plan so default gains come from its selected config.
        plan = loadTestPlan(plan_path, bindings);
        if (argc == 3)
            plan.output_root = argv[2];
        const auto limiters =
            raven_control::safety::loadJointLimiters(
                plan.joint_limits_path);
        validatePlan(plan, bindings, limiters);
        printPlan(plan);

        run_directory = createRunDirectory(plan.output_root, plan.name);
        copySnapshot(plan_path, *run_directory, "test_plan.yaml");
        copySnapshot(
            plan.joint_limits_path,
            *run_directory,
            "joint_limits.yaml");
        copySnapshot(
            plan.motor_config_path,
            *run_directory,
            "motor_config.yaml");
        writeMetadata(*run_directory, plan, plan_path);
        createNotesTemplate(*run_directory);

        logger.emplace(
            std::array<std::string, 3>{
                JOINT_NAMES[0], JOINT_NAMES[1], JOINT_NAMES[2]},
            100000);
        DiagnosticState diagnostic;
        raven_control::dynamics::GravityCompensator compensator(
            raven_control::dynamics::makeRavenUrdfGravityModel());
        raven_control::hal::CanInterface can(plan.can_interface);
        raven_control::hal::MotorDriver driver(
            can,
            motorMap(bindings),
            limiters,
            0xFD,
            runtime_config.feedback_timeout);
        StopGuard stop_guard(driver);
        if (!driver.stopAll())
            throw std::runtime_error("Failed to send startup stop");
        requestInitialFeedback(driver, runtime_config);

        const JointVector current = readJointPositions(driver);
        validatePose(
            current,
            "Current pose",
            limiters,
            0.0);
        std::cout << "\nCurrent pose: ";
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            std::cout << JOINT_NAMES[index] << '=' << std::fixed
                      << std::setprecision(1)
                      << radiansToDegrees(current[index]) << "deg";
            if (index + 1 < JOINT_NAMES.size())
                std::cout << " | ";
        }
        std::cout << '\n';

        TerminalMode terminal;
        if (!waitForStart()) {
            writeRunStatus(*run_directory, "cancelled", "Cancelled", 0);
            std::cout << "\nCancelled. Results: "
                      << run_directory->string() << '\n';
            return 0;
        }

        const auto enable_result = driver.enableAll();
        if (enable_result != raven_control::hal::MotorCommandResult::Sent) {
            throw std::runtime_error(
                "Enable failed: " + std::string(
                    raven_control::hal::toString(enable_result)));
        }

        const JointVector base_kp = defaultKp(bindings);
        const JointVector base_kd = defaultKd(bindings);
        Phase startup;
        startup.name = "startup";
        startup.motion = MotionKind::Quintic;
        startup.target_rad = plan.startup_target_rad;
        startup.kp = base_kp;
        startup.kd = base_kd;
        startup.duration_seconds = plan.startup_transition_seconds;
        for (std::size_t index = 0;
             index < JOINT_NAMES.size(); ++index) {
            const double minimum_duration =
                1.875 * std::abs(
                    startup.target_rad[index] - current[index]) /
                bindings[index]->position_control.max_slew_rate_rad_s;
            startup.duration_seconds = std::max(
                startup.duration_seconds,
                minimum_duration * 1.05);
        }
        startup.gravity_scale = plan.startup_gravity_scale;
        runTrajectory(
            makeTrajectory(startup, current),
            "startup",
            startup.gravity_scale,
            startup.kp,
            startup.kd,
            driver,
            runtime_config,
            compensator,
            plan.vbus_request_period,
            *logger,
            diagnostic);

        JointVector previous = plan.startup_target_rad;
        for (std::size_t repeat = 0;
             repeat < plan.repeat_count; ++repeat) {
            for (const Phase& phase : plan.phases) {
                const std::string prefix =
                    "r" + std::to_string(repeat + 1) + "/" +
                    phase.name +
                    (phase.analysis_window ? "/analysis" : "");
                runTrajectory(
                    makeTrajectory(phase, previous),
                    prefix,
                    phase.gravity_scale,
                    phase.kp,
                    phase.kd,
                    driver,
                    runtime_config,
                    compensator,
                    plan.vbus_request_period,
                    *logger,
                    diagnostic);
                previous = phase.target_rad;
                if (phase.hold_seconds > 0.0) {
                    const Phase hold = makeHoldPhase(
                        phase.name + "_settle",
                        previous,
                        phase.kp,
                        phase.kd,
                        phase.hold_seconds,
                        phase.gravity_scale);
                    runTrajectory(
                        makeTrajectory(hold, previous),
                        prefix + "/settle",
                        hold.gravity_scale,
                        hold.kp,
                        hold.kd,
                        driver,
                        runtime_config,
                        compensator,
                        plan.vbus_request_period,
                        *logger,
                        diagnostic);
                }
            }
        }

        if (plan.final_hold_seconds > 0.0) {
            const Phase final_hold = makeHoldPhase(
                "final_hold",
                previous,
                base_kp,
                base_kd,
                plan.final_hold_seconds,
                plan.startup_gravity_scale);
            runTrajectory(
                makeTrajectory(final_hold, previous),
                "final_hold",
                final_hold.gravity_scale,
                final_hold.kp,
                final_hold.kd,
                driver,
                runtime_config,
                compensator,
                plan.vbus_request_period,
                *logger,
                diagnostic);
        }

        if (!driver.stopAll())
            throw std::runtime_error("Failed to stop all motors");
        stop_guard.release();
        logger->saveCsv((*run_directory / "raw.csv").string());
        writeRunStatus(
            *run_directory,
            "completed",
            "Test plan completed",
            logger->sampleCount());
        std::cout << "\nCompleted. Results: "
                  << run_directory->string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        if (run_directory && logger) {
            try {
                logger->saveCsv((*run_directory / "raw.csv").string());
                writeRunStatus(
                    *run_directory,
                    "aborted",
                    error.what(),
                    logger->sampleCount());
            } catch (...) {
            }
        }
        std::cerr << "\nError: " << error.what() << '\n';
        if (run_directory)
            std::cerr << "Partial results: " << run_directory->string() << '\n';
        return 1;
    }
}
