#include "raven_control/config/motor_config.hpp"
#include "raven_control/control/gravity_feedforward_controller.hpp"
#include "raven_control/control/gravity_validation_session.hpp"
#include "raven_control/dynamics/pinocchio_gravity_model.hpp"
#include "raven_control/hal/can_interface.hpp"
#include "raven_control/hal/mit_torque_codec.hpp"
#include "raven_control/hal/motor_driver.hpp"
#include "raven_control/logging/motion_logger.hpp"
#include "raven_control/safety/joint_limiter.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
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

using Clock = std::chrono::steady_clock;
using JointVector = raven_control::dynamics::JointVector;
using JointBindings = std::array<
    const raven_control::config::JointMotorRuntimeConfig*,
    raven_control::dynamics::RAVEN_JOINT_COUNT>;
using TorqueAudits = std::array<
    raven_control::hal::MitTorqueAudit,
    raven_control::dynamics::RAVEN_JOINT_COUNT>;

constexpr std::array<const char*, 3> JOINT_NAMES{{
    "shoulder_Joint",
    "upperArm_Joint",
    "foreArm_Joint",
}};
constexpr std::size_t TEST_JOINT_INDEX = 1;
constexpr std::chrono::milliseconds GATE_DURATION{30000};
constexpr std::chrono::milliseconds MEASUREMENT_DURATION{5000};
constexpr std::chrono::milliseconds MINIMUM_LIVE_RAMP{1000};
constexpr std::array<double, 3> COMMISSIONING_TORQUE_CAP_NM{{
    0.5,
    3.0,
    1.0,
}};
constexpr double MAX_BUS_VOLTAGE_V = 35.0;
constexpr double MAX_MOTOR_TEMPERATURE_C = 60.0;
constexpr double PI = 3.14159265358979323846;
constexpr std::size_t LOG_RESERVE_SAMPLES = 30000;

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

    StopGuard(const StopGuard&) = delete;
    StopGuard& operator=(const StopGuard&) = delete;

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

JointBindings bindConfiguredJoints(
    const raven_control::config::MotorRuntimeConfig& config)
{
    if (config.joints.size() != JOINT_NAMES.size()) {
        throw std::runtime_error(
            "gravity validation requires exactly three configured joints");
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

std::vector<raven_control::hal::JointMotorConfig> motorMap(
    const JointBindings& bindings)
{
    std::vector<raven_control::hal::JointMotorConfig> result;
    result.reserve(bindings.size());
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

std::array<std::string, 3> loggingJointNames()
{
    return {
        JOINT_NAMES[0],
        JOINT_NAMES[1],
        JOINT_NAMES[2],
    };
}

void validateLiveConfiguration(
    const raven_control::config::MotorRuntimeConfig& config,
    const JointBindings& bindings,
    const raven_control::safety::JointLimiterMap& limiters)
{
    const auto& gravity = config.gravity_compensation;
    if (!gravity.enabled) {
        throw std::runtime_error(
            "Refusing to arm: gravity_compensation.enabled must be true");
    }
    if (gravity.dry_run) {
        throw std::runtime_error(
            "Refusing to arm: gravity_compensation.dry_run must be false. "
            "A zero-gain test with zero feedforward torque can fall.");
    }
    if (gravity.scale <= 0.0) {
        throw std::runtime_error(
            "Refusing to arm: gravity_compensation.scale must be positive");
    }
    if (gravity.ramp_duration < MINIMUM_LIVE_RAMP) {
        throw std::runtime_error(
            "Refusing to arm: gravity ramp must be at least 1000 ms");
    }
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const double limit = gravity.max_joint_torque_nm[index];
        if (limit <= 0.0) {
            throw std::runtime_error(
                "Refusing to arm: gravity torque limit for '" +
                std::string(JOINT_NAMES[index]) +
                "' must be positive");
        }
        if (limit > COMMISSIONING_TORQUE_CAP_NM[index]) {
            throw std::runtime_error(
                "Refusing to arm: gravity torque limit for '" +
                std::string(JOINT_NAMES[index]) +
                "' exceeds this validation app's commissioning cap of " +
                std::to_string(COMMISSIONING_TORQUE_CAP_NM[index]) +
                " N.m");
        }

        const auto limiter = limiters.find(JOINT_NAMES[index]);
        if (limiter == limiters.end() ||
            !limiter->second.isConfirmed()) {
            throw std::runtime_error(
                "Refusing to arm: confirmed limits are required for '" +
                std::string(JOINT_NAMES[index]) + "'");
        }
    }

    for (const std::size_t index : {std::size_t{0}, std::size_t{2}}) {
        if (bindings[index]->position_control.kp <= 0.0 &&
            bindings[index]->position_control.kd <= 0.0) {
            throw std::runtime_error(
                "Refusing to arm: non-test joint '" +
                std::string(JOINT_NAMES[index]) +
                "' requires a configured PD hold gain");
        }
    }
}

void requestBootstrapFeedback(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config)
{
    const auto deadline = Clock::now() +
        std::max(
            std::chrono::milliseconds(500),
            config.feedback_timeout * 2);
    auto next_request = Clock::now();
    while (!driver.allFeedbackValid() && Clock::now() < deadline) {
        const auto now = Clock::now();
        if (now >= next_request) {
            if (!driver.requestMechanicalPositions()) {
                throw std::runtime_error(
                    "Failed to request mechanical positions");
            }
            next_request = now + config.position_request_period;
        }
        (void)driver.poll();
        if (driver.faultLatched())
            throw std::runtime_error(driver.faultReason());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!driver.allFeedbackValid()) {
        throw std::runtime_error(
            "Fresh Type 17 feedback is unavailable for all joints");
    }
}

JointVector readPositions(raven_control::hal::MotorDriver& driver)
{
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

void validateCapturedPose(
    const JointVector& pose,
    const raven_control::safety::JointLimiterMap& limiters)
{
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        const auto limiter = limiters.find(JOINT_NAMES[index]);
        if (limiter == limiters.end())
            throw std::runtime_error("Missing joint limiter");
        const auto safe = limiter->second.clampTarget(pose[index]);
        if (!safe || *safe != pose[index]) {
            throw std::runtime_error(
                "Captured pose for '" + std::string(JOINT_NAMES[index]) +
                "' is outside its soft limits");
        }
    }
}

TorqueAudits auditTorques(
    const JointBindings& bindings,
    const JointVector& torque)
{
    TorqueAudits audits{};
    for (std::size_t index = 0; index < audits.size(); ++index) {
        audits[index] = raven_control::hal::auditMitTorqueCommand(
            bindings[index]->position_sign,
            bindings[index]->joint_to_motor_ratio,
            torque[index]);
        const double one_motor_lsb =
            2.0 *
            raven_control::hal::RS02_OPERATION_MAX_TORQUE_NM /
            65535.0;
        const double allowed_joint_error =
            one_motor_lsb * bindings[index]->joint_to_motor_ratio + 1e-9;
        if (std::abs(
                audits[index].decoded_joint_torque_nm - torque[index]) >
            allowed_joint_error) {
            throw std::runtime_error(
                "MIT torque audit exceeded one wire LSB on '" +
                std::string(JOINT_NAMES[index]) + "'");
        }
    }
    return audits;
}

void printPreflight(
    const JointVector& pose,
    const JointVector& raw_torque,
    const JointVector& scaled_torque,
    const TorqueAudits& audits,
    const raven_control::config::MotorRuntimeConfig& config)
{
    std::cout
        << "\nFinal preflight at the captured pose\n"
        << "Test joint: " << JOINT_NAMES[TEST_JOINT_INDEX]
        << " (Kp=0, Kd=0)\n"
        << "Non-test joints retain configured PD gains.\n"
        << "Gravity: LIVE, scale="
        << config.gravity_compensation.scale << ", ramp="
        << config.gravity_compensation.ramp_duration.count() << " ms\n\n"
        << std::fixed << std::setprecision(6);
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        std::cout
            << JOINT_NAMES[index]
            << " q=" << radiansToDegrees(pose[index]) << " deg"
            << " raw=" << raw_torque[index] << " N.m"
            << " scaled=" << scaled_torque[index] << " N.m"
            << " motor=" << audits[index].motor_torque_nm << " N.m"
            << " encoded=" << audits[index].encoded_torque
            << " decoded_joint="
            << audits[index].decoded_joint_torque_nm << " N.m\n";
    }
    std::cout << std::defaultfloat;
}

bool waitForArmConfirmation(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config)
{
    std::cout
        << "\nSUPPORT THE ARM BEFORE ARMING.\n"
        << "No automatic position command will be generated.\n"
        << "SPACE: capture a preflight preview\n"
        << "Q: quit without enabling motors\n"
        << std::flush;

    auto next_request = Clock::now();
    while (!stop_requested) {
        const auto now = Clock::now();
        if (now >= next_request) {
            if (!driver.requestMechanicalPositions()) {
                throw std::runtime_error(
                    "Failed to request mechanical positions");
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
    return false;
}

bool waitForFinalEnableConfirmation(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& config)
{
    std::cout
        << "\nReview the preflight values above. Keep the arm supported "
           "and do not move it.\n"
        << "SPACE: confirm the printed command path and enable once\n"
        << "Q: quit without enabling motors\n"
        << std::flush;
    auto next_request = Clock::now();
    while (!stop_requested) {
        const auto now = Clock::now();
        if (now >= next_request) {
            if (!driver.requestMechanicalPositions()) {
                throw std::runtime_error(
                    "Failed to request mechanical positions");
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
    return false;
}

std::string timestampString()
{
    const std::time_t now = std::time(nullptr);
    std::tm local_time{};
    localtime_r(&now, &local_time);
    std::ostringstream output;
    output << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return output.str();
}

std::filesystem::path createRunDirectory(
    const std::filesystem::path& output_root)
{
    std::filesystem::create_directories(output_root);
    const std::string stem = timestampString() + "_gravity_feedforward_j1";
    for (int suffix = 0; suffix < 1000; ++suffix) {
        const std::string name = suffix == 0
            ? stem
            : stem + '_' + std::to_string(suffix);
        const auto candidate = output_root / name;
        if (std::filesystem::create_directory(candidate))
            return candidate;
    }
    throw std::runtime_error("Cannot allocate a unique run directory");
}

std::string yamlQuote(const std::string& value)
{
    std::string result = "\"";
    for (char character : value) {
        if (character == '\\' || character == '"')
            result += '\\';
        result += character;
    }
    result += '"';
    return result;
}

void copySnapshot(
    const std::filesystem::path& source,
    const std::filesystem::path& destination)
{
    std::error_code error;
    std::filesystem::copy_file(
        source,
        destination,
        std::filesystem::copy_options::overwrite_existing,
        error);
    if (error) {
        throw std::runtime_error(
            "Cannot snapshot '" + source.string() + "': " +
            error.message());
    }
}

void writeMetadata(
    const std::filesystem::path& path,
    const std::string& interface_name,
    const std::string& limits_path,
    const std::string& motor_config_path,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointVector& captured_pose)
{
    std::ofstream output(path);
    if (!output)
        throw std::runtime_error("Cannot create metadata file");
    const char* commit = std::getenv("RAVEN_GIT_COMMIT");
    output
        << "schema_version: 1\n"
        << "application: gravity_feedforward_validation\n"
        << "created_at: " << yamlQuote(timestampString()) << "\n"
        << "git_commit: "
        << yamlQuote(commit == nullptr ? "unknown" : commit) << "\n"
        << "can_interface: " << yamlQuote(interface_name) << "\n"
        << "joint_limits_source: " << yamlQuote(limits_path) << "\n"
        << "motor_config_source: " << yamlQuote(motor_config_path) << "\n"
        << "test_joint: " << yamlQuote(JOINT_NAMES[TEST_JOINT_INDEX])
        << "\n"
        << "test_joint_kp: 0\n"
        << "test_joint_kd: 0\n"
        << "gate_duration_ms: " << GATE_DURATION.count() << "\n"
        << "measurement_duration_ms: "
        << MEASUREMENT_DURATION.count() << "\n"
        << "gravity_scale: " << config.gravity_compensation.scale << "\n"
        << "gravity_ramp_duration_ms: "
        << config.gravity_compensation.ramp_duration.count() << "\n"
        << "captured_pose_rad:\n";
    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        output << "  " << JOINT_NAMES[index] << ": "
               << std::setprecision(12) << captured_pose[index] << "\n";
    }
}

void writeRunStatus(
    const std::filesystem::path& path,
    const std::string& status,
    std::size_t measurements,
    const std::string& message)
{
    std::ofstream output(path);
    if (!output)
        throw std::runtime_error("Cannot write run status");
    output
        << "status: " << yamlQuote(status) << "\n"
        << "completed_measurements: " << measurements << "\n"
        << "message: " << yamlQuote(message) << "\n";
}

void checkLiveSafety(raven_control::hal::MotorDriver& driver)
{
    for (const char* joint_name : JOINT_NAMES) {
        const auto feedback = driver.feedback(joint_name);
        if (!feedback)
            continue;
        if (feedback->operation_feedback_valid &&
            feedback->temperature_celsius > MAX_MOTOR_TEMPERATURE_C) {
            throw std::runtime_error(
                "Motor temperature exceeded 60 C on '" +
                std::string(joint_name) + "'");
        }
        if (feedback->bus_voltage_valid &&
            feedback->bus_voltage_v > MAX_BUS_VOLTAGE_V) {
            throw std::runtime_error(
                "Bus voltage exceeded 35 V on '" +
                std::string(joint_name) + "'");
        }
    }
}

void recordSample(
    raven_control::logging::MotionLogger& logger,
    std::uint64_t cycle_index,
    const std::string& phase,
    Clock::time_point scheduled_at,
    Clock::time_point actual_at,
    const raven_control::config::MotorRuntimeConfig& config,
    const JointVector& target,
    const JointVector& kp,
    const JointVector& kd,
    const raven_control::control::GravityFeedforwardResult& gravity,
    const TorqueAudits& audits,
    raven_control::hal::MotorDriver& driver)
{
    const auto observed_at = Clock::now();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    raven_control::logging::MotionSample sample;
    sample.cycle_index = cycle_index;
    sample.phase = phase;
    sample.scheduled_at = scheduled_at;
    sample.actual_at = actual_at;
    sample.control_period =
        std::chrono::duration_cast<std::chrono::microseconds>(
            config.control_period);

    for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
        raven_control::logging::JointMotionSample joint;
        joint.command_position_rad = target[index];
        joint.trajectory_velocity_rad_s = 0.0;
        joint.sent_velocity_rad_s = 0.0;
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
        joint.sent_feedforward_torque_nm =
            gravity.commanded_torque_nm[index];
        joint.motor_feedforward_torque_nm =
            audits[index].motor_torque_nm;
        joint.encoded_feedforward_torque =
            audits[index].encoded_torque;
        joint.decoded_joint_feedforward_torque_nm =
            audits[index].decoded_joint_torque_nm;
        joint.feedforward_audit_valid = true;
        joint.kp = kp[index];
        joint.kd = kd[index];

        const auto feedback = driver.feedback(JOINT_NAMES[index]);
        if (feedback && feedback->valid) {
            joint.feedback_valid = true;
            joint.actual_position_rad = feedback->position_rad;
            joint.feedback_age_ms =
                std::chrono::duration<double, std::milli>(
                    observed_at - feedback->received_at).count();
            const bool operation_fresh =
                feedback->operation_feedback_valid &&
                observed_at >= feedback->operation_received_at &&
                observed_at - feedback->operation_received_at <=
                    config.feedback_timeout;
            joint.operation_feedback_valid = operation_fresh;
            if (operation_fresh) {
                joint.actual_velocity_rad_s = feedback->velocity_rad_s;
                joint.measured_torque_nm = feedback->torque_nm;
                joint.motor_temperature_celsius =
                    feedback->temperature_celsius;
                joint.operation_feedback_age_ms =
                    std::chrono::duration<double, std::milli>(
                        observed_at - feedback->operation_received_at)
                        .count();
                joint.motor_fault_flags = feedback->fault_flags;
                joint.motor_mode_state = feedback->mode_state;
            } else {
                joint.actual_velocity_rad_s = nan;
                joint.measured_torque_nm = nan;
                joint.motor_temperature_celsius = nan;
                joint.operation_feedback_age_ms = nan;
            }
            joint.bus_voltage_valid = feedback->bus_voltage_valid;
            if (feedback->bus_voltage_valid) {
                joint.bus_voltage_v = feedback->bus_voltage_v;
                joint.bus_voltage_age_ms =
                    std::chrono::duration<double, std::milli>(
                        observed_at - feedback->bus_voltage_received_at)
                        .count();
            } else {
                joint.bus_voltage_v = nan;
                joint.bus_voltage_age_ms = nan;
            }
            joint.position_error_rad =
                target[index] - feedback->position_rad;
            joint.estimated_p_torque_nm =
                kp[index] * joint.position_error_rad;
            joint.estimated_d_torque_nm = operation_fresh
                ? kd[index] * (-feedback->velocity_rad_s)
                : nan;
            joint.estimated_control_torque_nm = operation_fresh
                ? joint.estimated_p_torque_nm +
                      joint.estimated_d_torque_nm +
                      joint.sent_feedforward_torque_nm
                : nan;
        } else {
            joint.actual_position_rad = nan;
            joint.actual_velocity_rad_s = nan;
            joint.position_error_rad = nan;
            joint.estimated_p_torque_nm = nan;
            joint.estimated_d_torque_nm = nan;
            joint.estimated_control_torque_nm = nan;
            joint.measured_torque_nm = nan;
            joint.motor_temperature_celsius = nan;
            joint.bus_voltage_v = nan;
            joint.feedback_age_ms = nan;
            joint.operation_feedback_age_ms = nan;
            joint.bus_voltage_age_ms = nan;
        }
        sample.joints[index] = joint;
    }
    logger.record(std::move(sample));
}

void printStatus(
    const raven_control::control::GravityValidationSession& session,
    const raven_control::control::GravityFeedforwardResult& gravity,
    raven_control::hal::MotorDriver& driver)
{
    const auto feedback = driver.feedback(JOINT_NAMES[TEST_JOINT_INDEX]);
    std::cout
        << "\r\033[K" << session.phaseLabel()
        << " | ramp=" << std::fixed << std::setprecision(0)
        << gravity.ramp_factor * 100.0 << "%";
    if (session.remaining(Clock::now()).count() > 0) {
        std::cout << " | remaining=" << std::setprecision(1)
                  << session.remaining(Clock::now()).count() / 1000.0
                  << "s";
    }
    if (feedback && feedback->valid) {
        std::cout
            << " | J1=" << std::setprecision(2)
            << radiansToDegrees(feedback->position_rad) << "deg"
            << " tau_ff=" << std::setprecision(3)
            << gravity.commanded_torque_nm[TEST_JOINT_INDEX] << "Nm";
    }
    std::cout << std::defaultfloat << std::flush;
}

}  // namespace

int main(int argc, char* argv[])
{
    const std::string interface_name = argc > 1 ? argv[1] : "can0";
    const std::string limits_path = argc > 2
        ? argv[2]
        : "config/joint_limits.yaml";
    const std::string motor_config_path = argc > 3
        ? argv[3]
        : "config/motor_config.yaml";
    const std::filesystem::path output_root = argc > 4
        ? argv[4]
        : "logs/gravity_validation";

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::optional<raven_control::logging::MotionLogger> logger;
    std::filesystem::path run_directory;
    bool log_saved = false;
    std::size_t completed_measurements = 0;

    try {
        const auto motor_config =
            raven_control::config::loadMotorRuntimeConfig(
                motor_config_path);
        auto limiters =
            raven_control::safety::loadJointLimiters(limits_path);
        const JointBindings bindings =
            bindConfiguredJoints(motor_config);
        validateLiveConfiguration(motor_config, bindings, limiters);

        auto gravity_model = std::make_unique<
            raven_control::dynamics::PinocchioGravityModel>(
                motor_config.gravity_compensation.urdf_path);
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
        requestBootstrapFeedback(driver, motor_config);

        std::cout
            << "RAVEN gravity feedforward validation\n"
            << "CAN interface: " << interface_name << '\n'
            << "Joint limits: " << limits_path << '\n'
            << "Motor config: " << motor_config_path << '\n'
            << "Automatic motion: DISABLED\n"
            << "Live safety limits: VBUS <= " << MAX_BUS_VOLTAGE_V
            << " V, temperature <= " << MAX_MOTOR_TEMPERATURE_C
            << " C\n";

        TerminalMode terminal;
        if (!waitForArmConfirmation(driver, motor_config)) {
            std::cout << "\nValidation cancelled before enable\n";
            return 0;
        }

        requestBootstrapFeedback(driver, motor_config);
        const JointVector captured_pose = readPositions(driver);
        validateCapturedPose(captured_pose, limiters);
        const JointVector initial_raw_torque =
            gravity_model->compute(captured_pose);
        JointVector initial_scaled_torque{};
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            initial_scaled_torque[index] =
                initial_raw_torque[index] *
                motor_config.gravity_compensation.scale;
            if (std::abs(initial_scaled_torque[index]) >
                motor_config.gravity_compensation
                    .max_joint_torque_nm[index]) {
                throw std::runtime_error(
                    "Initial gravity prediction exceeds the configured "
                    "torque limit on '" +
                    std::string(JOINT_NAMES[index]) +
                    "'; refusing to enable with a clamped command");
            }
        }
        const TorqueAudits initial_audits =
            auditTorques(bindings, initial_scaled_torque);
        printPreflight(
            captured_pose,
            initial_raw_torque,
            initial_scaled_torque,
            initial_audits,
            motor_config);

        if (!waitForFinalEnableConfirmation(driver, motor_config)) {
            std::cout << "\nValidation cancelled before enable\n";
            return 0;
        }
        requestBootstrapFeedback(driver, motor_config);
        const JointVector confirmed_pose = readPositions(driver);
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            constexpr double MAX_CONFIRMATION_MOTION_RAD =
                0.1 * PI / 180.0;
            if (std::abs(confirmed_pose[index] - captured_pose[index]) >
                MAX_CONFIRMATION_MOTION_RAD) {
                throw std::runtime_error(
                    "Pose changed by more than 0.1 deg after the preflight "
                    "preview on '" + std::string(JOINT_NAMES[index]) +
                    "'; rerun and confirm without moving the arm");
            }
        }

        run_directory = createRunDirectory(output_root);
        copySnapshot(
            motor_config_path,
            run_directory / "motor_config.yaml");
        copySnapshot(limits_path, run_directory / "joint_limits.yaml");
        writeMetadata(
            run_directory / "metadata.yaml",
            interface_name,
            limits_path,
            motor_config_path,
            motor_config,
            captured_pose);
        writeRunStatus(
            run_directory / "run_status.yaml",
            "prepared",
            0,
            "Preflight confirmed; motor enable not attempted yet");

        logger.emplace(loggingJointNames(), LOG_RESERVE_SAMPLES);
        raven_control::control::GravityFeedforwardController gravity(
            std::move(gravity_model),
            motor_config.gravity_compensation);
        raven_control::control::GravityValidationSessionConfig
            session_config;
        session_config.gate_duration = GATE_DURATION;
        session_config.measurement_duration = MEASUREMENT_DURATION;
        raven_control::control::GravityValidationSession session(
            session_config);

        JointVector kp{};
        JointVector kd{};
        for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
            kp[index] = bindings[index]->position_control.kp;
            kd[index] = bindings[index]->position_control.kd;
        }
        kp[TEST_JOINT_INDEX] = 0.0;
        kd[TEST_JOINT_INDEX] = 0.0;

        const auto enable_result = driver.enableAll();
        if (enable_result != raven_control::hal::MotorCommandResult::Sent) {
            throw std::runtime_error(
                "Enable rejected: " + std::string(
                    raven_control::hal::toString(enable_result)));
        }
        writeRunStatus(
            run_directory / "run_status.yaml",
            "armed",
            0,
            "Motor enable accepted; validation in progress");

        const auto enabled_at = Clock::now();
        gravity.reset(enabled_at);
        session.start(enabled_at);
        auto next_cycle = enabled_at;
        auto next_vbus = enabled_at;
        auto next_status = enabled_at;
        auto operation_feedback_deadline =
            enabled_at + motor_config.feedback_timeout * 2;
        std::uint64_t cycle_index = 0;
        auto previous_phase = session.phase();

        std::cout
            << "\nLIVE. Keep supporting the arm during ramp and gate.\n"
            << "SPACE: normal stop and exit | Q: abort and stop\n"
            << "After the 30 s gate, move J1 slowly and press M immediately "
               "after release to mark a 5 s measurement.\n";

        while (!stop_requested) {
            const auto cycle_started_at = Clock::now();
            if (keyAvailable()) {
                const auto key = readKey();
                if (key && *key == ' ') {
                    if (session.phase() == raven_control::control::
                            GravityValidationPhase::Measurement) {
                        session.abort();
                    } else {
                        session.complete();
                    }
                    break;
                }
                if (key && (*key == 'q' || *key == 'Q')) {
                    session.abort();
                    break;
                }
                if (key && (*key == 'm' || *key == 'M')) {
                    if (!session.requestMeasurement(cycle_started_at)) {
                        std::cout
                            << "\nMeasurement mark ignored: wait for "
                               "manual_ready\n";
                    } else {
                        std::cout
                            << "\nMeasurement "
                            << session.activeMeasurement()
                            << " started for 5 seconds\n";
                    }
                }
            }

            (void)driver.poll();
            // poll() timestamps newly received Type 2 frames. Observe the
            // clock afterwards so those frames are not rejected as being
            // newer than a timestamp captured at the start of the cycle.
            const auto feedback_observed_at = Clock::now();
            if (driver.faultLatched())
                throw std::runtime_error(driver.faultReason());
            if (cycle_started_at >= next_vbus) {
                if (!driver.requestBusVoltages())
                    throw std::runtime_error("Failed to request VBUS");
                next_vbus =
                    cycle_started_at + motor_config.position_request_period;
            }
            checkLiveSafety(driver);

            JointVector positions{};
            bool operation_feedback_fresh = true;
            std::string operation_feedback_issue;
            for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
                const auto feedback = driver.feedback(JOINT_NAMES[index]);
                if (!feedback) {
                    operation_feedback_issue =
                        "no feedback record for '" +
                        std::string(JOINT_NAMES[index]) + "'";
                    operation_feedback_fresh = false;
                    break;
                }
                if (!feedback->valid) {
                    operation_feedback_issue =
                        "feedback is invalid for '" +
                        std::string(JOINT_NAMES[index]) + "'";
                    operation_feedback_fresh = false;
                    break;
                }
                if (!feedback->operation_feedback_valid) {
                    operation_feedback_issue =
                        "Type 2 has not been received for '" +
                        std::string(JOINT_NAMES[index]) + "'";
                    operation_feedback_fresh = false;
                    break;
                }
                if (!raven_control::control::isFreshFeedbackTimestamp(
                        feedback->operation_received_at,
                        feedback_observed_at,
                        motor_config.feedback_timeout)) {
                    std::ostringstream issue;
                    issue << "Type 2 timestamp is invalid for '"
                          << JOINT_NAMES[index] << "'";
                    if (feedback_observed_at <
                        feedback->operation_received_at) {
                        issue << " (feedback was timestamped after the "
                                 "observation time)";
                    } else {
                        issue << " (age="
                              << std::chrono::duration<double, std::milli>(
                                     feedback_observed_at -
                                     feedback->operation_received_at)
                                     .count()
                              << " ms, timeout="
                              << motor_config.feedback_timeout.count()
                              << " ms)";
                    }
                    operation_feedback_issue = issue.str();
                    operation_feedback_fresh = false;
                    break;
                }
                positions[index] = feedback->position_rad;
            }

            const auto gravity_result = gravity.compute(
                positions,
                operation_feedback_fresh,
                feedback_observed_at);
            if (!gravity_result.input_valid &&
                feedback_observed_at >= operation_feedback_deadline) {
                throw std::runtime_error(
                    "Fresh Type 2 feedback did not become available: " +
                    (operation_feedback_issue.empty()
                         ? std::string("unknown feedback state")
                         : operation_feedback_issue));
            }
            if (gravity_result.input_valid) {
                operation_feedback_deadline =
                    feedback_observed_at +
                    motor_config.feedback_timeout * 2;
            }
            for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
                if (gravity_result.torque_clamped[index]) {
                    throw std::runtime_error(
                        "Gravity command reached its configured torque "
                        "limit on '" + std::string(JOINT_NAMES[index]) +
                        "'; run is invalid and has been stopped");
                }
            }

            const TorqueAudits audits = auditTorques(
                bindings,
                gravity_result.commanded_torque_nm);
            for (std::size_t index = 0; index < JOINT_NAMES.size(); ++index) {
                const auto command_result = driver.sendMitCommand(
                    JOINT_NAMES[index],
                    captured_pose[index],
                    0.0,
                    kp[index],
                    kd[index],
                    gravity_result.commanded_torque_nm[index]);
                if (command_result ==
                    raven_control::hal::MotorCommandResult::FeedbackHold) {
                    throw std::runtime_error(
                        "Feedback Hold entered; validation stopped");
                }
                if (command_result !=
                    raven_control::hal::MotorCommandResult::Sent) {
                    throw std::runtime_error(
                        "MIT command failed on '" +
                        std::string(JOINT_NAMES[index]) + "': " +
                        raven_control::hal::toString(command_result));
                }
            }

            session.update(
                feedback_observed_at,
                gravity_result.input_valid &&
                    gravity_result.ramp_factor >= 1.0 - 1e-12);
            if (session.phase() != previous_phase) {
                std::cout << "\nPhase: " << session.phaseLabel() << '\n';
                if (session.phase() == raven_control::control::
                        GravityValidationPhase::Ready) {
                    std::cout
                        << "Move only J1 slowly. Release it, then press M "
                           "to record 5 seconds.\n";
                }
                previous_phase = session.phase();
            }

            recordSample(
                *logger,
                cycle_index++,
                session.phaseLabel(),
                next_cycle,
                cycle_started_at,
                motor_config,
                captured_pose,
                kp,
                kd,
                gravity_result,
                audits,
                driver);
            if (cycle_started_at >= next_status) {
                printStatus(session, gravity_result, driver);
                next_status =
                    cycle_started_at + std::chrono::milliseconds(200);
            }

            next_cycle += motor_config.control_period;
            std::this_thread::sleep_until(next_cycle);
        }

        if (stop_requested)
            session.abort();
        completed_measurements = session.completedMeasurements();
        if (!driver.stopAll())
            throw std::runtime_error("Failed to send final stop");
        stop_guard.release();

        logger->saveCsv((run_directory / "raw.csv").string());
        log_saved = true;
        const bool completed =
            session.phase() == raven_control::control::
                GravityValidationPhase::Complete;
        writeRunStatus(
            run_directory / "run_status.yaml",
            completed ? "completed" : "aborted",
            completed_measurements,
            completed
                ? "Stopped normally by user"
                : "Stopped by abort request or signal");
        std::cout
            << "\nMotors disabled. Run bundle: "
            << run_directory << '\n';
        return completed ? 0 : 1;
    } catch (const std::exception& error) {
        if (logger && !log_saved && !run_directory.empty() &&
            logger->sampleCount() > 0) {
            try {
                logger->saveCsv((run_directory / "raw.csv").string());
                log_saved = true;
            } catch (const std::exception& log_error) {
                std::cerr
                    << "\nFailed to save partial log: "
                    << log_error.what() << '\n';
            }
        }
        if (!run_directory.empty()) {
            try {
                writeRunStatus(
                    run_directory / "run_status.yaml",
                    "failed",
                    completed_measurements,
                    error.what());
            } catch (const std::exception&) {
            }
        }
        std::cerr << "\nFatal error: " << error.what() << '\n';
        return 1;
    }
}
