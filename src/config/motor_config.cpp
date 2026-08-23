#include "raven_control/config/motor_config.hpp"

#include "raven_control/hal/motor_driver.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <unordered_set>

#ifndef RAVEN_HAS_YAML_CPP
#define RAVEN_HAS_YAML_CPP 0
#endif

#if RAVEN_HAS_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif

namespace raven_control::config {
namespace {

constexpr std::array<const char*, GRAVITY_COMPENSATION_JOINT_COUNT>
    RAVEN_JOINT_NAMES{{
        "shoulder_Joint",
        "upperArm_Joint",
        "foreArm_Joint",
    }};

void requireFinite(
    double value,
    const std::string& field_name,
    const std::string& joint_name)
{
    if (!std::isfinite(value)) {
        throw std::invalid_argument(
            "Joint '" + joint_name + "' has a non-finite " +
            field_name);
    }
}

#if RAVEN_HAS_YAML_CPP
template <typename T>
T requiredValue(
    const YAML::Node& node,
    const char* field_name,
    const std::string& context)
{
    const YAML::Node value = node[field_name];
    if (!value) {
        throw std::runtime_error(
            context + " is missing '" + field_name + "'");
    }
    try {
        return value.as<T>();
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            context + " has invalid '" + field_name +
            "': " + error.what());
    }
}

std::chrono::milliseconds requiredMilliseconds(
    const YAML::Node& node,
    const char* field_name)
{
    const int value =
        requiredValue<int>(node, field_name, "Runtime config");
    return std::chrono::milliseconds(value);
}
#endif

}  // namespace

void JointMotorRuntimeConfig::validate() const
{
    if (joint_name.empty())
        throw std::invalid_argument("Motor config joint name is empty");
    if (motor_id > 127) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' requires motor_id in range 0..127");
    }
    if (position_sign != -1 && position_sign != 1) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' requires position_sign to be -1 or 1");
    }

    requireFinite(
        joint_zero_at_motor_rad,
        "joint_zero_at_motor_rad",
        joint_name);
    requireFinite(
        joint_to_motor_ratio,
        "joint_to_motor_ratio",
        joint_name);
    requireFinite(position_control.kp, "kp", joint_name);
    requireFinite(position_control.kd, "kd", joint_name);
    requireFinite(
        position_control.max_slew_rate_rad_s,
        "max_slew_rate_rad_s",
        joint_name);

    if (joint_to_motor_ratio <= 0.0) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' requires joint_to_motor_ratio > 0");
    }
    if (position_control.kp < 0.0 ||
        position_control.kp >
            hal::RS02_OPERATION_MAX_KP) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' requires kp in range 0..500");
    }
    if (position_control.kd < 0.0 ||
        position_control.kd >
            hal::RS02_OPERATION_MAX_KD) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' requires kd in range 0..5");
    }
    if (position_control.max_slew_rate_rad_s <= 0.0 ||
        position_control.max_slew_rate_rad_s >
            hal::RS02_OPERATION_MAX_VELOCITY_RAD_S) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' requires max_slew_rate_rad_s in range (0, 44]");
    }
}

void MotorRuntimeConfig::validate() const
{
    if (control_period.count() <= 0)
        throw std::invalid_argument("control_period_ms must be positive");
    if (feedback_timeout.count() <= 0)
        throw std::invalid_argument("feedback_timeout_ms must be positive");
    if (position_request_period.count() <= 0) {
        throw std::invalid_argument(
            "position_request_period_ms must be positive");
    }
    if (control_period > position_request_period) {
        throw std::invalid_argument(
            "control_period_ms must not exceed "
            "position_request_period_ms");
    }
    if (position_request_period >= feedback_timeout) {
        throw std::invalid_argument(
            "position_request_period_ms must be less than "
            "feedback_timeout_ms");
    }
    if (joints.empty())
        throw std::invalid_argument("Motor config contains no joints");
    if (gravity_compensation.urdf_path.empty()) {
        throw std::invalid_argument(
            "gravity_compensation.urdf_path must not be empty");
    }
    if (!std::isfinite(gravity_compensation.scale) ||
        gravity_compensation.scale < 0.0 ||
        gravity_compensation.scale > 1.0) {
        throw std::invalid_argument(
            "gravity_compensation.scale must be in range 0..1");
    }
    if (gravity_compensation.ramp_duration.count() <= 0) {
        throw std::invalid_argument(
            "gravity_compensation.ramp_duration_ms must be positive");
    }
    for (double limit : gravity_compensation.max_joint_torque_nm) {
        if (!std::isfinite(limit) || limit < 0.0) {
            throw std::invalid_argument(
                "gravity compensation torque limits must be finite and "
                "nonnegative");
        }
    }

    std::unordered_set<std::string> joint_names;
    std::unordered_set<std::uint8_t> motor_ids;
    for (const auto& joint : joints) {
        joint.validate();
        if (!joint_names.insert(joint.joint_name).second) {
            throw std::invalid_argument(
                "Duplicate motor config joint: " + joint.joint_name);
        }
        if (!motor_ids.insert(joint.motor_id).second) {
            throw std::invalid_argument(
                "Duplicate motor ID in motor config: " +
                std::to_string(joint.motor_id));
        }
    }

    for (std::size_t index = 0;
         index < RAVEN_JOINT_NAMES.size();
         ++index) {
        const auto* joint = findJoint(RAVEN_JOINT_NAMES[index]);
        if (joint == nullptr)
            continue;
        const double maximum_joint_torque =
            hal::RS02_OPERATION_MAX_TORQUE_NM *
            joint->joint_to_motor_ratio;
        if (gravity_compensation.max_joint_torque_nm[index] >
            maximum_joint_torque) {
            throw std::invalid_argument(
                "Gravity torque limit for '" +
                std::string(RAVEN_JOINT_NAMES[index]) +
                "' exceeds the calibrated RS02 range");
        }
    }
}

const JointMotorRuntimeConfig* MotorRuntimeConfig::findJoint(
    const std::string& joint_name) const noexcept
{
    for (const auto& joint : joints) {
        if (joint.joint_name == joint_name)
            return &joint;
    }
    return nullptr;
}

MotorRuntimeConfig loadMotorRuntimeConfig(
    const std::string& yaml_path)
{
#if RAVEN_HAS_YAML_CPP
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            "Cannot load motor config file '" + yaml_path +
            "': " + error.what());
    }

    const YAML::Node runtime = root["runtime"];
    if (!runtime || !runtime.IsMap()) {
        throw std::runtime_error(
            "Motor config file '" + yaml_path +
            "' must contain a 'runtime' map");
    }

    MotorRuntimeConfig config;
    config.control_period =
        requiredMilliseconds(runtime, "control_period_ms");
    config.feedback_timeout =
        requiredMilliseconds(runtime, "feedback_timeout_ms");
    config.position_request_period =
        requiredMilliseconds(runtime, "position_request_period_ms");
    // Backward-compatible read of the legacy switch. New configuration
    // belongs to the top-level gravity_compensation map below.
    if (const YAML::Node enabled = runtime["gravity_compensation_enabled"]) {
        try {
            config.gravity_compensation.enabled = enabled.as<bool>();
        } catch (const YAML::Exception& error) {
            throw std::runtime_error(
                "Runtime config has invalid "
                "'gravity_compensation_enabled': " +
                std::string(error.what()));
        }
    }

    const YAML::Node joints = root["joints"];
    if (!joints || !joints.IsMap()) {
        throw std::runtime_error(
            "Motor config file '" + yaml_path +
            "' must contain a 'joints' map");
    }

    for (const auto& entry : joints) {
        const std::string joint_name = entry.first.as<std::string>();
        const YAML::Node node = entry.second;
        const std::string context = "Joint '" + joint_name + "'";
        if (!node.IsMap())
            throw std::runtime_error(context + " must be a map");

        const int motor_id =
            requiredValue<int>(node, "motor_id", context);
        if (motor_id < 0 || motor_id > 127) {
            throw std::runtime_error(
                context + " requires motor_id in range 0..127");
        }

        const YAML::Node calibration = node["calibration"];
        if (!calibration || !calibration.IsMap()) {
            throw std::runtime_error(
                context + " must contain a 'calibration' map");
        }
        const YAML::Node position_control =
            node["position_control"];
        if (!position_control || !position_control.IsMap()) {
            throw std::runtime_error(
                context +
                " must contain a 'position_control' map");
        }

        JointMotorRuntimeConfig joint;
        joint.joint_name = joint_name;
        joint.motor_id = static_cast<std::uint8_t>(motor_id);
        joint.position_sign = requiredValue<int>(
            calibration,
            "position_sign",
            context + " calibration");
        joint.joint_zero_at_motor_rad = requiredValue<double>(
            calibration,
            "joint_zero_at_motor_rad",
            context + " calibration");
        joint.joint_to_motor_ratio = requiredValue<double>(
            calibration,
            "joint_to_motor_ratio",
            context + " calibration");
        joint.position_control.kp = requiredValue<double>(
            position_control,
            "kp",
            context + " position_control");
        joint.position_control.kd = requiredValue<double>(
            position_control,
            "kd",
            context + " position_control");
        joint.position_control.max_slew_rate_rad_s =
            requiredValue<double>(
                position_control,
                "max_slew_rate_rad_s",
                context + " position_control");
        config.joints.push_back(joint);
    }

    if (const YAML::Node gravity = root["gravity_compensation"]) {
        if (!gravity.IsMap()) {
            throw std::runtime_error(
                "gravity_compensation must be a map");
        }
        const std::string context = "Gravity compensation config";
        config.gravity_compensation.urdf_path =
            requiredValue<std::string>(gravity, "urdf_path", context);
        config.gravity_compensation.enabled =
            requiredValue<bool>(gravity, "enabled", context);
        config.gravity_compensation.dry_run =
            requiredValue<bool>(gravity, "dry_run", context);
        config.gravity_compensation.scale =
            requiredValue<double>(gravity, "scale", context);
        config.gravity_compensation.ramp_duration =
            std::chrono::milliseconds(requiredValue<int>(
                gravity, "ramp_duration_ms", context));

        const YAML::Node limits = gravity["max_joint_torque_nm"];
        if (!limits || !limits.IsMap()) {
            throw std::runtime_error(
                context + " requires a max_joint_torque_nm map");
        }
        for (std::size_t index = 0;
             index < RAVEN_JOINT_NAMES.size();
             ++index) {
            config.gravity_compensation.max_joint_torque_nm[index] =
                requiredValue<double>(
                    limits,
                    RAVEN_JOINT_NAMES[index],
                    context + " torque limits");
        }
    }

    config.validate();
    return config;
#else
    (void)yaml_path;
    throw std::runtime_error(
        "RAVEN_control was built without yaml-cpp support");
#endif
}

void saveMotorRuntimeConfig(
    const MotorRuntimeConfig& config,
    const std::string& yaml_path)
{
#if RAVEN_HAS_YAML_CPP
    config.validate();

    YAML::Emitter output;
    output << YAML::BeginMap;
    output << YAML::Key << "runtime" << YAML::Value
           << YAML::BeginMap;
    output << YAML::Key << "control_period_ms" << YAML::Value
           << config.control_period.count();
    output << YAML::Key << "feedback_timeout_ms" << YAML::Value
           << config.feedback_timeout.count();
    output << YAML::Key << "position_request_period_ms"
           << YAML::Value
           << config.position_request_period.count();
    output << YAML::EndMap;

    output << YAML::Key << "gravity_compensation" << YAML::Value
           << YAML::BeginMap;
    output << YAML::Key << "urdf_path" << YAML::Value
           << config.gravity_compensation.urdf_path;
    output << YAML::Key << "enabled" << YAML::Value
           << config.gravity_compensation.enabled;
    output << YAML::Key << "dry_run" << YAML::Value
           << config.gravity_compensation.dry_run;
    output << YAML::Key << "scale" << YAML::Value
           << config.gravity_compensation.scale;
    output << YAML::Key << "ramp_duration_ms" << YAML::Value
           << config.gravity_compensation.ramp_duration.count();
    output << YAML::Key << "max_joint_torque_nm" << YAML::Value
           << YAML::BeginMap;
    for (std::size_t index = 0;
         index < RAVEN_JOINT_NAMES.size();
         ++index) {
        output << YAML::Key << RAVEN_JOINT_NAMES[index]
               << YAML::Value
               << config.gravity_compensation.max_joint_torque_nm[index];
    }
    output << YAML::EndMap;
    output << YAML::EndMap;

    output << YAML::Key << "joints" << YAML::Value
           << YAML::BeginMap;
    for (const auto& joint : config.joints) {
        output << YAML::Key << joint.joint_name << YAML::Value
               << YAML::BeginMap;
        output << YAML::Key << "motor_id" << YAML::Value
               << static_cast<int>(joint.motor_id);

        output << YAML::Key << "calibration" << YAML::Value
               << YAML::BeginMap;
        output << YAML::Key << "position_sign" << YAML::Value
               << joint.position_sign;
        output << YAML::Key << "joint_zero_at_motor_rad"
               << YAML::Value << joint.joint_zero_at_motor_rad;
        output << YAML::Key << "joint_to_motor_ratio"
               << YAML::Value << joint.joint_to_motor_ratio;
        output << YAML::EndMap;

        output << YAML::Key << "position_control" << YAML::Value
               << YAML::BeginMap;
        output << YAML::Key << "kp" << YAML::Value
               << joint.position_control.kp;
        output << YAML::Key << "kd" << YAML::Value
               << joint.position_control.kd;
        output << YAML::Key << "max_slew_rate_rad_s"
               << YAML::Value
               << joint.position_control.max_slew_rate_rad_s;
        output << YAML::EndMap;
        output << YAML::EndMap;
    }
    output << YAML::EndMap;
    output << YAML::EndMap;

    if (!output.good()) {
        throw std::runtime_error(
            "Cannot serialize motor config: " +
            output.GetLastError());
    }

    const std::string temporary_path = yaml_path + ".tmp";
    {
        std::ofstream file(
            temporary_path,
            std::ios::binary | std::ios::trunc);
        if (!file) {
            throw std::runtime_error(
                "Cannot open temporary motor config file '" +
                temporary_path + "'");
        }
        file << output.c_str() << '\n';
        file.close();
        if (!file) {
            std::remove(temporary_path.c_str());
            throw std::runtime_error(
                "Cannot write temporary motor config file '" +
                temporary_path + "'");
        }
    }

    if (std::rename(
            temporary_path.c_str(),
            yaml_path.c_str()) != 0) {
        const std::string reason = std::strerror(errno);
        std::remove(temporary_path.c_str());
        throw std::runtime_error(
            "Cannot replace motor config file '" + yaml_path +
            "': " + reason);
    }
#else
    (void)config;
    (void)yaml_path;
    throw std::runtime_error(
        "RAVEN_control was built without yaml-cpp support");
#endif
}

}  // namespace raven_control::config
