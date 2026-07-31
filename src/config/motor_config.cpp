#include "raven_control/config/motor_config.hpp"

#include "raven_control/hal/motor_driver.hpp"

#include <cmath>
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

    config.validate();
    return config;
#else
    (void)yaml_path;
    throw std::runtime_error(
        "RAVEN_control was built without yaml-cpp support");
#endif
}

}  // namespace raven_control::config
