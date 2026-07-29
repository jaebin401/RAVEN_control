#include "raven_control/safety/joint_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#ifndef RAVEN_HAS_YAML_CPP
#define RAVEN_HAS_YAML_CPP 0
#endif

#if RAVEN_HAS_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif

namespace raven_control::safety {
namespace {

void requireFinite(double value, const std::string& field_name,
                   const std::string& joint_name)
{
    if (!std::isfinite(value)) {
        throw std::invalid_argument(
            "Joint '" + joint_name + "' has a non-finite " + field_name);
    }
}

#if RAVEN_HAS_YAML_CPP
template <typename T>
T requiredValue(const YAML::Node& node, const char* field_name,
                const std::string& joint_name)
{
    const YAML::Node value = node[field_name];
    if (!value) {
        throw std::runtime_error(
            "Joint '" + joint_name + "' is missing '" + field_name + "'");
    }
    try {
        return value.as<T>();
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            "Joint '" + joint_name + "' has invalid '" + field_name +
            "': " + error.what());
    }
}
#endif

}  // namespace

void JointLimitConfig::validate() const
{
    if (joint_name.empty())
        throw std::invalid_argument("Joint limit name must not be empty");

    requireFinite(hard_min_rad, "hard_min_rad", joint_name);
    requireFinite(hard_max_rad, "hard_max_rad", joint_name);
    requireFinite(soft_margin_rad, "soft_margin_rad", joint_name);
    requireFinite(
        soft_wall_stiffness_nm_per_rad,
        "soft_wall_stiffness_nm_per_rad",
        joint_name);
    requireFinite(
        soft_wall_max_torque_nm,
        "soft_wall_max_torque_nm",
        joint_name);

    if (hard_min_rad >= hard_max_rad) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' requires hard_min_rad < hard_max_rad");
    }
    if (soft_margin_rad < 0.0) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' requires soft_margin_rad >= 0");
    }

    const double position_range = hard_max_rad - hard_min_rad;
    if (2.0 * soft_margin_rad >= position_range) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' soft margins leave no commandable range");
    }
    if (soft_wall_stiffness_nm_per_rad < 0.0 ||
        soft_wall_max_torque_nm < 0.0) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' soft-wall gains must not be negative");
    }
    if (soft_wall_enabled &&
        (soft_wall_stiffness_nm_per_rad <= 0.0 ||
         soft_wall_max_torque_nm <= 0.0)) {
        throw std::invalid_argument(
            "Joint '" + joint_name +
            "' enables soft wall without positive stiffness and max torque");
    }
}

JointLimiter::JointLimiter(JointLimitConfig config)
    : config_(std::move(config))
{
    config_.validate();
}

std::optional<double> JointLimiter::clampTarget(
    double target_position_rad) const noexcept
{
    if (!std::isfinite(target_position_rad))
        return std::nullopt;
    return std::clamp(
        target_position_rad,
        softMinRad(),
        softMaxRad());
}

double JointLimiter::softWallTorque(
    double measured_position_rad) const noexcept
{
    if (!config_.soft_wall_enabled ||
        !std::isfinite(measured_position_rad)) {
        return 0.0;
    }

    if (measured_position_rad < softMinRad()) {
        const double restoring_torque =
            config_.soft_wall_stiffness_nm_per_rad *
            (softMinRad() - measured_position_rad);
        return std::min(
            restoring_torque,
            config_.soft_wall_max_torque_nm);
    }
    if (measured_position_rad > softMaxRad()) {
        const double restoring_torque =
            config_.soft_wall_stiffness_nm_per_rad *
            (measured_position_rad - softMaxRad());
        return -std::min(
            restoring_torque,
            config_.soft_wall_max_torque_nm);
    }
    return 0.0;
}

bool JointLimiter::isHardViolation(
    double measured_position_rad) const noexcept
{
    return !std::isfinite(measured_position_rad) ||
           measured_position_rad < config_.hard_min_rad ||
           measured_position_rad > config_.hard_max_rad;
}

bool JointLimiter::isConfirmed() const noexcept
{
    return config_.confirmed;
}

double JointLimiter::softMinRad() const noexcept
{
    return config_.hard_min_rad + config_.soft_margin_rad;
}

double JointLimiter::softMaxRad() const noexcept
{
    return config_.hard_max_rad - config_.soft_margin_rad;
}

const JointLimitConfig& JointLimiter::config() const noexcept
{
    return config_;
}

JointLimiterMap loadJointLimiters(const std::string& yaml_path)
{
#if RAVEN_HAS_YAML_CPP
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            "Cannot load joint limit file '" + yaml_path +
            "': " + error.what());
    }

    const YAML::Node joints = root["joints"];
    if (!joints || !joints.IsMap()) {
        throw std::runtime_error(
            "Joint limit file '" + yaml_path +
            "' must contain a 'joints' map");
    }

    JointLimiterMap limiters;
    for (const auto& entry : joints) {
        const std::string joint_name = entry.first.as<std::string>();
        const YAML::Node node = entry.second;
        if (!node.IsMap()) {
            throw std::runtime_error(
                "Joint '" + joint_name + "' must contain a settings map");
        }

        JointLimitConfig config;
        config.joint_name = joint_name;
        config.confirmed =
            requiredValue<bool>(node, "confirmed", joint_name);
        config.hard_min_rad =
            requiredValue<double>(node, "hard_min_rad", joint_name);
        config.hard_max_rad =
            requiredValue<double>(node, "hard_max_rad", joint_name);
        config.soft_margin_rad =
            requiredValue<double>(node, "soft_margin_rad", joint_name);
        config.soft_wall_enabled =
            requiredValue<bool>(node, "soft_wall_enabled", joint_name);
        config.soft_wall_stiffness_nm_per_rad = requiredValue<double>(
            node, "soft_wall_stiffness_nm_per_rad", joint_name);
        config.soft_wall_max_torque_nm = requiredValue<double>(
            node, "soft_wall_max_torque_nm", joint_name);
        config.validate();

        const auto insertion =
            limiters.emplace(joint_name, JointLimiter(config));
        if (!insertion.second) {
            throw std::runtime_error(
                "Duplicate joint limit entry: " + joint_name);
        }
    }

    if (limiters.empty()) {
        throw std::runtime_error(
            "Joint limit file '" + yaml_path + "' contains no joints");
    }
    return limiters;
#else
    (void)yaml_path;
    throw std::runtime_error(
        "RAVEN_control was built without yaml-cpp support");
#endif
}

}  // namespace raven_control::safety
