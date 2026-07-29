#pragma once

#include <optional>
#include <string>
#include <unordered_map>

namespace raven_control::safety {

struct JointLimitConfig {
    std::string joint_name;
    bool confirmed = false;
    double hard_min_rad = 0.0;
    double hard_max_rad = 0.0;
    double soft_margin_rad = 0.0;
    bool soft_wall_enabled = false;
    double soft_wall_stiffness_nm_per_rad = 0.0;
    double soft_wall_max_torque_nm = 0.0;

    void validate() const;
};

class JointLimiter {
public:
    explicit JointLimiter(JointLimitConfig config);

    [[nodiscard]] std::optional<double> clampTarget(
        double target_position_rad) const noexcept;
    [[nodiscard]] double softWallTorque(
        double measured_position_rad) const noexcept;
    [[nodiscard]] bool isHardViolation(
        double measured_position_rad) const noexcept;
    [[nodiscard]] bool isConfirmed() const noexcept;
    [[nodiscard]] double softMinRad() const noexcept;
    [[nodiscard]] double softMaxRad() const noexcept;
    [[nodiscard]] const JointLimitConfig& config() const noexcept;

private:
    JointLimitConfig config_;
};

using JointLimiterMap = std::unordered_map<std::string, JointLimiter>;

[[nodiscard]] JointLimiterMap loadJointLimiters(
    const std::string& yaml_path);

}  // namespace raven_control::safety
