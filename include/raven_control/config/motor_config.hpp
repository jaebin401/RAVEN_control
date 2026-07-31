#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace raven_control::config {

struct PositionControlConfig {
    double kp = 0.0;
    double kd = 0.0;
    double max_slew_rate_rad_s = 0.0;
};

struct JointMotorRuntimeConfig {
    std::string joint_name;
    std::uint8_t motor_id = 0;
    int position_sign = 1;
    double joint_zero_at_motor_rad = 0.0;
    double joint_to_motor_ratio = 1.0;
    PositionControlConfig position_control;

    void validate() const;
};

struct MotorRuntimeConfig {
    std::chrono::milliseconds control_period{20};
    std::chrono::milliseconds feedback_timeout{250};
    std::chrono::milliseconds position_request_period{100};
    std::vector<JointMotorRuntimeConfig> joints;

    void validate() const;

    [[nodiscard]] const JointMotorRuntimeConfig* findJoint(
        const std::string& joint_name) const noexcept;
};

[[nodiscard]] MotorRuntimeConfig loadMotorRuntimeConfig(
    const std::string& yaml_path);

}  // namespace raven_control::config
