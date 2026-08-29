#pragma once

#include <cstdint>

namespace raven_control::hal {

struct MitTorqueAudit {
    double requested_joint_torque_nm = 0.0;
    double motor_torque_nm = 0.0;
    std::uint16_t encoded_torque = 0;
    double decoded_motor_torque_nm = 0.0;
    double decoded_joint_torque_nm = 0.0;
};

[[nodiscard]] double jointToMotorTorque(
    int position_sign,
    double joint_to_motor_ratio,
    double joint_torque_nm);

[[nodiscard]] double motorToJointTorque(
    int position_sign,
    double joint_to_motor_ratio,
    double motor_torque_nm);

[[nodiscard]] std::uint16_t encodeRs02Torque(
    double motor_torque_nm) noexcept;

[[nodiscard]] double decodeRs02Torque(
    std::uint16_t encoded_torque) noexcept;

[[nodiscard]] MitTorqueAudit auditMitTorqueCommand(
    int position_sign,
    double joint_to_motor_ratio,
    double requested_joint_torque_nm);

}  // namespace raven_control::hal
