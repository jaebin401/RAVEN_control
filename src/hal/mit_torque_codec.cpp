#include "raven_control/hal/mit_torque_codec.hpp"

#include "raven_control/hal/rs02_operation_feedback.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace raven_control::hal {
namespace {

void validateCalibration(int position_sign, double joint_to_motor_ratio)
{
    if (position_sign != -1 && position_sign != 1) {
        throw std::invalid_argument(
            "position_sign must be either -1 or 1");
    }
    if (!std::isfinite(joint_to_motor_ratio) ||
        joint_to_motor_ratio <= 0.0) {
        throw std::invalid_argument(
            "joint_to_motor_ratio must be finite and positive");
    }
}

}  // namespace

double jointToMotorTorque(
    int position_sign,
    double joint_to_motor_ratio,
    double joint_torque_nm)
{
    validateCalibration(position_sign, joint_to_motor_ratio);
    if (!std::isfinite(joint_torque_nm))
        throw std::invalid_argument("joint torque must be finite");

    // Preserve virtual work for q_motor = sign * ratio * q_joint.
    return static_cast<double>(position_sign) *
           joint_torque_nm / joint_to_motor_ratio;
}

double motorToJointTorque(
    int position_sign,
    double joint_to_motor_ratio,
    double motor_torque_nm)
{
    validateCalibration(position_sign, joint_to_motor_ratio);
    if (!std::isfinite(motor_torque_nm))
        throw std::invalid_argument("motor torque must be finite");

    return static_cast<double>(position_sign) *
           joint_to_motor_ratio * motor_torque_nm;
}

std::uint16_t encodeRs02Torque(double motor_torque_nm) noexcept
{
    const double clamped = std::clamp(
        motor_torque_nm,
        -RS02_OPERATION_MAX_TORQUE_NM,
        RS02_OPERATION_MAX_TORQUE_NM);
    const double normalized =
        ((clamped / RS02_OPERATION_MAX_TORQUE_NM) + 1.0) * 32767.5;
    return static_cast<std::uint16_t>(normalized);
}

double decodeRs02Torque(std::uint16_t encoded_torque) noexcept
{
    return
        ((static_cast<double>(encoded_torque) / 65535.0) * 2.0 - 1.0) *
        RS02_OPERATION_MAX_TORQUE_NM;
}

MitTorqueAudit auditMitTorqueCommand(
    int position_sign,
    double joint_to_motor_ratio,
    double requested_joint_torque_nm)
{
    MitTorqueAudit result;
    result.requested_joint_torque_nm = requested_joint_torque_nm;
    result.motor_torque_nm = jointToMotorTorque(
        position_sign,
        joint_to_motor_ratio,
        requested_joint_torque_nm);
    result.encoded_torque = encodeRs02Torque(result.motor_torque_nm);
    result.decoded_motor_torque_nm =
        decodeRs02Torque(result.encoded_torque);
    result.decoded_joint_torque_nm = motorToJointTorque(
        position_sign,
        joint_to_motor_ratio,
        result.decoded_motor_torque_nm);
    return result;
}

}  // namespace raven_control::hal
