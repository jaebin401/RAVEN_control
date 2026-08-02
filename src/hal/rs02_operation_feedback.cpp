#include "raven_control/hal/rs02_operation_feedback.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace raven_control::hal {
namespace {

constexpr std::uint8_t COMM_OPERATION_FEEDBACK = 2;
constexpr double PI = 3.14159265358979323846;
constexpr double POSITION_SCALE_RAD = 4.0 * PI;

std::uint16_t unpackU16BigEndian(
    const std::array<std::uint8_t, 8>& data,
    std::size_t offset) noexcept
{
    return
        (std::uint16_t(data[offset]) << 8) |
        std::uint16_t(data[offset + 1]);
}

double decodeSymmetric(
    std::uint16_t encoded,
    double absolute_limit) noexcept
{
    return
        ((static_cast<double>(encoded) / 65535.0) * 2.0 - 1.0) *
        absolute_limit;
}

}  // namespace

std::optional<Rs02OperationFeedback> decodeRs02OperationFeedback(
    const CanFrame& frame,
    std::uint8_t host_id) noexcept
{
    if (frame.dlc != 8)
        return std::nullopt;

    const auto communication_type = static_cast<std::uint8_t>(
        (frame.id >> 24) & 0x1F);
    const auto destination_host_id = static_cast<std::uint8_t>(
        frame.id & 0xFF);
    if (communication_type != COMM_OPERATION_FEEDBACK ||
        destination_host_id != host_id) {
        return std::nullopt;
    }

    Rs02OperationFeedback feedback;
    feedback.motor_id = static_cast<std::uint8_t>(
        (frame.id >> 8) & 0xFF);
    feedback.motor_position_rad = decodeSymmetric(
        unpackU16BigEndian(frame.data, 0),
        POSITION_SCALE_RAD);
    feedback.motor_velocity_rad_s = decodeSymmetric(
        unpackU16BigEndian(frame.data, 2),
        RS02_OPERATION_MAX_VELOCITY_RAD_S);
    feedback.motor_torque_nm = decodeSymmetric(
        unpackU16BigEndian(frame.data, 4),
        RS02_OPERATION_MAX_TORQUE_NM);
    const auto raw_temperature = static_cast<std::int16_t>(
        unpackU16BigEndian(frame.data, 6));
    feedback.temperature_celsius =
        static_cast<double>(raw_temperature) / 10.0;
    feedback.fault_flags = static_cast<std::uint8_t>(
        (frame.id >> 16) & 0x3F);
    feedback.mode_state = static_cast<std::uint8_t>(
        (frame.id >> 22) & 0x03);
    return feedback;
}

}  // namespace raven_control::hal
