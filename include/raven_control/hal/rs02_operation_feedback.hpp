#pragma once

#include "raven_control/hal/can_interface.hpp"

#include <cstdint>
#include <optional>

namespace raven_control::hal {

inline constexpr double RS02_OPERATION_MAX_KP = 500.0;
inline constexpr double RS02_OPERATION_MAX_KD = 5.0;
inline constexpr double RS02_OPERATION_MAX_VELOCITY_RAD_S = 44.0;
inline constexpr double RS02_OPERATION_MAX_TORQUE_NM = 17.0;

struct Rs02OperationFeedback {
    std::uint8_t motor_id = 0;
    double motor_position_rad = 0.0;
    double motor_velocity_rad_s = 0.0;
    double motor_torque_nm = 0.0;
    double temperature_celsius = 0.0;
    std::uint8_t fault_flags = 0;
    std::uint8_t mode_state = 0;
};

// Decodes communication Type 2 without sending anything to the CAN bus.
// Returns nullopt for unrelated, malformed, or differently addressed frames.
[[nodiscard]] std::optional<Rs02OperationFeedback>
decodeRs02OperationFeedback(
    const CanFrame& frame,
    std::uint8_t host_id = 0xFD) noexcept;

}  // namespace raven_control::hal
