#pragma once

#include "raven_control/config/motor_config.hpp"
#include "raven_control/hal/rs02_operation_feedback.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace raven_control::telemetry {

struct JointStateSample {
    std::string joint_name;
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double effort_nm = 0.0;
    double temperature_celsius = 0.0;
    std::uint8_t fault_flags = 0;
    std::uint8_t mode_state = 0;
    std::chrono::steady_clock::time_point received_at{};
};

struct JointStateSnapshot {
    std::vector<JointStateSample> joints;
    std::chrono::steady_clock::time_point captured_at{};
};

// Read-only aggregation of Type 2 feedback. This class never owns a CAN
// transport and cannot enable, stop, or command a motor.
class JointStateAggregator {
public:
    explicit JointStateAggregator(
        std::vector<config::JointMotorRuntimeConfig> joints);

    [[nodiscard]] bool ingest(
        const hal::Rs02OperationFeedback& feedback,
        std::chrono::steady_clock::time_point received_at =
            std::chrono::steady_clock::now());

    [[nodiscard]] std::optional<JointStateSnapshot> snapshot(
        std::chrono::steady_clock::time_point now,
        std::chrono::milliseconds freshness_timeout) const;

    [[nodiscard]] std::size_t jointCount() const noexcept;

private:
    struct Channel {
        config::JointMotorRuntimeConfig config;
        JointStateSample sample;
        bool valid = false;
    };

    std::vector<Channel> channels_;
    std::unordered_map<std::uint8_t, std::size_t> motor_to_index_;
};

}  // namespace raven_control::telemetry
