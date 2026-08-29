#pragma once

#include <chrono>
#include <cstddef>
#include <string>

namespace raven_control::control {

enum class GravityValidationPhase {
    Disarmed,
    Ramp,
    Gate,
    Ready,
    Measurement,
    Complete,
    Aborted,
};

[[nodiscard]] const char* toString(
    GravityValidationPhase phase) noexcept;

[[nodiscard]] bool isFreshFeedbackTimestamp(
    std::chrono::steady_clock::time_point received_at,
    std::chrono::steady_clock::time_point observed_at,
    std::chrono::milliseconds timeout) noexcept;

struct GravityValidationSessionConfig {
    std::chrono::milliseconds gate_duration{30000};
    std::chrono::milliseconds measurement_duration{5000};

    void validate() const;
};

class GravityValidationSession {
public:
    explicit GravityValidationSession(
        GravityValidationSessionConfig config = {});

    void start(std::chrono::steady_clock::time_point now);
    void update(
        std::chrono::steady_clock::time_point now,
        bool gravity_ramp_complete);
    [[nodiscard]] bool requestMeasurement(
        std::chrono::steady_clock::time_point now);
    void complete() noexcept;
    void abort() noexcept;

    [[nodiscard]] GravityValidationPhase phase() const noexcept;
    [[nodiscard]] std::string phaseLabel() const;
    [[nodiscard]] std::size_t activeMeasurement() const noexcept;
    [[nodiscard]] std::size_t completedMeasurements() const noexcept;
    [[nodiscard]] std::chrono::milliseconds remaining(
        std::chrono::steady_clock::time_point now) const noexcept;

private:
    GravityValidationSessionConfig config_;
    GravityValidationPhase phase_ = GravityValidationPhase::Disarmed;
    std::chrono::steady_clock::time_point phase_started_at_{};
    std::size_t active_measurement_ = 0;
    std::size_t completed_measurements_ = 0;
};

}  // namespace raven_control::control
