#include "raven_control/control/gravity_validation_session.hpp"

#include <algorithm>
#include <stdexcept>

namespace raven_control::control {

const char* toString(GravityValidationPhase phase) noexcept
{
    switch (phase) {
    case GravityValidationPhase::Disarmed:
        return "disarmed";
    case GravityValidationPhase::Ramp:
        return "gravity_ramp";
    case GravityValidationPhase::Gate:
        return "command_gate";
    case GravityValidationPhase::Ready:
        return "manual_ready";
    case GravityValidationPhase::Measurement:
        return "measurement";
    case GravityValidationPhase::Complete:
        return "complete";
    case GravityValidationPhase::Aborted:
        return "aborted";
    }
    return "unknown";
}

bool isFreshFeedbackTimestamp(
    std::chrono::steady_clock::time_point received_at,
    std::chrono::steady_clock::time_point observed_at,
    std::chrono::milliseconds timeout) noexcept
{
    return timeout.count() > 0 &&
           received_at != std::chrono::steady_clock::time_point{} &&
           observed_at >= received_at &&
           observed_at - received_at <= timeout;
}

void GravityValidationSessionConfig::validate() const
{
    if (gate_duration.count() <= 0)
        throw std::invalid_argument("gate duration must be positive");
    if (measurement_duration.count() <= 0) {
        throw std::invalid_argument(
            "measurement duration must be positive");
    }
}

GravityValidationSession::GravityValidationSession(
    GravityValidationSessionConfig config)
    : config_(config)
{
    config_.validate();
}

void GravityValidationSession::start(
    std::chrono::steady_clock::time_point now)
{
    if (phase_ != GravityValidationPhase::Disarmed) {
        throw std::logic_error(
            "gravity validation session can only start once");
    }
    phase_ = GravityValidationPhase::Ramp;
    phase_started_at_ = now;
}

void GravityValidationSession::update(
    std::chrono::steady_clock::time_point now,
    bool gravity_ramp_complete)
{
    if (now < phase_started_at_)
        return;

    if (phase_ == GravityValidationPhase::Ramp &&
        gravity_ramp_complete) {
        phase_ = GravityValidationPhase::Gate;
        phase_started_at_ = now;
        return;
    }
    if (phase_ == GravityValidationPhase::Gate &&
        now - phase_started_at_ >= config_.gate_duration) {
        phase_ = GravityValidationPhase::Ready;
        phase_started_at_ = now;
        return;
    }
    if (phase_ == GravityValidationPhase::Measurement &&
        now - phase_started_at_ >= config_.measurement_duration) {
        ++completed_measurements_;
        phase_ = GravityValidationPhase::Ready;
        phase_started_at_ = now;
    }
}

bool GravityValidationSession::requestMeasurement(
    std::chrono::steady_clock::time_point now)
{
    if (phase_ != GravityValidationPhase::Ready)
        return false;
    active_measurement_ = completed_measurements_ + 1;
    phase_ = GravityValidationPhase::Measurement;
    phase_started_at_ = now;
    return true;
}

void GravityValidationSession::complete() noexcept
{
    if (phase_ != GravityValidationPhase::Aborted)
        phase_ = GravityValidationPhase::Complete;
}

void GravityValidationSession::abort() noexcept
{
    phase_ = GravityValidationPhase::Aborted;
}

GravityValidationPhase GravityValidationSession::phase() const noexcept
{
    return phase_;
}

std::string GravityValidationSession::phaseLabel() const
{
    if (phase_ != GravityValidationPhase::Measurement)
        return toString(phase_);
    return std::string(toString(phase_)) + '_' +
           std::to_string(active_measurement_);
}

std::size_t GravityValidationSession::activeMeasurement() const noexcept
{
    return active_measurement_;
}

std::size_t GravityValidationSession::completedMeasurements() const noexcept
{
    return completed_measurements_;
}

std::chrono::milliseconds GravityValidationSession::remaining(
    std::chrono::steady_clock::time_point now) const noexcept
{
    std::chrono::milliseconds duration{0};
    if (phase_ == GravityValidationPhase::Gate)
        duration = config_.gate_duration;
    else if (phase_ == GravityValidationPhase::Measurement)
        duration = config_.measurement_duration;
    else
        return std::chrono::milliseconds(0);

    if (now <= phase_started_at_)
        return duration;
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - phase_started_at_);
    return std::max(std::chrono::milliseconds(0), duration - elapsed);
}

}  // namespace raven_control::control
