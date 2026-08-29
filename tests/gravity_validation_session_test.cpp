#include "raven_control/control/gravity_validation_session.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using Clock = std::chrono::steady_clock;
using Phase = raven_control::control::GravityValidationPhase;

void check(bool condition, const std::string& message)
{
    if (!condition)
        throw std::runtime_error(message);
}

void testNoAutomaticMotionAndTimedPhases()
{
    raven_control::control::GravityValidationSessionConfig config;
    config.gate_duration = std::chrono::milliseconds(30000);
    config.measurement_duration = std::chrono::milliseconds(5000);
    raven_control::control::GravityValidationSession session(config);
    const auto start = Clock::now();

    check(session.phase() == Phase::Disarmed,
          "session must begin disarmed");
    check(!session.requestMeasurement(start),
          "measurement must be rejected while disarmed");

    session.start(start);
    check(session.phase() == Phase::Ramp,
          "start must enter gravity ramp without a motion phase");
    session.update(start + std::chrono::seconds(60), false);
    check(session.phase() == Phase::Ramp,
          "time alone must not bypass an incomplete gravity ramp");

    const auto gate_start = start + std::chrono::seconds(61);
    session.update(gate_start, true);
    check(session.phase() == Phase::Gate,
          "completed ramp must enter the stationary command gate");
    check(session.remaining(gate_start).count() == 30000,
          "gate must expose its full initial duration");

    session.update(gate_start + std::chrono::milliseconds(29999), true);
    check(session.phase() == Phase::Gate,
          "gate must not finish early");
    session.update(gate_start + std::chrono::milliseconds(30000), true);
    check(session.phase() == Phase::Ready,
          "gate must end in manual-ready state");

    const auto measurement_start =
        gate_start + std::chrono::milliseconds(31000);
    check(session.requestMeasurement(measurement_start),
          "manual-ready state must accept an explicit measurement mark");
    check(session.phaseLabel() == "measurement_1",
          "measurement label must carry its sequence number");
    session.update(
        measurement_start + std::chrono::milliseconds(4999), true);
    check(session.phase() == Phase::Measurement,
          "measurement must retain the full configured window");
    session.update(
        measurement_start + std::chrono::milliseconds(5000), true);
    check(session.phase() == Phase::Ready,
          "measurement must return to manual-ready state");
    check(session.completedMeasurements() == 1,
          "completed measurement count must advance once");
}

void testStopStates()
{
    const auto now = Clock::now();
    raven_control::control::GravityValidationSession completed;
    completed.start(now);
    completed.complete();
    check(completed.phase() == Phase::Complete,
          "explicit stop must complete the session");

    raven_control::control::GravityValidationSession aborted;
    aborted.start(now);
    aborted.abort();
    aborted.complete();
    check(aborted.phase() == Phase::Aborted,
          "abort state must not be overwritten by normal completion");
}

void testFeedbackFreshnessUsesPostPollObservationTime()
{
    const auto received_at = Clock::now();
    const auto timeout = std::chrono::milliseconds(250);

    check(
        raven_control::control::isFreshFeedbackTimestamp(
            received_at,
            received_at + std::chrono::milliseconds(1),
            timeout),
        "fresh feedback observed after reception must be accepted");
    check(
        raven_control::control::isFreshFeedbackTimestamp(
            received_at,
            received_at + timeout,
            timeout),
        "feedback exactly at the timeout must remain valid");
    check(
        !raven_control::control::isFreshFeedbackTimestamp(
            received_at,
            received_at + timeout + std::chrono::milliseconds(1),
            timeout),
        "feedback older than the timeout must be rejected");
    check(
        !raven_control::control::isFreshFeedbackTimestamp(
            received_at,
            received_at - std::chrono::milliseconds(1),
            timeout),
        "a pre-poll observation time must not validate later feedback");
}

}  // namespace

int main()
{
    testNoAutomaticMotionAndTimedPhases();
    testStopStates();
    testFeedbackFreshnessUsesPostPollObservationTime();
    std::cout << "gravity_validation_session_test passed\n";
    return 0;
}
