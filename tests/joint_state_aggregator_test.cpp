#include "raven_control/hal/rs02_operation_feedback.hpp"
#include "raven_control/telemetry/joint_state_aggregator.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>

namespace {

int failures = 0;

void check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

void packU16BigEndian(
    std::array<std::uint8_t, 8>& data,
    std::size_t offset,
    std::uint16_t value)
{
    data[offset] = static_cast<std::uint8_t>(value >> 8);
    data[offset + 1] = static_cast<std::uint8_t>(value & 0xFF);
}

void testType2DecoderRejectsUnrelatedFrames()
{
    raven_control::hal::CanFrame frame;
    frame.id = (std::uint32_t(1) << 24) | 0xFD;
    frame.dlc = 8;
    check(
        !raven_control::hal::decodeRs02OperationFeedback(frame),
        "Type 1 command must not be decoded as Type 2 feedback");

    frame.id = (std::uint32_t(2) << 24) | 0x01;
    check(
        !raven_control::hal::decodeRs02OperationFeedback(frame),
        "feedback addressed to another host must be ignored");
}

void testType2DecoderExtractsMetadata()
{
    raven_control::hal::CanFrame frame;
    constexpr std::uint8_t motor_id = 3;
    constexpr std::uint8_t fault_flags = 5;
    constexpr std::uint8_t mode_state = 2;
    frame.id =
        (std::uint32_t(2) << 24) |
        (std::uint32_t(mode_state) << 22) |
        (std::uint32_t(fault_flags) << 16) |
        (std::uint32_t(motor_id) << 8) |
        0xFD;
    frame.dlc = 8;
    packU16BigEndian(frame.data, 0, 32768);
    packU16BigEndian(frame.data, 2, 32768);
    packU16BigEndian(frame.data, 4, 32768);
    packU16BigEndian(frame.data, 6, 253);

    const auto decoded =
        raven_control::hal::decodeRs02OperationFeedback(frame);
    check(decoded.has_value(), "valid Type 2 feedback must decode");
    if (!decoded)
        return;
    check(decoded->motor_id == motor_id, "motor ID must decode");
    check(decoded->fault_flags == fault_flags, "fault flags must decode");
    check(decoded->mode_state == mode_state, "mode state must decode");
    check(
        std::abs(decoded->temperature_celsius - 25.3) < 1e-12,
        "temperature must decode in degrees Celsius");
}

raven_control::config::JointMotorRuntimeConfig makeJoint(
    std::string name,
    std::uint8_t motor_id,
    int sign = 1,
    double zero = 0.0,
    double ratio = 1.0)
{
    raven_control::config::JointMotorRuntimeConfig result;
    result.joint_name = std::move(name);
    result.motor_id = motor_id;
    result.position_sign = sign;
    result.joint_zero_at_motor_rad = zero;
    result.joint_to_motor_ratio = ratio;
    return result;
}

void testAggregatorConvertsAndRequiresFreshCompleteState()
{
    using Clock = std::chrono::steady_clock;
    const auto time = Clock::time_point(std::chrono::seconds(10));
    raven_control::telemetry::JointStateAggregator aggregator({
        makeJoint("joint_a", 1),
        makeJoint("joint_b", 2, -1, 1.0, 2.0),
    });

    raven_control::hal::Rs02OperationFeedback first;
    first.motor_id = 1;
    first.motor_position_rad = 0.5;
    check(aggregator.ingest(first, time), "known motor must ingest");
    check(
        !aggregator.snapshot(time, std::chrono::milliseconds(100)),
        "snapshot must wait for every configured joint");

    raven_control::hal::Rs02OperationFeedback second;
    second.motor_id = 2;
    second.motor_position_rad = 3.0;
    second.motor_velocity_rad_s = 4.0;
    second.motor_torque_nm = 5.0;
    second.temperature_celsius = 30.0;
    check(aggregator.ingest(second, time), "second motor must ingest");

    const auto snapshot = aggregator.snapshot(
        time + std::chrono::milliseconds(50),
        std::chrono::milliseconds(100));
    check(snapshot.has_value(), "complete fresh state must snapshot");
    if (snapshot) {
        check(snapshot->joints.size() == 2, "snapshot must contain two joints");
        const auto& joint = snapshot->joints[1];
        check(std::abs(joint.position_rad + 1.0) < 1e-12,
              "position calibration must be applied once");
        check(std::abs(joint.velocity_rad_s + 2.0) < 1e-12,
              "velocity calibration must be applied once");
        check(std::abs(joint.effort_nm + 10.0) < 1e-12,
              "effort must preserve virtual work");
    }

    check(
        !aggregator.snapshot(
            time + std::chrono::milliseconds(101),
            std::chrono::milliseconds(100)),
        "stale state must not be published");

    raven_control::hal::Rs02OperationFeedback unknown;
    unknown.motor_id = 99;
    check(!aggregator.ingest(unknown, time), "unknown motor must be ignored");
}

}  // namespace

int main()
{
    testType2DecoderRejectsUnrelatedFrames();
    testType2DecoderExtractsMetadata();
    testAggregatorConvertsAndRequiresFreshCompleteState();

    if (failures != 0) {
        std::cerr << failures << " joint telemetry test(s) failed\n";
        return 1;
    }
    std::cout << "All joint telemetry tests passed\n";
    return 0;
}
