#include "raven_control/hal/motor_driver.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr std::uint8_t HOST_ID = 0xFD;
constexpr std::uint8_t COMM_OPERATION_CONTROL = 1;
constexpr std::uint8_t COMM_ENABLE = 3;
constexpr std::uint8_t COMM_STOP = 4;
constexpr std::uint8_t COMM_READ_PARAMETER = 17;
constexpr std::uint16_t PARAM_MECHANICAL_POSITION = 0x7019;
constexpr double PI = 3.14159265358979323846;

int failures = 0;

void check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

std::uint8_t communicationType(
    const raven_control::hal::CanFrame& frame)
{
    return static_cast<std::uint8_t>((frame.id >> 24) & 0x1F);
}

class FakeCanTransport final
    : public raven_control::hal::CanTransport {
public:
    bool send(const raven_control::hal::CanFrame& frame) override
    {
        sent.push_back(frame);
        return send_success;
    }

    raven_control::hal::CanReceiveResult receive(
        raven_control::hal::CanFrame& frame,
        std::chrono::milliseconds) override
    {
        if (incoming.empty())
            return raven_control::hal::CanReceiveResult::Timeout;
        frame = incoming.front();
        incoming.pop_front();
        return raven_control::hal::CanReceiveResult::Received;
    }

    bool send_success = true;
    std::vector<raven_control::hal::CanFrame> sent;
    std::deque<raven_control::hal::CanFrame> incoming;
};

std::vector<raven_control::hal::JointMotorConfig> motorMap()
{
    return {
        {"joint_1", 1},
        {"joint_2", 2},
        {"joint_3", 3},
    };
}

raven_control::safety::JointLimiterMap limiters(bool confirmed)
{
    raven_control::safety::JointLimiterMap result;
    for (const auto& motor : motorMap()) {
        raven_control::safety::JointLimitConfig config;
        config.joint_name = motor.joint_name;
        config.confirmed = confirmed;
        config.hard_min_rad = -1.0;
        config.hard_max_rad = 1.0;
        config.soft_margin_rad = 0.1;
        result.emplace(
            config.joint_name,
            raven_control::safety::JointLimiter(config));
    }
    return result;
}

std::uint16_t decodeUnsignedU16(
    const raven_control::hal::CanFrame& frame,
    std::size_t offset)
{
    return
        (std::uint16_t(frame.data[offset]) << 8) |
        std::uint16_t(frame.data[offset + 1]);
}

raven_control::hal::CanFrame positionFeedback(
    std::uint8_t motor_id,
    float position_rad)
{
    raven_control::hal::CanFrame frame;
    frame.id =
        (std::uint32_t(COMM_READ_PARAMETER) << 24) |
        (std::uint32_t(motor_id) << 8) |
        HOST_ID;
    frame.dlc = 8;
    frame.data[0] =
        static_cast<std::uint8_t>(
            PARAM_MECHANICAL_POSITION & 0xFF);
    frame.data[1] =
        static_cast<std::uint8_t>(
            PARAM_MECHANICAL_POSITION >> 8);
    std::memcpy(
        frame.data.data() + 4,
        &position_rad,
        sizeof(position_rad));
    return frame;
}

void queueHealthyFeedback(FakeCanTransport& transport)
{
    transport.incoming.push_back(positionFeedback(1, 0.0F));
    transport.incoming.push_back(positionFeedback(2, 0.1F));
    transport.incoming.push_back(positionFeedback(3, -0.1F));
}

double decodePosition(
    const raven_control::hal::CanFrame& frame)
{
    const std::uint16_t encoded =
        (std::uint16_t(frame.data[0]) << 8) |
        std::uint16_t(frame.data[1]);
    return ((static_cast<double>(encoded) / 65535.0) * 2.0 - 1.0) *
           (4.0 * PI);
}

double decodeVelocity(
    const raven_control::hal::CanFrame& frame)
{
    const std::uint16_t encoded = decodeUnsignedU16(frame, 2);
    return ((static_cast<double>(encoded) / 65535.0) * 2.0 - 1.0) *
           raven_control::hal::RS02_OPERATION_MAX_VELOCITY_RAD_S;
}

double decodeTorque(
    const raven_control::hal::CanFrame& frame)
{
    const std::uint16_t encoded =
        static_cast<std::uint16_t>((frame.id >> 8) & 0xFFFF);
    return ((static_cast<double>(encoded) / 65535.0) * 2.0 - 1.0) *
           raven_control::hal::RS02_OPERATION_MAX_TORQUE_NM;
}

std::size_t countFrames(
    const FakeCanTransport& transport,
    std::uint8_t type)
{
    std::size_t count = 0;
    for (const auto& frame : transport.sent) {
        if (communicationType(frame) == type)
            ++count;
    }
    return count;
}

void testEnableRequiresConfirmedLimits()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(false));
    queueHealthyFeedback(transport);
    driver.poll();

    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::UnconfirmedLimits,
        "enable must be rejected while limits are unconfirmed");
    check(countFrames(transport, COMM_ENABLE) == 0,
          "unconfirmed limits must not emit enable frames");
}

void testTargetIsClampedBeforeCanWrite()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(true));
    queueHealthyFeedback(transport);
    driver.poll();
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "healthy, confirmed joints must enable");

    transport.sent.clear();
    const auto result =
        driver.sendMitCommand(
            "joint_1", 2.0, 1.0, 40.0, 5.0, 0.0);

    check(
        result ==
            raven_control::hal::MotorCommandResult::TargetClamped,
        "out-of-soft-range command must report clamping");
    check(transport.sent.size() == 1,
          "clamped target must emit exactly one operation frame");
    if (transport.sent.size() == 1) {
        check(
            communicationType(transport.sent.front()) ==
                COMM_OPERATION_CONTROL,
            "clamped target must use operation-control frame");
        check(
            std::abs(
                decodePosition(transport.sent.front()) - 0.9) <
                0.001,
            "CAN frame must contain the clamped soft maximum");
    }
}

void testOfficialGainEncoding()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(true));
    queueHealthyFeedback(transport);
    driver.poll();
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "healthy joints must enable before gain encoding test");
    transport.sent.clear();

    check(
        driver.sendMitCommand(
            "joint_1",
            0.0,
            0.0,
            50.0,
            0.5,
            8.5) ==
            raven_control::hal::MotorCommandResult::Sent,
        "valid RS02 gains must be sent");
    check(transport.sent.size() == 1,
          "gain test must emit exactly one frame");
    if (transport.sent.size() == 1) {
        check(
            decodeUnsignedU16(transport.sent.front(), 4) == 6553,
            "Kp must be encoded against the official 0..500 range");
        check(
            decodeUnsignedU16(transport.sent.front(), 6) == 6553,
            "Kd must be encoded against the official 0..5 range");
        check(
            std::abs(decodeTorque(transport.sent.front()) - 8.5) <
                0.001,
            "feedforward torque must use the official -17..17 N.m range");
    }
}

void testPositionCalibration()
{
    FakeCanTransport transport;
    auto map = motorMap();
    map[0].position_sign = -1;
    map[0].joint_zero_at_motor_rad = 0.5;
    map[0].joint_to_motor_ratio = 2.0;

    raven_control::hal::MotorDriver driver(
        transport,
        map,
        limiters(true));
    transport.incoming.push_back(positionFeedback(1, 0.3F));
    transport.incoming.push_back(positionFeedback(2, 0.1F));
    transport.incoming.push_back(positionFeedback(3, -0.1F));
    driver.poll();

    const auto feedback = driver.feedback("joint_1");
    check(feedback && std::abs(feedback->position_rad - 0.1) < 1e-6,
          "motor feedback must be converted into joint coordinates");
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "calibrated joints must enable with valid joint feedback");
    transport.sent.clear();

    check(
        driver.sendMitCommand(
            "joint_1",
            0.2,
            0.25,
            8.0,
            0.15,
            1.2) ==
            raven_control::hal::MotorCommandResult::Sent,
        "calibrated joint target must be accepted");
    check(transport.sent.size() == 1,
          "calibrated target must emit exactly one frame");
    if (transport.sent.size() == 1) {
        check(
            std::abs(
                decodePosition(transport.sent.front()) - 0.1) <
                0.001,
            "joint target must be converted into motor coordinates");
        check(
            std::abs(
                decodeVelocity(transport.sent.front()) + 0.5) <
                0.002,
            "joint velocity must be converted into motor coordinates");
        check(
            std::abs(
                decodeTorque(transport.sent.front()) + 0.6) <
                0.001,
            "joint torque must preserve virtual work through calibration");
    }
}

void testClampedTargetStopsOutwardDesiredVelocity()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(true));
    queueHealthyFeedback(transport);
    driver.poll();
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "healthy joints must enable before velocity clamp test");
    transport.sent.clear();

    check(
        driver.sendMitCommand(
            "joint_1",
            2.0,
            3.0,
            8.0,
            0.15,
            0.0) ==
            raven_control::hal::MotorCommandResult::TargetClamped,
        "out-of-range target must report clamping");
    check(transport.sent.size() == 1,
          "velocity clamp test must emit one MIT frame");
    if (transport.sent.size() == 1) {
        check(
            std::abs(decodeVelocity(transport.sent.front())) < 0.002,
            "clamped target must send zero desired velocity");
    }
}

void testOutOfRangeVelocityLatchesAndStopsAll()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(true));
    queueHealthyFeedback(transport);
    driver.poll();
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "healthy joints must enable before velocity range test");
    transport.sent.clear();

    check(
        driver.sendMitCommand(
            "joint_1",
            0.0,
            raven_control::hal::RS02_OPERATION_MAX_VELOCITY_RAD_S +
                0.1,
            8.0,
            0.15,
            0.0) ==
            raven_control::hal::MotorCommandResult::InvalidCommand,
        "velocity outside the RS02 range must be rejected");
    check(driver.faultLatched(),
          "out-of-range velocity must latch a fault");
    check(countFrames(transport, COMM_STOP) == 3,
          "out-of-range velocity must stop every configured motor");
}

void testOutOfRangeGainLatchesAndStopsAll()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(true));
    queueHealthyFeedback(transport);
    driver.poll();
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "healthy joints must enable before gain range test");
    transport.sent.clear();

    check(
        driver.sendMitCommand(
            "joint_1",
            0.0,
            0.0,
            500.1,
            0.15,
            0.0) ==
            raven_control::hal::MotorCommandResult::InvalidCommand,
        "gain outside the RS02 range must be rejected");
    check(driver.faultLatched(),
          "out-of-range gain must latch a fault");
    check(countFrames(transport, COMM_STOP) == 3,
          "out-of-range gain must stop every configured motor");
}

void testHardViolationLatchesAndStopsAll()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(true));
    queueHealthyFeedback(transport);
    driver.poll();
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "healthy joints must enable before hard-limit test");
    transport.sent.clear();

    transport.incoming.push_back(positionFeedback(2, 1.1F));
    driver.poll();

    check(driver.faultLatched(),
          "hard-limit feedback must latch a fault");
    check(!driver.isEnabled(),
          "hard-limit fault must disable the driver");
    check(countFrames(transport, COMM_STOP) == 3,
          "hard-limit fault must stop every configured motor");
    check(
        driver.sendMitCommand(
            "joint_1", 0.0, 0.0, 40.0, 5.0, 0.0) ==
            raven_control::hal::MotorCommandResult::FaultLatched,
        "latched fault must reject subsequent commands");
}

void testInvalidTargetLatchesAndStopsAll()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(true));
    queueHealthyFeedback(transport);
    driver.poll();
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "healthy joints must enable before invalid-command test");
    transport.sent.clear();

    const auto result = driver.sendMitCommand(
        "joint_1",
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        40.0,
        5.0,
        0.0);

    check(
        result ==
            raven_control::hal::MotorCommandResult::InvalidCommand,
        "non-finite command must be rejected");
    check(driver.faultLatched(),
          "non-finite command must latch a fault");
    check(countFrames(transport, COMM_STOP) == 3,
          "non-finite command must stop every configured motor");
}

void testOutOfRangeTorqueLatchesAndStopsAll()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(true));
    queueHealthyFeedback(transport);
    driver.poll();
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "healthy joints must enable before torque range test");
    transport.sent.clear();

    check(
        driver.sendMitCommand(
            "joint_1",
            0.0,
            0.0,
            8.0,
            0.15,
            raven_control::hal::RS02_OPERATION_MAX_TORQUE_NM +
                0.1) ==
            raven_control::hal::MotorCommandResult::InvalidCommand,
        "feedforward torque outside the RS02 range must be rejected");
    check(driver.faultLatched(),
          "out-of-range torque must latch a fault");
    check(countFrames(transport, COMM_STOP) == 3,
          "out-of-range torque must stop every configured motor");
}

}  // namespace

int main()
{
    testEnableRequiresConfirmedLimits();
    testTargetIsClampedBeforeCanWrite();
    testOfficialGainEncoding();
    testPositionCalibration();
    testClampedTargetStopsOutwardDesiredVelocity();
    testOutOfRangeVelocityLatchesAndStopsAll();
    testOutOfRangeGainLatchesAndStopsAll();
    testHardViolationLatchesAndStopsAll();
    testInvalidTargetLatchesAndStopsAll();
    testOutOfRangeTorqueLatchesAndStopsAll();

    if (failures != 0) {
        std::cerr << failures << " motor driver safety test(s) failed\n";
        return 1;
    }
    std::cout << "All motor driver safety tests passed\n";
    return 0;
}
