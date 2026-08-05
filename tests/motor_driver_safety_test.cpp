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
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr std::uint8_t HOST_ID = 0xFD;
constexpr std::uint8_t COMM_OPERATION_CONTROL = 1;
constexpr std::uint8_t COMM_OPERATION_FEEDBACK = 2;
constexpr std::uint8_t COMM_ENABLE = 3;
constexpr std::uint8_t COMM_STOP = 4;
constexpr std::uint8_t COMM_READ_PARAMETER = 17;
constexpr std::uint16_t PARAM_MECHANICAL_POSITION = 0x7019;
constexpr std::uint16_t PARAM_BUS_VOLTAGE = 0x701C;
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

raven_control::hal::CanFrame busVoltageFeedback(
    std::uint8_t motor_id,
    float voltage_v)
{
    raven_control::hal::CanFrame frame;
    frame.id =
        (std::uint32_t(COMM_READ_PARAMETER) << 24) |
        (std::uint32_t(motor_id) << 8) |
        HOST_ID;
    frame.dlc = 8;
    frame.data[0] = static_cast<std::uint8_t>(
        PARAM_BUS_VOLTAGE & 0xFF);
    frame.data[1] = static_cast<std::uint8_t>(
        PARAM_BUS_VOLTAGE >> 8);
    std::memcpy(
        frame.data.data() + 4,
        &voltage_v,
        sizeof(voltage_v));
    return frame;
}

std::uint16_t encodeSymmetric(double value, double absolute_limit)
{
    return static_cast<std::uint16_t>(
        ((value / absolute_limit) + 1.0) * 32767.5);
}

void packU16BigEndian(
    raven_control::hal::CanFrame& frame,
    std::size_t offset,
    std::uint16_t value)
{
    frame.data[offset] = static_cast<std::uint8_t>(value >> 8);
    frame.data[offset + 1] =
        static_cast<std::uint8_t>(value & 0xFF);
}

raven_control::hal::CanFrame operationFeedback(
    std::uint8_t motor_id,
    double position_rad,
    double velocity_rad_s,
    double torque_nm,
    double temperature_celsius,
    std::uint8_t fault_flags = 0,
    std::uint8_t mode_state = 2)
{
    raven_control::hal::CanFrame frame;
    frame.id =
        (std::uint32_t(COMM_OPERATION_FEEDBACK) << 24) |
        (std::uint32_t(mode_state & 0x03) << 22) |
        (std::uint32_t(fault_flags & 0x3F) << 16) |
        (std::uint32_t(motor_id) << 8) |
        HOST_ID;
    frame.dlc = 8;
    packU16BigEndian(
        frame,
        0,
        encodeSymmetric(position_rad, 4.0 * PI));
    packU16BigEndian(
        frame,
        2,
        encodeSymmetric(
            velocity_rad_s,
            raven_control::hal::RS02_OPERATION_MAX_VELOCITY_RAD_S));
    packU16BigEndian(
        frame,
        4,
        encodeSymmetric(
            torque_nm,
            raven_control::hal::RS02_OPERATION_MAX_TORQUE_NM));
    packU16BigEndian(
        frame,
        6,
        static_cast<std::uint16_t>(temperature_celsius * 10.0));
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

void testType2OperationFeedbackIsDecodedInJointCoordinates()
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
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "Type 17 position must bootstrap enable before Type 2 feedback");

    transport.incoming.push_back(operationFeedback(
        1,
        0.3,
        -1.25,
        2.5,
        35.6,
        0,
        2));
    driver.poll();

    const auto feedback = driver.feedback("joint_1");
    check(feedback && feedback->valid,
          "Type 2 feedback must provide a valid position");
    check(feedback && feedback->operation_feedback_valid,
          "Type 2 feedback must mark the full operation state valid");
    if (feedback) {
        check(
            feedback->source ==
                raven_control::hal::MotorFeedbackSource::Operation,
            "Type 2 feedback must record its source");
        check(std::abs(feedback->position_rad - 0.1) < 0.001,
              "Type 2 position must use joint calibration");
        check(std::abs(feedback->velocity_rad_s - 0.625) < 0.002,
              "Type 2 velocity must use joint calibration");
        check(std::abs(feedback->torque_nm + 5.0) < 0.003,
              "Type 2 torque must preserve virtual work");
        check(std::abs(feedback->temperature_celsius - 35.6) < 1e-9,
              "Type 2 temperature must decode tenths of a degree");
        check(feedback->fault_flags == 0,
              "healthy Type 2 feedback must have no fault flags");
        check(feedback->mode_state == 2,
              "Type 2 mode state must be decoded from the CAN ID");
    }

    transport.sent.clear();
    check(driver.requestMechanicalPositions(),
          "Type 17 request API must remain successful after enable");
    check(countFrames(transport, COMM_READ_PARAMETER) == 0,
          "enabled operation must rely on Type 2 instead of polling Type 17");
}

void testType2FaultLatchesAndStopsAll()
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
        "healthy Type 17 feedback must bootstrap the Type 2 fault test");
    transport.sent.clear();

    transport.incoming.push_back(operationFeedback(
        2,
        0.0,
        0.0,
        0.0,
        30.0,
        1));
    driver.poll();

    check(driver.faultLatched(),
          "nonzero Type 2 motor fault flags must latch a fault");
    check(!driver.isEnabled(),
          "Type 2 motor fault must disable the driver");
    check(countFrames(transport, COMM_STOP) == 3,
          "Type 2 motor fault must stop every configured motor");
}

void testType2HardLimitLatchesAndStopsAll()
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
        "healthy feedback must bootstrap the Type 2 limit test");
    transport.sent.clear();

    transport.incoming.push_back(operationFeedback(
        2,
        1.1,
        0.0,
        0.0,
        30.0));
    driver.poll();

    check(driver.faultLatched(),
          "Type 2 hard-limit position must latch a fault");
    check(!driver.isEnabled(),
          "Type 2 hard-limit fault must disable the driver");
    check(countFrames(transport, COMM_STOP) == 3,
          "Type 2 hard-limit fault must stop every configured motor");
    check(
        driver.faultReason().find("source=Type2") !=
            std::string::npos,
        "Type 2 hard-limit reason must identify its feedback source");
    check(
        driver.faultReason().find("limits=[-1.000000, 1.000000]") !=
            std::string::npos,
        "hard-limit reason must include the configured bounds");
}

void testStopRequiresFreshType17BeforeReenable()
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
        "healthy Type 17 feedback must allow the initial enable");

    transport.incoming.push_back(operationFeedback(
        1,
        0.2,
        0.0,
        0.0,
        30.0));
    driver.poll();
    check(driver.stopAll(),
          "explicit stop must succeed before re-enable test");
    check(!driver.allFeedbackValid(),
          "stop must invalidate feedback from the previous mode");

    transport.incoming.push_back(operationFeedback(
        1,
        0.9,
        0.0,
        0.0,
        30.0));
    driver.poll();
    check(!driver.allFeedbackValid(),
          "delayed Type 2 feedback must be ignored while stopped");
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::InvalidFeedback,
        "re-enable must wait for fresh Type 17 feedback");

    queueHealthyFeedback(transport);
    driver.poll();
    check(driver.allFeedbackValid(),
          "fresh Type 17 feedback must restore bootstrap validity");
    for (const auto& motor : motorMap()) {
        const auto feedback = driver.feedback(motor.joint_name);
        check(
            feedback &&
                feedback->source == raven_control::hal::
                                        MotorFeedbackSource::
                                            MechanicalPosition,
            "stopped feedback must come from Type 17");
    }
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "fresh Type 17 feedback must allow re-enable");
}

void testEnabledDriverIgnoresDelayedType17()
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
        "healthy Type 17 feedback must allow enable");

    transport.incoming.push_back(operationFeedback(
        1,
        0.2,
        0.0,
        0.0,
        30.0));
    driver.poll();
    transport.incoming.push_back(positionFeedback(1, 1.1F));
    driver.poll();

    check(!driver.faultLatched(),
          "delayed Type 17 feedback must not fault an enabled driver");
    const auto feedback = driver.feedback("joint_1");
    check(
        feedback &&
            feedback->source ==
                raven_control::hal::MotorFeedbackSource::Operation,
        "enabled feedback source must remain Type 2");
    check(
        feedback && std::abs(feedback->position_rad - 0.2) < 0.001,
        "delayed Type 17 feedback must not overwrite Type 2 position");
}

void testBusVoltageCanBeReadWhileEnabled()
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
        "healthy feedback must allow enable before VBUS test");
    transport.incoming.push_back(operationFeedback(
        1,
        0.2,
        0.0,
        0.0,
        30.0));
    driver.poll();

    transport.sent.clear();
    check(driver.requestBusVoltages(),
          "VBUS Type 17 requests must be sent while enabled");
    check(countFrames(transport, COMM_READ_PARAMETER) == 3,
          "VBUS request must emit one Type 17 frame per motor");
    for (const auto& frame : transport.sent) {
        check(
            frame.data[0] ==
                    static_cast<std::uint8_t>(
                        PARAM_BUS_VOLTAGE & 0xFF) &&
                frame.data[1] ==
                    static_cast<std::uint8_t>(
                        PARAM_BUS_VOLTAGE >> 8),
            "VBUS request must address parameter 0x701C");
    }

    transport.incoming.push_back(busVoltageFeedback(1, 31.25F));
    driver.poll();
    const auto feedback = driver.feedback("joint_1");
    check(feedback && feedback->bus_voltage_valid,
          "VBUS response must set a separate validity flag");
    check(
        feedback &&
            std::abs(feedback->bus_voltage_v - 31.25) < 1e-6,
        "VBUS response must decode the float voltage");
    check(
        feedback &&
            feedback->source ==
                raven_control::hal::MotorFeedbackSource::
                    Operation,
        "VBUS response must not replace the position source");
}

void testMissingType2FeedbackEntersPositionHold()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(true),
        HOST_ID,
        std::chrono::milliseconds(20));
    queueHealthyFeedback(transport);
    driver.poll();
    check(
        driver.enableAll() ==
            raven_control::hal::MotorCommandResult::Sent,
        "Type 17 feedback must allow initial enable");

    check(
        driver.sendMitCommand(
            "joint_1", 0.2, 0.4, 8.0, 0.15, 0.3) ==
            raven_control::hal::MotorCommandResult::Sent,
        "joint 1 must cache a safe MIT command");
    check(
        driver.sendMitCommand(
            "joint_2", -0.2, -0.4, 9.0, 0.2, -0.3) ==
            raven_control::hal::MotorCommandResult::Sent,
        "joint 2 must cache a safe MIT command");
    check(
        driver.sendMitCommand(
            "joint_3", 0.1, 0.2, 10.0, 0.25, 0.1) ==
            raven_control::hal::MotorCommandResult::Sent,
        "joint 3 must cache a safe MIT command");
    transport.sent.clear();

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    check(driver.requestMechanicalPositions(),
          "enabled Type 17 request must remain a no-op success");
    check(
        driver.sendMitCommand(
            "joint_1", 0.8, 2.0, 20.0, 1.0, 1.0) ==
            raven_control::hal::MotorCommandResult::FeedbackHold,
        "missing Type 2 feedback must enter position hold");
    check(driver.feedbackHoldLatched(),
          "feedback hold must remain latched");
    check(driver.isEnabled(),
          "feedback hold must keep the motors enabled");
    check(!driver.faultLatched(),
          "feedback hold alone must not latch a shutdown fault");
    check(countFrames(transport, COMM_STOP) == 0,
          "feedback hold must not send Type 4 stop frames");
    check(countFrames(transport, COMM_OPERATION_CONTROL) == 1,
          "feedback hold must continue sending Type 1 commands");
    check(
        std::abs(decodePosition(transport.sent.back()) - 0.2) < 0.001,
        "feedback hold must reuse the last safe position");
    check(
        std::abs(decodeVelocity(transport.sent.back())) < 0.002,
        "feedback hold must force desired velocity to zero");

    check(
        driver.sendMitCommand(
            "joint_2", 0.7, 3.0, 30.0, 2.0, 2.0) ==
            raven_control::hal::MotorCommandResult::FeedbackHold,
        "latched feedback hold must ignore later motion commands");
    check(
        std::abs(decodePosition(transport.sent.back()) + 0.2) < 0.001,
        "latched hold must preserve each joint's cached position");
    check(
        std::abs(decodeVelocity(transport.sent.back())) < 0.002,
        "every latched hold command must use zero velocity");

    check(driver.stopAll(),
          "explicit user stop must disable all motors from hold");
    check(!driver.isEnabled(),
          "explicit stop must leave the driver disabled");
    check(!driver.feedbackHoldLatched(),
          "explicit stop must clear feedback hold state");
    check(countFrames(transport, COMM_STOP) == 3,
          "explicit stop must send one Type 4 frame per motor");
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
        driver.faultReason().find("source=Type17") !=
            std::string::npos,
        "Type 17 hard-limit reason must identify its feedback source");
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
    testType2OperationFeedbackIsDecodedInJointCoordinates();
    testType2FaultLatchesAndStopsAll();
    testType2HardLimitLatchesAndStopsAll();
    testStopRequiresFreshType17BeforeReenable();
    testEnabledDriverIgnoresDelayedType17();
    testBusVoltageCanBeReadWhileEnabled();
    testMissingType2FeedbackEntersPositionHold();
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
