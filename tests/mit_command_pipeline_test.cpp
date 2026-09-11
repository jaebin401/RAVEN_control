#include "raven_control/control/mit_command_pipeline.hpp"

#include "raven_control/hal/rs02_operation_feedback.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using JointVector = raven_control::dynamics::JointVector;

constexpr std::uint8_t HOST_ID = 0xFD;
constexpr std::uint8_t COMM_OPERATION_FEEDBACK = 2;
constexpr std::uint8_t COMM_OPERATION_CONTROL = 1;
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

class FakeCanTransport final : public raven_control::hal::CanTransport {
public:
    bool send(const raven_control::hal::CanFrame& frame) override
    {
        sent.push_back(frame);
        return true;
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

    std::vector<raven_control::hal::CanFrame> sent;
    std::deque<raven_control::hal::CanFrame> incoming;
};

std::uint16_t encodeSymmetric(double value, double limit)
{
    return static_cast<std::uint16_t>(
        ((value / limit) + 1.0) * 32767.5);
}

void packU16(
    raven_control::hal::CanFrame& frame,
    std::size_t offset,
    std::uint16_t value)
{
    frame.data[offset] = static_cast<std::uint8_t>(value >> 8);
    frame.data[offset + 1] = static_cast<std::uint8_t>(value & 0xFF);
}

raven_control::hal::CanFrame operationFeedback(
    std::uint8_t motor_id,
    double position_rad)
{
    raven_control::hal::CanFrame frame;
    frame.id =
        (std::uint32_t(COMM_OPERATION_FEEDBACK) << 24) |
        (std::uint32_t(2) << 22) |
        (std::uint32_t(motor_id) << 8) |
        HOST_ID;
    frame.dlc = 8;
    packU16(frame, 0, encodeSymmetric(position_rad, 4.0 * PI));
    packU16(
        frame,
        2,
        encodeSymmetric(
            0.0,
            raven_control::hal::RS02_OPERATION_MAX_VELOCITY_RAD_S));
    packU16(
        frame,
        4,
        encodeSymmetric(
            0.0,
            raven_control::hal::RS02_OPERATION_MAX_TORQUE_NM));
    packU16(frame, 6, 250);
    return frame;
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
    frame.data[0] = static_cast<std::uint8_t>(
        PARAM_MECHANICAL_POSITION & 0xFF);
    frame.data[1] = static_cast<std::uint8_t>(
        PARAM_MECHANICAL_POSITION >> 8);
    std::memcpy(frame.data.data() + 4, &position_rad, sizeof(position_rad));
    return frame;
}

class FixedGravityModel final
    : public raven_control::dynamics::GravityModel {
public:
    JointVector compute(const JointVector& positions) const override
    {
        last_positions = positions;
        return {0.0, 1.0, -0.5};
    }

    mutable JointVector last_positions{};
};

raven_control::config::MotorRuntimeConfig runtimeConfig()
{
    raven_control::config::MotorRuntimeConfig config;
    config.feedback_timeout = std::chrono::milliseconds(250);
    config.gravity_compensation.enabled = true;
    config.gravity_compensation.dry_run = false;
    config.gravity_compensation.scale = 1.0;
    config.gravity_compensation.ramp_duration =
        std::chrono::milliseconds(100);
    config.gravity_compensation.max_joint_torque_nm = {1.0, 2.0, 2.0};
    config.joints = {
        {"shoulder_Joint", 1, 1, 0.0, 1.0, {10.0, 1.0, 1.0}},
        {"upperArm_Joint", 2, 1, 0.0, 1.0, {10.0, 1.0, 1.0}},
        {"foreArm_Joint", 3, 1, 0.0, 1.0, {10.0, 1.0, 1.0}},
    };
    return config;
}

std::vector<raven_control::hal::JointMotorConfig> motorMap()
{
    return {
        {"shoulder_Joint", 1},
        {"upperArm_Joint", 2},
        {"foreArm_Joint", 3},
    };
}

raven_control::safety::JointLimiterMap limiters()
{
    raven_control::safety::JointLimiterMap result;
    for (const auto& joint : motorMap()) {
        raven_control::safety::JointLimitConfig config;
        config.joint_name = joint.joint_name;
        config.confirmed = true;
        config.hard_min_rad = -2.0;
        config.hard_max_rad = 2.0;
        config.soft_margin_rad = 0.1;
        result.emplace(
            config.joint_name,
            raven_control::safety::JointLimiter(config));
    }
    return result;
}

std::uint8_t communicationType(const raven_control::hal::CanFrame& frame)
{
    return static_cast<std::uint8_t>((frame.id >> 24) & 0x1F);
}

void testAddsGravityToEveryJointAndDispatchesOnce()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport, motorMap(), limiters());
    for (std::uint8_t id = 1; id <= 3; ++id)
        transport.incoming.push_back(positionFeedback(id, 0.1F * id));
    driver.poll();
    check(
        driver.enableAll() == raven_control::hal::MotorCommandResult::Sent,
        "pipeline fixture must enable");
    for (std::uint8_t id = 1; id <= 3; ++id)
        transport.incoming.push_back(operationFeedback(id, 0.1 * id));
    driver.poll();

    auto model = std::make_unique<FixedGravityModel>();
    auto* model_ptr = model.get();
    const auto config = runtimeConfig();
    raven_control::control::MitCommandPipeline pipeline(
        driver,
        std::move(model),
        config,
        {"shoulder_Joint", "upperArm_Joint", "foreArm_Joint"});
    const auto start = Clock::now();
    pipeline.reset(start);

    raven_control::control::MitCommandPipeline::Commands commands{};
    for (auto& command : commands) {
        command.target_position_rad = 0.0;
        command.kp = 10.0;
        command.kd = 1.0;
    }
    commands[0].application_feedforward_torque_nm = 0.25;

    transport.sent.clear();
    const auto result = pipeline.send(
        commands,
        start + std::chrono::milliseconds(100));
    check(result.all_sent, "healthy pipeline cycle must send all joints");
    check(!result.feedback_hold, "healthy cycle must not enter hold");
    check(std::abs(result.final_feedforward_torque_nm[0] - 0.25) < 1e-12,
          "application torque must be preserved on J0");
    check(std::abs(result.final_feedforward_torque_nm[1] - 1.0) < 1e-12,
          "gravity torque must be added on J1");
    check(std::abs(result.final_feedforward_torque_nm[2] + 0.5) < 1e-12,
          "gravity torque must be added on J2");
    check(std::abs(model_ptr->last_positions[1] - 0.2) < 0.001,
          "gravity model must receive live joint feedback");

    std::size_t operation_frames = 0;
    for (const auto& frame : transport.sent)
        operation_frames += communicationType(frame) == COMM_OPERATION_CONTROL;
    check(operation_frames == 3,
          "one pipeline cycle must emit one MIT frame per joint");
}

void testStaleFeedbackEntersExistingHalHold()
{
    FakeCanTransport transport;
    raven_control::hal::MotorDriver driver(
        transport,
        motorMap(),
        limiters(),
        HOST_ID,
        std::chrono::milliseconds(20));
    for (std::uint8_t id = 1; id <= 3; ++id)
        transport.incoming.push_back(positionFeedback(id, 0.0F));
    driver.poll();
    check(
        driver.enableAll() == raven_control::hal::MotorCommandResult::Sent,
        "stale fixture must enable");
    for (std::uint8_t id = 1; id <= 3; ++id)
        transport.incoming.push_back(operationFeedback(id, 0.0));
    driver.poll();

    auto config = runtimeConfig();
    config.feedback_timeout = std::chrono::milliseconds(20);
    config.gravity_compensation.enabled = false;
    raven_control::control::MitCommandPipeline pipeline(
        driver,
        std::make_unique<FixedGravityModel>(),
        config,
        {"shoulder_Joint", "upperArm_Joint", "foreArm_Joint"});
    raven_control::control::MitCommandPipeline::Commands commands{};
    for (auto& command : commands) {
        command.kp = 10.0;
        command.kd = 1.0;
    }

    const auto initial = pipeline.send(commands, Clock::now());
    check(initial.all_sent,
          "stale fixture must establish a complete safe command snapshot");
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const auto result = pipeline.send(commands, Clock::now());
    check(result.feedback_hold,
          "stale feedback must be delegated to HAL Feedback Hold");
}

}  // namespace

int main()
{
    testAddsGravityToEveryJointAndDispatchesOnce();
    testStaleFeedbackEntersExistingHalHold();
    if (failures != 0) {
        std::cerr << failures << " MIT pipeline test(s) failed\n";
        return 1;
    }
    std::cout << "All MIT command pipeline tests passed\n";
    return 0;
}
