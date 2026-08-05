#include "raven_control/hal/motor_driver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace raven_control::hal {
namespace {

constexpr std::uint8_t COMM_OPERATION_CONTROL = 1;
constexpr std::uint8_t COMM_OPERATION_FEEDBACK = 2;
constexpr std::uint8_t COMM_ENABLE = 3;
constexpr std::uint8_t COMM_STOP = 4;
constexpr std::uint8_t COMM_READ_PARAMETER = 17;
constexpr std::uint16_t PARAM_MECHANICAL_POSITION = 0x7019;
constexpr std::uint16_t PARAM_BUS_VOLTAGE = 0x701C;

constexpr double PI = 3.14159265358979323846;
constexpr double POSITION_SCALE_RAD = 4.0 * PI;

std::uint32_t buildExtendedId(
    std::uint8_t communication_type,
    std::uint16_t data_field,
    std::uint8_t motor_id)
{
    return (std::uint32_t(communication_type & 0x1F) << 24) |
           (std::uint32_t(data_field) << 8) |
           std::uint32_t(motor_id);
}

std::uint8_t communicationType(const CanFrame& frame)
{
    return static_cast<std::uint8_t>((frame.id >> 24) & 0x1F);
}

std::uint8_t sourceMotorId(const CanFrame& frame)
{
    return static_cast<std::uint8_t>((frame.id >> 8) & 0xFF);
}

std::uint8_t destinationHostId(const CanFrame& frame)
{
    return static_cast<std::uint8_t>(frame.id & 0xFF);
}

void packU16LittleEndian(
    std::array<std::uint8_t, 8>& data,
    std::size_t offset,
    std::uint16_t value)
{
    data[offset] = static_cast<std::uint8_t>(value & 0xFF);
    data[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

void packU16BigEndian(
    std::array<std::uint8_t, 8>& data,
    std::size_t offset,
    std::uint16_t value)
{
    data[offset] = static_cast<std::uint8_t>(value >> 8);
    data[offset + 1] = static_cast<std::uint8_t>(value & 0xFF);
}

double unpackFloatLittleEndian(
    const std::array<std::uint8_t, 8>& data,
    std::size_t offset)
{
    float value = 0.0F;
    std::memcpy(&value, data.data() + offset, sizeof(value));
    return static_cast<double>(value);
}

std::uint16_t encodeSymmetric(
    double value,
    double absolute_limit)
{
    const double clamped =
        std::clamp(value, -absolute_limit, absolute_limit);
    const double normalized =
        ((clamped / absolute_limit) + 1.0) * 32767.5;
    return static_cast<std::uint16_t>(normalized);
}

std::uint16_t encodeUnsigned(double value, double maximum)
{
    const double clamped = std::clamp(value, 0.0, maximum);
    const double normalized = (clamped / maximum) * 65535.0;
    return static_cast<std::uint16_t>(normalized);
}

double motorToJointPosition(
    const JointMotorConfig& motor,
    double motor_position_rad)
{
    return static_cast<double>(motor.position_sign) *
           (motor_position_rad - motor.joint_zero_at_motor_rad) /
           motor.joint_to_motor_ratio;
}

double jointToMotorPosition(
    const JointMotorConfig& motor,
    double joint_position_rad)
{
    return motor.joint_zero_at_motor_rad +
           static_cast<double>(motor.position_sign) *
               motor.joint_to_motor_ratio *
               joint_position_rad;
}

double jointToMotorVelocity(
    const JointMotorConfig& motor,
    double joint_velocity_rad_s)
{
    return static_cast<double>(motor.position_sign) *
           motor.joint_to_motor_ratio *
           joint_velocity_rad_s;
}

double motorToJointVelocity(
    const JointMotorConfig& motor,
    double motor_velocity_rad_s)
{
    return static_cast<double>(motor.position_sign) *
           motor_velocity_rad_s / motor.joint_to_motor_ratio;
}

double jointToMotorTorque(
    const JointMotorConfig& motor,
    double joint_torque_nm)
{
    // Preserve virtual work for q_motor = sign * ratio * q_joint.
    return static_cast<double>(motor.position_sign) *
           joint_torque_nm / motor.joint_to_motor_ratio;
}

double motorToJointTorque(
    const JointMotorConfig& motor,
    double motor_torque_nm)
{
    return static_cast<double>(motor.position_sign) *
           motor.joint_to_motor_ratio * motor_torque_nm;
}

std::string hardLimitViolationReason(
    const std::string& joint_name,
    MotorFeedbackSource source,
    double position_rad,
    const safety::JointLimiter& limiter,
    std::optional<double> previous_position_rad = std::nullopt)
{
    std::ostringstream message;
    message
        << std::fixed << std::setprecision(6)
        << "Hard-limit violation on " << joint_name
        << " [source=" << toString(source)
        << ", q=" << position_rad
        << " rad, limits=["
        << limiter.config().hard_min_rad << ", "
        << limiter.config().hard_max_rad << "] rad";
    if (previous_position_rad) {
        message << ", previous_q=" << *previous_position_rad
                << " rad";
    }
    message << ']';
    return message.str();
}

}  // namespace

const char* toString(MotorFeedbackSource source) noexcept
{
    switch (source) {
    case MotorFeedbackSource::None:
        return "none";
    case MotorFeedbackSource::MechanicalPosition:
        return "Type17";
    case MotorFeedbackSource::Operation:
        return "Type2";
    }
    return "unknown";
}

const char* toString(MotorCommandResult result) noexcept
{
    switch (result) {
    case MotorCommandResult::Sent:
        return "sent";
    case MotorCommandResult::TargetClamped:
        return "target clamped";
    case MotorCommandResult::FeedbackHold:
        return "feedback-loss position hold";
    case MotorCommandResult::NotEnabled:
        return "motors are not enabled";
    case MotorCommandResult::UnknownJoint:
        return "unknown joint";
    case MotorCommandResult::InvalidCommand:
        return "invalid command";
    case MotorCommandResult::InvalidFeedback:
        return "invalid feedback";
    case MotorCommandResult::StaleFeedback:
        return "stale feedback";
    case MotorCommandResult::UnconfirmedLimits:
        return "joint limits are not confirmed";
    case MotorCommandResult::HardLimitViolation:
        return "hard-limit violation";
    case MotorCommandResult::FaultLatched:
        return "safety fault is latched";
    case MotorCommandResult::CanWriteFailure:
        return "CAN write failure";
    }
    return "unknown result";
}

MotorDriver::MotorDriver(
    CanTransport& transport,
    std::vector<JointMotorConfig> motor_map,
    safety::JointLimiterMap joint_limiters,
    std::uint8_t host_id,
    std::chrono::milliseconds feedback_timeout)
    : transport_(transport),
      host_id_(host_id),
      feedback_timeout_(feedback_timeout)
{
    if (motor_map.empty())
        throw std::invalid_argument("Motor map must not be empty");
    if (feedback_timeout_.count() <= 0) {
        throw std::invalid_argument(
            "Feedback timeout must be positive");
    }

    for (auto& motor : motor_map) {
        if (motor.joint_name.empty())
            throw std::invalid_argument("Motor map contains an empty joint name");
        if (motor.motor_id > 127) {
            throw std::invalid_argument(
                "Motor ID must be in range 0..127");
        }
        if (motor.position_sign != -1 &&
            motor.position_sign != 1) {
            throw std::invalid_argument(
                "Position sign must be -1 or 1 for '" +
                motor.joint_name + "'");
        }
        if (!std::isfinite(motor.joint_zero_at_motor_rad) ||
            !std::isfinite(motor.joint_to_motor_ratio) ||
            motor.joint_to_motor_ratio <= 0.0) {
            throw std::invalid_argument(
                "Invalid position calibration for '" +
                motor.joint_name + "'");
        }

        auto limiter = joint_limiters.find(motor.joint_name);
        if (limiter == joint_limiters.end()) {
            throw std::invalid_argument(
                "Missing joint limits for '" + motor.joint_name + "'");
        }
        if (channels_.find(motor.joint_name) != channels_.end()) {
            throw std::invalid_argument(
                "Duplicate joint in motor map: " + motor.joint_name);
        }
        if (motor_to_joint_.find(motor.motor_id) != motor_to_joint_.end()) {
            throw std::invalid_argument(
                "Duplicate motor ID in motor map: " +
                std::to_string(motor.motor_id));
        }
        const double motor_at_hard_min = jointToMotorPosition(
            motor,
            limiter->second.config().hard_min_rad);
        const double motor_at_hard_max = jointToMotorPosition(
            motor,
            limiter->second.config().hard_max_rad);
        if (std::abs(motor_at_hard_min) > POSITION_SCALE_RAD ||
            std::abs(motor_at_hard_max) > POSITION_SCALE_RAD) {
            throw std::invalid_argument(
                "Joint limits and calibration exceed the RS02 "
                "operation-control position range for '" +
                motor.joint_name + "'");
        }

        const std::string joint_name = motor.joint_name;
        const std::uint8_t motor_id = motor.motor_id;
        channels_.emplace(
            joint_name,
            Channel{
                std::move(motor),
                std::move(limiter->second),
                MotorFeedback{},
                LastMitCommand{}});
        motor_to_joint_.emplace(motor_id, joint_name);
    }
}

MotorCommandResult MotorDriver::enableAll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fault_latched_)
        return MotorCommandResult::FaultLatched;

    const auto now = std::chrono::steady_clock::now();
    for (const auto& entry : channels_) {
        const Channel& channel = entry.second;
        if (!channel.limiter.isConfirmed())
            return MotorCommandResult::UnconfirmedLimits;
        if (!channel.feedback.valid)
            return MotorCommandResult::InvalidFeedback;
        if (channel.feedback.source !=
            MotorFeedbackSource::MechanicalPosition) {
            return MotorCommandResult::InvalidFeedback;
        }
        if (!feedbackFresh(channel.feedback, now))
            return MotorCommandResult::StaleFeedback;
        if (channel.limiter.isHardViolation(
                channel.feedback.position_rad)) {
            latchFaultUnlocked(
                hardLimitViolationReason(
                    channel.motor.joint_name,
                    channel.feedback.source,
                    channel.feedback.position_rad,
                    channel.limiter));
            return MotorCommandResult::HardLimitViolation;
        }
    }

    feedback_hold_latched_ = false;
    feedback_hold_reason_.clear();
    for (auto& entry : channels_) {
        entry.second.feedback.operation_feedback_valid = false;
        entry.second.last_mit_command = LastMitCommand{};
    }

    for (const auto& entry : channels_) {
        if (!sendEnable(entry.second.motor.motor_id)) {
            latchFaultUnlocked(
                "CAN enable failed on " +
                entry.second.motor.joint_name);
            return MotorCommandResult::CanWriteFailure;
        }
    }
    enabled_ = true;
    return MotorCommandResult::Sent;
}

bool MotorDriver::stopAll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return stopAllUnlocked();
}

bool MotorDriver::requestMechanicalPositions()
{
    std::lock_guard<std::mutex> lock(mutex_);
    // Type 17 is the startup/bootstrap position source. Once enabled,
    // Type 2 replies to Type 1 commands become the primary feedback.
    if (enabled_)
        return true;

    bool success = true;
    for (const auto& entry : channels_) {
        success =
            sendReadMechanicalPosition(entry.second.motor.motor_id) &&
            success;
    }
    return success;
}

bool MotorDriver::requestBusVoltages()
{
    std::lock_guard<std::mutex> lock(mutex_);
    bool success = true;
    for (const auto& entry : channels_) {
        success = sendReadBusVoltage(entry.second.motor.motor_id) &&
                  success;
    }
    return success;
}

std::size_t MotorDriver::poll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t processed = 0;

    while (true) {
        CanFrame frame;
        const CanReceiveResult result =
            transport_.receive(frame, std::chrono::milliseconds(0));
        if (result == CanReceiveResult::Error) {
            if (!fault_latched_)
                latchFaultUnlocked("CAN receive failure");
            break;
        }
        if (result == CanReceiveResult::Timeout)
            break;
        processFrameUnlocked(frame);
        ++processed;
    }
    return processed;
}

MotorCommandResult MotorDriver::sendMitCommand(
    const std::string& joint_name,
    double target_position_rad,
    double target_velocity_rad_s,
    double kp,
    double kd,
    double feedforward_torque_nm)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (fault_latched_)
        return MotorCommandResult::FaultLatched;
    if (!enabled_)
        return MotorCommandResult::NotEnabled;

    auto channel_entry = channels_.find(joint_name);
    if (channel_entry == channels_.end())
        return MotorCommandResult::UnknownJoint;
    Channel& channel = channel_entry->second;

    if (feedback_hold_latched_) {
        if (!sendFeedbackHoldUnlocked(channel)) {
            latchFaultUnlocked(
                "CAN feedback-hold command failed on " + joint_name);
            return MotorCommandResult::CanWriteFailure;
        }
        return MotorCommandResult::FeedbackHold;
    }

    if (!std::isfinite(target_position_rad) ||
        !std::isfinite(target_velocity_rad_s) ||
        !std::isfinite(kp) ||
        !std::isfinite(kd) ||
        !std::isfinite(feedforward_torque_nm)) {
        latchFaultUnlocked("Non-finite MIT command on " + joint_name);
        return MotorCommandResult::InvalidCommand;
    }
    if (kp < 0.0 || kp > RS02_OPERATION_MAX_KP ||
        kd < 0.0 || kd > RS02_OPERATION_MAX_KD) {
        latchFaultUnlocked(
            "Out-of-range position gain on " + joint_name);
        return MotorCommandResult::InvalidCommand;
    }
    const auto now = std::chrono::steady_clock::now();
    for (const auto& entry : channels_) {
        if (!entry.second.feedback.valid) {
            return enterFeedbackHoldUnlocked(
                "Missing Type 2 feedback on " + entry.first,
                channel);
        }
        if (!feedbackFresh(entry.second.feedback, now)) {
            return enterFeedbackHoldUnlocked(
                "Stale Type 2 feedback on " + entry.first,
                channel);
        }
    }
    if (channel.limiter.isHardViolation(
            channel.feedback.position_rad)) {
        latchFaultUnlocked(
            hardLimitViolationReason(
                joint_name,
                channel.feedback.source,
                channel.feedback.position_rad,
                channel.limiter));
        return MotorCommandResult::HardLimitViolation;
    }

    const std::optional<double> safe_target =
        channel.limiter.clampTarget(target_position_rad);
    if (!safe_target) {
        latchFaultUnlocked(
            "Cannot limit invalid target on " + joint_name);
        return MotorCommandResult::InvalidCommand;
    }
    const double safe_joint_velocity =
        *safe_target == target_position_rad
        ? target_velocity_rad_s
        : 0.0;
    const double motor_velocity = jointToMotorVelocity(
        channel.motor,
        safe_joint_velocity);
    const double motor_torque = jointToMotorTorque(
        channel.motor,
        feedforward_torque_nm);
    if (std::abs(motor_velocity) >
        RS02_OPERATION_MAX_VELOCITY_RAD_S) {
        latchFaultUnlocked(
            "Out-of-range MIT velocity on " + joint_name);
        return MotorCommandResult::InvalidCommand;
    }
    if (std::abs(motor_torque) > RS02_OPERATION_MAX_TORQUE_NM) {
        latchFaultUnlocked(
            "Out-of-range MIT feedforward torque on " + joint_name);
        return MotorCommandResult::InvalidCommand;
    }
    const double motor_position =
        jointToMotorPosition(channel.motor, *safe_target);
    if (!sendMitFrame(
            channel.motor.motor_id,
            motor_position,
            motor_velocity,
            kp,
            kd,
            motor_torque)) {
        latchFaultUnlocked("CAN MIT command failed on " + joint_name);
        return MotorCommandResult::CanWriteFailure;
    }

    channel.last_mit_command = LastMitCommand{
        motor_position,
        kp,
        kd,
        motor_torque,
        true};

    return *safe_target == target_position_rad
        ? MotorCommandResult::Sent
        : MotorCommandResult::TargetClamped;
}

std::optional<MotorFeedback> MotorDriver::feedback(
    const std::string& joint_name) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto channel = channels_.find(joint_name);
    if (channel == channels_.end())
        return std::nullopt;
    return channel->second.feedback;
}

bool MotorDriver::allFeedbackValid() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    return std::all_of(
        channels_.begin(),
        channels_.end(),
        [&](const auto& entry) {
            return entry.second.feedback.valid &&
                   feedbackFresh(entry.second.feedback, now);
        });
}

bool MotorDriver::isEnabled() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
}

bool MotorDriver::feedbackHoldLatched() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return feedback_hold_latched_;
}

std::string MotorDriver::feedbackHoldReason() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return feedback_hold_reason_;
}

bool MotorDriver::faultLatched() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_latched_;
}

std::string MotorDriver::faultReason() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_reason_;
}

bool MotorDriver::feedbackFresh(
    const MotorFeedback& feedback,
    std::chrono::steady_clock::time_point now) const noexcept
{
    return feedback.valid &&
           now - feedback.received_at <= feedback_timeout_;
}

bool MotorDriver::sendEnable(std::uint8_t motor_id)
{
    CanFrame frame;
    frame.id = buildExtendedId(
        COMM_ENABLE,
        host_id_,
        motor_id);
    frame.dlc = 0;
    return transport_.send(frame);
}

bool MotorDriver::sendStop(std::uint8_t motor_id)
{
    CanFrame frame;
    frame.id = buildExtendedId(
        COMM_STOP,
        host_id_,
        motor_id);
    frame.dlc = 0;
    return transport_.send(frame);
}

bool MotorDriver::sendReadMechanicalPosition(
    std::uint8_t motor_id)
{
    CanFrame frame;
    frame.id = buildExtendedId(
        COMM_READ_PARAMETER,
        host_id_,
        motor_id);
    frame.dlc = 8;
    packU16LittleEndian(
        frame.data,
        0,
        PARAM_MECHANICAL_POSITION);
    return transport_.send(frame);
}

bool MotorDriver::sendReadBusVoltage(std::uint8_t motor_id)
{
    CanFrame frame;
    frame.id = buildExtendedId(
        COMM_READ_PARAMETER,
        host_id_,
        motor_id);
    frame.dlc = 8;
    packU16LittleEndian(frame.data, 0, PARAM_BUS_VOLTAGE);
    return transport_.send(frame);
}

bool MotorDriver::sendMitFrame(
    std::uint8_t motor_id,
    double position_rad,
    double velocity_rad_s,
    double kp,
    double kd,
    double torque_nm)
{
    CanFrame frame;
    frame.id = buildExtendedId(
        COMM_OPERATION_CONTROL,
        encodeSymmetric(torque_nm, RS02_OPERATION_MAX_TORQUE_NM),
        motor_id);
    frame.dlc = 8;

    packU16BigEndian(
        frame.data,
        0,
        encodeSymmetric(position_rad, POSITION_SCALE_RAD));
    packU16BigEndian(
        frame.data,
        2,
        encodeSymmetric(
            velocity_rad_s,
            RS02_OPERATION_MAX_VELOCITY_RAD_S));
    packU16BigEndian(
        frame.data,
        4,
        encodeUnsigned(kp, RS02_OPERATION_MAX_KP));
    packU16BigEndian(
        frame.data,
        6,
        encodeUnsigned(kd, RS02_OPERATION_MAX_KD));
    return transport_.send(frame);
}

bool MotorDriver::allLastMitCommandsValidUnlocked() const
{
    return std::all_of(
        channels_.begin(),
        channels_.end(),
        [](const auto& entry) {
            return entry.second.last_mit_command.valid;
        });
}

bool MotorDriver::sendFeedbackHoldUnlocked(Channel& channel)
{
    const LastMitCommand& command = channel.last_mit_command;
    return command.valid &&
           sendMitFrame(
               channel.motor.motor_id,
               command.motor_position_rad,
               0.0,
               command.kp,
               command.kd,
               command.motor_torque_nm);
}

MotorCommandResult MotorDriver::enterFeedbackHoldUnlocked(
    std::string reason,
    Channel& channel)
{
    if (!allLastMitCommandsValidUnlocked()) {
        latchFaultUnlocked(
            std::move(reason) +
            "; no complete safe command snapshot is available");
        return MotorCommandResult::StaleFeedback;
    }

    feedback_hold_latched_ = true;
    feedback_hold_reason_ = std::move(reason);
    if (!sendFeedbackHoldUnlocked(channel)) {
        latchFaultUnlocked(
            "CAN feedback-hold command failed on " +
            channel.motor.joint_name);
        return MotorCommandResult::CanWriteFailure;
    }
    return MotorCommandResult::FeedbackHold;
}

bool MotorDriver::stopAllUnlocked()
{
    bool success = true;
    for (const auto& entry : channels_) {
        success =
            sendStop(entry.second.motor.motor_id) &&
            success;
    }
    enabled_ = false;
    feedback_hold_latched_ = false;
    feedback_hold_reason_.clear();
    for (auto& entry : channels_) {
        entry.second.last_mit_command = LastMitCommand{};
        entry.second.feedback.valid = false;
        entry.second.feedback.operation_feedback_valid = false;
        entry.second.feedback.source = MotorFeedbackSource::None;
        entry.second.feedback.received_at = {};
        entry.second.feedback.operation_received_at = {};
    }
    return success;
}

void MotorDriver::latchFaultUnlocked(std::string reason)
{
    fault_latched_ = true;
    fault_reason_ = std::move(reason);
    (void)stopAllUnlocked();
}

void MotorDriver::processFrameUnlocked(const CanFrame& frame)
{
    if (destinationHostId(frame) != host_id_ || frame.dlc != 8)
        return;

    const std::uint8_t communication_type = communicationType(frame);
    if (communication_type == COMM_OPERATION_FEEDBACK) {
        // Type 2 frames queued during a stop transition must not become the
        // next enable's bootstrap position.
        if (!enabled_)
            return;

        const auto decoded = decodeRs02OperationFeedback(
            frame,
            host_id_);
        if (!decoded)
            return;

        const auto joint = motor_to_joint_.find(decoded->motor_id);
        if (joint == motor_to_joint_.end())
            return;

        Channel& channel = channels_.at(joint->second);
        const std::optional<double> previous_position =
            channel.feedback.valid
            ? std::optional<double>(channel.feedback.position_rad)
            : std::nullopt;
        const auto received_at = std::chrono::steady_clock::now();

        channel.feedback.position_rad = motorToJointPosition(
            channel.motor,
            decoded->motor_position_rad);
        channel.feedback.velocity_rad_s = motorToJointVelocity(
            channel.motor,
            decoded->motor_velocity_rad_s);
        channel.feedback.torque_nm = motorToJointTorque(
            channel.motor,
            decoded->motor_torque_nm);
        channel.feedback.temperature_celsius =
            decoded->temperature_celsius;
        channel.feedback.fault_flags = decoded->fault_flags;
        channel.feedback.mode_state = decoded->mode_state;
        channel.feedback.source = MotorFeedbackSource::Operation;
        channel.feedback.valid = true;
        channel.feedback.operation_feedback_valid = true;
        channel.feedback.received_at = received_at;
        channel.feedback.operation_received_at = received_at;

        if (channel.feedback.fault_flags != 0) {
            latchFaultUnlocked(
                "RS02 Type 2 fault flags " +
                std::to_string(channel.feedback.fault_flags) +
                " on " + joint->second);
            return;
        }
        if (channel.limiter.isHardViolation(
                channel.feedback.position_rad)) {
            latchFaultUnlocked(
                hardLimitViolationReason(
                    joint->second,
                    channel.feedback.source,
                    channel.feedback.position_rad,
                    channel.limiter,
                    previous_position));
        }
        return;
    }

    if (communication_type != COMM_READ_PARAMETER)
        return;

    const std::uint16_t parameter =
        std::uint16_t(frame.data[0]) |
        (std::uint16_t(frame.data[1]) << 8);

    const auto joint = motor_to_joint_.find(sourceMotorId(frame));
    if (joint == motor_to_joint_.end())
        return;

    Channel& channel = channels_.at(joint->second);
    if (parameter == PARAM_BUS_VOLTAGE) {
        const double bus_voltage_v =
            unpackFloatLittleEndian(frame.data, 4);
        if (!std::isfinite(bus_voltage_v) || bus_voltage_v < 0.0)
            return;
        channel.feedback.bus_voltage_v = bus_voltage_v;
        channel.feedback.bus_voltage_valid = true;
        channel.feedback.bus_voltage_received_at =
            std::chrono::steady_clock::now();
        return;
    }

    // A delayed mechanical-position response must not replace active
    // Type 2 feedback.
    if (enabled_)
        return;

    if (parameter != PARAM_MECHANICAL_POSITION) {
        return;
    }
    const std::optional<double> previous_position =
        channel.feedback.valid
        ? std::optional<double>(channel.feedback.position_rad)
        : std::nullopt;
    channel.feedback.position_rad =
        motorToJointPosition(
            channel.motor,
            unpackFloatLittleEndian(frame.data, 4));
    channel.feedback.source =
        MotorFeedbackSource::MechanicalPosition;
    channel.feedback.valid = true;
    channel.feedback.received_at =
        std::chrono::steady_clock::now();

    if (channel.limiter.isHardViolation(
            channel.feedback.position_rad)) {
        latchFaultUnlocked(
            hardLimitViolationReason(
                joint->second,
                channel.feedback.source,
                channel.feedback.position_rad,
                channel.limiter,
                previous_position));
    }
}

}  // namespace raven_control::hal
