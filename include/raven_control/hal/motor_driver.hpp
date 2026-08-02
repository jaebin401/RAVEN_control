#pragma once

#include "raven_control/hal/can_interface.hpp"
#include "raven_control/hal/rs02_operation_feedback.hpp"
#include "raven_control/safety/joint_limiter.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace raven_control::hal {

enum class MotorFeedbackSource {
    None,
    MechanicalPosition,
    Operation,
};

[[nodiscard]] const char* toString(
    MotorFeedbackSource source) noexcept;

struct JointMotorConfig {
    std::string joint_name;
    std::uint8_t motor_id = 0;
    int position_sign = 1;
    double joint_zero_at_motor_rad = 0.0;
    double joint_to_motor_ratio = 1.0;
};

struct MotorFeedback {
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double torque_nm = 0.0;
    double temperature_celsius = 0.0;
    std::uint8_t fault_flags = 0;
    std::uint8_t mode_state = 0;
    MotorFeedbackSource source = MotorFeedbackSource::None;
    bool valid = false;
    bool operation_feedback_valid = false;
    std::chrono::steady_clock::time_point received_at{};
    std::chrono::steady_clock::time_point operation_received_at{};
};

enum class MotorCommandResult {
    Sent,
    TargetClamped,
    FeedbackHold,
    NotEnabled,
    UnknownJoint,
    InvalidCommand,
    InvalidFeedback,
    StaleFeedback,
    UnconfirmedLimits,
    HardLimitViolation,
    FaultLatched,
    CanWriteFailure,
};

[[nodiscard]] const char* toString(MotorCommandResult result) noexcept;

class MotorDriver {
public:
    MotorDriver(
        CanTransport& transport,
        std::vector<JointMotorConfig> motor_map,
        safety::JointLimiterMap joint_limiters,
        std::uint8_t host_id = 0xFD,
        std::chrono::milliseconds feedback_timeout =
            std::chrono::milliseconds(250));

    MotorDriver(const MotorDriver&) = delete;
    MotorDriver& operator=(const MotorDriver&) = delete;

    [[nodiscard]] MotorCommandResult enableAll();
    [[nodiscard]] bool stopAll();
    [[nodiscard]] bool requestMechanicalPositions();
    std::size_t poll();

    [[nodiscard]] MotorCommandResult sendMitCommand(
        const std::string& joint_name,
        double target_position_rad,
        double target_velocity_rad_s,
        double kp,
        double kd,
        double feedforward_torque_nm);

    [[nodiscard]] std::optional<MotorFeedback> feedback(
        const std::string& joint_name) const;
    [[nodiscard]] bool allFeedbackValid() const;
    [[nodiscard]] bool isEnabled() const;
    [[nodiscard]] bool feedbackHoldLatched() const;
    [[nodiscard]] std::string feedbackHoldReason() const;
    [[nodiscard]] bool faultLatched() const;
    [[nodiscard]] std::string faultReason() const;

private:
    struct LastMitCommand {
        double motor_position_rad = 0.0;
        double kp = 0.0;
        double kd = 0.0;
        double motor_torque_nm = 0.0;
        bool valid = false;
    };

    struct Channel {
        JointMotorConfig motor;
        safety::JointLimiter limiter;
        MotorFeedback feedback;
        LastMitCommand last_mit_command;
    };

    using ChannelMap = std::unordered_map<std::string, Channel>;

    [[nodiscard]] bool feedbackFresh(
        const MotorFeedback& feedback,
        std::chrono::steady_clock::time_point now) const noexcept;
    [[nodiscard]] bool sendEnable(std::uint8_t motor_id);
    [[nodiscard]] bool sendStop(std::uint8_t motor_id);
    [[nodiscard]] bool sendReadMechanicalPosition(
        std::uint8_t motor_id);
    [[nodiscard]] bool sendMitFrame(
        std::uint8_t motor_id,
        double position_rad,
        double velocity_rad_s,
        double kp,
        double kd,
        double torque_nm);
    [[nodiscard]] bool allLastMitCommandsValidUnlocked() const;
    [[nodiscard]] bool sendFeedbackHoldUnlocked(Channel& channel);
    [[nodiscard]] MotorCommandResult enterFeedbackHoldUnlocked(
        std::string reason,
        Channel& channel);
    [[nodiscard]] bool stopAllUnlocked();
    void latchFaultUnlocked(std::string reason);
    void processFrameUnlocked(const CanFrame& frame);

    CanTransport& transport_;
    ChannelMap channels_;
    std::unordered_map<std::uint8_t, std::string> motor_to_joint_;
    std::uint8_t host_id_;
    std::chrono::milliseconds feedback_timeout_;
    bool enabled_ = false;
    bool feedback_hold_latched_ = false;
    std::string feedback_hold_reason_;
    bool fault_latched_ = false;
    std::string fault_reason_;
    mutable std::mutex mutex_;
};

}  // namespace raven_control::hal
