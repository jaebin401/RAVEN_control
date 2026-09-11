#pragma once

#include "raven_control/config/motor_config.hpp"
#include "raven_control/control/gravity_feedforward_controller.hpp"
#include "raven_control/hal/motor_driver.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

namespace raven_control::control {

struct JointMitCommand {
    double target_position_rad = 0.0;
    double target_velocity_rad_s = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double application_feedforward_torque_nm = 0.0;
};

struct MitCommandPipelineResult {
    GravityFeedforwardResult gravity;
    dynamics::JointVector final_feedforward_torque_nm{};
    std::array<hal::MotorCommandResult, dynamics::RAVEN_JOINT_COUNT>
        motor_results{};
    bool all_sent = false;
    bool target_clamped = false;
    bool feedback_hold = false;
    std::string error;
};

// Shared command path for every normal RAVEN MIT-mode application.  It adds
// URDF gravity feedforward to application-specific feedforward before the
// final joint commands cross the HAL boundary.
class MitCommandPipeline {
public:
    using JointNames = std::array<
        std::string,
        dynamics::RAVEN_JOINT_COUNT>;
    using Commands = std::array<
        JointMitCommand,
        dynamics::RAVEN_JOINT_COUNT>;

    MitCommandPipeline(
        hal::MotorDriver& driver,
        std::unique_ptr<dynamics::GravityModel> gravity_model,
        const config::MotorRuntimeConfig& config,
        JointNames joint_names);

    MitCommandPipeline(const MitCommandPipeline&) = delete;
    MitCommandPipeline& operator=(const MitCommandPipeline&) = delete;

    void reset(std::chrono::steady_clock::time_point now) noexcept;
    void setGravityEnabled(
        bool enabled,
        std::chrono::steady_clock::time_point now) noexcept;

    [[nodiscard]] bool gravityEnabled() const noexcept;
    [[nodiscard]] double gravityRampFactor() const noexcept;
    [[nodiscard]] const config::GravityCompensationConfig& gravityConfig()
        const noexcept;
    [[nodiscard]] const GravityFeedforwardResult& lastGravityResult()
        const noexcept;
    [[nodiscard]] const dynamics::JointVector& lastAppliedFeedforwardTorque()
        const noexcept;

    [[nodiscard]] MitCommandPipelineResult send(
        const Commands& commands,
        std::chrono::steady_clock::time_point now);

private:
    hal::MotorDriver& driver_;
    GravityFeedforwardController gravity_;
    config::MotorRuntimeConfig config_;
    JointNames joint_names_;
    GravityFeedforwardResult last_gravity_result_{};
    dynamics::JointVector last_applied_feedforward_torque_nm_{};
};

}  // namespace raven_control::control
