#include "raven_control/control/mit_command_pipeline.hpp"

#include "raven_control/hal/mit_torque_codec.hpp"
#include "raven_control/hal/rs02_operation_feedback.hpp"

#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace raven_control::control {
namespace {

bool finite(const JointMitCommand& command)
{
    return std::isfinite(command.target_position_rad) &&
           std::isfinite(command.target_velocity_rad_s) &&
           std::isfinite(command.kp) &&
           std::isfinite(command.kd) &&
           std::isfinite(command.application_feedforward_torque_nm);
}

}  // namespace

MitCommandPipeline::MitCommandPipeline(
    hal::MotorDriver& driver,
    std::unique_ptr<dynamics::GravityModel> gravity_model,
    const config::MotorRuntimeConfig& config,
    JointNames joint_names)
    : driver_(driver),
      gravity_(std::move(gravity_model), config.gravity_compensation),
      config_(config),
      joint_names_(std::move(joint_names))
{
    std::unordered_map<std::string, bool> seen;
    for (const auto& joint_name : joint_names_) {
        if (joint_name.empty() || config_.findJoint(joint_name) == nullptr) {
            throw std::invalid_argument(
                "MIT pipeline joint is missing from motor config: " +
                joint_name);
        }
        if (!seen.emplace(joint_name, true).second) {
            throw std::invalid_argument(
                "MIT pipeline joint names must be unique");
        }
    }
}

void MitCommandPipeline::reset(
    std::chrono::steady_clock::time_point now) noexcept
{
    gravity_.reset(now);
    last_gravity_result_ = {};
    last_applied_feedforward_torque_nm_ = {};
}

void MitCommandPipeline::setGravityEnabled(
    bool enabled,
    std::chrono::steady_clock::time_point now) noexcept
{
    gravity_.setEnabled(enabled, now);
}

bool MitCommandPipeline::gravityEnabled() const noexcept
{
    return gravity_.enabled();
}

double MitCommandPipeline::gravityRampFactor() const noexcept
{
    return gravity_.rampFactor();
}

const config::GravityCompensationConfig&
MitCommandPipeline::gravityConfig() const noexcept
{
    return gravity_.config();
}

const GravityFeedforwardResult&
MitCommandPipeline::lastGravityResult() const noexcept
{
    return last_gravity_result_;
}

const dynamics::JointVector&
MitCommandPipeline::lastAppliedFeedforwardTorque() const noexcept
{
    return last_applied_feedforward_torque_nm_;
}

MitCommandPipelineResult MitCommandPipeline::send(
    const Commands& commands,
    std::chrono::steady_clock::time_point now)
{
    MitCommandPipelineResult result;
    dynamics::JointVector positions{};
    bool feedback_fresh = true;

    for (std::size_t index = 0; index < joint_names_.size(); ++index) {
        const auto feedback = driver_.feedback(joint_names_[index]);
        if (!feedback || !feedback->valid ||
            !feedback->operation_feedback_valid ||
            feedback->operation_received_at ==
                std::chrono::steady_clock::time_point{} ||
            now < feedback->operation_received_at ||
            now - feedback->operation_received_at >
                config_.feedback_timeout) {
            feedback_fresh = false;
            break;
        }
        positions[index] = feedback->position_rad;
    }

    result.gravity = gravity_.compute(positions, feedback_fresh, now);
    last_gravity_result_ = result.gravity;

    for (std::size_t index = 0; index < commands.size(); ++index) {
        if (!finite(commands[index])) {
            result.error = "Non-finite MIT command for '" +
                joint_names_[index] + "'";
            return result;
        }
        if (result.gravity.torque_clamped[index]) {
            result.error = "Gravity torque reached configured limit for '" +
                joint_names_[index] + "'";
            return result;
        }

        result.final_feedforward_torque_nm[index] =
            result.gravity.commanded_torque_nm[index] +
            commands[index].application_feedforward_torque_nm;
        if (!std::isfinite(result.final_feedforward_torque_nm[index])) {
            result.error = "Non-finite combined feedforward torque for '" +
                joint_names_[index] + "'";
            return result;
        }

        const auto* joint = config_.findJoint(joint_names_[index]);
        const double motor_torque = hal::jointToMotorTorque(
            joint->position_sign,
            joint->joint_to_motor_ratio,
            result.final_feedforward_torque_nm[index]);
        if (std::abs(motor_torque) >
            hal::RS02_OPERATION_MAX_TORQUE_NM) {
            result.error = "Combined feedforward torque exceeds RS02 range for '" +
                joint_names_[index] + "'";
            return result;
        }
    }

    for (std::size_t index = 0; index < commands.size(); ++index) {
        const auto& command = commands[index];
        result.motor_results[index] = driver_.sendMitCommand(
            joint_names_[index],
            command.target_position_rad,
            command.target_velocity_rad_s,
            command.kp,
            command.kd,
            result.final_feedforward_torque_nm[index]);

        const auto motor_result = result.motor_results[index];
        if (motor_result == hal::MotorCommandResult::TargetClamped) {
            result.target_clamped = true;
            continue;
        }
        if (motor_result == hal::MotorCommandResult::FeedbackHold) {
            result.feedback_hold = true;
            result.final_feedforward_torque_nm =
                last_applied_feedforward_torque_nm_;
            return result;
        }
        if (motor_result != hal::MotorCommandResult::Sent) {
            result.error = "MIT command failed for '" +
                joint_names_[index] + "': " + hal::toString(motor_result);
            return result;
        }
    }

    result.all_sent = true;
    last_applied_feedforward_torque_nm_ =
        result.final_feedforward_torque_nm;
    return result;
}

}  // namespace raven_control::control
