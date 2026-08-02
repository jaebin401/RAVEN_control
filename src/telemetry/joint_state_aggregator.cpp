#include "raven_control/telemetry/joint_state_aggregator.hpp"

#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace raven_control::telemetry {

JointStateAggregator::JointStateAggregator(
    std::vector<config::JointMotorRuntimeConfig> joints)
{
    if (joints.empty())
        throw std::invalid_argument("Joint telemetry map must not be empty");

    std::unordered_map<std::string, bool> joint_names;
    channels_.reserve(joints.size());
    for (auto& joint : joints) {
        if (joint.joint_name.empty()) {
            throw std::invalid_argument(
                "Joint telemetry map contains an empty joint name");
        }
        if (joint.position_sign != -1 && joint.position_sign != 1) {
            throw std::invalid_argument(
                "Position sign must be -1 or 1 for '" +
                joint.joint_name + "'");
        }
        if (!std::isfinite(joint.joint_zero_at_motor_rad) ||
            !std::isfinite(joint.joint_to_motor_ratio) ||
            joint.joint_to_motor_ratio <= 0.0) {
            throw std::invalid_argument(
                "Invalid telemetry calibration for '" +
                joint.joint_name + "'");
        }
        if (!joint_names.emplace(joint.joint_name, true).second) {
            throw std::invalid_argument(
                "Duplicate telemetry joint: " + joint.joint_name);
        }
        if (motor_to_index_.find(joint.motor_id) !=
            motor_to_index_.end()) {
            throw std::invalid_argument(
                "Duplicate telemetry motor ID: " +
                std::to_string(joint.motor_id));
        }

        const std::size_t index = channels_.size();
        motor_to_index_.emplace(joint.motor_id, index);
        JointStateSample sample;
        sample.joint_name = joint.joint_name;
        channels_.push_back(Channel{
            std::move(joint),
            std::move(sample),
            false});
    }
}

bool JointStateAggregator::ingest(
    const hal::Rs02OperationFeedback& feedback,
    std::chrono::steady_clock::time_point received_at)
{
    const auto channel_entry = motor_to_index_.find(feedback.motor_id);
    if (channel_entry == motor_to_index_.end())
        return false;

    Channel& channel = channels_[channel_entry->second];
    const double sign = static_cast<double>(
        channel.config.position_sign);
    const double ratio = channel.config.joint_to_motor_ratio;
    channel.sample.position_rad =
        sign *
        (feedback.motor_position_rad -
         channel.config.joint_zero_at_motor_rad) /
        ratio;
    channel.sample.velocity_rad_s =
        sign * feedback.motor_velocity_rad_s / ratio;
    // Preserve virtual work for q_motor = sign * ratio * q_joint.
    channel.sample.effort_nm =
        sign * ratio * feedback.motor_torque_nm;
    channel.sample.temperature_celsius =
        feedback.temperature_celsius;
    channel.sample.fault_flags = feedback.fault_flags;
    channel.sample.mode_state = feedback.mode_state;
    channel.sample.received_at = received_at;
    channel.valid = true;
    return true;
}

std::optional<JointStateSnapshot> JointStateAggregator::snapshot(
    std::chrono::steady_clock::time_point now,
    std::chrono::milliseconds freshness_timeout) const
{
    if (freshness_timeout.count() <= 0)
        return std::nullopt;

    JointStateSnapshot result;
    result.captured_at = now;
    result.joints.reserve(channels_.size());
    for (const Channel& channel : channels_) {
        if (!channel.valid ||
            now < channel.sample.received_at ||
            now - channel.sample.received_at > freshness_timeout) {
            return std::nullopt;
        }
        result.joints.push_back(channel.sample);
    }
    return result;
}

std::size_t JointStateAggregator::jointCount() const noexcept
{
    return channels_.size();
}

}  // namespace raven_control::telemetry
