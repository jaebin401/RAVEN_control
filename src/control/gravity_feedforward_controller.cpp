#include "raven_control/control/gravity_feedforward_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace raven_control::control {
namespace {

constexpr std::chrono::duration<double> MAX_RAMP_UPDATE{0.1};

void validateConfig(const config::GravityCompensationConfig& config)
{
    if (config.urdf_path.empty())
        throw std::invalid_argument("Gravity URDF path must not be empty");
    if (!std::isfinite(config.scale) ||
        config.scale < 0.0 || config.scale > 1.0) {
        throw std::invalid_argument(
            "Gravity compensation scale must be in range 0..1");
    }
    if (config.ramp_duration.count() <= 0) {
        throw std::invalid_argument(
            "Gravity compensation ramp duration must be positive");
    }
    for (double limit : config.max_joint_torque_nm) {
        if (!std::isfinite(limit) || limit < 0.0) {
            throw std::invalid_argument(
                "Gravity torque limits must be finite and nonnegative");
        }
    }
}

bool finite(const dynamics::JointVector& value)
{
    return std::all_of(
        value.begin(),
        value.end(),
        [](double element) { return std::isfinite(element); });
}

}  // namespace

GravityFeedforwardController::GravityFeedforwardController(
    std::unique_ptr<dynamics::GravityModel> gravity_model,
    config::GravityCompensationConfig config)
    : gravity_model_(std::move(gravity_model)),
      config_(std::move(config)),
      enabled_(config_.enabled)
{
    if (!gravity_model_)
        throw std::invalid_argument("Gravity model must not be null");
    validateConfig(config_);
}

void GravityFeedforwardController::reset(
    std::chrono::steady_clock::time_point now) noexcept
{
    ramp_factor_ = 0.0;
    last_update_ = now;
}

void GravityFeedforwardController::setEnabled(
    bool enabled,
    std::chrono::steady_clock::time_point now) noexcept
{
    enabled_ = enabled;
    if (!enabled_) {
        ramp_factor_ = 0.0;
        last_update_ = now;
    } else if (last_update_ ==
               std::chrono::steady_clock::time_point{}) {
        last_update_ = now;
    }
}

bool GravityFeedforwardController::enabled() const noexcept
{
    return enabled_;
}

double GravityFeedforwardController::rampFactor() const noexcept
{
    return ramp_factor_;
}

const config::GravityCompensationConfig&
GravityFeedforwardController::config() const noexcept
{
    return config_;
}

GravityFeedforwardResult GravityFeedforwardController::compute(
    const dynamics::JointVector& joint_positions_rad,
    bool feedback_fresh,
    std::chrono::steady_clock::time_point now)
{
    GravityFeedforwardResult result;
    result.scale = config_.scale;
    result.enabled = enabled_;
    result.dry_run = config_.dry_run;

    if (!enabled_) {
        reset(now);
        return result;
    }
    if (!feedback_fresh || !finite(joint_positions_rad)) {
        reset(now);
        return result;
    }

    if (last_update_ == std::chrono::steady_clock::time_point{} ||
        now < last_update_) {
        last_update_ = now;
    }
    const auto elapsed = std::min(
        std::chrono::duration<double>(now - last_update_),
        MAX_RAMP_UPDATE);
    last_update_ = now;
    const double ramp_seconds =
        std::chrono::duration<double>(config_.ramp_duration).count();
    ramp_factor_ = std::clamp(
        ramp_factor_ + elapsed.count() / ramp_seconds,
        0.0,
        1.0);

    result.raw_gravity_torque_nm =
        gravity_model_->compute(joint_positions_rad);
    if (!finite(result.raw_gravity_torque_nm)) {
        reset(now);
        return result;
    }

    result.input_valid = true;
    result.ramp_factor = ramp_factor_;
    for (std::size_t index = 0;
         index < dynamics::RAVEN_JOINT_COUNT;
         ++index) {
        result.ramped_gravity_torque_nm[index] =
            result.raw_gravity_torque_nm[index] *
            config_.scale * ramp_factor_;
        const double limit = config_.max_joint_torque_nm[index];
        result.limited_gravity_torque_nm[index] = std::clamp(
            result.ramped_gravity_torque_nm[index],
            -limit,
            limit);
        result.torque_clamped[index] =
            result.limited_gravity_torque_nm[index] !=
            result.ramped_gravity_torque_nm[index];
        result.commanded_torque_nm[index] = config_.dry_run
            ? 0.0
            : result.limited_gravity_torque_nm[index];
    }
    return result;
}

}  // namespace raven_control::control
