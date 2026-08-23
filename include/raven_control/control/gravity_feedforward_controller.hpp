#pragma once

#include "raven_control/config/gravity_compensation_config.hpp"
#include "raven_control/dynamics/gravity_model.hpp"

#include <array>
#include <chrono>
#include <memory>

namespace raven_control::control {

struct GravityFeedforwardResult {
    dynamics::JointVector raw_gravity_torque_nm{};
    dynamics::JointVector ramped_gravity_torque_nm{};
    dynamics::JointVector limited_gravity_torque_nm{};
    dynamics::JointVector commanded_torque_nm{};
    std::array<bool, dynamics::RAVEN_JOINT_COUNT> torque_clamped{};
    double scale = 0.0;
    double ramp_factor = 0.0;
    bool enabled = false;
    bool dry_run = true;
    bool input_valid = false;
};

class GravityFeedforwardController {
public:
    GravityFeedforwardController(
        std::unique_ptr<dynamics::GravityModel> gravity_model,
        config::GravityCompensationConfig config);

    GravityFeedforwardController(const GravityFeedforwardController&) =
        delete;
    GravityFeedforwardController& operator=(
        const GravityFeedforwardController&) = delete;

    void reset(std::chrono::steady_clock::time_point now) noexcept;
    void setEnabled(
        bool enabled,
        std::chrono::steady_clock::time_point now) noexcept;

    [[nodiscard]] bool enabled() const noexcept;
    [[nodiscard]] double rampFactor() const noexcept;
    [[nodiscard]] const config::GravityCompensationConfig& config()
        const noexcept;

    [[nodiscard]] GravityFeedforwardResult compute(
        const dynamics::JointVector& joint_positions_rad,
        bool feedback_fresh,
        std::chrono::steady_clock::time_point now);

private:
    std::unique_ptr<dynamics::GravityModel> gravity_model_;
    config::GravityCompensationConfig config_;
    bool enabled_ = false;
    double ramp_factor_ = 0.0;
    std::chrono::steady_clock::time_point last_update_{};
};

}  // namespace raven_control::control
