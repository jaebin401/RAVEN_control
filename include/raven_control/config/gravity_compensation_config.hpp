#pragma once

#include <array>
#include <chrono>
#include <string>

namespace raven_control::config {

inline constexpr std::size_t GRAVITY_COMPENSATION_JOINT_COUNT = 3;

struct GravityCompensationConfig {
    std::string urdf_path =
        "../RAVEN_hardware/urdf/urdf/RAVEN.urdf";
    bool enabled = false;
    bool dry_run = true;
    double scale = 0.0;
    std::chrono::milliseconds ramp_duration{3000};
    std::array<double, GRAVITY_COMPENSATION_JOINT_COUNT>
        max_joint_torque_nm{};
};

}  // namespace raven_control::config
