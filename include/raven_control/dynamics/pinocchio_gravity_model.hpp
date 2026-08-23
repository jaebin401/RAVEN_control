#pragma once

#include "raven_control/dynamics/gravity_model.hpp"

#include <array>
#include <memory>
#include <string>

namespace raven_control::dynamics {

inline const std::array<std::string, RAVEN_JOINT_COUNT>
    RAVEN_URDF_JOINT_NAMES{
        "shoulder_Joint",
        "upperArm_Joint",
        "foreArm_Joint",
    };

class PinocchioGravityModel final : public GravityModel {
public:
    explicit PinocchioGravityModel(
        std::string urdf_path,
        std::array<std::string, RAVEN_JOINT_COUNT> joint_names =
            RAVEN_URDF_JOINT_NAMES,
        Vector3 gravity_base_m_s2 = {0.0, 0.0, -9.81});

    ~PinocchioGravityModel() override;

    PinocchioGravityModel(PinocchioGravityModel&&) noexcept;
    PinocchioGravityModel& operator=(PinocchioGravityModel&&) noexcept;

    PinocchioGravityModel(const PinocchioGravityModel&) = delete;
    PinocchioGravityModel& operator=(const PinocchioGravityModel&) = delete;

    // Reuses Pinocchio Data allocated at construction. Do not call this
    // method concurrently on the same model instance.
    [[nodiscard]] JointVector compute(
        const JointVector& joint_positions_rad) const override;

    [[nodiscard]] const std::string& urdfPath() const noexcept;
    [[nodiscard]] const std::array<std::string, RAVEN_JOINT_COUNT>&
    jointNames() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace raven_control::dynamics
