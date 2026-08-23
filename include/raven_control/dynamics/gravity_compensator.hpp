#pragma once

#include "raven_control/dynamics/gravity_model.hpp"

#include <array>

namespace raven_control::dynamics {

struct GravityLinkParameters {
    Vector3 joint_origin_parent_m{};
    Vector3 joint_origin_rpy_rad{};
    Vector3 joint_axis{};
    double mass_kg = 0.0;
    Vector3 center_of_mass_child_m{};
};

struct GravityModelParameters {
    std::array<GravityLinkParameters, RAVEN_JOINT_COUNT> links{};
    Vector3 gravity_base_m_s2{0.0, 0.0, -9.81};

    void validate() const;
};

class GravityCompensator final : public GravityModel {
public:
    explicit GravityCompensator(GravityModelParameters parameters);

    [[nodiscard]] JointVector compute(
        const JointVector& joint_positions_rad) const override;

    [[nodiscard]] const GravityModelParameters& parameters() const noexcept;

private:
    GravityModelParameters parameters_;
};

// Parameters copied from RAVEN_hardware/urdf/urdf/RAVEN.urdf.
[[nodiscard]] GravityModelParameters makeRavenUrdfGravityModel();

}  // namespace raven_control::dynamics
