#pragma once

#include <array>
#include <cstddef>

namespace raven_control::dynamics {

inline constexpr std::size_t RAVEN_JOINT_COUNT = 3;

using Vector3 = std::array<double, 3>;
using JointVector = std::array<double, RAVEN_JOINT_COUNT>;

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

class GravityCompensator {
public:
    explicit GravityCompensator(GravityModelParameters parameters);

    // Returns joint-coordinate torque that balances gravity at q.
    [[nodiscard]] JointVector compute(
        const JointVector& joint_positions_rad) const;

    [[nodiscard]] const GravityModelParameters& parameters() const noexcept;

private:
    GravityModelParameters parameters_;
};

// Parameters copied from RAVEN_hardware/urdf/urdf/RAVEN.urdf.
[[nodiscard]] GravityModelParameters makeRavenUrdfGravityModel();

}  // namespace raven_control::dynamics
