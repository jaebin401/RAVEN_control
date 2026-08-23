#pragma once

#include <array>
#include <cstddef>

namespace raven_control::dynamics {

inline constexpr std::size_t RAVEN_JOINT_COUNT = 3;

using Vector3 = std::array<double, 3>;
using JointVector = std::array<double, RAVEN_JOINT_COUNT>;

class GravityModel {
public:
    virtual ~GravityModel() = default;

    // Returns joint-coordinate torque that balances gravity at q.
    [[nodiscard]] virtual JointVector compute(
        const JointVector& joint_positions_rad) const = 0;
};

}  // namespace raven_control::dynamics
