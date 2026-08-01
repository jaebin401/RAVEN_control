#include "raven_control/dynamics/gravity_compensator.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace raven_control::dynamics {
namespace {

using Matrix3 = std::array<std::array<double, 3>, 3>;

bool isFinite(const Vector3& value)
{
    return std::isfinite(value[0]) &&
           std::isfinite(value[1]) &&
           std::isfinite(value[2]);
}

Vector3 add(const Vector3& lhs, const Vector3& rhs)
{
    return {
        lhs[0] + rhs[0],
        lhs[1] + rhs[1],
        lhs[2] + rhs[2],
    };
}

Vector3 subtract(const Vector3& lhs, const Vector3& rhs)
{
    return {
        lhs[0] - rhs[0],
        lhs[1] - rhs[1],
        lhs[2] - rhs[2],
    };
}

Vector3 scale(const Vector3& value, double factor)
{
    return {
        value[0] * factor,
        value[1] * factor,
        value[2] * factor,
    };
}

double dot(const Vector3& lhs, const Vector3& rhs)
{
    return lhs[0] * rhs[0] +
           lhs[1] * rhs[1] +
           lhs[2] * rhs[2];
}

Vector3 cross(const Vector3& lhs, const Vector3& rhs)
{
    return {
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    };
}

double norm(const Vector3& value)
{
    return std::sqrt(dot(value, value));
}

Vector3 normalized(const Vector3& value)
{
    return scale(value, 1.0 / norm(value));
}

Matrix3 identity()
{
    return {{{1.0, 0.0, 0.0},
             {0.0, 1.0, 0.0},
             {0.0, 0.0, 1.0}}};
}

Matrix3 multiply(const Matrix3& lhs, const Matrix3& rhs)
{
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            for (std::size_t index = 0; index < 3; ++index) {
                result[row][column] +=
                    lhs[row][index] * rhs[index][column];
            }
        }
    }
    return result;
}

Vector3 multiply(const Matrix3& matrix, const Vector3& vector)
{
    return {
        dot(matrix[0], vector),
        dot(matrix[1], vector),
        dot(matrix[2], vector),
    };
}

Matrix3 axisAngle(const Vector3& axis, double angle)
{
    const Vector3 unit = normalized(axis);
    const double x = unit[0];
    const double y = unit[1];
    const double z = unit[2];
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double one_minus_cosine = 1.0 - cosine;

    return {{
        {cosine + x * x * one_minus_cosine,
         x * y * one_minus_cosine - z * sine,
         x * z * one_minus_cosine + y * sine},
        {y * x * one_minus_cosine + z * sine,
         cosine + y * y * one_minus_cosine,
         y * z * one_minus_cosine - x * sine},
        {z * x * one_minus_cosine - y * sine,
         z * y * one_minus_cosine + x * sine,
         cosine + z * z * one_minus_cosine},
    }};
}

Matrix3 rotationFromRpy(const Vector3& rpy)
{
    const Matrix3 roll = axisAngle({1.0, 0.0, 0.0}, rpy[0]);
    const Matrix3 pitch = axisAngle({0.0, 1.0, 0.0}, rpy[1]);
    const Matrix3 yaw = axisAngle({0.0, 0.0, 1.0}, rpy[2]);
    return multiply(yaw, multiply(pitch, roll));
}

}  // namespace

void GravityModelParameters::validate() const
{
    if (!isFinite(gravity_base_m_s2))
        throw std::invalid_argument("Gravity vector must be finite");

    for (const auto& link : links) {
        if (!isFinite(link.joint_origin_parent_m) ||
            !isFinite(link.joint_origin_rpy_rad) ||
            !isFinite(link.joint_axis) ||
            !isFinite(link.center_of_mass_child_m) ||
            !std::isfinite(link.mass_kg)) {
            throw std::invalid_argument(
                "Gravity model parameters must be finite");
        }
        if (link.mass_kg <= 0.0)
            throw std::invalid_argument("Link mass must be positive");
        if (norm(link.joint_axis) <= 0.0)
            throw std::invalid_argument("Joint axis must be non-zero");
    }
}

GravityCompensator::GravityCompensator(
    GravityModelParameters parameters)
    : parameters_(std::move(parameters))
{
    parameters_.validate();
}

JointVector GravityCompensator::compute(
    const JointVector& joint_positions_rad) const
{
    for (double position : joint_positions_rad) {
        if (!std::isfinite(position)) {
            throw std::invalid_argument(
                "Joint positions must be finite");
        }
    }

    JointVector compensation_torque_nm{};
    std::array<Vector3, RAVEN_JOINT_COUNT> joint_origins_base{};
    std::array<Vector3, RAVEN_JOINT_COUNT> joint_axes_base{};

    Vector3 parent_origin_base{};
    Matrix3 parent_rotation_base = identity();

    for (std::size_t link_index = 0;
         link_index < RAVEN_JOINT_COUNT;
         ++link_index) {
        const auto& link = parameters_.links[link_index];
        const Vector3 joint_origin_base = add(
            parent_origin_base,
            multiply(
                parent_rotation_base,
                link.joint_origin_parent_m));
        const Matrix3 joint_rotation_base = multiply(
            parent_rotation_base,
            rotationFromRpy(link.joint_origin_rpy_rad));
        const Vector3 axis_base = multiply(
            joint_rotation_base,
            normalized(link.joint_axis));
        const Matrix3 child_rotation_base = multiply(
            joint_rotation_base,
            axisAngle(link.joint_axis, joint_positions_rad[link_index]));
        const Vector3 center_of_mass_base = add(
            joint_origin_base,
            multiply(
                child_rotation_base,
                link.center_of_mass_child_m));
        const Vector3 gravity_force = scale(
            parameters_.gravity_base_m_s2,
            link.mass_kg);

        joint_origins_base[link_index] = joint_origin_base;
        joint_axes_base[link_index] = axis_base;

        for (std::size_t joint_index = 0;
             joint_index <= link_index;
             ++joint_index) {
            const Vector3 lever_arm = subtract(
                center_of_mass_base,
                joint_origins_base[joint_index]);
            const double gravity_generalized_torque = dot(
                joint_axes_base[joint_index],
                cross(lever_arm, gravity_force));
            compensation_torque_nm[joint_index] -=
                gravity_generalized_torque;
        }

        parent_origin_base = joint_origin_base;
        parent_rotation_base = child_rotation_base;
    }

    return compensation_torque_nm;
}

const GravityModelParameters& GravityCompensator::parameters() const noexcept
{
    return parameters_;
}

GravityModelParameters makeRavenUrdfGravityModel()
{
    GravityModelParameters parameters;
    parameters.gravity_base_m_s2 = {0.0, 0.0, -9.81};
    parameters.links = {{
        {
            {0.0, 0.0, 0.0504},
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 1.0},
            0.598911,
            {0.000083, 0.001708, 0.057466},
        },
        {
            {0.0, 0.027, 0.07},
            {0.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            0.718905,
            {-0.180709, -0.02605, -0.000062},
        },
        {
            {-0.225, -0.000025, -0.000075},
            {0.0, 0.0, 0.0},
            {0.0, -1.0, 0.0},
            0.14102,
            {0.065872, -0.02672, 0.051838},
        },
    }};
    return parameters;
}

}  // namespace raven_control::dynamics
