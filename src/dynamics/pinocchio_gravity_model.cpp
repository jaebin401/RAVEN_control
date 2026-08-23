#include "pinocchio/fwd.hpp"

#include "raven_control/dynamics/pinocchio_gravity_model.hpp"

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <Eigen/Core>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <system_error>
#include <unordered_set>
#include <utility>

namespace raven_control::dynamics {
namespace {

struct LoadedModel {
    pinocchio::Model model;
    std::array<Eigen::Index, RAVEN_JOINT_COUNT> q_indices{};
    std::array<Eigen::Index, RAVEN_JOINT_COUNT> v_indices{};
};

void validateFinite(const Vector3& value, const char* field_name)
{
    for (double element : value) {
        if (!std::isfinite(element)) {
            throw std::invalid_argument(
                std::string(field_name) + " must be finite");
        }
    }
}

LoadedModel loadModel(
    const std::string& urdf_path,
    const std::array<std::string, RAVEN_JOINT_COUNT>& joint_names,
    const Vector3& gravity_base_m_s2)
{
    if (urdf_path.empty())
        throw std::invalid_argument("URDF path must not be empty");

    std::error_code filesystem_error;
    if (!std::filesystem::is_regular_file(urdf_path, filesystem_error)) {
        throw std::runtime_error(
            "URDF file does not exist or is not a regular file: " +
            urdf_path);
    }
    validateFinite(gravity_base_m_s2, "Gravity vector");

    LoadedModel loaded;
    try {
        // RAVEN is a fixed-base manipulator, so no free-flyer root joint is
        // added here.
        pinocchio::urdf::buildModel(urdf_path, loaded.model);
    } catch (const std::exception& error) {
        throw std::runtime_error(
            "Failed to build Pinocchio model from URDF '" + urdf_path +
            "': " + error.what());
    }

    loaded.model.gravity.linear() = Eigen::Vector3d(
        gravity_base_m_s2[0],
        gravity_base_m_s2[1],
        gravity_base_m_s2[2]);
    loaded.model.gravity.angular().setZero();

    std::unordered_set<std::string> unique_names;
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
        const std::string& joint_name = joint_names[index];
        if (joint_name.empty())
            throw std::invalid_argument("URDF joint name must not be empty");
        if (!unique_names.insert(joint_name).second) {
            throw std::invalid_argument(
                "Duplicate URDF joint name: " + joint_name);
        }

        if (!loaded.model.existJointName(joint_name)) {
            throw std::runtime_error(
                "Required joint '" + joint_name +
                "' was not found in URDF '" + urdf_path + "'");
        }
        const pinocchio::JointIndex joint_id =
            loaded.model.getJointId(joint_name);

        const auto& joint = loaded.model.joints[joint_id];
        if (joint.nq() != 1 || joint.nv() != 1) {
            throw std::runtime_error(
                "Joint '" + joint_name +
                "' must have exactly one configuration and velocity DOF");
        }

        loaded.q_indices[index] =
            static_cast<Eigen::Index>(joint.idx_q());
        loaded.v_indices[index] =
            static_cast<Eigen::Index>(joint.idx_v());
    }

    if (loaded.model.nq != static_cast<int>(RAVEN_JOINT_COUNT) ||
        loaded.model.nv != static_cast<int>(RAVEN_JOINT_COUNT)) {
        throw std::runtime_error(
            "RAVEN gravity model expects exactly " +
            std::to_string(RAVEN_JOINT_COUNT) +
            " configuration and velocity DOFs, but URDF provides nq=" +
            std::to_string(loaded.model.nq) + " and nv=" +
            std::to_string(loaded.model.nv));
    }

    return loaded;
}

}  // namespace

class PinocchioGravityModel::Impl {
public:
    Impl(
        std::string urdf_path,
        std::array<std::string, RAVEN_JOINT_COUNT> joint_names,
        const Vector3& gravity_base_m_s2)
        : urdf_path_(std::move(urdf_path)),
          joint_names_(std::move(joint_names)),
          loaded_(loadModel(
              urdf_path_, joint_names_, gravity_base_m_s2)),
          data_(loaded_.model),
          configuration_(Eigen::VectorXd::Zero(loaded_.model.nq))
    {
    }

    JointVector compute(const JointVector& joint_positions_rad) const
    {
        for (std::size_t index = 0;
             index < joint_positions_rad.size();
             ++index) {
            if (!std::isfinite(joint_positions_rad[index])) {
                throw std::invalid_argument(
                    "Joint positions must be finite");
            }
            configuration_[loaded_.q_indices[index]] =
                joint_positions_rad[index];
        }

        const auto& gravity_torque =
            pinocchio::computeGeneralizedGravity(
                loaded_.model,
                data_,
                configuration_);

        JointVector result{};
        for (std::size_t index = 0; index < result.size(); ++index)
            result[index] = gravity_torque[loaded_.v_indices[index]];
        return result;
    }

    std::string urdf_path_;
    std::array<std::string, RAVEN_JOINT_COUNT> joint_names_;
    LoadedModel loaded_;
    mutable pinocchio::Data data_;
    mutable Eigen::VectorXd configuration_;
};

PinocchioGravityModel::PinocchioGravityModel(
    std::string urdf_path,
    std::array<std::string, RAVEN_JOINT_COUNT> joint_names,
    Vector3 gravity_base_m_s2)
    : impl_(std::make_unique<Impl>(
          std::move(urdf_path),
          std::move(joint_names),
          gravity_base_m_s2))
{
}

PinocchioGravityModel::~PinocchioGravityModel() = default;

PinocchioGravityModel::PinocchioGravityModel(
    PinocchioGravityModel&&) noexcept = default;

PinocchioGravityModel& PinocchioGravityModel::operator=(
    PinocchioGravityModel&&) noexcept = default;

JointVector PinocchioGravityModel::compute(
    const JointVector& joint_positions_rad) const
{
    return impl_->compute(joint_positions_rad);
}

const std::string& PinocchioGravityModel::urdfPath() const noexcept
{
    return impl_->urdf_path_;
}

const std::array<std::string, RAVEN_JOINT_COUNT>&
PinocchioGravityModel::jointNames() const noexcept
{
    return impl_->joint_names_;
}

}  // namespace raven_control::dynamics
