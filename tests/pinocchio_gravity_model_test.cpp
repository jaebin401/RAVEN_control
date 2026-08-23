#include "raven_control/dynamics/gravity_compensator.hpp"
#include "raven_control/dynamics/pinocchio_gravity_model.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

constexpr double TOLERANCE_NM = 1e-9;

int failures = 0;

void check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

void compareModelsAtPoses(const std::string& urdf_path)
{
    const raven_control::dynamics::PinocchioGravityModel pinocchio_model(
        urdf_path);
    const raven_control::dynamics::GravityCompensator reference_model(
        raven_control::dynamics::makeRavenUrdfGravityModel());

    const std::array<raven_control::dynamics::JointVector, 8> poses{{
        {0.0, 0.0, 0.0},
        {0.5, 0.0, 0.0},
        {-1.2, 0.4, -0.7},
        {1.8, 0.4, -0.7},
        {0.0, 0.9, 0.2},
        {0.0, -0.4, 1.1},
        {2.1, 1.2, -1.0},
        {-2.3, -0.8, 0.6},
    }};

    for (std::size_t pose_index = 0;
         pose_index < poses.size();
         ++pose_index) {
        const auto pinocchio_torque =
            pinocchio_model.compute(poses[pose_index]);
        const auto reference_torque =
            reference_model.compute(poses[pose_index]);
        for (std::size_t joint_index = 0;
             joint_index < pinocchio_torque.size();
             ++joint_index) {
            check(
                std::abs(
                    pinocchio_torque[joint_index] -
                    reference_torque[joint_index]) < TOLERANCE_NM,
                "Pinocchio and analytic gravity torque differ at pose " +
                    std::to_string(pose_index) + ", joint " +
                    std::to_string(joint_index));
        }
    }
}

void testJointMetadata(const std::string& urdf_path)
{
    const raven_control::dynamics::PinocchioGravityModel model(urdf_path);
    check(model.urdfPath() == urdf_path,
          "gravity model must retain its URDF path");
    check(model.jointNames() ==
              raven_control::dynamics::RAVEN_URDF_JOINT_NAMES,
          "gravity model must retain the validated RAVEN joint order");
}

void testInvalidStateIsRejected(const std::string& urdf_path)
{
    const raven_control::dynamics::PinocchioGravityModel model(urdf_path);
    bool rejected = false;
    try {
        (void)model.compute({
            0.0,
            std::numeric_limits<double>::quiet_NaN(),
            0.0,
        });
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    check(rejected, "non-finite joint position must be rejected");
}

void testMissingJointIsRejected(const std::string& urdf_path)
{
    auto joint_names = raven_control::dynamics::RAVEN_URDF_JOINT_NAMES;
    joint_names[1] = "missing_joint";

    bool rejected = false;
    try {
        const raven_control::dynamics::PinocchioGravityModel model(
            urdf_path, joint_names);
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    check(rejected, "missing required URDF joint must be rejected");
}

void testMissingUrdfIsRejected()
{
    bool rejected = false;
    try {
        const raven_control::dynamics::PinocchioGravityModel model(
            "/path/that/does/not/exist/RAVEN.urdf");
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    check(rejected, "missing URDF file must be rejected");
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: pinocchio_gravity_model_test <urdf_path>\n";
        return 2;
    }

    try {
        compareModelsAtPoses(argv[1]);
        testJointMetadata(argv[1]);
        testInvalidStateIsRejected(argv[1]);
        testMissingJointIsRejected(argv[1]);
        testMissingUrdfIsRejected();
    } catch (const std::exception& error) {
        std::cerr << "Unexpected exception: " << error.what() << '\n';
        return 1;
    }

    if (failures != 0) {
        std::cerr << failures
                  << " Pinocchio gravity model test(s) failed\n";
        return 1;
    }
    std::cout << "All Pinocchio gravity model tests passed\n";
    return 0;
}
