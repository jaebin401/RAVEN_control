#include "raven_control/dynamics/gravity_compensator.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double TOLERANCE_NM = 1e-9;

int failures = 0;

void check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

void testHomePoseAgainstHandCalculation()
{
    const raven_control::dynamics::GravityCompensator compensator(
        raven_control::dynamics::makeRavenUrdfGravityModel());
    const auto torque = compensator.compute({0.0, 0.0, 0.0});

    check(std::abs(torque[0]) < TOLERANCE_NM,
          "vertical shoulder axis must need zero gravity torque");
    check(std::abs(torque[1] - 1.494581303551) < TOLERANCE_NM,
          "home upper-arm torque must match the hand calculation");
    check(std::abs(torque[2] - 0.091127733206) < TOLERANCE_NM,
          "home forearm torque must match the hand calculation");
}

void testShoulderYawDoesNotChangeGravityTorque()
{
    const raven_control::dynamics::GravityCompensator compensator(
        raven_control::dynamics::makeRavenUrdfGravityModel());
    const auto zero_yaw = compensator.compute({0.0, 0.4, -0.7});
    const auto turned = compensator.compute({1.8, 0.4, -0.7});

    for (std::size_t index = 0;
         index < raven_control::dynamics::RAVEN_JOINT_COUNT;
         ++index) {
        check(std::abs(zero_yaw[index] - turned[index]) < TOLERANCE_NM,
              "gravity torque must be invariant to shoulder yaw");
    }
}

void testForearmAxisDirectionIsRespected()
{
    const raven_control::dynamics::GravityCompensator compensator(
        raven_control::dynamics::makeRavenUrdfGravityModel());
    const auto positive = compensator.compute({0.0, 0.0, PI / 2.0});
    const auto negative = compensator.compute({0.0, 0.0, -PI / 2.0});

    check(positive[2] < 0.0,
          "positive forearm angle must follow the URDF -Y axis");
    check(negative[2] > 0.0,
          "negative forearm angle must follow the URDF -Y axis");
}

void testInvalidModelAndStateAreRejected()
{
    auto parameters =
        raven_control::dynamics::makeRavenUrdfGravityModel();
    parameters.links[1].mass_kg = 0.0;
    bool invalid_model_rejected = false;
    try {
        const raven_control::dynamics::GravityCompensator compensator(
            parameters);
    } catch (const std::invalid_argument&) {
        invalid_model_rejected = true;
    }
    check(invalid_model_rejected,
          "zero link mass must be rejected");

    const raven_control::dynamics::GravityCompensator compensator(
        raven_control::dynamics::makeRavenUrdfGravityModel());
    bool invalid_state_rejected = false;
    try {
        (void)compensator.compute({
            0.0,
            std::numeric_limits<double>::quiet_NaN(),
            0.0,
        });
    } catch (const std::invalid_argument&) {
        invalid_state_rejected = true;
    }
    check(invalid_state_rejected,
          "non-finite joint position must be rejected");
}

}  // namespace

int main()
{
    testHomePoseAgainstHandCalculation();
    testShoulderYawDoesNotChangeGravityTorque();
    testForearmAxisDirectionIsRespected();
    testInvalidModelAndStateAreRejected();

    if (failures != 0) {
        std::cerr << failures
                  << " gravity compensator test(s) failed\n";
        return 1;
    }
    std::cout << "All gravity compensator tests passed\n";
    return 0;
}
