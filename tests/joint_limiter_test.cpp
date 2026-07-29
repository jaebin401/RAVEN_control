#include "raven_control/safety/joint_limiter.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

bool near(double actual, double expected, double tolerance = 1e-12)
{
    return std::abs(actual - expected) <= tolerance;
}

raven_control::safety::JointLimitConfig validConfig()
{
    raven_control::safety::JointLimitConfig config;
    config.joint_name = "test_joint";
    config.confirmed = true;
    config.hard_min_rad = -1.0;
    config.hard_max_rad = 1.0;
    config.soft_margin_rad = 0.2;
    config.soft_wall_enabled = true;
    config.soft_wall_stiffness_nm_per_rad = 10.0;
    config.soft_wall_max_torque_nm = 3.0;
    return config;
}

void testTargetClamp()
{
    const raven_control::safety::JointLimiter limiter(validConfig());

    check(near(*limiter.clampTarget(0.25), 0.25),
          "target inside the soft range must pass unchanged");
    check(near(*limiter.clampTarget(-2.0), -0.8),
          "target below the soft range must clamp to soft minimum");
    check(near(*limiter.clampTarget(2.0), 0.8),
          "target above the soft range must clamp to soft maximum");
    check(!limiter.clampTarget(
              std::numeric_limits<double>::quiet_NaN()),
          "non-finite target must be rejected");
}

void testHardLimitDetection()
{
    const raven_control::safety::JointLimiter limiter(validConfig());

    check(!limiter.isHardViolation(-1.0),
          "hard minimum itself is allowed");
    check(!limiter.isHardViolation(1.0),
          "hard maximum itself is allowed");
    check(limiter.isHardViolation(-1.001),
          "position below hard minimum must violate");
    check(limiter.isHardViolation(1.001),
          "position above hard maximum must violate");
    check(limiter.isHardViolation(
              std::numeric_limits<double>::infinity()),
          "non-finite feedback must violate");
}

void testSoftWallTorque()
{
    const raven_control::safety::JointLimiter limiter(validConfig());

    check(near(limiter.softWallTorque(0.0), 0.0),
          "soft wall must be inactive inside the soft range");
    check(near(limiter.softWallTorque(-0.9), 1.0),
          "lower soft wall must push in the positive direction");
    check(near(limiter.softWallTorque(0.9), -1.0),
          "upper soft wall must push in the negative direction");
    check(near(limiter.softWallTorque(-2.0), 3.0),
          "lower soft wall torque must saturate");
    check(near(limiter.softWallTorque(2.0), -3.0),
          "upper soft wall torque must saturate");
}

void testValidation()
{
    auto config = validConfig();
    config.hard_min_rad = config.hard_max_rad;

    bool threw = false;
    try {
        const raven_control::safety::JointLimiter limiter(config);
        (void)limiter;
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    check(threw, "invalid hard-limit range must be rejected");

    config = validConfig();
    config.soft_margin_rad = 1.0;
    threw = false;
    try {
        const raven_control::safety::JointLimiter limiter(config);
        (void)limiter;
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    check(threw, "soft margins leaving no range must be rejected");
}

}  // namespace

int main()
{
    testTargetClamp();
    testHardLimitDetection();
    testSoftWallTorque();
    testValidation();

    if (failures != 0) {
        std::cerr << failures << " joint limiter test(s) failed\n";
        return 1;
    }
    std::cout << "All joint limiter tests passed\n";
    return 0;
}
