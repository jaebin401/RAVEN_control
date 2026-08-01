#include "raven_control/config/motor_config.hpp"

#include <iostream>
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

raven_control::config::MotorRuntimeConfig validConfig()
{
    raven_control::config::MotorRuntimeConfig config;
    config.control_period = std::chrono::milliseconds(20);
    config.feedback_timeout = std::chrono::milliseconds(250);
    config.position_request_period =
        std::chrono::milliseconds(100);
    config.joints = {
        {
            "joint_1",
            1,
            1,
            0.0,
            1.0,
            {8.0, 0.15, 0.524},
        },
        {
            "joint_2",
            2,
            -1,
            0.5,
            2.0,
            {12.0, 0.2, 0.524},
        },
    };
    return config;
}

template <typename Modify>
void checkInvalid(
    Modify modify,
    const std::string& message)
{
    auto config = validConfig();
    modify(config);

    bool threw = false;
    try {
        config.validate();
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    check(threw, message);
}

void testValidConfig()
{
    const auto config = validConfig();
    config.validate();
    check(config.gravity_compensation_enabled,
          "gravity compensation must default to enabled");
    check(config.findJoint("joint_2") != nullptr,
          "configured joint must be found");
    check(config.findJoint("missing") == nullptr,
          "unknown joint must not be found");
}

void testValidation()
{
    checkInvalid(
        [](auto& config) {
            config.joints[1].motor_id =
                config.joints[0].motor_id;
        },
        "duplicate motor IDs must be rejected");
    checkInvalid(
        [](auto& config) {
            config.joints[0].position_sign = 0;
        },
        "position sign other than -1 or 1 must be rejected");
    checkInvalid(
        [](auto& config) {
            config.joints[0].position_control.kp = 500.1;
        },
        "Kp above the RS02 protocol range must be rejected");
    checkInvalid(
        [](auto& config) {
            config.joints[0].position_control.kd = 5.1;
        },
        "Kd above the RS02 protocol range must be rejected");
    checkInvalid(
        [](auto& config) {
            config.position_request_period =
                config.feedback_timeout;
        },
        "feedback requests must be faster than the timeout");
    checkInvalid(
        [](auto& config) {
            config.control_period =
                config.position_request_period +
                std::chrono::milliseconds(1);
        },
        "control period must not exceed the feedback request period");
}

}  // namespace

int main()
{
    testValidConfig();
    testValidation();

    if (failures != 0) {
        std::cerr << failures << " motor config test(s) failed\n";
        return 1;
    }
    std::cout << "All motor config tests passed\n";
    return 0;
}
