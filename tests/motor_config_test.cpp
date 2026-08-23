#include "raven_control/config/motor_config.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#ifndef RAVEN_HAS_YAML_CPP
#define RAVEN_HAS_YAML_CPP 0
#endif

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
    check(!config.gravity_compensation.enabled,
          "gravity compensation must default to disabled");
    check(config.gravity_compensation.dry_run,
          "gravity compensation must default to dry-run");
    check(config.gravity_compensation.scale == 0.0,
          "gravity compensation must default to zero scale");
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
    checkInvalid(
        [](auto& config) {
            config.gravity_compensation.scale = 1.1;
        },
        "gravity scale above one must be rejected");
    checkInvalid(
        [](auto& config) {
            config.gravity_compensation.ramp_duration =
                std::chrono::milliseconds(0);
        },
        "zero gravity ramp duration must be rejected");
    checkInvalid(
        [](auto& config) {
            config.gravity_compensation.max_joint_torque_nm[0] = -0.1;
        },
        "negative gravity torque limits must be rejected");
}

void testYamlRoundTrip(const std::string& example_path)
{
#if RAVEN_HAS_YAML_CPP
    auto config =
        raven_control::config::loadMotorRuntimeConfig(example_path);
    check(!config.gravity_compensation.enabled,
          "example config must keep gravity disabled");
    check(config.gravity_compensation.dry_run,
          "example config must keep gravity in dry-run");
    check(config.gravity_compensation.scale == 0.0,
          "example config must keep gravity scale at zero");

    config.gravity_compensation.enabled = true;
    config.gravity_compensation.scale = 0.2;
    config.gravity_compensation.max_joint_torque_nm = {0.1, 0.2, 0.3};
    const auto round_trip_path =
        std::filesystem::temp_directory_path() /
        "raven_motor_config_round_trip.yaml";
    raven_control::config::saveMotorRuntimeConfig(
        config, round_trip_path.string());
    const auto loaded = raven_control::config::loadMotorRuntimeConfig(
        round_trip_path.string());
    std::filesystem::remove(round_trip_path);

    check(loaded.gravity_compensation.enabled,
          "saved gravity enabled state must round-trip");
    check(loaded.gravity_compensation.dry_run,
          "saved gravity dry-run state must round-trip");
    check(loaded.gravity_compensation.scale == 0.2,
          "saved gravity scale must round-trip");
    check(loaded.gravity_compensation.max_joint_torque_nm[2] == 0.3,
          "saved gravity torque limits must round-trip");
#else
    (void)example_path;
#endif
}

}  // namespace

int main(int argc, char* argv[])
{
    testValidConfig();
    testValidation();
    if (argc > 1)
        testYamlRoundTrip(argv[1]);

    if (failures != 0) {
        std::cerr << failures << " motor config test(s) failed\n";
        return 1;
    }
    std::cout << "All motor config tests passed\n";
    return 0;
}
