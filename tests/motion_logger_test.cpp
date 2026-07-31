#include "raven_control/logging/motion_logger.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

void check(bool condition, const std::string& message)
{
    if (!condition)
        throw std::runtime_error(message);
}

std::string readFile(const std::filesystem::path& path)
{
    std::ifstream input(path);
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
}

}  // namespace

int main()
{
    const auto origin = std::chrono::steady_clock::now();
    raven_control::logging::MotionLogger logger(
        {
            "shoulder_Joint",
            "upperArm_Joint",
            "foreArm_Joint",
        },
        2);

    raven_control::logging::JointMotionSample joint;
    joint.command_position_rad = 0.2;
    joint.actual_position_rad = 0.1;
    joint.trajectory_velocity_rad_s = 0.3;
    joint.sent_velocity_rad_s = 0.0;
    joint.actual_velocity_rad_s = 0.25;
    joint.position_error_rad = 0.1;
    joint.estimated_p_torque_nm = 2.0;
    joint.estimated_d_torque_nm = -0.25;
    joint.sent_feedforward_torque_nm = 0.0;
    joint.estimated_control_torque_nm = 1.75;
    joint.feedback_age_ms = 3.0;
    joint.feedback_valid = true;
    joint.kp = 20.0;
    joint.kd = 1.0;

    raven_control::logging::MotionSample first;
    first.cycle_index = 0;
    first.phase = "Pose A";
    first.scheduled_at = origin;
    first.actual_at = origin + std::chrono::microseconds(500);
    first.control_period = std::chrono::microseconds(20000);
    first.joints[0] = joint;
    first.joints[1] = joint;
    first.joints[2] = joint;
    logger.record(first);

    raven_control::logging::MotionSample second = first;
    second.cycle_index = 1;
    second.scheduled_at = origin + std::chrono::microseconds(20000);
    second.actual_at = origin + std::chrono::microseconds(41000);
    logger.record(second);

    check(logger.sampleCount() == 2, "logger must retain samples");

    const auto path =
        std::filesystem::temp_directory_path() /
        "raven_motion_logger_test.csv";
    logger.saveCsv(path.string());
    const std::string csv = readFile(path);
    std::filesystem::remove(path);

    check(
        csv.find("shoulder_Joint.command_position_rad") !=
            std::string::npos,
        "CSV must include joint command columns");
    check(
        csv.find("0,Pose A,0,500,0,500,0") !=
            std::string::npos,
        "first sample timing must be serialized");
    check(
        csv.find("1,Pose A,20000,41000,40500,21000,1") !=
            std::string::npos,
        "deadline miss must be serialized");

    std::cout << "motion_logger_test passed\n";
    return 0;
}
