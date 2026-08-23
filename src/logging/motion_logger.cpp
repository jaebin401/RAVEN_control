#include "raven_control/logging/motion_logger.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace raven_control::logging {
namespace {

std::string csvEscape(const std::string& value)
{
    if (value.find_first_of(",\"\n\r") == std::string::npos)
        return value;

    std::string escaped = "\"";
    for (const char character : value) {
        if (character == '\"')
            escaped += '\"';
        escaped += character;
    }
    escaped += '\"';
    return escaped;
}

void writeNumber(std::ostream& output, double value)
{
    if (std::isfinite(value))
        output << value;
    else
        output << "nan";
}

}  // namespace

MotionLogger::MotionLogger(
    std::array<std::string, MOTION_LOG_JOINT_COUNT> joint_names,
    std::size_t reserve_samples)
    : joint_names_(std::move(joint_names))
{
    for (const std::string& name : joint_names_) {
        if (name.empty()) {
            throw std::invalid_argument(
                "MotionLogger joint names must not be empty");
        }
    }
    samples_.reserve(reserve_samples);
}

void MotionLogger::record(MotionSample sample)
{
    if (sample.control_period.count() <= 0) {
        throw std::invalid_argument(
            "Motion sample control period must be positive");
    }
    samples_.push_back(std::move(sample));
}

void MotionLogger::saveCsv(const std::string& path) const
{
    if (path.empty())
        throw std::invalid_argument("Motion log path must not be empty");

    const std::filesystem::path output_path(path);
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(
            output_path.parent_path());
    }

    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error(
            "Cannot open motion log '" + path + "'");
    }

    output
        << "cycle_index,phase,scheduled_time_us,actual_time_us,"
           "dt_us,lateness_us,deadline_missed";
    for (const std::string& joint_name : joint_names_) {
        const std::string prefix = csvEscape(joint_name);
        output
            << ',' << prefix << ".command_position_rad"
            << ',' << prefix << ".actual_position_rad"
            << ',' << prefix << ".trajectory_velocity_rad_s"
            << ',' << prefix << ".sent_velocity_rad_s"
            << ',' << prefix << ".actual_velocity_rad_s"
            << ',' << prefix << ".position_error_rad"
            << ',' << prefix << ".estimated_p_torque_nm"
            << ',' << prefix << ".estimated_d_torque_nm"
            << ',' << prefix << ".raw_gravity_torque_nm"
            << ',' << prefix << ".ramped_gravity_torque_nm"
            << ',' << prefix << ".limited_gravity_torque_nm"
            << ',' << prefix << ".gravity_scale"
            << ',' << prefix << ".gravity_ramp_factor"
            << ',' << prefix << ".sent_feedforward_torque_nm"
            << ',' << prefix << ".estimated_control_torque_nm"
            << ',' << prefix << ".measured_torque_nm"
            << ',' << prefix << ".motor_temperature_celsius"
            << ',' << prefix << ".bus_voltage_v"
            << ',' << prefix << ".feedback_age_ms"
            << ',' << prefix << ".operation_feedback_age_ms"
            << ',' << prefix << ".bus_voltage_age_ms"
            << ',' << prefix << ".feedback_valid"
            << ',' << prefix << ".operation_feedback_valid"
            << ',' << prefix << ".bus_voltage_valid"
            << ',' << prefix << ".gravity_enabled"
            << ',' << prefix << ".gravity_dry_run"
            << ',' << prefix << ".gravity_input_valid"
            << ',' << prefix << ".gravity_torque_clamped"
            << ',' << prefix << ".motor_fault_flags"
            << ',' << prefix << ".motor_mode_state"
            << ',' << prefix << ".kp"
            << ',' << prefix << ".kd";
    }
    output << '\n';

    if (samples_.empty())
        return;

    const auto origin = samples_.front().scheduled_at;
    std::chrono::steady_clock::time_point previous_actual{};
    output << std::setprecision(12);

    for (std::size_t index = 0; index < samples_.size(); ++index) {
        const MotionSample& sample = samples_[index];
        const auto scheduled_us =
            std::chrono::duration<double, std::micro>(
                sample.scheduled_at - origin).count();
        const auto actual_us =
            std::chrono::duration<double, std::micro>(
                sample.actual_at - origin).count();
        const double dt_us = index == 0
            ? 0.0
            : std::chrono::duration<double, std::micro>(
                  sample.actual_at - previous_actual).count();
        const double lateness_us =
            std::chrono::duration<double, std::micro>(
                sample.actual_at - sample.scheduled_at).count();
        const bool deadline_missed =
            sample.actual_at - sample.scheduled_at >
            sample.control_period;

        output
            << sample.cycle_index << ','
            << csvEscape(sample.phase) << ','
            << scheduled_us << ','
            << actual_us << ','
            << dt_us << ','
            << lateness_us << ','
            << (deadline_missed ? 1 : 0);

        for (const JointMotionSample& joint : sample.joints) {
            output << ',';
            writeNumber(output, joint.command_position_rad);
            output << ',';
            writeNumber(output, joint.actual_position_rad);
            output << ',';
            writeNumber(output, joint.trajectory_velocity_rad_s);
            output << ',';
            writeNumber(output, joint.sent_velocity_rad_s);
            output << ',';
            writeNumber(output, joint.actual_velocity_rad_s);
            output << ',';
            writeNumber(output, joint.position_error_rad);
            output << ',';
            writeNumber(output, joint.estimated_p_torque_nm);
            output << ',';
            writeNumber(output, joint.estimated_d_torque_nm);
            output << ',';
            writeNumber(output, joint.raw_gravity_torque_nm);
            output << ',';
            writeNumber(output, joint.ramped_gravity_torque_nm);
            output << ',';
            writeNumber(output, joint.limited_gravity_torque_nm);
            output << ',';
            writeNumber(output, joint.gravity_scale);
            output << ',';
            writeNumber(output, joint.gravity_ramp_factor);
            output << ',';
            writeNumber(output, joint.sent_feedforward_torque_nm);
            output << ',';
            writeNumber(output, joint.estimated_control_torque_nm);
            output << ',';
            writeNumber(output, joint.measured_torque_nm);
            output << ',';
            writeNumber(output, joint.motor_temperature_celsius);
            output << ',';
            writeNumber(output, joint.bus_voltage_v);
            output << ',';
            writeNumber(output, joint.feedback_age_ms);
            output << ',';
            writeNumber(output, joint.operation_feedback_age_ms);
            output << ',';
            writeNumber(output, joint.bus_voltage_age_ms);
            output << ',' << (joint.feedback_valid ? 1 : 0)
                   << ',' << (joint.operation_feedback_valid ? 1 : 0)
                   << ',' << (joint.bus_voltage_valid ? 1 : 0)
                   << ',' << (joint.gravity_enabled ? 1 : 0)
                   << ',' << (joint.gravity_dry_run ? 1 : 0)
                   << ',' << (joint.gravity_input_valid ? 1 : 0)
                   << ',' << (joint.gravity_torque_clamped ? 1 : 0)
                   << ',' << static_cast<unsigned int>(
                          joint.motor_fault_flags)
                   << ',' << static_cast<unsigned int>(
                          joint.motor_mode_state)
                   << ',';
            writeNumber(output, joint.kp);
            output << ',';
            writeNumber(output, joint.kd);
        }
        output << '\n';
        previous_actual = sample.actual_at;
    }

    if (!output) {
        throw std::runtime_error(
            "Failed while writing motion log '" + path + "'");
    }
}

std::size_t MotionLogger::sampleCount() const noexcept
{
    return samples_.size();
}

const std::array<std::string, MOTION_LOG_JOINT_COUNT>&
MotionLogger::jointNames() const noexcept
{
    return joint_names_;
}

std::string makeTimestampedLogPath(
    const std::string& directory,
    const std::string& prefix)
{
    const std::time_t now = std::time(nullptr);
    std::tm local_time{};
#if defined(_WIN32)
    localtime_s(&local_time, &now);
#else
    localtime_r(&now, &local_time);
#endif

    std::ostringstream name;
    name
        << prefix << '_'
        << std::put_time(&local_time, "%Y%m%d_%H%M%S")
        << ".csv";
    return (std::filesystem::path(directory) / name.str()).string();
}

}  // namespace raven_control::logging
