#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace raven_control::logging {

inline constexpr std::size_t MOTION_LOG_JOINT_COUNT = 3;

struct JointMotionSample {
    double command_position_rad = 0.0;
    double actual_position_rad = 0.0;
    double trajectory_velocity_rad_s = 0.0;
    double sent_velocity_rad_s = 0.0;
    double actual_velocity_rad_s = 0.0;
    double position_error_rad = 0.0;
    double estimated_p_torque_nm = 0.0;
    double estimated_d_torque_nm = 0.0;
    double sent_feedforward_torque_nm = 0.0;
    double estimated_control_torque_nm = 0.0;
    double measured_torque_nm = 0.0;
    double motor_temperature_celsius = 0.0;
    double feedback_age_ms = 0.0;
    double operation_feedback_age_ms = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    std::uint8_t motor_fault_flags = 0;
    std::uint8_t motor_mode_state = 0;
    bool feedback_valid = false;
    bool operation_feedback_valid = false;
};

struct MotionSample {
    std::uint64_t cycle_index = 0;
    std::string phase;
    std::chrono::steady_clock::time_point scheduled_at{};
    std::chrono::steady_clock::time_point actual_at{};
    std::chrono::microseconds control_period{0};
    std::array<JointMotionSample, MOTION_LOG_JOINT_COUNT> joints{};
};

class MotionLogger {
public:
    explicit MotionLogger(
        std::array<std::string, MOTION_LOG_JOINT_COUNT> joint_names,
        std::size_t reserve_samples = 0);

    void record(MotionSample sample);
    void saveCsv(const std::string& path) const;

    [[nodiscard]] std::size_t sampleCount() const noexcept;
    [[nodiscard]] const std::array<
        std::string,
        MOTION_LOG_JOINT_COUNT>& jointNames()
        const noexcept;

private:
    std::array<std::string, MOTION_LOG_JOINT_COUNT> joint_names_;
    std::vector<MotionSample> samples_;
};

[[nodiscard]] std::string makeTimestampedLogPath(
    const std::string& directory,
    const std::string& prefix);

}  // namespace raven_control::logging
