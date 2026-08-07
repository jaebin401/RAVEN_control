#include "raven_control/config/motor_config.hpp"
#include "raven_control/hal/can_interface.hpp"
#include "raven_control/hal/motor_driver.hpp"
#include "raven_control/safety/joint_limiter.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double STEP_DEG = 1.0;
constexpr double MAX_COMMAND_ACCELERATION_RAD_S2 = 3.0;
constexpr double POSITION_RESPONSE_GAIN_PER_S = 8.0;
constexpr double POSITION_SETTLE_TOLERANCE_RAD = 1.74532925199e-4;
constexpr double VELOCITY_SETTLE_TOLERANCE_RAD_S = 5.0e-3;

struct JointSpec {
    const char* name;
    char positive_key;
    char negative_key;
};

constexpr std::array<JointSpec, 3> JOINTS{{
    {"shoulder_Joint", 'w', 's'},
    {"upperArm_Joint", 'd', 'e'},
    {"foreArm_Joint", 'r', 'f'},
}};

struct ControlState {
    std::atomic<double> target_rad{0.0};
    std::atomic<double> setpoint_rad{0.0};
    std::atomic<double> setpoint_velocity_rad_s{0.0};
    std::atomic<double> kp{0.0};
    std::atomic<double> kd{0.0};
    double max_slew_rate_rad_s = 0.0;
};

struct SmoothedCommand {
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
};

using JointBindings = std::array<
    const raven_control::config::JointMotorRuntimeConfig*,
    JOINTS.size()>;

volatile std::sig_atomic_t stop_requested = 0;

double radiansToDegrees(double radians)
{
    return radians * 180.0 / PI;
}

double degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
}

double moveToward(
    double current,
    double target,
    double max_delta)
{
    return current + std::clamp(
        target - current,
        -max_delta,
        max_delta);
}

SmoothedCommand advanceSmoothedCommand(
    double current_position_rad,
    double current_velocity_rad_s,
    double target_position_rad,
    double max_velocity_rad_s,
    double control_period_seconds)
{
    const double position_error_rad =
        target_position_rad - current_position_rad;
    const double max_velocity_change_rad_s =
        MAX_COMMAND_ACCELERATION_RAD_S2 *
        control_period_seconds;

    if (std::abs(position_error_rad) <=
            POSITION_SETTLE_TOLERANCE_RAD &&
        std::abs(current_velocity_rad_s) <=
            VELOCITY_SETTLE_TOLERANCE_RAD_S) {
        return {target_position_rad, 0.0};
    }

    // Slow down continuously as the command approaches its target, while
    // retaining the configured maximum slew rate. Limiting the velocity
    // change on each cycle avoids the instantaneous velocity step that
    // occurred on every key press with the previous position-only limiter.
    const double desired_velocity_rad_s = std::clamp(
        POSITION_RESPONSE_GAIN_PER_S * position_error_rad,
        -max_velocity_rad_s,
        max_velocity_rad_s);
    const double next_velocity_rad_s = moveToward(
        current_velocity_rad_s,
        desired_velocity_rad_s,
        max_velocity_change_rad_s);
    const double next_position_rad =
        current_position_rad +
        0.5 * (current_velocity_rad_s + next_velocity_rad_s) *
            control_period_seconds;

    return {next_position_rad, next_velocity_rad_s};
}

void handleSignal(int)
{
    stop_requested = 1;
}

class TerminalMode {
public:
    TerminalMode()
    {
        if (::tcgetattr(STDIN_FILENO, &original_) != 0) {
            throw std::runtime_error(
                "Cannot read terminal settings: " +
                std::string(std::strerror(errno)));
        }
        enableRaw();
    }

    ~TerminalMode()
    {
        restore();
    }

    TerminalMode(const TerminalMode&) = delete;
    TerminalMode& operator=(const TerminalMode&) = delete;

    void enableRaw()
    {
        if (raw_enabled_)
            return;

        termios raw = original_;
        ::cfmakeraw(&raw);
        raw.c_lflag |= ISIG;
        raw.c_oflag |= OPOST;
        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            throw std::runtime_error(
                "Cannot enable terminal raw mode: " +
                std::string(std::strerror(errno)));
        }
        raw_enabled_ = true;
    }

    void restore() noexcept
    {
        if (raw_enabled_) {
            ::tcsetattr(STDIN_FILENO, TCSANOW, &original_);
            raw_enabled_ = false;
        }
    }

private:
    termios original_{};
    bool raw_enabled_ = false;
};

bool keyAvailable()
{
    timeval timeout{};
    fd_set descriptors;
    FD_ZERO(&descriptors);
    FD_SET(STDIN_FILENO, &descriptors);
    return ::select(
               STDIN_FILENO + 1,
               &descriptors,
               nullptr,
               nullptr,
               &timeout) > 0;
}

std::optional<char> readKey()
{
    char key = '\0';
    const ssize_t count = ::read(STDIN_FILENO, &key, sizeof(key));
    if (count == 1)
        return key;
    return std::nullopt;
}

JointBindings bindConfiguredJoints(
    const raven_control::config::MotorRuntimeConfig& config)
{
    if (config.joints.size() != JOINTS.size()) {
        throw std::runtime_error(
            "position_control_multi requires exactly three joints");
    }

    JointBindings bindings{};
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        bindings[index] = config.findJoint(JOINTS[index].name);
        if (bindings[index] == nullptr) {
            throw std::runtime_error(
                "Motor config is missing joint '" +
                std::string(JOINTS[index].name) + "'");
        }
    }
    return bindings;
}

std::vector<raven_control::hal::JointMotorConfig> motorMap(
    const JointBindings& bindings)
{
    std::vector<raven_control::hal::JointMotorConfig> result;
    result.reserve(JOINTS.size());
    for (const auto* joint : bindings) {
        result.push_back({
            joint->joint_name,
            joint->motor_id,
            joint->position_sign,
            joint->joint_zero_at_motor_rad,
            joint->joint_to_motor_ratio,
        });
    }
    return result;
}

void initializeControlStates(
    const JointBindings& bindings,
    std::array<ControlState, JOINTS.size()>& states)
{
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        states[index].kp.store(
            bindings[index]->position_control.kp);
        states[index].kd.store(
            bindings[index]->position_control.kd);
        states[index].max_slew_rate_rad_s =
            bindings[index]->
                position_control.max_slew_rate_rad_s;
    }
}

void printHelp(const JointBindings& bindings)
{
    std::cout
        << "========================================\n"
        << "RAVEN multi-joint debug controller\n"
        << "Space : enable/stop all motors\n"
        << "W/S   : shoulder +/-\n"
        << "E/D   : upper arm -/+\n"
        << "R/F   : forearm +/-\n"
        << "Step  : " << STEP_DEG << " deg per key press\n"
        << "Accel : " << MAX_COMMAND_ACCELERATION_RAD_S2
        << " rad/s^2 command limit\n"
        << "/     : enter three targets in degrees\n"
        << "Q     : quit\n"
        << "----------------------------------------\n";
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        std::cout
            << JOINTS[index].name
            << " ID=" << static_cast<int>(bindings[index]->motor_id)
            << " Kp=" << bindings[index]->position_control.kp
            << " Kd=" << bindings[index]->position_control.kd
            << " slew="
            << bindings[index]->
                position_control.max_slew_rate_rad_s
            << " rad/s\n";
    }
    std::cout << "========================================\n";
}

void printStatus(
    raven_control::hal::MotorDriver& driver,
    const JointBindings& bindings,
    const std::array<ControlState, JOINTS.size()>& states)
{
    std::cout << "\r\033[K[KEY] ";
    if (driver.feedbackHoldLatched())
        std::cout << "\033[33mFEEDBACK HOLD\033[0m | ";
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto feedback = driver.feedback(JOINTS[index].name);
        const double actual_deg =
            feedback && feedback->valid
            ? radiansToDegrees(feedback->position_rad)
            : 0.0;

        std::cout
            << "ID:"
            << static_cast<int>(bindings[index]->motor_id)
            << (driver.isEnabled()
                    ? " \033[32mON \033[0m"
                    : " \033[31mOFF\033[0m")
            << " T:" << std::fixed << std::setprecision(1)
            << std::setw(6)
            << radiansToDegrees(states[index].target_rad.load())
            << " A:" << std::setw(6) << actual_deg << " | ";
    }
    std::cout << std::flush;
}

void controlLoop(
    raven_control::hal::MotorDriver& driver,
    const raven_control::config::MotorRuntimeConfig& motor_config,
    const JointBindings& bindings,
    std::array<ControlState, JOINTS.size()>& states,
    std::atomic<bool>& running,
    std::atomic<bool>& monitor_active)
{
    auto next_cycle = std::chrono::steady_clock::now();
    auto next_position_request = next_cycle;
    auto next_status = next_cycle;
    const double control_period_seconds =
        std::chrono::duration<double>(
            motor_config.control_period).count();

    while (running.load() && !stop_requested) {
        next_cycle += motor_config.control_period;
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_position_request) {
            if (!driver.requestMechanicalPositions()) {
                (void)driver.stopAll();
                running.store(false);
                break;
            }
            do {
                next_position_request +=
                    motor_config.position_request_period;
            } while (next_position_request <= now);
        }
        (void)driver.poll();

        if (driver.faultLatched()) {
            running.store(false);
            break;
        }

        if (driver.isEnabled()) {
            for (std::size_t index = 0;
                 index < JOINTS.size();
                 ++index) {
                const double target =
                    states[index].target_rad.load();
                const double current =
                    states[index].setpoint_rad.load();
                const double current_velocity_rad_s =
                    states[index].setpoint_velocity_rad_s.load();
                const SmoothedCommand command =
                    advanceSmoothedCommand(
                        current,
                        current_velocity_rad_s,
                        target,
                        states[index].max_slew_rate_rad_s,
                        control_period_seconds);
                states[index].setpoint_rad.store(
                    command.position_rad);
                states[index].setpoint_velocity_rad_s.store(
                    command.velocity_rad_s);

                const auto result = driver.sendMitCommand(
                    JOINTS[index].name,
                    command.position_rad,
                    command.velocity_rad_s,
                    states[index].kp.load(),
                    states[index].kd.load(),
                    0.0);
                if (result == raven_control::hal::
                                  MotorCommandResult::NotEnabled &&
                    !driver.isEnabled()) {
                    // The user stopped the motors between the enabled check
                    // above and this command. This is a normal transition,
                    // not a controller failure.
                    break;
                }
                if (result !=
                        raven_control::hal::MotorCommandResult::Sent &&
                    result != raven_control::hal::
                        MotorCommandResult::TargetClamped &&
                    result != raven_control::hal::
                        MotorCommandResult::FeedbackHold) {
                    running.store(false);
                    break;
                }
            }
        } else {
            for (auto& state : states)
                state.setpoint_velocity_rad_s.store(0.0);
        }

        if (monitor_active.load() && now >= next_status) {
            printStatus(driver, bindings, states);
            next_status =
                now + std::chrono::milliseconds(100);
        }

        std::this_thread::sleep_until(next_cycle);
    }
}

bool initializeTargetsFromFeedback(
    raven_control::hal::MotorDriver& driver,
    std::array<ControlState, JOINTS.size()>& states,
    std::chrono::milliseconds feedback_timeout)
{
    if (!driver.allFeedbackValid())
        return false;

    const auto now = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto feedback = driver.feedback(JOINTS[index].name);
        if (!feedback ||
            !feedback->valid ||
            feedback->source != raven_control::hal::
                                    MotorFeedbackSource::
                                        MechanicalPosition ||
            feedback->received_at ==
                std::chrono::steady_clock::time_point{} ||
            now < feedback->received_at ||
            now - feedback->received_at > feedback_timeout) {
            return false;
        }
        states[index].setpoint_rad.store(feedback->position_rad);
        states[index].setpoint_velocity_rad_s.store(0.0);
        states[index].target_rad.store(feedback->position_rad);
    }
    return true;
}

bool parseTargets(
    const std::string& line,
    std::array<double, JOINTS.size()>& targets_deg)
{
    std::istringstream input(line);
    for (double& target : targets_deg) {
        if (!(input >> target) || !std::isfinite(target))
            return false;
    }
    input >> std::ws;
    return input.eof();
}

void adjustTarget(
    std::array<ControlState, JOINTS.size()>& states,
    char key)
{
    const double step_rad = degreesToRadians(STEP_DEG);
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        if (key == JOINTS[index].positive_key) {
            states[index].target_rad.store(
                states[index].target_rad.load() + step_rad);
            return;
        }
        if (key == JOINTS[index].negative_key) {
            states[index].target_rad.store(
                states[index].target_rad.load() - step_rad);
            return;
        }
    }
}

}  // namespace

int main(int argc, char* argv[])
{
    const std::string interface_name =
        argc > 1 ? argv[1] : "can0";
    const std::string limits_path =
        argc > 2 ? argv[2] : "config/joint_limits.yaml";
    const std::string motor_config_path =
        argc > 3 ? argv[3] : "config/motor_config.yaml";

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    try {
        auto limiters =
            raven_control::safety::loadJointLimiters(limits_path);
        const auto motor_config =
            raven_control::config::loadMotorRuntimeConfig(
                motor_config_path);
        const JointBindings bindings =
            bindConfiguredJoints(motor_config);
        raven_control::hal::CanInterface can(interface_name);
        raven_control::hal::MotorDriver driver(
            can,
            motorMap(bindings),
            std::move(limiters),
            0xFD,
            motor_config.feedback_timeout);
        std::array<ControlState, JOINTS.size()> states;
        initializeControlStates(bindings, states);
        std::atomic<bool> running{true};
        std::atomic<bool> monitor_active{true};

        if (!driver.stopAll())
            throw std::runtime_error("Failed to send startup stop");
        if (!driver.requestMechanicalPositions()) {
            throw std::runtime_error(
                "Failed to request initial position feedback");
        }

        const auto feedback_deadline =
            std::chrono::steady_clock::now() +
            std::max(
                std::chrono::milliseconds(500),
                motor_config.feedback_timeout * 2);
        while (!driver.allFeedbackValid() &&
               std::chrono::steady_clock::now() <
                   feedback_deadline) {
            (void)driver.poll();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(5));
        }

        if (!initializeTargetsFromFeedback(
                driver,
                states,
                motor_config.feedback_timeout)) {
            throw std::runtime_error(
                "Fresh Type 17 feedback was not received at startup");
        }
        std::cout
            << "Joint limits: " << limits_path << '\n'
            << "Motor config: " << motor_config_path << '\n';
        printHelp(bindings);

        TerminalMode terminal;
        std::thread control_thread(
            controlLoop,
            std::ref(driver),
            std::cref(motor_config),
            std::cref(bindings),
            std::ref(states),
            std::ref(running),
            std::ref(monitor_active));

        try {
            while (running.load() && !stop_requested) {
                if (!keyAvailable()) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(5));
                    continue;
                }

                const auto key = readKey();
                if (!key)
                    continue;

                if (*key == 'q' || *key == 'Q') {
                    running.store(false);
                } else if (*key == ' ') {
                    std::cout << "\n";
                    if (driver.isEnabled()) {
                        if (!driver.stopAll())
                            std::cerr << "Failed to stop all motors\n";
                    } else if (!initializeTargetsFromFeedback(
                                   driver,
                                   states,
                                   motor_config.feedback_timeout)) {
                        std::cerr
                            << "Enable rejected: fresh Type 17 feedback "
                               "is unavailable\n";
                    } else {
                        const auto result = driver.enableAll();
                        if (result !=
                            raven_control::hal::
                                MotorCommandResult::Sent) {
                            std::cerr
                                << "Enable rejected: "
                                << raven_control::hal::toString(result)
                                << '\n';
                        }
                    }
                } else if (*key == '/') {
                    monitor_active.store(false);
                    terminal.restore();
                    std::cout << "\nCMD (deg) > " << std::flush;

                    std::string line;
                    if (!std::getline(std::cin, line)) {
                        running.store(false);
                    } else {
                        std::array<double, JOINTS.size()> targets_deg{};
                        if (!parseTargets(line, targets_deg)) {
                            std::cout
                                << "Expected exactly three finite "
                                   "angles\n";
                        } else if (!driver.isEnabled()) {
                            std::cout
                                << "Targets ignored while motors "
                                   "are stopped\n";
                        } else {
                            for (std::size_t index = 0;
                                 index < JOINTS.size();
                                 ++index) {
                                states[index].target_rad.store(
                                    degreesToRadians(
                                        targets_deg[index]));
                            }
                        }
                    }

                    if (running.load()) {
                        terminal.enableRaw();
                        monitor_active.store(true);
                    }
                } else {
                    adjustTarget(states, *key);
                }
            }
        } catch (...) {
            running.store(false);
            control_thread.join();
            (void)driver.stopAll();
            throw;
        }

        terminal.restore();
        running.store(false);
        control_thread.join();
        const bool stopped = driver.stopAll();
        std::cout << "\n";

        if (driver.faultLatched()) {
            std::cerr
                << "Safety fault: " << driver.faultReason() << '\n';
            return 2;
        }
        if (!stopped) {
            std::cerr << "Failed to send final stop\n";
            return 1;
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal error: " << error.what() << '\n';
        return 1;
    }
}
