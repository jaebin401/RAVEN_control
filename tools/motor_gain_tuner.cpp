#include "raven_control/config/motor_config.hpp"
#include "raven_control/hal/can_interface.hpp"
#include "raven_control/hal/motor_driver.hpp"
#include "raven_control/safety/joint_limiter.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double TARGET_STEP_RAD = 1.0 * PI / 180.0;
constexpr double KP_STEP = 1.0;
constexpr double KD_STEP = 0.01;

struct JointUiSpec {
    const char* name;
};

constexpr std::array<JointUiSpec, 3> JOINTS{{
    {"shoulder_Joint"},
    {"upperArm_Joint"},
    {"foreArm_Joint"},
}};

using JointBindings = std::array<
    raven_control::config::JointMotorRuntimeConfig*,
    JOINTS.size()>;

struct TuningState {
    double target_rad = 0.0;
    double setpoint_rad = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double max_slew_rate_rad_s = 0.0;
};

enum class SelectedGain {
    Kp,
    Kd,
};

enum class Key {
    None,
    Up,
    Down,
    Left,
    Right,
    Tab,
    Space,
    TargetPositive,
    TargetNegative,
    Save,
    Quit,
};

volatile std::sig_atomic_t stop_requested = 0;

double radiansToDegrees(double radians)
{
    return radians * 180.0 / PI;
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

    ~TerminalMode()
    {
        restore();
    }

    TerminalMode(const TerminalMode&) = delete;
    TerminalMode& operator=(const TerminalMode&) = delete;

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

bool readByte(char& value, std::chrono::milliseconds timeout)
{
    timeval wait{};
    wait.tv_sec = static_cast<time_t>(timeout.count() / 1000);
    wait.tv_usec =
        static_cast<suseconds_t>((timeout.count() % 1000) * 1000);

    fd_set descriptors;
    FD_ZERO(&descriptors);
    FD_SET(STDIN_FILENO, &descriptors);
    const int ready = ::select(
        STDIN_FILENO + 1,
        &descriptors,
        nullptr,
        nullptr,
        &wait);
    if (ready <= 0)
        return false;
    return ::read(STDIN_FILENO, &value, 1) == 1;
}

Key readKey()
{
    char first = '\0';
    if (!readByte(first, std::chrono::milliseconds(0)))
        return Key::None;

    if (first == '\033') {
        char second = '\0';
        char third = '\0';
        if (!readByte(second, std::chrono::milliseconds(8)) ||
            second != '[' ||
            !readByte(third, std::chrono::milliseconds(8))) {
            return Key::None;
        }
        switch (third) {
        case 'A':
            return Key::Up;
        case 'B':
            return Key::Down;
        case 'C':
            return Key::Right;
        case 'D':
            return Key::Left;
        default:
            return Key::None;
        }
    }

    switch (first) {
    case '\t':
        return Key::Tab;
    case ' ':
        return Key::Space;
    case 'w':
    case 'W':
        return Key::TargetPositive;
    case 's':
    case 'S':
        return Key::TargetNegative;
    case 'v':
    case 'V':
        return Key::Save;
    case 'q':
    case 'Q':
        return Key::Quit;
    case 'k':
    case 'K':
        return Key::Up;
    case 'j':
    case 'J':
        return Key::Down;
    case 'h':
    case 'H':
        return Key::Left;
    case 'l':
    case 'L':
        return Key::Right;
    default:
        return Key::None;
    }
}

JointBindings bindConfiguredJoints(
    raven_control::config::MotorRuntimeConfig& config)
{
    if (config.joints.size() != JOINTS.size()) {
        throw std::runtime_error(
            "motor_gain_tuner requires exactly three joints");
    }

    JointBindings bindings{};
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        for (auto& joint : config.joints) {
            if (joint.joint_name == JOINTS[index].name) {
                bindings[index] = &joint;
                break;
            }
        }
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
    result.reserve(bindings.size());
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

std::array<TuningState, JOINTS.size()> tuningStates(
    const JointBindings& bindings)
{
    std::array<TuningState, JOINTS.size()> states{};
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        states[index].kp =
            bindings[index]->position_control.kp;
        states[index].kd =
            bindings[index]->position_control.kd;
        states[index].max_slew_rate_rad_s =
            bindings[index]->
                position_control.max_slew_rate_rad_s;
    }
    return states;
}

bool initializeTargetsFromFeedback(
    raven_control::hal::MotorDriver& driver,
    std::array<TuningState, JOINTS.size()>& states)
{
    if (!driver.allFeedbackValid())
        return false;

    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto feedback = driver.feedback(JOINTS[index].name);
        if (!feedback || !feedback->valid)
            return false;
        states[index].target_rad = feedback->position_rad;
        states[index].setpoint_rad = feedback->position_rad;
    }
    return true;
}

void render(
    raven_control::hal::MotorDriver& driver,
    const JointBindings& bindings,
    const std::array<TuningState, JOINTS.size()>& states,
    std::size_t selected_joint,
    SelectedGain selected_gain,
    const std::string& message,
    const std::string& config_path)
{
    std::cout << "\033[2J\033[H"
              << "RAVEN RS02 Gain Tuner\n"
              << "Config: " << config_path << "\n"
              << "Motors: "
              << (driver.isEnabled() ? "ENABLED" : "STOPPED")
              << "\n\n";

    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto feedback = driver.feedback(JOINTS[index].name);
        const bool selected = index == selected_joint;
        const double actual_deg =
            feedback && feedback->valid
            ? radiansToDegrees(feedback->position_rad)
            : 0.0;

        std::cout
            << (selected ? "> " : "  ")
            << std::left << std::setw(17) << JOINTS[index].name
            << " ID " << std::setw(3)
            << static_cast<int>(bindings[index]->motor_id)
            << std::right << std::fixed << std::setprecision(2)
            << " actual " << std::setw(8) << actual_deg
            << " deg  target " << std::setw(8)
            << radiansToDegrees(states[index].target_rad)
            << " deg  ";

        if (selected && selected_gain == SelectedGain::Kp)
            std::cout << "\033[7m";
        std::cout << "Kp " << std::setw(7)
                  << states[index].kp;
        if (selected && selected_gain == SelectedGain::Kp)
            std::cout << "\033[0m";
        std::cout << "  ";

        if (selected && selected_gain == SelectedGain::Kd)
            std::cout << "\033[7m";
        std::cout << "Kd " << std::setw(6)
                  << states[index].kd;
        if (selected && selected_gain == SelectedGain::Kd)
            std::cout << "\033[0m";
        std::cout << '\n';
    }

    std::cout
        << "\nLeft/Right : select joint"
        << "    Tab : select Kp/Kd\n"
        << "Up/Down    : adjust selected gain"
        << "    (Kp " << KP_STEP
        << ", Kd " << KD_STEP << ")\n"
        << "W/S        : selected target +/-2 deg"
        << "    Space : enable/stop all\n"
        << "V          : save gains while stopped"
        << "    Q : quit\n"
        << "H/J/K/L    : arrow-key fallback\n\n"
        << message << "\n"
        << std::flush;
}

void adjustGain(
    TuningState& state,
    SelectedGain selected_gain,
    double direction)
{
    if (selected_gain == SelectedGain::Kp) {
        state.kp = std::clamp(
            std::round(
                (state.kp + direction * KP_STEP) / KP_STEP) *
                KP_STEP,
            0.0,
            raven_control::hal::RS02_OPERATION_MAX_KP);
    } else {
        state.kd = std::clamp(
            std::round(
                (state.kd + direction * KD_STEP) / KD_STEP) *
                KD_STEP,
            0.0,
            raven_control::hal::RS02_OPERATION_MAX_KD);
    }
}

void copyGainsToConfig(
    const std::array<TuningState, JOINTS.size()>& states,
    const JointBindings& bindings)
{
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        bindings[index]->position_control.kp = states[index].kp;
        bindings[index]->position_control.kd = states[index].kd;
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
        auto motor_config =
            raven_control::config::loadMotorRuntimeConfig(
                motor_config_path);
        const JointBindings bindings =
            bindConfiguredJoints(motor_config);
        auto states = tuningStates(bindings);

        raven_control::hal::CanInterface can(interface_name);
        raven_control::hal::MotorDriver driver(
            can,
            motorMap(bindings),
            std::move(limiters),
            0xFD,
            motor_config.feedback_timeout);

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
        (void)initializeTargetsFromFeedback(driver, states);

        TerminalMode terminal;
        std::size_t selected_joint = 0;
        SelectedGain selected_gain = SelectedGain::Kp;
        std::string message =
            "Ready. Keep the mechanism supported during tuning.";
        bool running = true;
        auto next_cycle = std::chrono::steady_clock::now();
        auto next_position_request = next_cycle;
        auto next_render = next_cycle;

        try {
            while (running && !stop_requested) {
                next_cycle += motor_config.control_period;
                const auto now = std::chrono::steady_clock::now();

                if (now >= next_position_request) {
                    if (!driver.requestMechanicalPositions()) {
                        (void)driver.stopAll();
                        throw std::runtime_error(
                            "Failed to request position feedback");
                    }
                    do {
                        next_position_request +=
                            motor_config.position_request_period;
                    } while (next_position_request <= now);
                }
                (void)driver.poll();
                if (driver.faultLatched())
                    break;

                while (true) {
                    const Key key = readKey();
                    if (key == Key::None)
                        break;

                    switch (key) {
                    case Key::Left:
                        selected_joint =
                            (selected_joint + JOINTS.size() - 1) %
                            JOINTS.size();
                        message = "Selected " +
                            std::string(
                                JOINTS[selected_joint].name);
                        break;
                    case Key::Right:
                        selected_joint =
                            (selected_joint + 1) % JOINTS.size();
                        message = "Selected " +
                            std::string(
                                JOINTS[selected_joint].name);
                        break;
                    case Key::Tab:
                        selected_gain =
                            selected_gain == SelectedGain::Kp
                            ? SelectedGain::Kd
                            : SelectedGain::Kp;
                        message =
                            selected_gain == SelectedGain::Kp
                            ? "Kp selected"
                            : "Kd selected";
                        break;
                    case Key::Up:
                        adjustGain(
                            states[selected_joint],
                            selected_gain,
                            1.0);
                        message = "Gain increased";
                        break;
                    case Key::Down:
                        adjustGain(
                            states[selected_joint],
                            selected_gain,
                            -1.0);
                        message = "Gain decreased";
                        break;
                    case Key::TargetPositive:
                    case Key::TargetNegative:
                        if (!driver.isEnabled()) {
                            message =
                                "Enable motors before stepping target";
                        } else {
                            states[selected_joint].target_rad +=
                                key == Key::TargetPositive
                                ? TARGET_STEP_RAD
                                : -TARGET_STEP_RAD;
                            message = "Target changed by 2 degrees";
                        }
                        break;
                    case Key::Space:
                        if (driver.isEnabled()) {
                            if (!driver.stopAll()) {
                                throw std::runtime_error(
                                    "Failed to stop all motors");
                            }
                            message = "All motors stopped";
                        } else if (!initializeTargetsFromFeedback(
                                       driver,
                                       states)) {
                            message =
                                "Enable rejected: fresh feedback "
                                "is unavailable";
                        } else {
                            const auto result = driver.enableAll();
                            message =
                                result ==
                                    raven_control::hal::
                                        MotorCommandResult::Sent
                                ? "All motors enabled"
                                : "Enable rejected: " +
                                      std::string(
                                          raven_control::hal::
                                              toString(result));
                        }
                        break;
                    case Key::Save:
                        if (driver.isEnabled()) {
                            message =
                                "Stop motors before saving gains";
                        } else {
                            copyGainsToConfig(states, bindings);
                            raven_control::config::
                                saveMotorRuntimeConfig(
                                    motor_config,
                                    motor_config_path);
                            message =
                                "Saved gains to " +
                                motor_config_path;
                        }
                        break;
                    case Key::Quit:
                        running = false;
                        break;
                    case Key::None:
                        break;
                    }
                }

                if (driver.isEnabled()) {
                    for (std::size_t index = 0;
                         index < JOINTS.size();
                         ++index) {
                        const double difference =
                            states[index].target_rad -
                            states[index].setpoint_rad;
                        const double max_step =
                            states[index].max_slew_rate_rad_s *
                            std::chrono::duration<double>(
                                motor_config.control_period).count();
                        states[index].setpoint_rad =
                            std::abs(difference) <= max_step
                            ? states[index].target_rad
                            : states[index].setpoint_rad +
                                  std::copysign(
                                      max_step,
                                      difference);

                        const auto result =
                            driver.sendPositionCommand(
                                JOINTS[index].name,
                                states[index].setpoint_rad,
                                states[index].kp,
                                states[index].kd);
                        if (result ==
                            raven_control::hal::
                                MotorCommandResult::TargetClamped) {
                            message =
                                "Target clamped by joint limit";
                        } else if (
                            result !=
                            raven_control::hal::
                                MotorCommandResult::Sent) {
                            throw std::runtime_error(
                                "Position command failed: " +
                                std::string(
                                    raven_control::hal::
                                        toString(result)));
                        }
                    }
                }

                if (now >= next_render) {
                    render(
                        driver,
                        bindings,
                        states,
                        selected_joint,
                        selected_gain,
                        message,
                        motor_config_path);
                    next_render =
                        now + std::chrono::milliseconds(100);
                }

                std::this_thread::sleep_until(next_cycle);
                const auto after_sleep =
                    std::chrono::steady_clock::now();
                if (after_sleep >
                    next_cycle + motor_config.control_period) {
                    next_cycle = after_sleep;
                }
            }
        } catch (...) {
            (void)driver.stopAll();
            throw;
        }

        terminal.restore();
        const bool stopped = driver.stopAll();
        std::cout << "\033[2J\033[H";
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
