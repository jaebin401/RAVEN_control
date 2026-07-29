#include "raven_control/hal/can_interface.hpp"
#include "raven_control/hal/motor_driver.hpp"
#include "raven_control/safety/joint_limiter.hpp"

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
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
constexpr double STEP_DEG = 2.0;
constexpr double MAX_SPEED_DEG_PER_SEC = 500.0;
constexpr double CONTROL_PERIOD_SEC = 0.02;

struct JointSpec {
    const char* name;
    std::uint8_t motor_id;
    char positive_key;
    char negative_key;
};

constexpr std::array<JointSpec, 3> JOINTS{{
    {"shoulder_Joint", 1, 'w', 's'},
    {"upperArm_Joint", 2, 'e', 'd'},
    {"foreArm_Joint", 3, 'r', 'f'},
}};

struct ControlState {
    std::atomic<double> target_rad{0.0};
    std::atomic<double> setpoint_rad{0.0};
    std::atomic<double> kp{40.0};
    std::atomic<double> kd{5.0};
};

volatile std::sig_atomic_t stop_requested = 0;

double radiansToDegrees(double radians)
{
    return radians * 180.0 / PI;
}

double degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
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

std::vector<raven_control::hal::JointMotorConfig> motorMap()
{
    std::vector<raven_control::hal::JointMotorConfig> result;
    result.reserve(JOINTS.size());
    for (const auto& joint : JOINTS)
        result.push_back({joint.name, joint.motor_id});
    return result;
}

void printHelp()
{
    std::cout
        << "========================================\n"
        << "RAVEN multi-joint debug controller\n"
        << "Space : enable/stop all motors\n"
        << "W/S   : shoulder +/-\n"
        << "E/D   : upper arm +/-\n"
        << "R/F   : forearm +/-\n"
        << "/     : enter three targets in degrees\n"
        << "Q     : quit\n"
        << "========================================\n";
}

void printStatus(
    raven_control::hal::MotorDriver& driver,
    const std::array<ControlState, JOINTS.size()>& states)
{
    std::cout << "\r\033[K[KEY] ";
    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto feedback = driver.feedback(JOINTS[index].name);
        const double actual_deg =
            feedback && feedback->valid
            ? radiansToDegrees(feedback->position_rad)
            : 0.0;

        std::cout
            << "ID:" << static_cast<int>(JOINTS[index].motor_id)
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
    std::array<ControlState, JOINTS.size()>& states,
    std::atomic<bool>& running,
    std::atomic<bool>& monitor_active)
{
    const auto period = std::chrono::duration<double>(CONTROL_PERIOD_SEC);
    const double max_step_rad =
        degreesToRadians(MAX_SPEED_DEG_PER_SEC) * CONTROL_PERIOD_SEC;
    std::size_t cycle = 0;
    auto next_cycle = std::chrono::steady_clock::now();

    while (running.load() && !stop_requested) {
        next_cycle +=
            std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(period);
        ++cycle;

        if (cycle % 5 == 0 &&
            !driver.requestMechanicalPositions()) {
            (void)driver.stopAll();
            running.store(false);
            break;
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
                const double difference = target - current;
                const double next =
                    std::abs(difference) <= max_step_rad
                    ? target
                    : current + std::copysign(
                          max_step_rad,
                          difference);
                states[index].setpoint_rad.store(next);

                const auto result = driver.sendPositionCommand(
                    JOINTS[index].name,
                    next,
                    states[index].kp.load(),
                    states[index].kd.load());
                if (result !=
                        raven_control::hal::MotorCommandResult::Sent &&
                    result != raven_control::hal::
                        MotorCommandResult::TargetClamped) {
                    running.store(false);
                    break;
                }
            }
        }

        if (monitor_active.load() && cycle % 5 == 0)
            printStatus(driver, states);

        std::this_thread::sleep_until(next_cycle);
    }
}

bool initializeTargetsFromFeedback(
    raven_control::hal::MotorDriver& driver,
    std::array<ControlState, JOINTS.size()>& states)
{
    if (!driver.allFeedbackValid())
        return false;

    for (std::size_t index = 0; index < JOINTS.size(); ++index) {
        const auto feedback = driver.feedback(JOINTS[index].name);
        if (!feedback || !feedback->valid)
            return false;
        states[index].setpoint_rad.store(feedback->position_rad);
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

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    try {
        auto limiters =
            raven_control::safety::loadJointLimiters(limits_path);
        raven_control::hal::CanInterface can(interface_name);
        raven_control::hal::MotorDriver driver(
            can,
            motorMap(),
            std::move(limiters));
        std::array<ControlState, JOINTS.size()> states;
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
            std::chrono::milliseconds(500);
        while (!driver.allFeedbackValid() &&
               std::chrono::steady_clock::now() <
                   feedback_deadline) {
            (void)driver.poll();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(5));
        }

        initializeTargetsFromFeedback(driver, states);
        printHelp();

        TerminalMode terminal;
        std::thread control_thread(
            controlLoop,
            std::ref(driver),
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
                                   states)) {
                        std::cerr
                            << "Enable rejected: fresh feedback "
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
