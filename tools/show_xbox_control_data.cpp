/*
 * Xbox controller input diagnostic utility.
 *
 * Reads every axis and button exposed by the Linux joystick API and displays
 * their current values in one terminal screen. This tool does not control any
 * motor and does not communicate over CAN.
 */

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/joystick.h>

namespace {

constexpr const char* DEFAULT_DEVICE_PATH = "/dev/input/js0";
constexpr auto DISPLAY_PERIOD = std::chrono::milliseconds(40);  // 25 Hz

volatile std::sig_atomic_t stop_requested = 0;

void handle_signal(int)
{
    stop_requested = 1;
}

class TerminalDisplay {
public:
    TerminalDisplay()
    {
        // Clear the terminal once and hide the cursor while refreshing.
        std::cout << "\x1b[2J\x1b[H\x1b[?25l" << std::flush;
    }

    ~TerminalDisplay()
    {
        std::cout << "\x1b[?25h\n" << std::flush;
    }

    TerminalDisplay(const TerminalDisplay&) = delete;
    TerminalDisplay& operator=(const TerminalDisplay&) = delete;
};

class Joystick {
public:
    explicit Joystick(std::string device_path)
        : device_path_(std::move(device_path))
    {
        fd_ = ::open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            throw std::runtime_error(
                "Cannot open '" + device_path_ + "': " + std::strerror(errno));
        }

        try {
            query_device_info();
        } catch (...) {
            ::close(fd_);
            fd_ = -1;
            throw;
        }
    }

    ~Joystick()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    Joystick(const Joystick&) = delete;
    Joystick& operator=(const Joystick&) = delete;

    const std::string& device_path() const { return device_path_; }
    const std::string& name() const { return name_; }
    const std::vector<double>& axes() const { return axes_; }
    const std::vector<double>& buttons() const { return buttons_; }

    bool read_pending_events(std::string& error_message)
    {
        while (true) {
            js_event event{};
            const ssize_t bytes_read = ::read(fd_, &event, sizeof(event));

            if (bytes_read == static_cast<ssize_t>(sizeof(event))) {
                process_event(event);
                continue;
            }

            if (bytes_read < 0 && errno == EINTR) {
                if (stop_requested)
                    return true;
                continue;
            }

            if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return true;

            if (bytes_read == 0) {
                error_message = "Controller disconnected (end of device stream).";
                return false;
            }

            if (bytes_read < 0) {
                error_message =
                    "Controller read failed: " + std::string(std::strerror(errno));
                return false;
            }

            error_message = "Controller read returned an incomplete joystick event.";
            return false;
        }
    }

private:
    void query_device_info()
    {
        unsigned char axis_count = 0;
        unsigned char button_count = 0;

        if (::ioctl(fd_, JSIOCGAXES, &axis_count) < 0) {
            throw std::runtime_error(
                "Cannot query axis count: " + std::string(std::strerror(errno)));
        }

        if (::ioctl(fd_, JSIOCGBUTTONS, &button_count) < 0) {
            throw std::runtime_error(
                "Cannot query button count: " + std::string(std::strerror(errno)));
        }

        char name_buffer[128] = "Unknown joystick";
        if (::ioctl(fd_, JSIOCGNAME(sizeof(name_buffer)), name_buffer) >= 0)
            name_ = name_buffer;
        else
            name_ = "Unknown joystick";

        axes_.assign(axis_count, 0.0);
        buttons_.assign(button_count, 0.0);
    }

    void process_event(const js_event& event)
    {
        // JS_EVENT_INIT carries a valid initial axis/button state. Remove only
        // the INIT flag, then process it exactly like a normal input event.
        const unsigned char type =
            event.type & static_cast<unsigned char>(~JS_EVENT_INIT);

        if (type == JS_EVENT_AXIS && event.number < axes_.size()) {
            axes_[event.number] = normalize_axis(event.value);
        } else if (type == JS_EVENT_BUTTON && event.number < buttons_.size()) {
            buttons_[event.number] = event.value == 0 ? 0.0 : 1.0;
        }
    }

    static double normalize_axis(short value)
    {
        // The signed joystick range is asymmetric: -32768 to 32767.
        if (value >= 0)
            return static_cast<double>(value) / 32767.0;
        return static_cast<double>(value) / 32768.0;
    }

    std::string device_path_;
    std::string name_;
    int fd_ = -1;
    std::vector<double> axes_;
    std::vector<double> buttons_;
};

void print_state(const Joystick& joystick)
{
    constexpr std::size_t AXES_PER_ROW = 3;
    constexpr std::size_t BUTTONS_PER_ROW = 4;

    std::cout << "\x1b[H"
              << "RAVEN Xbox controller input monitor\n"
              << "Device  : " << joystick.device_path() << "\n"
              << "Name    : " << joystick.name() << "\n"
              << "Axes    : " << joystick.axes().size() << "\n"
              << "Buttons : " << joystick.buttons().size() << "\n"
              << "Refresh : 25 Hz\n"
              << "Stop    : Ctrl+C\n\n"
              << std::fixed << std::setprecision(4);

    std::cout << "AXES [-1.0, 1.0]\n";
    if (joystick.axes().empty()) {
        std::cout << "  (none)\n";
    } else {
        for (std::size_t index = 0; index < joystick.axes().size(); ++index) {
            std::cout << "  axis[" << std::setw(2) << index << "] = "
                      << std::setw(7) << joystick.axes()[index];
            if ((index + 1) % AXES_PER_ROW == 0 ||
                index + 1 == joystick.axes().size()) {
                std::cout << '\n';
            }
        }
    }

    std::cout << "\nBUTTONS {0.0, 1.0}\n";
    std::cout << std::setprecision(1);
    if (joystick.buttons().empty()) {
        std::cout << "  (none)\n";
    } else {
        for (std::size_t index = 0; index < joystick.buttons().size(); ++index) {
            std::cout << "  button[" << std::setw(2) << index << "] = "
                      << joystick.buttons()[index];
            if ((index + 1) % BUTTONS_PER_ROW == 0 ||
                index + 1 == joystick.buttons().size()) {
                std::cout << '\n';
            }
        }
    }

    // Clear any remnants if a reconnected device previously occupied more rows.
    std::cout << "\x1b[J" << std::flush;
}

void install_signal_handlers()
{
    struct sigaction action {};
    action.sa_handler = handle_signal;
    sigemptyset(&action.sa_mask);

    if (::sigaction(SIGINT, &action, nullptr) < 0 ||
        ::sigaction(SIGTERM, &action, nullptr) < 0) {
        throw std::runtime_error(
            "Cannot install signal handlers: " + std::string(std::strerror(errno)));
    }
}

void print_usage(const char* program_name)
{
    std::cerr << "Usage: " << program_name << " [device_path]\n"
              << "Default device_path: " << DEFAULT_DEVICE_PATH << '\n';
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc > 2) {
        print_usage(argv[0]);
        return 2;
    }

    const std::string device_path =
        argc == 2 ? argv[1] : DEFAULT_DEVICE_PATH;

    try {
        install_signal_handlers();
        Joystick joystick(device_path);
        TerminalDisplay display;

        auto next_display = std::chrono::steady_clock::now();
        std::string read_error;

        while (!stop_requested) {
            if (!joystick.read_pending_events(read_error)) {
                std::cerr << "\n" << read_error << '\n';
                return 1;
            }

            print_state(joystick);
            next_display += DISPLAY_PERIOD;
            std::this_thread::sleep_until(next_display);

            // Avoid a catch-up spin if the process was suspended or delayed.
            const auto now = std::chrono::steady_clock::now();
            if (next_display < now)
                next_display = now;
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
