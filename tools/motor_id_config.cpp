/*
 * RobStride CAN maintenance utility (private protocol / SocketCAN)
 *
 * Supported operations:
 *   - discover connected motor CAN IDs
 *   - read one motor's ID and MCU unique identifier
 *   - change a motor CAN ID
 *   - set the current mechanical position as zero
 *
 * Protocol reference: RS02User Manual 260112, section 4.1
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <cerrno>
#include <cstdlib>
#include <poll.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace {

constexpr uint8_t DEFAULT_HOST_ID = 0xFD;
constexpr int DEFAULT_TIMEOUT_MS = 20;
constexpr int DEFAULT_RETRIES = 2;
constexpr int MIN_MOTOR_ID = 0;
constexpr int MAX_MOTOR_ID = 127;
constexpr uint16_t PARAM_MECH_POS = 0x7019;

constexpr uint8_t COMM_GET_ID = 0;
constexpr uint8_t COMM_FEEDBACK = 2;
constexpr uint8_t COMM_STOP = 4;
constexpr uint8_t COMM_SET_ZERO = 6;
constexpr uint8_t COMM_SET_CAN_ID = 7;
constexpr uint8_t COMM_PARAM_READ = 17;

struct Config {
    std::string interface = "can0";
    uint8_t host_id = DEFAULT_HOST_ID;
    int timeout_ms = DEFAULT_TIMEOUT_MS;
    int retries = DEFAULT_RETRIES;
    bool assume_yes = false;
    bool dry_run = false;
    bool verbose = false;
};

struct DeviceInfo {
    uint8_t motor_id = 0;
    std::array<uint8_t, 8> uid{};
};

uint32_t build_ext_id(uint8_t comm_type, uint16_t data_field, uint8_t motor_id)
{
    return (uint32_t(comm_type & 0x1F) << 24) |
           (uint32_t(data_field) << 8) |
           uint32_t(motor_id);
}

uint8_t comm_type(const can_frame& frame)
{
    return static_cast<uint8_t>(((frame.can_id & CAN_EFF_MASK) >> 24) & 0x1F);
}

uint8_t source_motor_id(const can_frame& frame)
{
    return static_cast<uint8_t>(((frame.can_id & CAN_EFF_MASK) >> 8) & 0xFF);
}

uint8_t destination_id(const can_frame& frame)
{
    return static_cast<uint8_t>((frame.can_id & CAN_EFF_MASK) & 0xFF);
}

std::string frame_to_string(uint32_t raw_id, const uint8_t* data, size_t size)
{
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0')
        << std::setw(8) << (raw_id & CAN_EFF_MASK) << '#';
    for (size_t i = 0; i < size; ++i)
        out << std::setw(2) << unsigned(data[i]);
    return out.str();
}

std::string frame_to_string(const can_frame& frame)
{
    return frame_to_string(frame.can_id, frame.data, frame.can_dlc);
}

class CanBus {
public:
    explicit CanBus(const Config& config) : config_(config)
    {
        if (config_.dry_run)
            return;

        socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_ < 0)
            throw std::runtime_error("SocketCAN socket creation failed: " +
                                     std::string(std::strerror(errno)));

        ifreq ifr{};
        if (config_.interface.size() >= IFNAMSIZ) {
            close_socket();
            throw std::runtime_error("CAN interface name is too long: " + config_.interface);
        }
        std::strncpy(ifr.ifr_name, config_.interface.c_str(), IFNAMSIZ - 1);

        if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
            const std::string reason = std::strerror(errno);
            close_socket();
            throw std::runtime_error("Cannot find CAN interface '" + config_.interface +
                                     "': " + reason);
        }

        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = ifr.ifr_ifindex;
        if (::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            const std::string reason = std::strerror(errno);
            close_socket();
            throw std::runtime_error("Cannot bind to CAN interface '" + config_.interface +
                                     "': " + reason);
        }
    }

    ~CanBus() { close_socket(); }

    CanBus(const CanBus&) = delete;
    CanBus& operator=(const CanBus&) = delete;

    bool is_dry_run() const { return config_.dry_run; }

    bool send(uint32_t raw_id, const std::array<uint8_t, 8>& data)
    {
        if (config_.dry_run) {
            std::cout << "DRY-RUN cansend " << config_.interface << ' '
                      << frame_to_string(raw_id, data.data(), data.size()) << '\n';
            return true;
        }

        can_frame frame{};
        frame.can_id = (raw_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
        frame.can_dlc = static_cast<__u8>(data.size());
        std::copy(data.begin(), data.end(), frame.data);

        const ssize_t written = ::write(socket_, &frame, sizeof(frame));
        if (written != static_cast<ssize_t>(sizeof(frame))) {
            std::cerr << "CAN write failed: " << std::strerror(errno) << '\n';
            return false;
        }
        if (config_.verbose)
            std::cout << "TX " << frame_to_string(frame) << '\n';
        return true;
    }

    bool receive(can_frame& frame, int timeout_ms)
    {
        if (config_.dry_run)
            return false;

        pollfd descriptor{};
        descriptor.fd = socket_;
        descriptor.events = POLLIN;

        int poll_result;
        do {
            poll_result = ::poll(&descriptor, 1, timeout_ms);
        } while (poll_result < 0 && errno == EINTR);

        if (poll_result <= 0)
            return false;

        const ssize_t received = ::read(socket_, &frame, sizeof(frame));
        if (received != static_cast<ssize_t>(sizeof(frame)))
            return false;
        if ((frame.can_id & CAN_EFF_FLAG) == 0)
            return false;

        if (config_.verbose)
            std::cout << "RX " << frame_to_string(frame) << '\n';
        return true;
    }

    void drain()
    {
        can_frame ignored{};
        // Do not spin forever when firmware active reporting (Type 24) is enabled.
        for (int count = 0; count < 256 && receive(ignored, 0); ++count) {
        }
    }

private:
    void close_socket()
    {
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
    }

    const Config& config_;
    int socket_ = -1;
};

std::optional<can_frame> wait_for(
    CanBus& bus, int timeout_ms, const std::function<bool(const can_frame&)>& predicate)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        can_frame frame{};
        if (!bus.receive(frame, std::max(1, static_cast<int>(remaining.count()))))
            continue;
        if (predicate(frame))
            return frame;
    }
    return std::nullopt;
}

std::optional<DeviceInfo> get_device(CanBus& bus, const Config& config, uint8_t motor_id)
{
    const std::array<uint8_t, 8> empty{};
    for (int attempt = 0; attempt < config.retries; ++attempt) {
        bus.drain();
        const uint32_t id = build_ext_id(COMM_GET_ID, config.host_id, motor_id);
        if (!bus.send(id, empty))
            return std::nullopt;
        if (bus.is_dry_run())
            return DeviceInfo{motor_id, {}};

        auto reply = wait_for(bus, config.timeout_ms, [&](const can_frame& frame) {
            if (comm_type(frame) != COMM_GET_ID || source_motor_id(frame) != motor_id)
                return false;
            // The manual fixes GET_ID's reply destination at 0xFE. Some firmware
            // revisions instead echo the configured host ID, so accept both.
            const uint8_t destination = destination_id(frame);
            return destination == 0xFE || destination == config.host_id;
        });
        if (reply) {
            DeviceInfo device{};
            device.motor_id = source_motor_id(*reply);
            std::copy_n(reply->data, device.uid.size(), device.uid.begin());
            return device;
        }
    }
    return std::nullopt;
}

bool send_stop(CanBus& bus, const Config& config, uint8_t motor_id)
{
    const std::array<uint8_t, 8> empty{};
    bus.drain();
    return bus.send(build_ext_id(COMM_STOP, config.host_id, motor_id), empty);
}

bool send_set_id(CanBus& bus, const Config& config, uint8_t old_id, uint8_t new_id)
{
    const std::array<uint8_t, 8> empty{};
    const uint16_t data_field = (uint16_t(new_id) << 8) | config.host_id;
    bus.drain();
    return bus.send(build_ext_id(COMM_SET_CAN_ID, data_field, old_id), empty);
}

bool send_set_zero(CanBus& bus, const Config& config, uint8_t motor_id)
{
    std::array<uint8_t, 8> data{};
    data[0] = 1;
    bus.drain();
    return bus.send(build_ext_id(COMM_SET_ZERO, config.host_id, motor_id), data);
}

std::optional<double> read_mechanical_position(
    CanBus& bus, const Config& config, uint8_t motor_id)
{
    std::array<uint8_t, 8> data{};
    data[0] = static_cast<uint8_t>(PARAM_MECH_POS & 0xFF);
    data[1] = static_cast<uint8_t>(PARAM_MECH_POS >> 8);
    bus.drain();
    if (!bus.send(build_ext_id(COMM_PARAM_READ, config.host_id, motor_id), data))
        return std::nullopt;
    if (bus.is_dry_run())
        return 0.0;

    auto reply = wait_for(bus, config.timeout_ms * 3, [&](const can_frame& frame) {
        return comm_type(frame) == COMM_PARAM_READ &&
               source_motor_id(frame) == motor_id &&
               destination_id(frame) == config.host_id &&
               frame.can_dlc == 8 && frame.data[0] == data[0] && frame.data[1] == data[1];
    });
    if (!reply)
        return std::nullopt;

    float position = 0.0F;
    std::memcpy(&position, &reply->data[4], sizeof(position));
    return static_cast<double>(position);
}

std::string uid_to_string(const DeviceInfo& device)
{
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0');
    for (uint8_t byte : device.uid)
        out << std::setw(2) << unsigned(byte);
    return out.str();
}

bool confirm_destructive(const Config& config, const std::string& question)
{
    if (config.assume_yes || config.dry_run)
        return true;
    if (!::isatty(STDIN_FILENO)) {
        std::cerr << "Refusing a non-interactive destructive operation. Add --yes to confirm.\n";
        return false;
    }

    std::cout << question << " [y/N] " << std::flush;
    std::string answer;
    std::getline(std::cin, answer);
    return answer == "y" || answer == "Y" || answer == "yes" || answer == "YES";
}

int parse_integer(const std::string& text, int minimum, int maximum, const char* label)
{
    size_t parsed = 0;
    long value;
    try {
        value = std::stol(text, &parsed, 0);
    } catch (const std::exception&) {
        throw std::runtime_error(std::string("Invalid ") + label + ": " + text);
    }
    if (parsed != text.size() || value < minimum || value > maximum)
        throw std::runtime_error(std::string(label) + " must be in range " +
                                 std::to_string(minimum) + ".." +
                                 std::to_string(maximum) + ": " + text);
    return static_cast<int>(value);
}

void print_usage(const char* program)
{
    std::cout
        << "Usage: " << program << " [options] <command> [arguments]\n\n"
        << "Commands:\n"
        << "  scan [MIN [MAX]]       Discover motor IDs (default: 0..127)\n"
        << "  get ID                 Read one motor ID and MCU UID\n"
        << "  set-id OLD_ID NEW_ID   Change a motor ID and verify the new ID\n"
        << "  set-zero ID            Stop the motor and set its current angle to zero\n\n"
        << "Options:\n"
        << "  -i, --interface NAME   SocketCAN interface (default: can0)\n"
        << "  --host-id ID           Host ID in decimal or 0xNN (default: 0xFD)\n"
        << "  --timeout MS           Reply timeout per attempt (default: 20)\n"
        << "  --retries N            Query attempts (default: 2)\n"
        << "  -y, --yes              Confirm set-id/set-zero without a prompt\n"
        << "  --dry-run              Print equivalent cansend frames without sending\n"
        << "  -v, --verbose          Print transmitted and received CAN frames\n"
        << "  -h, --help             Show this help\n\n"
        << "Examples:\n"
        << "  " << program << " scan\n"
        << "  " << program << " get 127\n"
        << "  " << program << " --dry-run set-id 127 1\n"
        << "  " << program << " --yes set-zero 1\n";
}

int command_scan(CanBus& bus, const Config& config, const std::vector<std::string>& args)
{
    if (args.size() > 2)
        throw std::runtime_error("scan accepts at most MIN and MAX");
    const int minimum = args.empty() ? MIN_MOTOR_ID :
        parse_integer(args[0], MIN_MOTOR_ID, MAX_MOTOR_ID, "motor ID");
    const int maximum = args.size() < 2 ? MAX_MOTOR_ID :
        parse_integer(args[1], MIN_MOTOR_ID, MAX_MOTOR_ID, "motor ID");
    if (minimum > maximum)
        throw std::runtime_error("scan MIN must not be greater than MAX");

    if (bus.is_dry_run()) {
        std::cout << "Dry-run scan prints one representative query only.\n";
        get_device(bus, config, static_cast<uint8_t>(minimum));
        return 0;
    }

    std::cout << "Scanning " << config.interface << " for RobStride IDs "
              << minimum << ".." << maximum << " ...\n";
    std::vector<DeviceInfo> devices;
    for (int id = minimum; id <= maximum; ++id) {
        auto device = get_device(bus, config, static_cast<uint8_t>(id));
        if (device) {
            devices.push_back(*device);
            std::cout << "  ID " << std::setw(3) << id
                      << "  MCU UID " << uid_to_string(*device) << '\n';
        }
    }

    if (devices.empty()) {
        std::cout << "No motors responded. Check power, CAN-H/L, 120 ohm termination, "
                     "1 Mbps bitrate, and protocol mode.\n";
        return 1;
    }
    std::cout << "Found " << devices.size() << " motor(s).\n";
    return 0;
}

int command_get(CanBus& bus, const Config& config, const std::vector<std::string>& args)
{
    if (args.size() != 1)
        throw std::runtime_error("get requires exactly one motor ID");
    const auto id = static_cast<uint8_t>(
        parse_integer(args[0], MIN_MOTOR_ID, MAX_MOTOR_ID, "motor ID"));
    auto device = get_device(bus, config, id);
    if (!device) {
        std::cerr << "Motor ID " << unsigned(id) << " did not respond.\n";
        return 1;
    }
    if (!bus.is_dry_run())
        std::cout << "Motor ID: " << unsigned(device->motor_id)
                  << "\nMCU UID:  " << uid_to_string(*device) << '\n';
    return 0;
}

int command_set_id(CanBus& bus, const Config& config, const std::vector<std::string>& args)
{
    if (args.size() != 2)
        throw std::runtime_error("set-id requires OLD_ID and NEW_ID");
    const auto old_id = static_cast<uint8_t>(
        parse_integer(args[0], MIN_MOTOR_ID, MAX_MOTOR_ID, "old motor ID"));
    const auto new_id = static_cast<uint8_t>(
        parse_integer(args[1], MIN_MOTOR_ID, MAX_MOTOR_ID, "new motor ID"));
    if (old_id == new_id)
        throw std::runtime_error("OLD_ID and NEW_ID are identical");

    std::optional<DeviceInfo> old_device;
    if (!bus.is_dry_run()) {
        old_device = get_device(bus, config, old_id);
        if (!old_device) {
            std::cerr << "Motor ID " << unsigned(old_id) << " did not respond; no change made.\n";
            return 1;
        }
        if (get_device(bus, config, new_id)) {
            std::cerr << "Motor ID " << unsigned(new_id)
                      << " is already in use; no change made.\n";
            return 1;
        }
        std::cout << "Target MCU UID: " << uid_to_string(*old_device) << '\n';
    }

    if (!confirm_destructive(config,
            "Change motor ID " + std::to_string(old_id) + " -> " +
            std::to_string(new_id) + "?")) {
        std::cout << "Cancelled.\n";
        return 1;
    }

    // Stop first so an ID maintenance operation cannot leave an enabled motor moving.
    if (!send_stop(bus, config, old_id) || !send_set_id(bus, config, old_id, new_id))
        return 1;
    if (bus.is_dry_run())
        return 0;

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    auto changed = get_device(bus, config, new_id);
    if (!changed) {
        std::cerr << "ID-change frame was sent, but motor ID " << unsigned(new_id)
                  << " did not answer verification. Power-cycle only after checking the bus.\n";
        return 1;
    }
    if (old_device && changed->uid != old_device->uid) {
        std::cerr << "Motor ID " << unsigned(new_id)
                  << " answered with a different MCU UID; check for an ID collision.\n";
        return 1;
    }

    std::cout << "Motor ID changed successfully: " << unsigned(old_id)
              << " -> " << unsigned(new_id)
              << "\nMCU UID: " << uid_to_string(*changed) << '\n';
    return 0;
}

int command_set_zero(CanBus& bus, const Config& config, const std::vector<std::string>& args)
{
    if (args.size() != 1)
        throw std::runtime_error("set-zero requires exactly one motor ID");
    const auto id = static_cast<uint8_t>(
        parse_integer(args[0], MIN_MOTOR_ID, MAX_MOTOR_ID, "motor ID"));

    if (!bus.is_dry_run()) {
        auto device = get_device(bus, config, id);
        if (!device) {
            std::cerr << "Motor ID " << unsigned(id) << " did not respond; no change made.\n";
            return 1;
        }
        std::cout << "Target MCU UID: " << uid_to_string(*device) << '\n';
        if (auto before = read_mechanical_position(bus, config, id))
            std::cout << "Current mechanical position: " << *before << " rad\n";
    }

    if (!confirm_destructive(config,
            "Stop motor ID " + std::to_string(id) +
            " and set its current mechanical position to zero?")) {
        std::cout << "Cancelled.\n";
        return 1;
    }

    if (!send_stop(bus, config, id))
        return 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!send_set_zero(bus, config, id))
        return 1;
    if (bus.is_dry_run())
        return 0;

    auto acknowledgement = wait_for(bus, config.timeout_ms * 3, [&](const can_frame& frame) {
        return comm_type(frame) == COMM_FEEDBACK &&
               source_motor_id(frame) == id && destination_id(frame) == config.host_id;
    });
    if (!acknowledgement) {
        std::cerr << "Zero-setting frame was sent, but no feedback acknowledgement arrived.\n";
        return 1;
    }

    std::optional<double> position;
    for (int attempt = 0; attempt < 3 && !position; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        position = read_mechanical_position(bus, config, id);
    }
    if (position) {
        if (std::abs(*position) > 0.1) {
            std::cerr << "Zero command was acknowledged, but read-back is " << *position
                      << " rad (expected within +/-0.1 rad). Check firmware and shaft state.\n";
            return 1;
        }
        std::cout << "Mechanical zero set and verified. Reported position: "
                  << *position << " rad\n";
    } else {
        std::cout << "Mechanical zero command acknowledged. Position read-back was unavailable.\n";
    }
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Config config;
        std::string command;
        std::vector<std::string> command_args;

        for (int index = 1; index < argc; ++index) {
            const std::string arg = argv[index];
            if (command.empty() && (arg == "-h" || arg == "--help")) {
                print_usage(argv[0]);
                return 0;
            }
            if (command.empty() && (arg == "-i" || arg == "--interface")) {
                if (++index >= argc)
                    throw std::runtime_error(arg + " requires a value");
                config.interface = argv[index];
            } else if (command.empty() && arg == "--host-id") {
                if (++index >= argc)
                    throw std::runtime_error(arg + " requires a value");
                config.host_id = static_cast<uint8_t>(
                    parse_integer(argv[index], 0, 255, "host ID"));
            } else if (command.empty() && arg == "--timeout") {
                if (++index >= argc)
                    throw std::runtime_error(arg + " requires a value");
                config.timeout_ms = parse_integer(argv[index], 1, 10000, "timeout");
            } else if (command.empty() && arg == "--retries") {
                if (++index >= argc)
                    throw std::runtime_error(arg + " requires a value");
                config.retries = parse_integer(argv[index], 1, 20, "retries");
            } else if (command.empty() && (arg == "-y" || arg == "--yes")) {
                config.assume_yes = true;
            } else if (command.empty() && arg == "--dry-run") {
                config.dry_run = true;
            } else if (command.empty() && (arg == "-v" || arg == "--verbose")) {
                config.verbose = true;
            } else if (command.empty() && !arg.empty() && arg[0] == '-') {
                throw std::runtime_error("Unknown option: " + arg);
            } else if (command.empty()) {
                command = arg;
            } else {
                command_args.push_back(arg);
            }
        }

        if (command.empty()) {
            print_usage(argv[0]);
            return 2;
        }

        CanBus bus(config);
        if (command == "scan")
            return command_scan(bus, config, command_args);
        if (command == "get")
            return command_get(bus, config, command_args);
        if (command == "set-id")
            return command_set_id(bus, config, command_args);
        if (command == "set-zero")
            return command_set_zero(bus, config, command_args);

        throw std::runtime_error("Unknown command: " + command);
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 2;
    }
}
