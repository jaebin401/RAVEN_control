#include "raven_control/hal/can_interface.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <poll.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace raven_control::hal {

CanInterface::CanInterface(const std::string& interface_name)
    : interface_name_(interface_name)
{
    if (interface_name_.empty() || interface_name_.size() >= IFNAMSIZ) {
        throw std::invalid_argument(
            "Invalid CAN interface name: '" + interface_name_ + "'");
    }

    socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        throw std::runtime_error(
            "SocketCAN socket creation failed: " +
            std::string(std::strerror(errno)));
    }

    ifreq interface_request{};
    std::strncpy(
        interface_request.ifr_name,
        interface_name_.c_str(),
        IFNAMSIZ - 1);
    if (::ioctl(socket_, SIOCGIFINDEX, &interface_request) < 0) {
        const std::string reason = std::strerror(errno);
        closeSocket();
        throw std::runtime_error(
            "Cannot find CAN interface '" + interface_name_ +
            "': " + reason);
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = interface_request.ifr_ifindex;
    if (::bind(
            socket_,
            reinterpret_cast<sockaddr*>(&address),
            sizeof(address)) < 0) {
        const std::string reason = std::strerror(errno);
        closeSocket();
        throw std::runtime_error(
            "Cannot bind to CAN interface '" + interface_name_ +
            "': " + reason);
    }
}

CanInterface::~CanInterface()
{
    closeSocket();
}

bool CanInterface::send(const CanFrame& frame)
{
    if (socket_ < 0 || frame.dlc > frame.data.size())
        return false;

    can_frame linux_frame{};
    linux_frame.can_id = (frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    linux_frame.can_dlc = frame.dlc;
    std::copy_n(
        frame.data.begin(),
        frame.dlc,
        linux_frame.data);

    const ssize_t written =
        ::write(socket_, &linux_frame, sizeof(linux_frame));
    return written == static_cast<ssize_t>(sizeof(linux_frame));
}

CanReceiveResult CanInterface::receive(
    CanFrame& frame,
    std::chrono::milliseconds timeout)
{
    if (socket_ < 0)
        return CanReceiveResult::Error;

    pollfd descriptor{};
    descriptor.fd = socket_;
    descriptor.events = POLLIN;

    const auto bounded_timeout = std::clamp<long long>(
        timeout.count(),
        0,
        static_cast<long long>(std::numeric_limits<int>::max()));

    int poll_result;
    do {
        poll_result = ::poll(
            &descriptor,
            1,
            static_cast<int>(bounded_timeout));
    } while (poll_result < 0 && errno == EINTR);

    if (poll_result == 0)
        return CanReceiveResult::Timeout;
    if (poll_result < 0)
        return CanReceiveResult::Error;

    can_frame linux_frame{};
    const ssize_t received =
        ::read(socket_, &linux_frame, sizeof(linux_frame));
    if (received != static_cast<ssize_t>(sizeof(linux_frame)))
        return CanReceiveResult::Error;
    if ((linux_frame.can_id & CAN_EFF_FLAG) == 0)
        return CanReceiveResult::Timeout;

    frame = {};
    frame.id = linux_frame.can_id & CAN_EFF_MASK;
    frame.dlc = std::min<std::uint8_t>(
        linux_frame.can_dlc,
        static_cast<std::uint8_t>(frame.data.size()));
    std::copy_n(
        linux_frame.data,
        frame.dlc,
        frame.data.begin());
    return CanReceiveResult::Received;
}

const std::string& CanInterface::interfaceName() const noexcept
{
    return interface_name_;
}

void CanInterface::closeSocket() noexcept
{
    if (socket_ >= 0) {
        ::close(socket_);
        socket_ = -1;
    }
}

}  // namespace raven_control::hal
