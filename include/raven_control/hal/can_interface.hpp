#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace raven_control::hal {

struct CanFrame {
    std::uint32_t id = 0;
    std::uint8_t dlc = 0;
    std::array<std::uint8_t, 8> data{};
};

enum class CanReceiveResult {
    Received,
    Timeout,
    Error,
};

class CanTransport {
public:
    virtual ~CanTransport() = default;

    [[nodiscard]] virtual bool send(const CanFrame& frame) = 0;
    [[nodiscard]] virtual CanReceiveResult receive(
        CanFrame& frame,
        std::chrono::milliseconds timeout) = 0;
};

class CanInterface final : public CanTransport {
public:
    explicit CanInterface(const std::string& interface_name);
    ~CanInterface() override;

    CanInterface(const CanInterface&) = delete;
    CanInterface& operator=(const CanInterface&) = delete;

    [[nodiscard]] bool send(const CanFrame& frame) override;
    [[nodiscard]] CanReceiveResult receive(
        CanFrame& frame,
        std::chrono::milliseconds timeout) override;

    [[nodiscard]] const std::string& interfaceName() const noexcept;

private:
    void closeSocket() noexcept;

    std::string interface_name_;
    int socket_ = -1;
};

}  // namespace raven_control::hal
