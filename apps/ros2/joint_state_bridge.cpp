#include "raven_control/config/motor_config.hpp"
#include "raven_control/hal/can_interface.hpp"
#include "raven_control/hal/rs02_operation_feedback.hpp"
#include "raven_control/telemetry/joint_state_aggregator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

class JointStateBridge final : public rclcpp::Node {
public:
    JointStateBridge()
        : rclcpp::Node("raven_joint_state_bridge")
    {
        const std::string interface_name = declare_parameter<std::string>(
            "can_interface",
            "can0");
        const std::string motor_config_path =
            declare_parameter<std::string>(
                "motor_config_path",
                "config/motor_config.yaml");
        const int publish_rate_hz = declare_parameter<int>(
            "publish_rate_hz",
            50);

        const auto motor_config =
            raven_control::config::loadMotorRuntimeConfig(
                motor_config_path);
        const int feedback_timeout_ms = declare_parameter<int>(
            "feedback_timeout_ms",
            static_cast<int>(motor_config.feedback_timeout.count()));
        if (publish_rate_hz <= 0 || publish_rate_hz > 200) {
            throw std::invalid_argument(
                "publish_rate_hz must be in range 1..200");
        }
        if (feedback_timeout_ms <= 0) {
            throw std::invalid_argument(
                "feedback_timeout_ms must be positive");
        }

        feedback_timeout_ =
            std::chrono::milliseconds(feedback_timeout_ms);
        publish_period_ = std::chrono::microseconds(
            1000000 / publish_rate_hz);
        transport_ =
            std::make_unique<raven_control::hal::CanInterface>(
                interface_name);
        aggregator_ = std::make_unique<
            raven_control::telemetry::JointStateAggregator>(
                motor_config.joints);
        publisher_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::QoS(10).reliable());

        RCLCPP_INFO(
            get_logger(),
            "Read-only Type 17/Type 2 bridge listening on %s for %zu joints; "
            "publishing /joint_states at %d Hz",
            interface_name.c_str(),
            aggregator_->jointCount(),
            publish_rate_hz);
        RCLCPP_INFO(
            get_logger(),
            "This process never enables, stops, or commands a motor");
    }

    void run()
    {
        auto next_publish = std::chrono::steady_clock::now();
        auto next_stale_warning = next_publish;
        while (rclcpp::ok()) {
            receiveAvailableFeedback();

            const auto now = std::chrono::steady_clock::now();
            if (now < next_publish)
                continue;
            do {
                next_publish += publish_period_;
            } while (next_publish <= now);

            const auto snapshot = aggregator_->snapshot(
                now,
                feedback_timeout_);
            if (!snapshot) {
                if (now >= next_stale_warning) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Waiting for one fresh Type 17 or Type 2 frame from "
                        "every joint; "
                        "/joint_states is not being published");
                    next_stale_warning = now + std::chrono::seconds(1);
                }
                continue;
            }

            sensor_msgs::msg::JointState message;
            message.header.stamp = get_clock()->now();
            message.name.reserve(snapshot->joints.size());
            message.position.reserve(snapshot->joints.size());
            if (snapshot->velocity_and_effort_valid) {
                message.velocity.reserve(snapshot->joints.size());
                message.effort.reserve(snapshot->joints.size());
            }
            for (const auto& joint : snapshot->joints) {
                message.name.push_back(joint.joint_name);
                message.position.push_back(joint.position_rad);
                if (snapshot->velocity_and_effort_valid) {
                    message.velocity.push_back(joint.velocity_rad_s);
                    message.effort.push_back(joint.effort_nm);
                }
            }
            publisher_->publish(message);
        }
    }

private:
    void receiveAvailableFeedback()
    {
        constexpr std::size_t MAX_FRAMES_PER_PASS = 64;
        for (std::size_t count = 0;
             count < MAX_FRAMES_PER_PASS && rclcpp::ok();
             ++count) {
            raven_control::hal::CanFrame frame;
            const auto result = transport_->receive(
                frame,
                std::chrono::milliseconds(1));
            if (result == raven_control::hal::CanReceiveResult::Timeout)
                return;
            if (result == raven_control::hal::CanReceiveResult::Error) {
                throw std::runtime_error(
                    "SocketCAN receive failed in read-only bridge");
            }

            const auto decoded =
                raven_control::hal::decodeRs02OperationFeedback(frame);
            if (decoded) {
                (void)aggregator_->ingest(
                    *decoded,
                    std::chrono::steady_clock::now());
                continue;
            }

            const auto mechanical_position =
                raven_control::hal::
                    decodeRs02MechanicalPositionFeedback(frame);
            if (mechanical_position) {
                (void)aggregator_->ingest(
                    *mechanical_position,
                    std::chrono::steady_clock::now());
            }
        }
    }

    std::unique_ptr<raven_control::hal::CanInterface> transport_;
    std::unique_ptr<
        raven_control::telemetry::JointStateAggregator> aggregator_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::chrono::milliseconds feedback_timeout_{250};
    std::chrono::microseconds publish_period_{20000};
};

}  // namespace

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto bridge = std::make_shared<JointStateBridge>();
        bridge->run();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& error) {
        RCLCPP_FATAL(
            rclcpp::get_logger("raven_joint_state_bridge"),
            "%s",
            error.what());
        rclcpp::shutdown();
        return 1;
    }
}
