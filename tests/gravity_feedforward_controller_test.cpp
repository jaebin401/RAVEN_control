#include "raven_control/control/gravity_feedforward_controller.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using Clock = std::chrono::steady_clock;
using JointVector = raven_control::dynamics::JointVector;

constexpr double TOLERANCE = 1e-12;

int failures = 0;

void check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

class FixedGravityModel final
    : public raven_control::dynamics::GravityModel {
public:
    explicit FixedGravityModel(JointVector torque)
        : torque_(torque)
    {
    }

    JointVector compute(const JointVector&) const override
    {
        ++call_count;
        return torque_;
    }

    mutable int call_count = 0;

private:
    JointVector torque_;
};

raven_control::config::GravityCompensationConfig activeConfig()
{
    raven_control::config::GravityCompensationConfig config;
    config.enabled = true;
    config.dry_run = false;
    config.scale = 0.5;
    config.ramp_duration = std::chrono::milliseconds(1000);
    config.max_joint_torque_nm = {2.0, 0.4, 2.0};
    return config;
}

void testDisabledReturnsZeroWithoutEvaluatingModel()
{
    auto model = std::make_unique<FixedGravityModel>(
        JointVector{0.0, 1.0, -1.0});
    auto* model_ptr = model.get();
    raven_control::config::GravityCompensationConfig config;
    raven_control::control::GravityFeedforwardController controller(
        std::move(model), config);

    const auto result = controller.compute(
        {0.0, 0.0, 0.0}, true, Clock::now());
    check(!result.enabled, "disabled result must report disabled");
    check(result.commanded_torque_nm == JointVector{},
          "disabled controller must command zero torque");
    check(model_ptr->call_count == 0,
          "disabled controller must not evaluate the model");
}

void testRampScaleClampAndCommand()
{
    auto model = std::make_unique<FixedGravityModel>(
        JointVector{1.0, 2.0, -3.0});
    raven_control::control::GravityFeedforwardController controller(
        std::move(model), activeConfig());

    const auto start = Clock::now();
    controller.reset(start);
    raven_control::control::GravityFeedforwardResult result;
    for (int step = 1; step <= 10; ++step) {
        result = controller.compute(
            {0.0, 0.0, 0.0},
            true,
            start + std::chrono::milliseconds(step * 100));
    }

    check(std::abs(result.ramp_factor - 1.0) < TOLERANCE,
          "ramp must reach one after its configured duration");
    check(std::abs(result.raw_gravity_torque_nm[1] - 2.0) < TOLERANCE,
          "result must expose raw gravity torque");
    check(std::abs(result.ramped_gravity_torque_nm[1] - 1.0) <
              TOLERANCE,
          "scale and ramp must be applied before limiting");
    check(std::abs(result.limited_gravity_torque_nm[1] - 0.4) <
              TOLERANCE,
          "joint torque must be clamped to its configured limit");
    check(result.torque_clamped[1],
          "result must identify clamped torque");
    check(std::abs(result.commanded_torque_nm[1] - 0.4) < TOLERANCE,
          "non-dry-run controller must command limited torque");
}

void testDryRunComputesButCommandsZero()
{
    auto config = activeConfig();
    config.dry_run = true;
    config.ramp_duration = std::chrono::milliseconds(100);
    auto model = std::make_unique<FixedGravityModel>(
        JointVector{0.0, 1.0, 0.0});
    raven_control::control::GravityFeedforwardController controller(
        std::move(model), config);

    const auto start = Clock::now();
    controller.reset(start);
    const auto result = controller.compute(
        {0.0, 0.0, 0.0},
        true,
        start + std::chrono::milliseconds(100));
    check(result.input_valid, "dry-run must still evaluate valid input");
    check(result.limited_gravity_torque_nm[1] > 0.0,
          "dry-run must expose the torque it would apply");
    check(result.commanded_torque_nm == JointVector{},
          "dry-run must command zero torque");
}

void testInvalidFeedbackAndStateFailClosed()
{
    auto model = std::make_unique<FixedGravityModel>(
        JointVector{0.0, 1.0, 0.0});
    raven_control::control::GravityFeedforwardController controller(
        std::move(model), activeConfig());
    const auto start = Clock::now();
    controller.reset(start);

    const auto stale = controller.compute(
        {0.0, 0.0, 0.0},
        false,
        start + std::chrono::milliseconds(20));
    check(!stale.input_valid,
          "stale feedback must invalidate gravity output");
    check(stale.commanded_torque_nm == JointVector{},
          "stale feedback must command zero new torque");
    check(controller.rampFactor() == 0.0,
          "stale feedback must reset the enable ramp");

    const auto invalid = controller.compute(
        {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0},
        true,
        start + std::chrono::milliseconds(40));
    check(!invalid.input_valid,
          "non-finite state must invalidate gravity output");
    check(invalid.commanded_torque_nm == JointVector{},
          "non-finite state must command zero torque");
}

void testDisableIsImmediate()
{
    auto model = std::make_unique<FixedGravityModel>(
        JointVector{0.0, 1.0, 0.0});
    raven_control::control::GravityFeedforwardController controller(
        std::move(model), activeConfig());
    const auto now = Clock::now();
    controller.reset(now);
    (void)controller.compute(
        {0.0, 0.0, 0.0},
        true,
        now + std::chrono::milliseconds(100));
    controller.setEnabled(false, now + std::chrono::milliseconds(101));
    const auto result = controller.compute(
        {0.0, 0.0, 0.0},
        true,
        now + std::chrono::milliseconds(120));
    check(result.commanded_torque_nm == JointVector{},
          "disable must command zero without a ramp-down delay");
    check(controller.rampFactor() == 0.0,
          "disable must clear the ramp state");
}

}  // namespace

int main()
{
    testDisabledReturnsZeroWithoutEvaluatingModel();
    testRampScaleClampAndCommand();
    testDryRunComputesButCommandsZero();
    testInvalidFeedbackAndStateFailClosed();
    testDisableIsImmediate();

    if (failures != 0) {
        std::cerr << failures
                  << " gravity feedforward controller test(s) failed\n";
        return 1;
    }
    std::cout << "All gravity feedforward controller tests passed\n";
    return 0;
}
