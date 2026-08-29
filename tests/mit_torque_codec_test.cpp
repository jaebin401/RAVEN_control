#include "raven_control/hal/mit_torque_codec.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void check(bool condition, const std::string& message)
{
    if (!condition)
        throw std::runtime_error(message);
}

void testVirtualWorkAndWireAudit()
{
    const auto audit = raven_control::hal::auditMitTorqueCommand(
        -1,
        2.0,
        1.2);
    check(std::abs(audit.motor_torque_nm + 0.6) < 1e-12,
          "joint-to-motor torque must preserve virtual work");
    check(std::abs(audit.decoded_joint_torque_nm - 1.2) < 0.002,
          "wire round trip must recover joint torque within one LSB");
    check(
        audit.encoded_torque ==
            raven_control::hal::encodeRs02Torque(-0.6),
        "audit must expose the exact torque field used on the wire");
}

void testRs02Endpoints()
{
    check(raven_control::hal::encodeRs02Torque(-17.0) == 0,
          "negative RS02 endpoint must encode as zero");
    check(raven_control::hal::encodeRs02Torque(17.0) == 65535,
          "positive RS02 endpoint must encode as 65535");
    check(std::abs(raven_control::hal::decodeRs02Torque(0) + 17.0) <
              1e-12,
          "negative RS02 endpoint must decode exactly");
    check(std::abs(raven_control::hal::decodeRs02Torque(65535) - 17.0) <
              1e-12,
          "positive RS02 endpoint must decode exactly");
}

void testInvalidCalibrationIsRejected()
{
    bool rejected = false;
    try {
        (void)raven_control::hal::auditMitTorqueCommand(0, 1.0, 1.0);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    check(rejected, "invalid position sign must be rejected");
}

}  // namespace

int main()
{
    testVirtualWorkAndWireAudit();
    testRs02Endpoints();
    testInvalidCalibrationIsRejected();
    std::cout << "mit_torque_codec_test passed\n";
    return 0;
}
