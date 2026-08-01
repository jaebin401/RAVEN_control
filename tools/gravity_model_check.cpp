#include "raven_control/dynamics/gravity_compensator.hpp"

#include <array>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {

constexpr double PI = 3.14159265358979323846;

double degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
}

void printResult(
    const raven_control::dynamics::GravityCompensator& compensator,
    const std::array<double, 3>& degrees)
{
    raven_control::dynamics::JointVector radians{};
    for (std::size_t index = 0; index < degrees.size(); ++index)
        radians[index] = degreesToRadians(degrees[index]);

    const auto torque = compensator.compute(radians);
    std::cout
        << std::fixed << std::setprecision(6)
        << "q_deg: [" << degrees[0] << ", " << degrees[1]
        << ", " << degrees[2] << "]\n"
        << "tau_gravity_nm: [" << torque[0] << ", " << torque[1]
        << ", " << torque[2] << "]\n";
}

}  // namespace

int main(int argc, char* argv[])
{
    try {
        const raven_control::dynamics::GravityCompensator compensator(
            raven_control::dynamics::makeRavenUrdfGravityModel());

        if (argc == 1) {
            printResult(compensator, {0.0, 0.0, 0.0});
            printResult(compensator, {0.0, 45.0, 45.0});
            return 0;
        }
        if (argc != 4) {
            std::cerr
                << "Usage: gravity_model_check "
                << "[shoulder_deg upperArm_deg foreArm_deg]\n";
            return 2;
        }

        printResult(
            compensator,
            {
                std::stod(argv[1]),
                std::stod(argv[2]),
                std::stod(argv[3]),
            });
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Gravity model check failed: "
                  << error.what() << '\n';
        return 1;
    }
}
