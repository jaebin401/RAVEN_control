#include "raven_control/dynamics/gravity_compensator.hpp"
#include "raven_control/dynamics/pinocchio_gravity_model.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr double PI = 3.14159265358979323846;

double degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
}

void printVector(
    const char* label,
    const raven_control::dynamics::JointVector& value)
{
    std::cout
        << label << ": ["
        << value[0] << ", "
        << value[1] << ", "
        << value[2] << "]\n";
}

void compareAtPose(
    const raven_control::dynamics::PinocchioGravityModel& pinocchio_model,
    const raven_control::dynamics::GravityCompensator& reference_model,
    const std::array<double, 3>& degrees)
{
    raven_control::dynamics::JointVector radians{};
    for (std::size_t index = 0; index < degrees.size(); ++index)
        radians[index] = degreesToRadians(degrees[index]);

    const auto pinocchio_torque = pinocchio_model.compute(radians);
    const auto reference_torque = reference_model.compute(radians);
    raven_control::dynamics::JointVector difference{};
    for (std::size_t index = 0; index < difference.size(); ++index) {
        difference[index] =
            pinocchio_torque[index] - reference_torque[index];
    }

    std::cout
        << std::fixed << std::setprecision(9)
        << "q_deg: [" << degrees[0] << ", " << degrees[1]
        << ", " << degrees[2] << "]\n";
    printVector("tau_pinocchio_nm", pinocchio_torque);
    printVector("tau_reference_nm", reference_torque);
    printVector("difference_nm", difference);
}

}  // namespace

int main(int argc, char* argv[])
{
    try {
        if (argc != 2 && argc != 5) {
            std::cerr
                << "Usage: pinocchio_gravity_model_check <urdf_path> "
                << "[shoulder_deg upperArm_deg foreArm_deg]\n";
            return 2;
        }

        const raven_control::dynamics::PinocchioGravityModel
            pinocchio_model(argv[1]);
        const raven_control::dynamics::GravityCompensator reference_model(
            raven_control::dynamics::makeRavenUrdfGravityModel());

        std::cout << "URDF: " << pinocchio_model.urdfPath() << '\n';
        if (argc == 2) {
            compareAtPose(
                pinocchio_model, reference_model, {0.0, 0.0, 0.0});
            compareAtPose(
                pinocchio_model, reference_model, {0.0, 45.0, 45.0});
            return 0;
        }

        compareAtPose(
            pinocchio_model,
            reference_model,
            {
                std::stod(argv[2]),
                std::stod(argv[3]),
                std::stod(argv[4]),
            });
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Pinocchio gravity model check failed: "
                  << error.what() << '\n';
        return 1;
    }
}
