#include "physics/BusConfiguration.h"
#include "ProbeTempDirectory.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>

namespace {

bool nearlyEqual(double first, double second) {
    return std::abs(first - second) < 1.0e-9;
}

} // namespace

int main() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_articulated_config_probe");
    const std::filesystem::path busPath = root / "articulated.bus";
    {
        std::ofstream bus(busPath);
        bus << "[model]\nmodel.cfg\n"
               "[mass]\n12.0\n"
               "[boundingbox]\n2.4\n12.0\n3.2\n0\n0\n1.6\n"
               "[schwerpunkt]\n1.2\n"
               "[newachse]\nachse_long\n0\n"
               "achse_maxwidth\n2.2\n"
               "achse_minwidth\n1.8\n"
               "achse_raddurchmesser\n1.0\n"
               "achse_feder\n200\n"
               "achse_maxforce\n100\n"
               "achse_daempfer\n20\n"
               "achse_antrieb\n0\n"
               "[openbus_articulated]\n1\n"
               "[coupling_front]\n0\n4.788\n0.501\n"
               "[coupling_back]\n0\n-4.331\n0.315\n"
               "[coupling_front_character]\n52.5\n-20\n20\n1\n";
    }
    {
        std::ofstream model(root / "model.cfg");
        model << "[mesh]\nbody.obj\n";
    }
    {
        std::ofstream mesh(root / "body.obj");
        mesh << "v 0 0 0\n";
    }

    try {
        const BusConfiguration configuration = loadBusConfiguration(busPath);
        const bool couplingMetadataTransferred =
            configuration.articulated &&
            configuration.frontCouplingPoint ==
                std::optional<std::array<double, 3>>({0.0, 4.788, 0.501}) &&
            configuration.rearCouplingPoint ==
                std::optional<std::array<double, 3>>({0.0, -4.331, 0.315}) &&
            configuration.frontCouplingCharacter.has_value() &&
            nearlyEqual(configuration.frontCouplingCharacter->alphaDegrees, 52.5) &&
            nearlyEqual(configuration.frontCouplingCharacter->minimumBetaDegrees, -20.0) &&
            nearlyEqual(configuration.frontCouplingCharacter->maximumBetaDegrees, 20.0) &&
            configuration.frontCouplingCharacter->type == 1;
        const VehiclePlacement frontPlacement{{10.0, -4.0, 1.5}, 37.0, 12.0, -8.0};
        const VehiclePlacement rearPlacement =
            articulatedRearPlacement(configuration, frontPlacement, configuration);
        const auto worldCouplingPoint = [](const VehiclePlacement& placement,
                                           const std::array<double, 3>& point) {
            const double yawRadians = placement.yawDegrees * std::acos(-1.0) / 180.0;
            const double cosine = std::cos(yawRadians);
            const double sine = std::sin(yawRadians);
            return std::array<double, 3>{
                placement.position[0] + cosine * point[0] - sine * point[1],
                placement.position[1] + sine * point[0] + cosine * point[1],
                placement.position[2] + point[2]};
        };
        const std::array<double, 3> frontAnchor =
            worldCouplingPoint(frontPlacement, *configuration.rearCouplingPoint);
        const std::array<double, 3> rearAnchor =
            worldCouplingPoint(rearPlacement, *configuration.frontCouplingPoint);
        const bool couplingAnchorsAlign =
            std::equal(frontAnchor.begin(), frontAnchor.end(), rearAnchor.begin(), nearlyEqual);
        openbus::probe::removeProbeDirectory(root);
        if (!couplingMetadataTransferred || !couplingAnchorsAlign) {
            std::cerr << "Articulated config transfer or initial coupling-anchor alignment failed\n";
            return 1;
        }
    } catch (const std::exception& error) {
        openbus::probe::removeProbeDirectory(root);
        std::cerr << "Unable to load articulated configuration fixture: " << error.what() << '\n';
        return 2;
    }
    return 0;
}
