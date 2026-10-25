#include "config/VehicleCatalog.h"
#include "ProbeTempDirectory.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_vehicle_catalog_probe");
    const std::filesystem::path vehiclesDirectory = root / "Vehicles";
    const std::filesystem::path busDirectory = vehiclesDirectory / "Example" / "CityBus";
    const std::filesystem::path textureDirectory = busDirectory / "Texture" / "Paints";
    std::filesystem::create_directories(textureDirectory);

    const std::filesystem::path busPath = busDirectory / "city.bus";
    std::ofstream bus(busPath);
    bus << "[friendlyname]\nExample Motors\nCity Bus\nDefault Paint\n[model]\nmodel.cfg\n";
    bus.close();
    std::ofstream alternativeTexture(textureDirectory / "red.dds");
    alternativeTexture << "texture";
    alternativeTexture.close();

    const std::filesystem::path ctiPath = textureDirectory / "paint.cti";
    std::ofstream cti(ctiPath);
    cti << "[item]\nRed Livery\nBody\nred.dds\n"
           "[item]\nRed Livery\nInterior\nred.dds\n";
    cti.close();

    const std::filesystem::path parentTexturePath = busDirectory.parent_path() / "blue.dds";
    std::ofstream parentTexture(parentTexturePath);
    parentTexture << "texture";
    parentTexture.close();
    const std::filesystem::path parentCtiPath = busDirectory.parent_path() / "paint.cti";
    std::ofstream parentCti(parentCtiPath);
    parentCti << "[item]\nBlue Livery\nBody\nblue.dds\n";
    parentCti.close();

    const std::filesystem::path hofPath = busDirectory.parent_path() / "Example Depot.hof";
    std::ofstream hof(hofPath);
    hof << "[name]\nExample Depot\n";
    hof.close();

    const std::vector<openbus::config::VehicleCatalogEntry> vehicles =
        openbus::config::discoverVehicleCatalog(vehiclesDirectory);
    const std::vector<openbus::config::VehicleAppearance> appearances =
        openbus::config::discoverVehicleAppearances(busPath, vehiclesDirectory);
    const std::vector<openbus::config::VehicleDepot> depots =
        openbus::config::discoverVehicleDepots(busPath, vehiclesDirectory);
    const auto redAppearance =
        std::find_if(appearances.begin(), appearances.end(),
                     [](const openbus::config::VehicleAppearance& appearance) {
                         return appearance.name == "Red Livery";
                     });
    const auto blueAppearance =
        std::find_if(appearances.begin(), appearances.end(),
                     [](const openbus::config::VehicleAppearance& appearance) {
                         return appearance.name == "Blue Livery";
                     });
    const bool passed =
        vehicles.size() == 1 && vehicles[0].manufacturer == "Example Motors" &&
        vehicles[0].type == "City Bus" && vehicles[0].defaultAppearance == "Default Paint" &&
        appearances.size() == 2 && redAppearance != appearances.end() &&
        redAppearance->textureReplacements.size() == 2 &&
        redAppearance->textureReplacements.at("body") ==
            std::filesystem::absolute(textureDirectory / "red.dds").generic_string() &&
        blueAppearance != appearances.end() &&
        blueAppearance->textureReplacements.at("body") ==
            std::filesystem::absolute(parentTexturePath).generic_string() &&
        depots.size() == 1 && depots[0].name == "Example Depot" &&
        depots[0].path == hofPath;
    openbus::probe::removeProbeDirectory(root);
    if (!passed) {
        std::cerr << "Vehicle catalog discovery failed\n";
        return 1;
    }
    return 0;
}
