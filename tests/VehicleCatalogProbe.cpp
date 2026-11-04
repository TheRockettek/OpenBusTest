#include "config/VehicleCatalog.h"
#include "config/HofData.h"
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
    std::ofstream defaultTexture(textureDirectory / "default.dds");
    defaultTexture << "texture";
    defaultTexture.close();

    const std::filesystem::path ctiPath = textureDirectory / "paint.cti";
    std::ofstream cti(ctiPath);
    cti << "[item]\nDefault Paint\nBody\ndefault.dds\n"
           "[item]\nRed Livery\nBody\nred.dds\n"
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
    hof << "[name]\nExample Depot\n"
           "[infosystem_trip]\n1001\nOutbound\n0\n100\n"
           "[infosystem_busstop_list]\n2\nSTOP_A\nSTOP_B\n";
    hof.close();
    const std::filesystem::path otherBusDirectory = vehiclesDirectory / "OtherBus";
    std::filesystem::create_directories(otherBusDirectory);
    const std::filesystem::path otherHofPath = otherBusDirectory / "Other Depot.hof";
    std::ofstream otherHof(otherHofPath);
    otherHof << "[name]\nOther Depot\n";
    otherHof.close();

    const std::vector<openbus::config::VehicleCatalogEntry> vehicles =
        openbus::config::discoverVehicleCatalog(vehiclesDirectory);
    const std::vector<openbus::config::VehicleAppearance> appearances =
        openbus::config::discoverVehicleAppearances(busPath, vehiclesDirectory);
    const std::vector<openbus::config::VehicleDepot> depots =
        openbus::config::discoverVehicleDepots(busPath, vehiclesDirectory);
    const std::vector<openbus::config::VehicleDepot> allDepots =
        openbus::config::discoverAllVehicleDepots(vehiclesDirectory);
    const openbus::config::HofData parsedHof = openbus::config::loadHofData(hofPath);
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
    const auto* defaultAppearance =
        openbus::config::findVehicleAppearance(appearances, "default paint");
    const bool passed =
        vehicles.size() == 1 && vehicles[0].manufacturer == "Example Motors" &&
        vehicles[0].type == "City Bus" && vehicles[0].defaultAppearance == "Default Paint" &&
        appearances.size() == 3 && defaultAppearance != nullptr &&
        defaultAppearance->textureReplacements.at("body") ==
            std::filesystem::absolute(textureDirectory / "default.dds").generic_string() &&
        redAppearance != appearances.end() &&
        redAppearance->textureReplacements.size() == 2 &&
        redAppearance->textureReplacements.at("body") ==
            std::filesystem::absolute(textureDirectory / "red.dds").generic_string() &&
        blueAppearance != appearances.end() &&
        blueAppearance->textureReplacements.at("body") ==
            std::filesystem::absolute(parentTexturePath).generic_string() &&
        depots.size() == 1 && depots[0].name == "Example Depot" &&
        depots[0].path == hofPath && allDepots.size() == 2 &&
        parsedHof.routes.size() == 1 && parsedHof.routes[0].code == "1001" &&
        parsedHof.routes[0].line == "100" &&
        parsedHof.routes[0].busstopIdents == std::vector<std::string>({"STOP_A", "STOP_B"}) &&
        std::ranges::any_of(allDepots, [&](const openbus::config::VehicleDepot& depot) {
            return depot.name == "Other Depot" && depot.path == otherHofPath;
        });
    openbus::probe::removeProbeDirectory(root);
    if (!passed) {
        std::cerr << "Vehicle catalog discovery failed\n";
        return 1;
    }
    return 0;
}
