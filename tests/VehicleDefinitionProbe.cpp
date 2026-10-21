#include "app/vehicle/VehicleDefinition.h"
#include "ProbeTempDirectory.h"

#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_vehicle_definition_probe");
    std::filesystem::create_directories(root / "model");

    const std::filesystem::path vehicleConfigPath = root / "ai_car.ovh";
    std::ofstream vehicleConfig(vehicleConfigPath);
    vehicleConfig << "[friendlyname]\nTest\nAI car\nDefault\n"
                     "[model]\nmodel/model.cfg\n";
    vehicleConfig.close();

    std::ofstream modelConfig(root / "model/model.cfg");
    modelConfig << "[mesh]\ncar.o3d\n";
    modelConfig.close();
    std::ofstream mesh(root / "model/car.o3d", std::ios::binary);
    mesh.close();

    const std::filesystem::path modelConfigPath = root / "model/model.cfg";
    const std::shared_ptr<const SharedVehicleDefinition> definition =
        sharedVehicleDefinition(vehicleConfigPath, modelConfigPath, VehicleFileKind::Vehicle);
    const bool loaded = definition->vehicleConfiguration.friendlyVehicleName == "AI car" &&
                        definition->modelConfigPath == modelConfigPath &&
                        definition->modelRoot == root / "model" &&
                        !definition->modelConfiguration.diagnostics.hasErrors() &&
                        definition->modelConfiguration.parts.size() == 1 &&
                        definition->modelConfiguration.parts[0].objPath == root / "model/car.o3d";
    openbus::probe::removeProbeDirectory(root);

    if (!loaded) {
        std::cerr << "Shared vehicle definition did not load an OVH config and vehicle model\n";
        return 1;
    }
    return 0;
}
