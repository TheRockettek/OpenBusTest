#include "BusConfigLoader.h"

#include "ModelConfigLoader.h"

VehicleConfig loadBusConfig(const std::filesystem::path& configPath) {
    return loadVehicleConfig(configPath, VehicleFileKind::Bus);
}

ModelConfig loadBusModelConfig(const std::filesystem::path& configPath,
                               const std::filesystem::path& modelRoot, Variables& variables) {
    return loadModelConfig(configPath, modelRoot, ModelConfigKind::Bus, variables);
}
