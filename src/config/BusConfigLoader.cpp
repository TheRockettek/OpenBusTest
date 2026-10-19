#include "config/BusConfigLoader.h"

#include "config/ModelConfigLoader.h"
#include "core/Variables.h"

VehicleConfig loadBusConfig(const std::filesystem::path& configPath) {
    return loadVehicleConfig(configPath, VehicleFileKind::Bus);
}

ModelConfig loadBusModelConfig(const std::filesystem::path& configPath,
                               const std::filesystem::path& modelRoot,
                               openbus::scripting::Vehicle& variables) {
    return loadModelConfig(configPath, modelRoot, ModelConfigKind::Bus, variables);
}
