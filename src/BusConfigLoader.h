#pragma once

#include "ModelConfigTypes.h"
#include "VehicleConfigLoader.h"

#include <filesystem>

namespace openbus::scripting {
class Vehicle;
}

VehicleConfig loadBusConfig(const std::filesystem::path& configPath);
ModelConfig loadBusModelConfig(const std::filesystem::path& configPath,
                               const std::filesystem::path& modelRoot,
                               openbus::scripting::Vehicle& variables);
