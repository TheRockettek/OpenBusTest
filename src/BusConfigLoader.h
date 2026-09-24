#pragma once

#include "ModelConfigTypes.h"
#include "VehicleConfigLoader.h"

#include <filesystem>

class Variables;

VehicleConfig loadBusConfig(const std::filesystem::path& configPath);
ModelConfig loadBusModelConfig(const std::filesystem::path& configPath,
                               const std::filesystem::path& modelRoot, Variables& variables);
