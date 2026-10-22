#pragma once

// Parsed, immutable vehicle definitions shared between vehicles that use the
// same .bus/.ovh and model CFG pair, plus per-placement setup helpers.

#include "config/VehicleConfigLoader.h"
#include "core/BusTypes.h"
#include "render/BusModelLoader.h"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

struct SharedVehicleDefinition {
    std::string cacheKey;
    std::filesystem::path modelConfigPath;
    std::filesystem::path modelRoot;
    VehicleConfig vehicleConfiguration;
    openbus::rendering::BusModelLoadResult modelConfiguration;
    std::vector<std::string> modelNumericVariables;
    std::vector<std::string> modelStringVariables;
};

// Thread-safe; returns the cached definition when the same pair was loaded before.
[[nodiscard]] std::shared_ptr<const SharedVehicleDefinition>
sharedVehicleDefinition(const std::filesystem::path& busConfigPath,
                        const std::filesystem::path& modelConfigPath, VehicleFileKind configKind);

void applyVehiclePlacement(const VehiclePlacement& placement);

void selectRuntimeRegistration(VehicleConfig& configuration, const VehiclePlacement& placement);
