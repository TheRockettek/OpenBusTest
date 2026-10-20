#include "app/vehicle/VehicleDefinition.h"

#include "config/BusConfigLoader.h"
#include "core/Environment.h"
#include "core/Logger.h"
#include "core/Variables.h"
#include "render/CameraMath.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <unordered_map>

using openbus::rendering::rotate;
using openbus::rendering::translate;

namespace {

std::string normalizedConfigPath(const std::filesystem::path& path) {
    std::string key = std::filesystem::absolute(path).lexically_normal().generic_string();
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return key;
}

std::filesystem::path modelRootFor(const std::filesystem::path& modelConfigPath) {
    const std::filesystem::path directory = modelConfigPath.parent_path();
    std::string directoryName = directory.filename().string();
    std::transform(
        directoryName.begin(), directoryName.end(), directoryName.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return directoryName == "configuration files" ? directory.parent_path() : directory;
}

} // namespace

std::shared_ptr<const SharedVehicleDefinition>
sharedVehicleDefinition(const std::filesystem::path& busConfigPath,
                        const std::filesystem::path& modelConfigPath) {
    static std::mutex cacheMutex;
    static std::unordered_map<std::string, std::shared_ptr<const SharedVehicleDefinition>> cache;
    const std::string key =
        normalizedConfigPath(busConfigPath) + "|" + normalizedConfigPath(modelConfigPath);
    std::lock_guard<std::mutex> lock(cacheMutex);
    const auto existing = cache.find(key);
    if (existing != cache.end()) {
        return existing->second;
    }

    auto definition = std::make_shared<SharedVehicleDefinition>();
    definition->cacheKey = key;
    definition->modelConfigPath = modelConfigPath;
    definition->modelRoot = modelRootFor(modelConfigPath);
    gameLog.Log("Loading bus model with config: " + modelConfigPath.string() +
                " and model root: " + definition->modelRoot.string());
    definition->vehicleConfiguration = loadBusConfig(busConfigPath);

    openbus::scripting::Vehicle modelVariables;
    definition->modelConfiguration =
        openbus::rendering::loadBusModel(modelConfigPath, definition->modelRoot, modelVariables);
    for (const auto& [name, value] : modelVariables.numericValues()) {
        (void)value;
        definition->modelNumericVariables.push_back(name);
    }
    for (const auto& [name, value] : modelVariables.stringValues()) {
        (void)value;
        definition->modelStringVariables.push_back(name);
    }
    for (const ConfigurationDiagnostic& diagnostic :
         definition->modelConfiguration.diagnostics.entries) {
        const std::string severity =
            diagnostic.severity == ConfigurationDiagnostic::Severity::Error ? "error" : "warning";
        gameLog.Log("CFG " + severity + " line " + std::to_string(diagnostic.line) + " [" +
                    diagnostic.keyword + "]: " + diagnostic.message);
    }

    std::shared_ptr<const SharedVehicleDefinition> shared = std::move(definition);
    cache.emplace(key, shared);
    return shared;
}

void applyVehiclePlacement(const VehiclePlacement& placement) {
    translate(placement.position[0], placement.position[1], placement.position[2]);
    rotate(placement.yawDegrees, 0.0, 0.0, 1.0);
}

void selectRuntimeRegistration(VehicleConfig& configuration, const VehiclePlacement& placement) {
    if (const char* overrideValue = openbus::getEnvironment("OPENBUS_REGISTRATION");
        overrideValue != nullptr && *overrideValue != '\0') {
        configuration.selectedRegistration = overrideValue;
        return;
    }

    const std::size_t registrationCount = registrationOptionCount(configuration);
    if (configuration.registrationFree && registrationCount > 0) {
        std::size_t selectedIndex = 0;
        if (const char* indexValue = openbus::getEnvironment("OPENBUS_REGISTRATION_INDEX");
            indexValue != nullptr && *indexValue != '\0') {
            try {
                selectedIndex = std::stoull(indexValue);
            } catch (const std::exception&) {
                selectedIndex = 0;
            }
        }
        selectRegistrationAtIndex(configuration, selectedIndex);
        return;
    }

    if (configuration.registrationAutomatic) {
        // Keep automatic numbers repeatable for a vehicle placement while avoiding
        // process-global random state. The explicit override remains available for
        // multiplayer/replay workflows that need a prescribed registration.
        std::string seed = configuration.sourcePath.generic_string();
        seed += ':' + std::to_string(placement.position[0]);
        seed += ':' + std::to_string(placement.position[1]);
        seed += ':' + std::to_string(placement.position[2]);
        seed += ':' + std::to_string(placement.yawDegrees);
        std::uint32_t hash = 2166136261U;
        for (const unsigned char character : seed) {
            hash ^= character;
            hash *= 16777619U;
        }
        std::ostringstream generated;
        generated << configuration.registrationPrefix << std::setfill('0') << std::setw(3)
                  << (hash % 1000U);
        configuration.selectedRegistration = generated.str();
    }
}
