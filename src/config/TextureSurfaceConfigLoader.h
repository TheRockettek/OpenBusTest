#pragma once

#include "config/ConfigurationTypes.h"

#include <filesystem>
#include <optional>

namespace openbus::config {

struct TextureSurfaceConfiguration {
    bool configFound = false;
    bool supportsPuddles = false;
    bool supportsMoisture = false;
    std::optional<int> surfaceType;
    std::optional<int> nightMapMode;
    ConfigurationDiagnostics diagnostics;
};

TextureSurfaceConfiguration
loadTextureSurfaceConfiguration(const std::filesystem::path& texturePath);

} // namespace openbus::config
