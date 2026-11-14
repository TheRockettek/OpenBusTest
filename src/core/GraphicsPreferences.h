#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace openbus::core {

struct GraphicsPreferences {
    std::string windowMode = "windowed";
    int width = 1920;
    int height = 1080;
    int refreshRate = 60;
    int monitor = 0;
    int anisotropicFiltering = 1;
    bool generateTextureMipmaps = false;
    double mapTextureScale = 1.0;
    double vehicleTextureScale = 1.0;
    bool vehicleFrustumCulling = true;
    std::string antiAliasingMode = "off";
    double renderScale = 1.0;
    bool vsync = true;
    double fpsCap = 0.0;
    int reflectionSize = 1024;
    int reflectionInterval = 1;
    double reflectionMaxFps = 0.0;
    double minimumObjectSizeMainPercent = 0.0;
    double minimumObjectSizeMirrorsPercent = 0.0;

    bool operator==(const GraphicsPreferences&) const = default;
};

struct GraphicsPreferencesFile {
    GraphicsPreferences values;
    std::vector<std::string> lines;
};

[[nodiscard]] std::filesystem::path graphicsPreferencesPath();
[[nodiscard]] double parseGraphicsFpsCapOverride(std::string_view value);
[[nodiscard]] GraphicsPreferencesFile loadGraphicsPreferences(const std::filesystem::path& path);
[[nodiscard]] GraphicsPreferences mergeGraphicsPreferences(const GraphicsPreferences& baseline,
                                                           const GraphicsPreferences& edited,
                                                           GraphicsPreferences disk);
void saveGraphicsPreferences(const std::filesystem::path& path,
                             const GraphicsPreferencesFile& preferences);

} // namespace openbus::core
