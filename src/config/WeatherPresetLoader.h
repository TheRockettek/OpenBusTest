#pragma once

#include "core/WeatherState.h"

#include <filesystem>
#include <string>
#include <vector>

namespace openbus::config {

struct WeatherPresetFile {
    std::filesystem::path path;
    std::string name;
};

struct LoadedWeatherPreset {
    WeatherState weather;
    std::vector<std::string> warnings;
};

std::vector<WeatherPresetFile> discoverWeatherPresets(const std::filesystem::path& directory);
LoadedWeatherPreset loadWeatherPreset(const std::filesystem::path& path);

} // namespace openbus::config
