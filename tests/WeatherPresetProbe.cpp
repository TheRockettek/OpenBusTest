#include "config/WeatherPresetLoader.h"
#include "core/Variables.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--installed") {
        const auto presets = openbus::config::discoverWeatherPresets(argv[2]);
        for (const auto& preset : presets) {
            try {
                static_cast<void>(openbus::config::loadWeatherPreset(preset.path));
            } catch (const std::exception& error) {
                std::cerr << preset.name << ": " << error.what() << '\n';
                return 1;
            }
        }
        std::cout << "Loaded " << presets.size() << " installed weather presets\n";
        return 0;
    }
    if (argc != 2) {
        std::cerr << "Expected weather fixture directory\n";
        return 1;
    }
    bool passed = true;
    const auto check = [&](bool condition, const char* message) {
        if (!condition) {
            std::cerr << message << '\n';
            passed = false;
        }
    };
    const std::filesystem::path directory = argv[1];
    const auto presets = openbus::config::discoverWeatherPresets(directory);
    check(presets.size() == 3, "weather discovery missed or included non-OWT files");
    check(std::any_of(presets.begin(), presets.end(),
                      [](const auto& preset) { return preset.name == "Test Rain (English)"; }),
          "English preset label was not loaded");
    const auto rain = openbus::config::loadWeatherPreset(directory / "Rain.owt");
    check(rain.weather.precipitationType == 1 &&
              std::abs(rain.weather.precipitationRate - 128.0 / 255.0) < 1e-10 &&
              rain.weather.visibilityMeters == 1200 && rain.weather.windDirectionDegrees == 270 &&
              rain.weather.windSpeedMetersPerSecond == 8 &&
              rain.weather.cloudType == openbus::WeatherCloudType::Overcast1 &&
              rain.weather.cloudBaseAboveGroundMeters == 900 &&
              rain.weather.temperatureCelsius == 10 &&
              rain.weather.atmosphericPressureHectopascals == 1000 &&
              rain.weather.omsiFogHeight == 0.5 &&
              rain.weather.omsiPrecipitationParameters ==
                  std::array<double, 3>{0.0, -2.0, 0.0} &&
              rain.weather.omsiGroundWetness == std::array<double, 3>{20.0, 255.0, 0.0} &&
              rain.weather.absoluteHumidityGramsPerCubicMeter > 6 &&
              rain.weather.absoluteHumidityGramsPerCubicMeter < 7 && !rain.warnings.empty(),
          "weather preset values were not converted");
    SystemVariables variables;
    variables.setWeather(rain.weather);
    check(variables.get("preciptype") == 1 && variables.get("weather_temperature") == 10 &&
              variables.weatherCloudType() == openbus::WeatherCloudType::Overcast1 &&
              variables.weather().omsiFogHeight == 0.5 &&
              variables.weather().omsiGroundWetness == rain.weather.omsiGroundWetness,
          "weather preset did not reach script and renderer state");
    const auto clear = openbus::config::loadWeatherPreset(directory / "Clear.owt");
    check(clear.weather.precipitationType == 0 &&
              std::abs(clear.weather.precipitationRate - 192.0 / 255.0) < 1e-10 &&
              !clear.weather.cloudType && clear.weather.windDirectionDegrees == 0 &&
              !clear.weather.omsiPrecipitationParameters && !clear.weather.omsiGroundWetness &&
              !clear.weather.omsiSnow && !clear.weather.omsiSnowOnRoad &&
              std::none_of(clear.warnings.begin(), clear.warnings.end(), [](const auto& warning) {
                  return warning == "Additional precipitation fields are not applied.";
              }),
          "clear preset or 360-degree wind was converted incorrectly");
    variables.setWeather(clear.weather);
    check(variables.get("preciptype") == 0 &&
              std::abs(variables.get("preciprate") - 192.0F / 255.0F) < 1e-6F,
          "clear preset intensity was not published to scripts");
    bool invalid = false;
    try {
        static_cast<void>(openbus::config::loadWeatherPreset(directory / "Invalid.owt"));
    } catch (const std::exception&) {
        invalid = true;
    }
    check(invalid, "malformed preset was accepted");
    bool missing = false;
    try {
        static_cast<void>(openbus::config::discoverWeatherPresets(directory / "missing"));
    } catch (const std::exception&) {
        missing = true;
    }
    check(missing, "missing weather directory was not reported");
    const auto temp = std::filesystem::temp_directory_path() /
                      ("openbus-weather-" +
                       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(temp);
    const auto utf16 = temp / "weather.owt";
    {
        std::ifstream source(directory / "Clear.owt", std::ios::binary);
        std::ofstream output(utf16, std::ios::binary);
        output.put(static_cast<char>(0xff));
        output.put(static_cast<char>(0xfe));
        for (char c; source.get(c);) {
            output.put(c);
            output.put('\0');
        }
        if (!source.eof() || !output)
            throw std::runtime_error("Could not write UTF-16 fixture");
    }
    check(std::abs(openbus::config::loadWeatherPreset(utf16).weather.precipitationRate -
                   192.0 / 255.0) < 1e-10,
          "UTF-16LE preset was not parsed");
    const auto localized = temp / "weather_ENG.dsc";
    {
        std::ofstream output(localized, std::ios::binary);
        output.put(static_cast<char>(0xff));
        output.put(static_cast<char>(0xfe));
        for (char c : std::string("[name]\nEnglish weather\n")) {
            output.put(c);
            output.put('\0');
        }
        if (!output)
            throw std::runtime_error("Could not write UTF-16 label fixture");
    }
    check(openbus::config::discoverWeatherPresets(temp).front().name == "English weather",
          "UTF-16 English label was not loaded");
    std::filesystem::remove(localized);
    check(openbus::config::discoverWeatherPresets(temp).front().name == "weather",
          "missing English label did not fall back to the file stem");
    {
        std::ofstream output(utf16, std::ios::binary | std::ios::app);
        output.put('\0');
    }
    bool malformedUtf16 = false;
    try {
        static_cast<void>(openbus::config::loadWeatherPreset(utf16));
    } catch (const std::exception&) {
        malformedUtf16 = true;
    }
    check(malformedUtf16, "odd-length UTF-16 preset was accepted");
    {
        std::ifstream source(directory / "Clear.owt", std::ios::binary);
        std::ofstream output(utf16, std::ios::binary | std::ios::trunc);
        output << source.rdbuf() << "\n[wind]\n0\n1\n";
        if (!output)
            throw std::runtime_error("Could not write duplicate-tag fixture");
    }
    bool duplicate = false;
    try {
        static_cast<void>(openbus::config::loadWeatherPreset(utf16));
    } catch (const std::exception&) {
        duplicate = true;
    }
    check(duplicate, "duplicate preset tag was accepted");
    {
        std::ifstream source(directory / "Clear.owt");
        std::string contents((std::istreambuf_iterator<char>(source)), {});
        const auto fog = contents.find("[fog]\n50000\n");
        if (fog == std::string::npos) throw std::runtime_error("Missing fixture fog record");
        contents.replace(fog, std::string("[fog]\n50000\n").size(), "[fog]\n0\n");
        std::ofstream output(utf16, std::ios::binary | std::ios::trunc);
        output << contents;
        if (!output) throw std::runtime_error("Could not write zero-fog fixture");
    }
    check(openbus::config::loadWeatherPreset(utf16).weather.visibilityMeters == 100000,
          "non-positive preset fog range did not use OMSI's 100000 fallback");
    {
        std::ifstream source(directory / "Rain.owt");
        std::string contents((std::istreambuf_iterator<char>(source)), {});
        const auto precip = contents.find("[precip]\n1\n128\n");
        if (precip == std::string::npos) throw std::runtime_error("Missing fixture precipitation");
        contents.replace(precip, std::string("[precip]\n1\n128\n").size(),
                         "[precip]\n1\n128.5\n");
        std::ofstream output(utf16, std::ios::binary | std::ios::trunc);
        output << contents;
        if (!output) throw std::runtime_error("Could not write fractional precipitation fixture");
    }
    bool fractionalPrecip = false;
    try {
        static_cast<void>(openbus::config::loadWeatherPreset(utf16));
    } catch (const std::exception&) {
        fractionalPrecip = true;
    }
    check(fractionalPrecip, "fractional native precipitation byte was accepted");
    {
        std::ifstream source(directory / "Clear.owt");
        std::ofstream output(utf16, std::ios::binary | std::ios::trunc);
        output << source.rdbuf() << "\n[snow]\n[snowOnRoad]\n";
        if (!output) throw std::runtime_error("Could not write snow-flags fixture");
    }
    const auto snow = openbus::config::loadWeatherPreset(utf16);
    check(snow.weather.omsiSnow && snow.weather.omsiSnowOnRoad,
          "presence-only OMSI snow flags were not retained");
    variables.setWeather(snow.weather);
    variables.setWeather(clear.weather);
    check(!variables.weather().omsiSnow && !variables.weather().omsiSnowOnRoad,
          "switching presets retained stale snow flags");
    {
        std::ifstream source(directory / "Clear.owt");
        std::ofstream output(utf16, std::ios::binary | std::ios::trunc);
        output << source.rdbuf() << "\n[groundwet]\n1\n2\n[snow]\n";
        if (!output) throw std::runtime_error("Could not write malformed wetness fixture");
    }
    bool malformedWetness = false;
    try {
        static_cast<void>(openbus::config::loadWeatherPreset(utf16));
    } catch (const std::exception&) {
        malformedWetness = true;
    }
    check(malformedWetness, "incomplete ground wetness was accepted");
    {
        std::ifstream source(directory / "Clear.owt");
        std::ofstream output(utf16, std::ios::binary | std::ios::trunc);
        output << source.rdbuf() << "nan\n0\n0\n";
        if (!output) throw std::runtime_error("Could not write malformed precipitation fixture");
    }
    bool malformedAdditionalPrecip = false;
    try {
        static_cast<void>(openbus::config::loadWeatherPreset(utf16));
    } catch (const std::exception&) {
        malformedAdditionalPrecip = true;
    }
    check(malformedAdditionalPrecip, "invalid additional precipitation was accepted");
    std::filesystem::remove(utf16);
    std::filesystem::remove(temp);
    return passed ? 0 : 1;
}
