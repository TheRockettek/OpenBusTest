#include "core/WeatherEnvironment.h"

#include <array>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace {

bool setEnvironment(const char* name, const char* value) {
#ifdef _WIN32
    return _putenv_s(name, value == nullptr ? "" : value) == 0;
#else
    return value == nullptr ? unsetenv(name) == 0 : setenv(name, value, 1) == 0;
#endif
}

void set(const char* name, const char* value) {
    if (!setEnvironment(name, value)) {
        throw std::runtime_error(std::string("Could not set ") + name);
    }
}

bool require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
    }
    return condition;
}

} // namespace

int main(int argc, char** argv) {
    for (const char* name :
         {"OPENBUS_WEATHER", "OPENBUS_WEATHER_RATE", "OPENBUS_WEATHER_TEMPERATURE",
          "OPENBUS_WEATHER_ABSHUM", "OPENBUS_WEATHER_VISIBILITY", "OPENBUS_WEATHER_BRIGHTNESS",
          "OPENBUS_WEATHER_FOG_HEIGHT",
          "OPENBUS_WEATHER_CLOUD_TYPE", "OPENBUS_WEATHER_CLOUD_BASE",
          "OPENBUS_WEATHER_WIND_DIRECTION", "OPENBUS_WEATHER_WIND_SPEED",
          "OPENBUS_METAR_STATION"}) {
        set(name, nullptr);
    }
    if (argc == 3 && std::string_view(argv[1]) == "--live") {
        set("OPENBUS_METAR_STATION", argv[2]);
        set("OPENBUS_WEATHER_BRIGHTNESS", "0.25");
        set("OPENBUS_WEATHER_CLOUD_TYPE", "cumulus1");
        try {
            const auto observed = openbus::weatherFromEnvironment();
            if (observed.brightness != 0.25 ||
                observed.cloudType != openbus::WeatherCloudType::Cumulus1 ||
                !observed.visibilityMeters || !observed.windSpeedMetersPerSecond) {
                std::cerr << "Live NOAA weather did not reach the startup weather snapshot\n";
                return 1;
            }
            std::cout << "Live NOAA report reached the startup weather snapshot\n";
            return 0;
        } catch (const std::exception& error) {
            std::cerr << "Live NOAA weather initialization failed: " << error.what() << '\n';
            return 1;
        }
    }
    bool passed = true;
    auto weather = openbus::weatherFromEnvironment();
    passed &= require(weather.precipitationType == 0 && weather.precipitationRate == 0.0 &&
                          weather.temperatureCelsius == 20.0 &&
                          weather.absoluteHumidityGramsPerCubicMeter == 10.0 &&
                          !weather.visibilityMeters && !weather.brightness,
                      "unset weather settings did not preserve defaults");
    set("OPENBUS_WEATHER_BRIGHTNESS", "0.25");
    weather = openbus::weatherFromEnvironment();
    passed &= require(weather.brightness == 0.25 && !weather.visibilityMeters,
                      "brightness was not parsed independently of visibility");
    set("OPENBUS_WEATHER_BRIGHTNESS", nullptr);
    set("OPENBUS_WEATHER_FOG_HEIGHT", "40");
    weather = openbus::weatherFromEnvironment();
    passed &= require(weather.fogTopAboveGroundMeters == 40.0 && !weather.visibilityMeters,
                      "fog height was not parsed independently of visibility");
    set("OPENBUS_WEATHER_VISIBILITY", "1250");
    weather = openbus::weatherFromEnvironment();
    passed &= require(weather.visibilityMeters == 1250.0 &&
                          weather.fogTopAboveGroundMeters == 40.0 && !weather.brightness,
                      "visibility and fog height were not parsed independently of brightness");
    set("OPENBUS_WEATHER", "SnOw");
    set("OPENBUS_WEATHER_RATE", "0.75");
    set("OPENBUS_WEATHER_TEMPERATURE", "-5");
    set("OPENBUS_WEATHER_ABSHUM", "3.5");
    set("OPENBUS_WEATHER_BRIGHTNESS", "0");
    weather = openbus::weatherFromEnvironment();
    passed &= require(weather.precipitationType == 2 && weather.precipitationRate == 0.75 &&
                          weather.temperatureCelsius == -5.0 &&
                          weather.absoluteHumidityGramsPerCubicMeter == 3.5 &&
                          weather.visibilityMeters == 1250.0 && weather.brightness == 0.0,
                      "combined weather settings were not parsed correctly");
    set("OPENBUS_WEATHER_RATE", nullptr);
    set("OPENBUS_WEATHER", "rain");
    passed &= require(openbus::weatherFromEnvironment().precipitationRate == 0.5,
                      "rain did not retain its default precipitation rate");
    set("OPENBUS_WEATHER_CLOUD_TYPE", "CuMuLuS2");
    set("OPENBUS_WEATHER_CLOUD_BASE", "1200");
    set("OPENBUS_WEATHER_WIND_DIRECTION", "270");
    set("OPENBUS_WEATHER_WIND_SPEED", "8");
    weather = openbus::weatherFromEnvironment();
    passed &= require(weather.cloudType == openbus::WeatherCloudType::Cumulus2 &&
                          weather.cloudBaseAboveGroundMeters == 1200.0 &&
                          weather.windDirectionDegrees == 270.0 &&
                          weather.windSpeedMetersPerSecond == 8.0,
                      "cloud and wind settings were not parsed correctly");
    set("OPENBUS_WEATHER_CLOUD_TYPE", nullptr);
    set("OPENBUS_WEATHER_CLOUD_BASE", nullptr);
    set("OPENBUS_WEATHER_WIND_DIRECTION", nullptr);
    set("OPENBUS_WEATHER_WIND_SPEED", nullptr);
    set("OPENBUS_WEATHER_BRIGHTNESS", nullptr);
    set("OPENBUS_WEATHER_CLOUD_TYPE", "overcast1");
    weather = openbus::weatherFromEnvironment();
    passed &=
        require(weather.cloudType == openbus::WeatherCloudType::Overcast1 && !weather.brightness,
                "cloud type was not parsed independently of brightness");
    set("OPENBUS_WEATHER_CLOUD_TYPE", nullptr);

    for (const auto& test : std::array{std::pair{"OPENBUS_WEATHER_BRIGHTNESS", "-0.01"},
                                       std::pair{"OPENBUS_WEATHER_BRIGHTNESS", "1.01"},
                                       std::pair{"OPENBUS_WEATHER_BRIGHTNESS", "nan"},
                                       std::pair{"OPENBUS_WEATHER_BRIGHTNESS", "0.5extra"},
                                       std::pair{"OPENBUS_WEATHER_VISIBILITY", "-1"},
                                       std::pair{"OPENBUS_WEATHER_VISIBILITY", "inf"},
                                       std::pair{"OPENBUS_WEATHER_FOG_HEIGHT", "-1"},
                                       std::pair{"OPENBUS_WEATHER_FOG_HEIGHT", "inf"},
                                       std::pair{"OPENBUS_WEATHER_RATE", "1.1"},
                                       std::pair{"OPENBUS_WEATHER_TEMPERATURE", "-273.15"},
                                       std::pair{"OPENBUS_WEATHER_ABSHUM", "-1"},
                                       std::pair{"OPENBUS_WEATHER_CLOUD_TYPE", "cirrus"},
                                       std::pair{"OPENBUS_WEATHER_CLOUD_BASE", "-1"},
                                       std::pair{"OPENBUS_WEATHER_CLOUD_BASE", "inf"},
                                       std::pair{"OPENBUS_WEATHER_WIND_DIRECTION", "360"},
                                       std::pair{"OPENBUS_WEATHER_WIND_DIRECTION", "-1"},
                                       std::pair{"OPENBUS_WEATHER_WIND_SPEED", "-1"},
                                       std::pair{"OPENBUS_WEATHER", "hail"}}) {
        set(test.first, test.second);
        bool rejected = false;
        try {
            static_cast<void>(openbus::weatherFromEnvironment());
        } catch (const std::exception& error) {
            rejected = std::string(error.what()).find(test.first) != std::string::npos;
            if (!rejected) {
                std::cerr << test.first << " produced an unexpected error: " << error.what()
                          << '\n';
            }
        }
        if (!rejected)
            std::cerr << "invalid setting: " << test.first << '=' << test.second << '\n';
        passed &= require(rejected, "invalid weather setting was not rejected with its name");
        set(test.first, nullptr);
        if (std::string(test.first) == "OPENBUS_WEATHER") {
            set("OPENBUS_WEATHER", "rain");
        }
    }
    set("OPENBUS_WEATHER", "clear");
    set("OPENBUS_WEATHER_RATE", "0.1");
    bool clearRateRejected = false;
    try {
        static_cast<void>(openbus::weatherFromEnvironment());
    } catch (const std::runtime_error&) {
        clearRateRejected = true;
    }
    passed &= require(clearRateRejected, "clear weather accepted a nonzero precipitation rate");
    set("OPENBUS_WEATHER_RATE", nullptr);
    set("OPENBUS_WEATHER", nullptr);
    set("OPENBUS_METAR_STATION", "invalid/station");
    bool invalidStationRejected = false;
    try {
        static_cast<void>(openbus::weatherFromEnvironment());
    } catch (const std::invalid_argument&) {
        invalidStationRejected = true;
    }
    passed &= require(invalidStationRejected, "invalid METAR station was not rejected");
    set("OPENBUS_METAR_STATION", "KJFK");
    for (const auto& conflict : std::array{
             std::pair{"OPENBUS_WEATHER", "rain"}, std::pair{"OPENBUS_WEATHER_RATE", "0.5"},
             std::pair{"OPENBUS_WEATHER_TEMPERATURE", "10"},
             std::pair{"OPENBUS_WEATHER_ABSHUM", "5"},
             std::pair{"OPENBUS_WEATHER_VISIBILITY", "1000"},
             std::pair{"OPENBUS_WEATHER_CLOUD_BASE", "1000"},
             std::pair{"OPENBUS_WEATHER_WIND_DIRECTION", "90"},
             std::pair{"OPENBUS_WEATHER_WIND_SPEED", "5"}}) {
        set(conflict.first, conflict.second);
        bool conflictingWeatherRejected = false;
        try {
            static_cast<void>(openbus::weatherFromEnvironment());
        } catch (const std::invalid_argument& error) {
            conflictingWeatherRejected = std::string(error.what()).find(conflict.first) !=
                                         std::string::npos;
        }
        passed &= require(conflictingWeatherRejected,
                          "manual and METAR weather conflict was not rejected");
        set(conflict.first, nullptr);
    }
    return passed ? 0 : 1;
}
