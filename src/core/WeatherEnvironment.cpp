#include "core/WeatherEnvironment.h"

#include "core/Environment.h"
#include "core/MetarNoaa.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>

namespace openbus {

WeatherState weatherFromEnvironment() {
    const std::string station = environmentValue("OPENBUS_METAR_STATION");
    if (!station.empty()) {
        static_cast<void>(noaaMetarStation(station));
        for (const char* name :
             {"OPENBUS_WEATHER", "OPENBUS_WEATHER_RATE", "OPENBUS_WEATHER_TEMPERATURE",
              "OPENBUS_WEATHER_ABSHUM", "OPENBUS_WEATHER_VISIBILITY", "OPENBUS_WEATHER_CLOUD_BASE",
              "OPENBUS_WEATHER_WIND_DIRECTION", "OPENBUS_WEATHER_WIND_SPEED"}) {
            if (!environmentValue(name).empty()) {
                throw std::invalid_argument(std::string(name) +
                                            " cannot be combined with OPENBUS_METAR_STATION");
            }
        }
    }
    WeatherState weather;
    std::string precipitation = environmentValue("OPENBUS_WEATHER");
    std::transform(
        precipitation.begin(), precipitation.end(), precipitation.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    if (precipitation.empty() || precipitation == "clear") {
        weather.precipitationType = 0;
    } else if (precipitation == "rain") {
        weather.precipitationType = 1;
    } else if (precipitation == "snow") {
        weather.precipitationType = 2;
    } else {
        throw std::runtime_error("OPENBUS_WEATHER must be clear, rain, or snow");
    }

    weather.precipitationRate = weather.precipitationType == 0 ? 0.0 : 0.5;
    const std::string configuredRate = environmentValue("OPENBUS_WEATHER_RATE");
    if (!configuredRate.empty()) {
        char* end = nullptr;
        weather.precipitationRate = std::strtod(configuredRate.c_str(), &end);
        if (end == configuredRate.c_str() || *end != '\0' ||
            !std::isfinite(weather.precipitationRate) || weather.precipitationRate < 0.0 ||
            weather.precipitationRate > 1.0 ||
            (weather.precipitationType == 0 && weather.precipitationRate != 0.0)) {
            throw std::runtime_error(
                "OPENBUS_WEATHER_RATE must be in [0, 1] and non-zero only for rain or snow");
        }
    }
    weather.temperatureCelsius =
        finiteEnvironmentDouble("OPENBUS_WEATHER_TEMPERATURE", weather.temperatureCelsius);
    weather.absoluteHumidityGramsPerCubicMeter = finiteEnvironmentDouble(
        "OPENBUS_WEATHER_ABSHUM", weather.absoluteHumidityGramsPerCubicMeter);
    if (!environmentValue("OPENBUS_WEATHER_VISIBILITY").empty()) {
        weather.visibilityMeters = finiteEnvironmentDouble("OPENBUS_WEATHER_VISIBILITY", 0.0);
        if (*weather.visibilityMeters < 0.0) {
            throw std::runtime_error("OPENBUS_WEATHER_VISIBILITY must be non-negative");
        }
    }
    if (!environmentValue("OPENBUS_WEATHER_FOG_HEIGHT").empty()) {
        weather.fogTopAboveGroundMeters =
            finiteEnvironmentDouble("OPENBUS_WEATHER_FOG_HEIGHT", 0.0);
        if (*weather.fogTopAboveGroundMeters < 0.0) {
            throw std::runtime_error("OPENBUS_WEATHER_FOG_HEIGHT must be non-negative");
        }
    }
    if (!environmentValue("OPENBUS_WEATHER_BRIGHTNESS").empty()) {
        weather.brightness = finiteEnvironmentDouble("OPENBUS_WEATHER_BRIGHTNESS", 1.0);
        if (*weather.brightness < 0.0 || *weather.brightness > 1.0) {
            throw std::runtime_error("OPENBUS_WEATHER_BRIGHTNESS must be in [0, 1]");
        }
    }
    std::string cloudType = environmentValue("OPENBUS_WEATHER_CLOUD_TYPE");
    std::transform(
        cloudType.begin(), cloudType.end(), cloudType.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    if (cloudType == "cumulus1") {
        weather.cloudType = WeatherCloudType::Cumulus1;
    } else if (cloudType == "cumulus2") {
        weather.cloudType = WeatherCloudType::Cumulus2;
    } else if (cloudType == "cumulus3") {
        weather.cloudType = WeatherCloudType::Cumulus3;
    } else if (cloudType == "overcast1") {
        weather.cloudType = WeatherCloudType::Overcast1;
    } else if (!cloudType.empty()) {
        throw std::runtime_error(
            "OPENBUS_WEATHER_CLOUD_TYPE must be cumulus1, cumulus2, cumulus3, or overcast1");
    }
    for (const auto& setting :
         {std::pair{"OPENBUS_WEATHER_CLOUD_BASE", &weather.cloudBaseAboveGroundMeters},
          std::pair{"OPENBUS_WEATHER_WIND_DIRECTION", &weather.windDirectionDegrees},
          std::pair{"OPENBUS_WEATHER_WIND_SPEED", &weather.windSpeedMetersPerSecond}}) {
        if (!environmentValue(setting.first).empty()) {
            const double value = finiteEnvironmentDouble(setting.first, 0.0);
            if (value < 0.0) {
                throw std::runtime_error(std::string(setting.first) + " must be non-negative");
            }
            if (setting.second == &weather.windDirectionDegrees && value >= 360.0) {
                throw std::runtime_error(std::string(setting.first) + " must be below 360");
            }
            *setting.second = value;
        }
    }
    if (weather.temperatureCelsius <= -273.15) {
        throw std::runtime_error("OPENBUS_WEATHER_TEMPERATURE must be above absolute zero");
    }
    if (weather.absoluteHumidityGramsPerCubicMeter < 0.0) {
        throw std::runtime_error("OPENBUS_WEATHER_ABSHUM must be non-negative");
    }
    validateWeatherState(weather);
    if (!station.empty()) {
        const auto report =
            parseNoaaMetar(station, fetchNoaaMetar(station), std::chrono::system_clock::now());
        WeatherState observed = weatherFromMetar(report.observation);
        observed.brightness = weather.brightness;
        observed.cloudType = weather.cloudType;
        observed.fogTopAboveGroundMeters = weather.fogTopAboveGroundMeters;
        validateWeatherState(observed);
        return observed;
    }
    return weather;
}

} // namespace openbus
