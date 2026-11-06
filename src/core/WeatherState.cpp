#include "core/WeatherState.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace openbus {

void validateWeatherState(const WeatherState& weather) {
    const auto nonnegative = [](const std::optional<double>& value) {
        return !value || (std::isfinite(*value) && *value >= 0.0);
    };
    const auto finiteTriple = [](const std::optional<std::array<double, 3>>& values) {
        return !values || (std::isfinite((*values)[0]) && std::isfinite((*values)[1]) &&
                           std::isfinite((*values)[2]));
    };
    const double maxScriptValue = static_cast<double>(std::numeric_limits<float>::max());
    if (weather.precipitationType < 0 || weather.precipitationType > 2 ||
        !std::isfinite(weather.precipitationRate) || weather.precipitationRate < 0.0 ||
        weather.precipitationRate > 1.0 || !std::isfinite(weather.temperatureCelsius) ||
        weather.temperatureCelsius <= -273.15 || weather.temperatureCelsius > maxScriptValue ||
        !std::isfinite(weather.absoluteHumidityGramsPerCubicMeter) ||
        weather.absoluteHumidityGramsPerCubicMeter < 0.0 ||
        weather.absoluteHumidityGramsPerCubicMeter > maxScriptValue ||
        !nonnegative(weather.visibilityMeters) || !nonnegative(weather.windSpeedMetersPerSecond) ||
        !nonnegative(weather.cloudBaseAboveGroundMeters) ||
        (weather.cloudBaseAboveGroundMeters &&
         *weather.cloudBaseAboveGroundMeters > MAX_CLOUD_BASE_ABOVE_GROUND_METERS) ||
        !nonnegative(weather.fogTopAboveGroundMeters) ||
        (weather.fogTopAboveGroundMeters && *weather.fogTopAboveGroundMeters > maxScriptValue) ||
        (weather.omsiFogHeight && !std::isfinite(*weather.omsiFogHeight)) ||
        !finiteTriple(weather.omsiPrecipitationParameters) ||
        !finiteTriple(weather.omsiGroundWetness) ||
        (weather.brightness && (!std::isfinite(*weather.brightness) || *weather.brightness < 0.0 ||
                                *weather.brightness > 1.0)) ||
        (weather.windDirectionDegrees &&
         (!std::isfinite(*weather.windDirectionDegrees) || *weather.windDirectionDegrees < 0.0 ||
          *weather.windDirectionDegrees >= 360.0)) ||
        (weather.atmosphericPressureHectopascals &&
         (!std::isfinite(*weather.atmosphericPressureHectopascals) ||
          *weather.atmosphericPressureHectopascals <= 0.0))) {
        throw std::invalid_argument("Weather settings are invalid");
    }
    if (weather.cloudType) {
        switch (*weather.cloudType) {
        case WeatherCloudType::Cumulus1:
        case WeatherCloudType::Cumulus2:
        case WeatherCloudType::Cumulus3:
        case WeatherCloudType::Overcast1:
            break;
        default:
            throw std::invalid_argument("Weather cloud type is invalid");
        }
    }
}

WeatherState customWeatherFromDraft(const WeatherState& draft) {
    const double maxValue = static_cast<double>(std::numeric_limits<float>::max());
    const auto range = [](double value, double minimum, double maximum, const char* message) {
        if (!std::isfinite(value) || value < minimum || value > maximum) {
            throw std::invalid_argument(message);
        }
    };
    const auto optionalRange = [&](const std::optional<double>& value, double minimum,
                                   double maximum, const char* message) {
        if (value) {
            range(*value, minimum, maximum, message);
        }
    };
    range(draft.precipitationRate, 0.0, 1.0, "Precipitation intensity must be between 0 and 1.");
    if (draft.precipitationType < 0 || draft.precipitationType > 2) {
        throw std::invalid_argument("Select clear, rain or snow.");
    }
    if (!std::isfinite(draft.temperatureCelsius) || draft.temperatureCelsius <= -273.15 ||
        draft.temperatureCelsius > maxValue) {
        throw std::invalid_argument("Temperature must be finite and above absolute zero.");
    }
    range(draft.absoluteHumidityGramsPerCubicMeter, 0.0, maxValue,
          "Absolute humidity must be finite and non-negative.");
    optionalRange(draft.visibilityMeters, 0.0, maxValue,
                  "Visibility must be finite and non-negative.");
    optionalRange(draft.brightness, 0.0, 1.0, "Brightness must be between 0 and 1.");
    if (draft.windDirectionDegrees &&
        (!std::isfinite(*draft.windDirectionDegrees) || *draft.windDirectionDegrees < 0.0 ||
         *draft.windDirectionDegrees >= 360.0)) {
        throw std::invalid_argument("Wind direction must be at least 0 and less than 360 degrees.");
    }
    optionalRange(draft.windSpeedMetersPerSecond, 0.0, maxValue,
                  "Wind speed must be finite and non-negative.");
    optionalRange(draft.cloudBaseAboveGroundMeters, 0.0, MAX_CLOUD_BASE_ABOVE_GROUND_METERS,
                  "Cloud height must be between 0 and 10000000 meters.");
    optionalRange(draft.fogTopAboveGroundMeters, 0.0, maxValue,
                  "Fog height must be finite and non-negative.");
    if (draft.atmosphericPressureHectopascals &&
        (!std::isfinite(*draft.atmosphericPressureHectopascals) ||
         *draft.atmosphericPressureHectopascals <= 0.0 ||
         *draft.atmosphericPressureHectopascals > maxValue)) {
        throw std::invalid_argument("Pressure must be finite and greater than zero.");
    }
    WeatherState weather = draft;
    if (weather.precipitationType == 0) {
        weather.precipitationRate = 0.0;
    }
    weather.omsiFogHeight.reset();
    weather.omsiPrecipitationParameters.reset();
    weather.omsiGroundWetness.reset();
    weather.omsiSnow = false;
    weather.omsiSnowOnRoad = false;
    validateWeatherState(weather);
    return weather;
}

void validateRoadWeatherConditions(const RoadWeatherConditions& conditions) {
    if (!std::isfinite(conditions.water) || conditions.water < 0.0 || conditions.water > 1.0 ||
        !std::isfinite(conditions.snow) || conditions.snow < 0.0 || conditions.snow > 1.0 ||
        !std::isfinite(conditions.groundSnow) || conditions.groundSnow < 0.0 ||
        conditions.groundSnow > 1.0 || !std::isfinite(conditions.freezeFactor) ||
        conditions.freezeFactor < 0.0 || conditions.freezeFactor > 1.0) {
        throw std::invalid_argument("Road weather conditions must be finite and between 0 and 1");
    }
}

RoadWeatherConditions roadWeatherFromPreset(const WeatherState& weather) {
    validateWeatherState(weather);
    RoadWeatherConditions conditions;
    if (weather.omsiGroundWetness) {
        conditions.water = std::clamp((*weather.omsiGroundWetness)[0] / 79.0, 0.0, 1.0);
    }
    conditions.snow = weather.omsiSnowOnRoad ? 1.0 : 0.0;
    conditions.groundSnow = weather.omsiSnow ? 1.0 : 0.0;
    conditions.freezeFactor = weather.temperatureCelsius < 0.0 ? 1.0 : 0.0;
    return conditions;
}

void advanceRoadWeather(RoadWeatherConditions& conditions, const WeatherState& weather,
                        double elapsedSeconds) {
    validateRoadWeatherConditions(conditions);
    validateWeatherState(weather);
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0) {
        throw std::invalid_argument("Road weather elapsed time must be finite and non-negative");
    }
    if (elapsedSeconds == 0.0) {
        return;
    }

    const double rain = weather.precipitationType == 1 ? weather.precipitationRate / 600.0 : 0.0;
    const double snowfall =
        weather.precipitationType == 2 ? weather.precipitationRate / 1200.0 : 0.0;
    const double drying = weather.temperatureCelsius > 0.0 ? 1.0 / 3600.0 : 1.0 / 43200.0;
    const double melting = std::clamp(weather.temperatureCelsius / 5.0, 0.0, 1.0) / 1800.0;
    const double waterDecay = rain + drying;
    const double snowDecay = snowfall + melting + 1.0 / 86400.0;
    const double waterExp = std::exp(-waterDecay * elapsedSeconds);
    const double snowExp = std::exp(-snowDecay * elapsedSeconds);
    const double snowEquilibrium = snowfall / snowDecay;
    const double snowOffset = conditions.snow - snowEquilibrium;
    const double decayDifference = std::abs(waterDecay - snowDecay);
    const double snowIntegral = decayDifference == 0.0
                                    ? elapsedSeconds * waterExp
                                    : std::max(waterExp, snowExp) *
                                          (-std::expm1(-decayDifference * elapsedSeconds)) /
                                          decayDifference;
    const double water = conditions.water * waterExp +
                         (rain + 0.5 * melting * snowEquilibrium) *
                             (-std::expm1(-waterDecay * elapsedSeconds)) / waterDecay +
                         0.5 * melting * snowOffset * snowIntegral;
    conditions.water = std::clamp(water, 0.0, 1.0);
    conditions.snow = std::clamp(snowEquilibrium + snowOffset * snowExp, 0.0, 1.0);
    conditions.groundSnow =
        std::clamp(snowEquilibrium + (conditions.groundSnow - snowEquilibrium) * snowExp, 0.0, 1.0);
    const bool freezing = weather.temperatureCelsius < 0.0;
    const double targetFreeze = freezing ? 1.0 : 0.0;
    const double freezingTimeConstant = freezing ? 600.0 : 300.0;
    conditions.freezeFactor = targetFreeze + (conditions.freezeFactor - targetFreeze) *
                                                 std::exp(-elapsedSeconds / freezingTimeConstant);
}

} // namespace openbus
