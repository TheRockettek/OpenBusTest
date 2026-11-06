#pragma once

#include <array>
#include <optional>

namespace openbus {

inline constexpr double MAX_CLOUD_BASE_ABOVE_GROUND_METERS = 10000000.0;

enum class WeatherCloudType {
    Cumulus1,
    Cumulus2,
    Cumulus3,
    Overcast1,
};

struct WeatherState {
    // Precipitation uses OMSI's 0 = clear, 1 = rain, 2 = snow convention.
    int precipitationType = 0;
    double precipitationRate = 0.0;
    double temperatureCelsius = 20.0;
    double absoluteHumidityGramsPerCubicMeter = 10.0;

    // Unknown observations stay absent rather than being interpreted as clear weather.
    std::optional<double> visibilityMeters;
    std::optional<double> brightness; // Normalized, 0 to 1.
    std::optional<double>
        windDirectionDegrees; // Direction wind comes from, clockwise from true north.
    std::optional<double> windSpeedMetersPerSecond;
    std::optional<double> cloudBaseAboveGroundMeters;
    std::optional<double> fogTopAboveGroundMeters;
    std::optional<WeatherCloudType> cloudType;
    std::optional<double> atmosphericPressureHectopascals;

    // Preserved OMSI preset fields whose units or runtime effects are not fully mapped.
    std::optional<double> omsiFogHeight;
    std::optional<std::array<double, 3>> omsiPrecipitationParameters;
    std::optional<std::array<double, 3>> omsiGroundWetness;
    bool omsiSnow = false;
    bool omsiSnowOnRoad = false;
};

struct RoadWeatherConditions {
    double water = 0.0;
    double snow = 0.0;
    // Cold-exposure proxy for the frozen share of road water, not an ice depth.
    double freezeFactor = 0.0;
    double groundSnow = 0.0;
};

enum class SurfaceWeatherAction {
    DryRoadWater,
    ClearRoadSnow,
    ClearAllSnow,
};

// Throws std::invalid_argument if a field is invalid or cannot be published to scripts.
void validateWeatherState(const WeatherState& weather);
// Validates user-entered weather and removes preset-only metadata.
WeatherState customWeatherFromDraft(const WeatherState& draft);
void validateRoadWeatherConditions(const RoadWeatherConditions& conditions);
RoadWeatherConditions roadWeatherFromPreset(const WeatherState& weather);
void advanceRoadWeather(RoadWeatherConditions& conditions, const WeatherState& weather,
                        double elapsedSeconds);

} // namespace openbus
