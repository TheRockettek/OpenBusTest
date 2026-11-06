#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace openbus {

enum class MetarVisibilityBound {
    Reported,
    LessThan,
    GreaterThan,
    AtLeast,
};

struct MetarVisibility {
    // Reported values are quantized by the reporting station.
    double meters;
    MetarVisibilityBound bound;
};

enum class MetarAltimeterUnit {
    Hectopascals,
    InchesOfMercury,
};

struct MetarAltimeterSetting {
    double hectopascals;
    MetarAltimeterUnit reportedUnit;
};

enum class MetarCloudCoverage {
    Few,
    Scattered,
    Broken,
    Overcast,
    VerticalVisibility,
};

struct MetarCloudLayer {
    MetarCloudCoverage coverage;
    std::optional<double> heightAboveAerodromeMeters;
    std::string cloudSuffix;
};

struct MetarWindRange {
    double fromDegrees;
    double toDegrees;
};

struct MetarObservation {
    std::string station;
    // METAR does not include month or year; consumers must resolve age separately.
    int day;
    int hourUtc;
    int minuteUtc;
    bool specialReport = false;
    bool automated = false;
    bool corrected = false;
    std::optional<double> windDirectionDegrees; // Direction from, clockwise from north.
    std::optional<double> windSpeedMetersPerSecond;
    std::optional<double> gustSpeedMetersPerSecond;
    bool variableWindDirection = false;
    std::optional<MetarWindRange> variableWindRange;
    std::optional<MetarVisibility> visibility;
    bool cavok = false;
    std::optional<std::string> clearSkyCode;
    std::vector<MetarCloudLayer> cloudLayers;
    std::optional<double> temperatureCelsius;
    std::optional<double> dewPointCelsius;
    // Altimeter settings are not station-level atmospheric pressure.
    std::optional<MetarAltimeterSetting> altimeter;
    std::vector<std::string> presentWeather;
    std::vector<std::string> unparsedGroups;
    std::vector<std::string> trailingGroups;
};

// Parses the observation body only; remarks and forecast groups are retained separately.
// Throws std::invalid_argument for a malformed header or contradictory recognized fields.
MetarObservation parseMetar(std::string_view report);

// Approximate water-vapor density in g/m3 from Celsius temperatures.
// The liquid-water Magnus approximation is restricted to -40 through 50 C.
double absoluteHumidityFromDewPoint(double temperatureCelsius, double dewPointCelsius);
// Relative humidity is in percent. Conversions throw for invalid or unrepresentable inputs;
// dew point is unavailable at zero humidity, above saturation, or outside the approximation.
double relativeHumidityFromAbsolute(double temperatureCelsius, double absoluteHumidity);
double absoluteHumidityFromRelative(double temperatureCelsius, double relativeHumidity);
std::optional<double> dewPointFromAbsoluteHumidity(double temperatureCelsius,
                                                   double absoluteHumidity);

} // namespace openbus
