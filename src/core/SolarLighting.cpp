#include "core/SolarLighting.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace openbus {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double radians = pi / 180.0;

double brightnessForAltitude(double altitudeDegrees) {
    const double t = std::clamp((altitudeDegrees + 8.0) / 18.0, 0.0, 1.0);
    return 0.06 + 0.94 * t * t * (3.0 - 2.0 * t);
}
} // namespace

SolarLighting solarLighting(int dayOfYear, double timeSeconds, double latitudeDegrees) {
    if (dayOfYear < 1 || dayOfYear > 366 || !std::isfinite(timeSeconds) || timeSeconds < 0 ||
        timeSeconds >= 86400 || !std::isfinite(latitudeDegrees) || latitudeDegrees < -90 ||
        latitudeDegrees > 90) {
        throw std::invalid_argument("Solar date, time or latitude is invalid");
    }
    const double declination = 23.44 * radians * std::sin(2.0 * pi * (dayOfYear - 80.0) / 365.2422);
    const double hourAngle = (timeSeconds / 3600.0 - 12.0) * 15.0 * radians;
    const double latitude = latitudeDegrees * radians;
    const double east = -std::cos(declination) * std::sin(hourAngle);
    const double north = std::cos(latitude) * std::sin(declination) -
                         std::sin(latitude) * std::cos(declination) * std::cos(hourAngle);
    const double up = std::sin(latitude) * std::sin(declination) +
                      std::cos(latitude) * std::cos(declination) * std::cos(hourAngle);
    const double altitude = std::asin(std::clamp(up, -1.0, 1.0)) / radians;
    return {altitude, {east, north, up}, brightnessForAltitude(altitude)};
}

SolarLighting solarLightingWithAltitude(const SolarLighting& solar, double altitudeDegrees) {
    if (!std::isfinite(altitudeDegrees) || altitudeDegrees < -90 || altitudeDegrees > 90)
        throw std::invalid_argument("Sun altitude must be finite and between -90 and 90 degrees");
    const double horizontal = std::hypot(solar.direction[0], solar.direction[1]);
    const double east = horizontal > 1e-12 ? solar.direction[0] / horizontal : 0.0;
    const double north = horizontal > 1e-12 ? solar.direction[1] / horizontal : -1.0;
    const double elevation = altitudeDegrees * radians;
    return {altitudeDegrees,
            {east * std::cos(elevation), north * std::cos(elevation), std::sin(elevation)},
            brightnessForAltitude(altitudeDegrees)};
}
} // namespace openbus
