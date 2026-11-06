#pragma once

#include <array>

namespace openbus {

struct SolarLighting {
    double altitudeDegrees;
    std::array<double, 3> direction;
    double brightness;
};

// Approximate local solar time, with east/north/up world axes.
SolarLighting solarLighting(int dayOfYear, double timeSeconds, double latitudeDegrees);
SolarLighting solarLightingWithAltitude(const SolarLighting& solar, double altitudeDegrees);

} // namespace openbus
