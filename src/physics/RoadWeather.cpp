#include "physics/RoadWeather.h"

#include <algorithm>

namespace openbus::physics {

double roadWeatherFrictionMultiplier(const std::optional<CollisionSurface>& surface,
                                     const RoadWeatherConditions& conditions) {
    if (!surface) {
        return 1.0;
    }
    if (surface->terrain) {
        return 1.0 - conditions.groundSnow * 0.55;
    }

    double snow = conditions.snow;
    if (surface->type == 6 || surface->type == 7 || surface->type == 8) {
        snow = 1.0;
    }

    const double wet = surface->moisture ? conditions.water : 0.0;
    const double wetMultiplier = 0.75 - 0.45 * conditions.freezeFactor;
    return std::min(1.0 - wet * (1.0 - wetMultiplier), 1.0 - snow * 0.55);
}

} // namespace openbus::physics
