#pragma once

#include "core/BusTypes.h"
#include "core/WeatherState.h"

#include <optional>

namespace openbus::physics {

// Approximate multiplier for classified spline-road wheel contacts.
double roadWeatherFrictionMultiplier(const std::optional<CollisionSurface>& surface,
                                     const RoadWeatherConditions& conditions);

} // namespace openbus::physics
