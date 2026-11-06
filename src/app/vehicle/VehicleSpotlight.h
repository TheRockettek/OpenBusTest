#pragma once

#include "config/ModelConfigTypes.h"
#include "render/CoreRenderer.h"
#include "render/MatrixMath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace openbus::app::vehicle {

enum class SpotlightStatus { Off, Invalid, Active };

inline SpotlightStatus makeSpotlight(const std::vector<ModelSpotlight>& definitions,
                                     double selection, const openbus::rendering::Matrix4& modelView,
                                     openbus::rendering::VehicleSpotlight& result) {
    if (!std::isfinite(selection)) {
        return SpotlightStatus::Invalid;
    }
    const double index = std::nearbyint(selection);
    if (index < 0.0) {
        return SpotlightStatus::Off;
    }
    if (index >= static_cast<double>(definitions.size())) {
        return SpotlightStatus::Invalid;
    }
    const auto& p = definitions[static_cast<std::size_t>(index)].parameters;
    if (!std::all_of(p.begin(), p.end(), [](double value) { return std::isfinite(value); }) ||
        std::hypot(p[3], p[4], p[5]) < 1e-12 || p[6] < 0.0 || p[6] > 255.0 || p[7] < 0.0 ||
        p[7] > 255.0 || p[8] < 0.0 || p[8] > 255.0 || p[9] <= 0.0 ||
        p[9] > std::numeric_limits<float>::max() || p[10] < 0.0 || p[10] > p[11] || p[11] > 180.0) {
        return SpotlightStatus::Invalid;
    }
    const auto position = openbus::rendering::transformPoint(modelView, {p[1], -p[0], p[2], 1.0});
    const auto direction = openbus::rendering::transformPoint(modelView, {p[4], -p[3], p[5], 0.0});
    const auto representable = [](double value) {
        return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    };
    if (!std::all_of(position.begin(), position.end(), representable) ||
        !std::all_of(direction.begin(), direction.end(), representable) ||
        std::hypot(direction[0], direction[1], direction[2]) < 1e-12) {
        return SpotlightStatus::Invalid;
    }
    result = {{position[0], position[1], position[2]},
              {direction[0], direction[1], direction[2]},
              {p[6] / 255.0, p[7] / 255.0, p[8] / 255.0},
              p[9],
              p[10],
              p[11]};
    return SpotlightStatus::Active;
}

} // namespace openbus::app::vehicle
