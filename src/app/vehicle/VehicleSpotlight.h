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

inline bool spotlightSelectionChanged(double previous, double current) {
    return !std::isfinite(previous) || !std::isfinite(current) ||
           std::nearbyint(previous) != std::nearbyint(current);
}

inline SpotlightStatus spotlightSelectionStatus(std::size_t definitionCount, double selection) {
    if (!std::isfinite(selection)) {
        return SpotlightStatus::Invalid;
    }
    const double index = std::nearbyint(selection);
    if (index < 0.0) {
        return SpotlightStatus::Off;
    }
    return index >= static_cast<double>(definitionCount) ? SpotlightStatus::Invalid
                                                         : SpotlightStatus::Active;
}

inline bool validSpotlightParameters(const ModelSpotlight& definition) {
    const auto& p = definition.parameters;
    return std::all_of(p.begin(), p.end(), [](double value) { return std::isfinite(value); }) &&
           std::hypot(p[3], p[4], p[5]) >= 1e-12 && p[6] >= 0.0 && p[6] <= 255.0 && p[7] >= 0.0 &&
           p[7] <= 255.0 && p[8] >= 0.0 && p[8] <= 255.0 && p[9] > 0.0 &&
           p[9] <= std::numeric_limits<float>::max() && p[10] >= 0.0 && p[10] <= p[11] &&
           p[11] <= 180.0;
}

inline bool spotlightOutsideView(const std::vector<ModelSpotlight>& definitions, double selection,
                                 const openbus::rendering::Matrix4& modelToWorld,
                                 const openbus::rendering::ViewFrustum& frustum) {
    if (spotlightSelectionStatus(definitions.size(), selection) != SpotlightStatus::Active) {
        return false;
    }
    const auto& definition = definitions[static_cast<std::size_t>(std::nearbyint(selection))];
    if (!validSpotlightParameters(definition)) {
        return false;
    }
    const auto& p = definition.parameters;
    const auto position =
        openbus::rendering::transformPoint(modelToWorld, {p[1], -p[0], p[2], 1.0});
    if (!std::all_of(position.begin(), position.end(),
                     [](double value) { return std::isfinite(value); })) {
        return false;
    }
    return openbus::rendering::sphereOutsideFrustum(frustum, position, p[9] * 4.0);
}

struct SpotlightCandidate {
    openbus::rendering::VehicleSpotlight light;
    bool player = false;
};

inline std::vector<openbus::rendering::VehicleSpotlight>
selectSpotlights(std::vector<SpotlightCandidate>& candidates, std::size_t limit) {
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const SpotlightCandidate& a, const SpotlightCandidate& b) {
                         if (a.player != b.player) {
                             return a.player;
                         }
                         const auto& ap = a.light.position;
                         const auto& bp = b.light.position;
                         return std::hypot(ap[0], ap[1], ap[2]) < std::hypot(bp[0], bp[1], bp[2]);
                     });
    std::vector<openbus::rendering::VehicleSpotlight> lights;
    lights.reserve(std::min(limit, candidates.size()));
    for (std::size_t i = 0; i < std::min(limit, candidates.size()); ++i) {
        lights.push_back(candidates[i].light);
    }
    return lights;
}

inline SpotlightStatus makeSpotlight(const std::vector<ModelSpotlight>& definitions,
                                     double selection, const openbus::rendering::Matrix4& modelView,
                                     openbus::rendering::VehicleSpotlight& result) {
    const auto selectionStatus = spotlightSelectionStatus(definitions.size(), selection);
    if (selectionStatus != SpotlightStatus::Active) {
        return selectionStatus;
    }
    const double index = std::nearbyint(selection);
    const auto& definition = definitions[static_cast<std::size_t>(index)];
    if (!validSpotlightParameters(definition)) {
        return SpotlightStatus::Invalid;
    }
    const auto& p = definition.parameters;
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
