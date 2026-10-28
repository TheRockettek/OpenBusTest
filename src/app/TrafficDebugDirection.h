#pragma once

#include "map/MapSplineProfile.h"
#include "map/MapTrafficRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <utility>

namespace openbus::app {

inline bool trafficDebugHasBidirectionalPath(const std::vector<map::MapSplinePathOffset>& paths) {
    return std::any_of(paths.begin(), paths.end(),
                       [](const auto& path) { return path.bidirectional; });
}

inline map::MapTrafficRoute trafficDebugSplineRoute(const map::MapTrafficRouteSegment& source,
                                                    const map::MapSplinePathOffset& path) {
    map::MapTrafficRouteSegment segment = source;
    segment.reversed = path.reversed;
    segment.pathLateralMeters = path.lateralMeters;
    segment.pathHeightMeters = path.heightMeters;
    segment.splineDistanceAtRouteStartMeters.reset();
    map::MapTrafficRoute route;
    route.segments.push_back(std::move(segment));
    route.totalLength = source.spline.placement.length;
    route.cumulativeEndDistances.push_back(route.totalLength);
    return route;
}

inline std::optional<std::array<std::array<double, 3>, 6>>
trafficDirectionArrow(const std::array<double, 3>& center, double headingRadians, double length) {
    if (!std::isfinite(center[0]) || !std::isfinite(center[1]) || !std::isfinite(center[2]) ||
        !std::isfinite(headingRadians) || !std::isfinite(length) || length <= 0.0) {
        return std::nullopt;
    }
    const double forwardX = std::sin(headingRadians);
    const double forwardY = std::cos(headingRadians);
    const double rightX = forwardY;
    const double rightY = -forwardX;
    const std::array<double, 3> tip = {center[0] + forwardX * length * 0.5,
                                       center[1] + forwardY * length * 0.5, center[2]};
    const std::array<double, 3> tail = {center[0] - forwardX * length * 0.5,
                                        center[1] - forwardY * length * 0.5, center[2]};
    const auto wing = [&](double side) {
        return std::array<double, 3>{tip[0] - forwardX * length * 0.35 +
                                         rightX * length * 0.22 * side,
                                     tip[1] - forwardY * length * 0.35 +
                                         rightY * length * 0.22 * side,
                                     center[2]};
    };
    return std::array<std::array<double, 3>, 6>{tail, tip, wing(-1.0), tip, wing(1.0), tip};
}

} // namespace openbus::app
