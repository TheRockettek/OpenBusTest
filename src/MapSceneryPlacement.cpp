#include "MapSceneryPlacement.h"

#include "MapSplineGeometry.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <unordered_set>

namespace openbus::map {

std::vector<MapSceneryPose> placeMapSplineAttachment(const MapSplineAttachment& attachment,
                                                     const std::vector<MapSplinePlacement>& splines,
                                                     int tileX, int tileY) {
    std::vector<MapSceneryPose> placements;
    if (!attachment.transformValid || attachment.splineIndex < 0 ||
        static_cast<std::size_t>(attachment.splineIndex) >= splines.size()) {
        return placements;
    }
    const std::size_t ownerIndex = static_cast<std::size_t>(attachment.splineIndex);
    const MapSplinePlacement& owner = splines[ownerIndex];
    if (!owner.geometryValid || owner.length <= 0.0) {
        return placements;
    }
    const double interval = std::max(0.0, attachment.interval);
    const double range = std::max(0.0, attachment.range);
    const double rowCountValue = interval > 0.0 ? std::floor(range / interval) + 1.0 : 1.0;
    const std::size_t rowCount = static_cast<std::size_t>(std::clamp(rowCountValue, 1.0, 4096.0));
    const std::size_t firstRowIndex = attachment.repeater ? attachment.repeaterFirstObjectIndex : 0;
    if (firstRowIndex >= rowCount) {
        return placements;
    }

    const double ownerChainOffset = owner.chainOffsetValid ? owner.chainOffset : 0.0;
    const double tileOriginX = static_cast<double>(tileX) * OMSI_TILE_SIZE_METERS;
    const double tileOriginY = static_cast<double>(tileY) * OMSI_TILE_SIZE_METERS;
    std::size_t currentIndex = ownerIndex;
    double accumulatedLength = 0.0;
    bool backwards = false;
    std::unordered_set<int> visited;
    visited.insert(owner.splineId);

    for (std::size_t segmentCount = 0; segmentCount < splines.size(); ++segmentCount) {
        const MapSplinePlacement& spline = splines[currentIndex];
        if (!spline.geometryValid || spline.length <= 0.0) {
            break;
        }
        for (std::size_t rowIndex = firstRowIndex; rowIndex < rowCount; ++rowIndex) {
            const double distanceFromChainStart =
                attachment.offset[2] + static_cast<double>(rowIndex) * interval;
            const double distanceOnChainDirection =
                distanceFromChainStart - ownerChainOffset - accumulatedLength;
            if (distanceOnChainDirection < -1.0e-6 ||
                distanceOnChainDirection > spline.length + 1.0e-6 ||
                (segmentCount != 0 && distanceOnChainDirection <= 1.0e-6)) {
                continue;
            }
            const double along = std::clamp(distanceOnChainDirection, 0.0, spline.length);
            const double curveDistance = backwards ? spline.length - along : along;
            const auto center = sampleMapSpline(tileOriginX + spline.localX,
                                                tileOriginY + spline.localY, spline.rotationDegrees,
                                                spline.length, spline.radius, curveDistance);
            if (!center) {
                continue;
            }

            const double lateral = backwards ? -attachment.offset[0] : attachment.offset[0];
            const double headingDegrees = center->headingRadians * 180.0 / std::numbers::pi;
            const auto lateralOffset = mapSplineRightOffset(center->headingRadians, lateral);
            const double t = std::clamp(curveDistance / spline.length, 0.0, 1.0);
            const double cant = spline.cantStart + (spline.cantEnd - spline.cantStart) * t;
            const double cantLift = -std::clamp(lateral, -10.0, 10.0) * cant / 100.0;
            const double elevation = mapSplineElevation(
                spline.elevation, spline.length, spline.gradientStart, spline.gradientEnd,
                spline.elevated ? std::optional<double>(spline.heightDelta) : std::nullopt,
                curveDistance);
            const double pitch = attachment.tilt
                                     ? std::atan((spline.gradientStart +
                                                  (spline.gradientEnd - spline.gradientStart) * t) /
                                                 100.0) *
                                           180.0 / std::numbers::pi
                                     : 0.0;
            const double bank = attachment.tilt && std::abs(lateral) <= 10.0
                                    ? std::atan(cant / 100.0) * 180.0 / std::numbers::pi
                                    : 0.0;
            const double reverseTurn = backwards ? 180.0 : 0.0;
            placements.push_back(
                {center->x + lateralOffset[0],
                 center->y + lateralOffset[1],
                 elevation + attachment.offset[1] + cantLift,
                 {headingDegrees, pitch, bank, attachment.rotationDegrees[0] + reverseTurn,
                  attachment.rotationDegrees[1], attachment.rotationDegrees[2]},
                 6});
        }

        accumulatedLength += spline.length;
        const int nextId = backwards ? spline.previousSplineId : spline.nextSplineId;
        if (nextId == 0 || visited.contains(nextId)) {
            break;
        }
        const auto next = std::find_if(
            splines.begin(), splines.end(),
            [nextId](const MapSplinePlacement& candidate) { return candidate.splineId == nextId; });
        if (next == splines.end()) {
            break; // A repeater in the next tile continues this chain.
        }
        if (next->previousSplineId == spline.splineId) {
            backwards = false;
        } else if (next->nextSplineId == spline.splineId) {
            backwards = true;
        } else {
            break;
        }
        currentIndex = static_cast<std::size_t>(std::distance(splines.begin(), next));
        visited.insert(nextId);
    }
    return placements;
}

} // namespace openbus::map
