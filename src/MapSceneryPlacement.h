#pragma once

#include "MapConfigLoader.h"

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace openbus::map {

struct MapSceneryPose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    // Renderer rotation components; spline poses use heading/pitch/bank before object angles.
    std::array<double, 6> rotationDegrees = {};
    std::size_t rotationCount = 0;
};

// Converts a tile-local ordinary [object] placement to map coordinates without changing its
// authored rotation component order. Invalid parsed transforms produce no pose.
std::optional<MapSceneryPose> placeMapSceneryObject(const MapSceneryPlacement& object, int tileX,
                                                    int tileY);

// Absolute-height assets use their authored Z directly; other assets are terrain-relative.
double mapSceneryWorldHeight(double authoredZ, double terrainHeight, bool absoluteHeight);

// Places a spline-attachment row on its spline and subsequent linked splines in the same tile.
// A repeater resumes the row at the encoded first-object index on its own spline.
std::vector<MapSceneryPose> placeMapSplineAttachment(const MapSplineAttachment& attachment,
                                                     const std::vector<MapSplinePlacement>& splines,
                                                     int tileX, int tileY);

} // namespace openbus::map
