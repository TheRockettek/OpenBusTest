#pragma once

#include "MapConfigLoader.h"

#include <array>
#include <cstddef>
#include <vector>

namespace openbus::map {

struct MapSceneryPose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    // Spline-frame (heading, pitch, bank), followed by object-frame angles.
    std::array<double, 6> rotationDegrees = {};
    std::size_t rotationCount = 0;
};

// Places a spline-attachment row on its spline and subsequent linked splines in the same tile.
// A repeater resumes the row at the encoded first-object index on its own spline.
std::vector<MapSceneryPose> placeMapSplineAttachment(const MapSplineAttachment& attachment,
                                                     const std::vector<MapSplinePlacement>& splines,
                                                     int tileX, int tileY);

} // namespace openbus::map
