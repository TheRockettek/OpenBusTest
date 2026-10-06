#pragma once

#include "MapConfigLoader.h"
#include "MapSplineGeometry.h"
#include "MapSplineProfile.h"

#include <vector>

namespace openbus::map {

struct MapRoadVertex {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double textureU = 0.0;
    double textureV = 0.0;
};

struct MapRoadSectionGeometry {
    std::vector<MapRoadVertex> vertices;
    std::vector<int> indices;
};

// Builds one visible .sli cross-section strip in world coordinates. Rendering and collision
// consume this same geometry so road physics cannot drift from the displayed profile.
MapRoadSectionGeometry buildMapRoadSectionGeometry(const MapSplinePlacement& spline,
                                                   const std::vector<MapSplineSample>& centerline,
                                                   const MapSplineProfileSection& section);

} // namespace openbus::map
