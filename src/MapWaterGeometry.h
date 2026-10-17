#pragma once

#include "MapConfigLoader.h"

#include <array>

namespace openbus::map {

struct MapWaterVertex {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Builds the four corners of one water quad in SW, SE, NE, NW order. The
// corner interpretation is reverse-engineered from neighboring OMSI sidecars.
std::array<MapWaterVertex, 4> buildMapWaterSurfaceVertices(int tileX, int tileY,
                                                          const WaterSurface& surface);

} // namespace openbus::map
