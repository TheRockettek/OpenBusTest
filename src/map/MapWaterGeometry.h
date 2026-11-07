#pragma once

#include "map/MapConfigLoader.h"

#include <array>

namespace openbus::map {

struct MapWaterVertex {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Builds the four corners of one water quad in SW, SE, NE, NW order. The
// Sidecar heights are stored SW, SE, NW, NE; the returned corners are wound
// SW, SE, NE, NW. The interpretation is inferred from neighboring sidecars.
std::array<MapWaterVertex, 4> buildMapWaterSurfaceVertices(int tileX, int tileY,
                                                           const WaterSurface& surface,
                                                           double tileSize = OMSI_TILE_SIZE_METERS);

} // namespace openbus::map
