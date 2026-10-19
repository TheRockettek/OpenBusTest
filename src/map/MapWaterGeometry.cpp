#include "map/MapWaterGeometry.h"

#include "map/MapConfigLoader.h"

namespace openbus::map {

std::array<MapWaterVertex, 4> buildMapWaterSurfaceVertices(int tileX, int tileY,
                                                           const WaterSurface& surface) {
    const double west = static_cast<double>(tileX) * OMSI_TILE_SIZE_METERS;
    const double south = static_cast<double>(tileY) * OMSI_TILE_SIZE_METERS;
    const double east = west + OMSI_TILE_SIZE_METERS;
    const double north = south + OMSI_TILE_SIZE_METERS;
    return {{{west, south, surface.heights[0]},
             {east, south, surface.heights[1]},
             {east, north, surface.heights[2]},
             {west, north, surface.heights[3]}}};
}

} // namespace openbus::map
