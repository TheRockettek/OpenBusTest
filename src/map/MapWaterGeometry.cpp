#include "map/MapWaterGeometry.h"

#include "map/MapConfigLoader.h"

namespace openbus::map {

std::array<MapWaterVertex, 4>
buildMapWaterSurfaceVertices(int tileX, int tileY, const WaterSurface& surface, double tileSize) {
    if (!std::isfinite(tileSize) || tileSize <= 0.0) {
        throw std::invalid_argument("Water tile size must be finite and positive");
    }
    const double west = static_cast<double>(tileX) * tileSize;
    const double south = static_cast<double>(tileY) * tileSize;
    const double east = west + tileSize;
    const double north = south + tileSize;
    return {{{west, south, surface.heights[0]},
             {east, south, surface.heights[1]},
             {east, north, surface.heights[2]},
             {west, north, surface.heights[3]}}};
}

} // namespace openbus::map
