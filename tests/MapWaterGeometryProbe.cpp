#include "MapWaterGeometry.h"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, const char* message) {
    if (std::abs(actual - expected) > 1.0e-9) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main() {
    try {
        const openbus::map::WaterSurface surface{{1.25F, 2.5F, 3.75F, 5.0F}};
        const auto vertices = openbus::map::buildMapWaterSurfaceVertices(-2, 3, surface);
        constexpr double tileSize = openbus::map::OMSI_TILE_SIZE_METERS;
        requireNear(vertices[0].x, -2.0 * tileSize, "south-west X uses negative tile coordinate");
        requireNear(vertices[0].y, 3.0 * tileSize, "south-west Y uses tile coordinate");
        requireNear(vertices[0].z, 1.25, "height 0 maps to south-west corner");
        requireNear(vertices[1].x, -1.0 * tileSize, "south-east X advances one tile");
        requireNear(vertices[1].y, 3.0 * tileSize, "south-east Y remains south edge");
        requireNear(vertices[1].z, 2.5, "height 1 maps to south-east corner");
        requireNear(vertices[2].x, -1.0 * tileSize, "north-east X matches east edge");
        requireNear(vertices[2].y, 4.0 * tileSize, "north-east Y advances one tile");
        requireNear(vertices[2].z, 3.75, "height 2 maps to north-east corner");
        requireNear(vertices[3].x, -2.0 * tileSize, "north-west X matches west edge");
        requireNear(vertices[3].y, 4.0 * tileSize, "north-west Y advances one tile");
        requireNear(vertices[3].z, 5.0, "height 3 maps to north-west corner");

        const openbus::map::WaterSurface eastNeighbor{{2.5F, 9.0F, 8.0F, 3.75F}};
        const auto adjacent = openbus::map::buildMapWaterSurfaceVertices(-1, 3, eastNeighbor);
        requireNear(vertices[1].x, adjacent[0].x, "east neighbor shares the west/east boundary");
        requireNear(vertices[1].z, adjacent[0].z,
                    "east neighbor south corner retains shared-edge water height");
        requireNear(vertices[2].z, adjacent[3].z,
                    "east neighbor north corner retains shared-edge water height");
    } catch (const std::exception& error) {
        std::cerr << "map water geometry probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
