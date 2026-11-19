#pragma once

#include "map/MapConfigLoader.h"
#include "render/ModelVertex.h"

#include <cstdint>
#include <vector>

namespace openbus::map {

struct MapTerrainMesh {
    std::vector<openbus::rendering::ModelVertex> vertices;
    std::vector<std::uint32_t> indices;
    float minimumHeight = 0.0F;
    float maximumHeight = 0.0F;
};

struct MapWaterMesh {
    std::vector<openbus::rendering::ModelVertex> vertices;
    std::vector<std::uint32_t> indices;
    double minimumHeight = 0.0;
    double maximumHeight = 0.0;
    double meanHeight = 0.0;
};

// Terrain and water vertices are tile-local; height remains in world Z. Terrain UVs use
// grid intervals while each water quad spans [0,1]. Index winding matches production.
[[nodiscard]] MapTerrainMesh prepareMapTerrainMesh(const TerrainGrid& grid, double tileSize);
[[nodiscard]] MapWaterMesh prepareMapWaterMesh(const WaterData& water, int tileX, int tileY,
                                               double tileSize);

} // namespace openbus::map
