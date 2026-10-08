#pragma once

#include "MapConfigLoader.h"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <utility>
#include <vector>

class SimulationState;

namespace openbus::rendering {

struct MapRenderStats {
    std::size_t activeTiles = 0;
    std::size_t totalTiles = 0;
    std::size_t activeTerrainTiles = 0;
    std::size_t totalTerrainTiles = 0;
    std::size_t activeSceneryObjects = 0;
    std::size_t totalSceneryObjects = 0;
    std::size_t activeRoadBatches = 0;
    std::size_t totalRoadBatches = 0;
};

class MapRenderer {
  public:
    MapRenderer(const openbus::map::MapDefinition& map, std::size_t centerTileIndex,
                double spawnWorldX, double spawnWorldY, std::size_t groundTextureIndex,
                const std::filesystem::path& omsiRoot, SimulationState& simulationState);
    ~MapRenderer();

    MapRenderer(const MapRenderer&) = delete;
    MapRenderer& operator=(const MapRenderer&) = delete;

    void updateScripts();
    void updateStreaming(double worldX, double worldY);
    const std::vector<std::pair<int, int>>& activeTileCoordinates() const;
    MapRenderStats renderStats() const;
    void draw() const;

  private:
    void initializeActiveScripts();

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering