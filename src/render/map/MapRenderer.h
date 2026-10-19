#pragma once

#include "map/MapConfigLoader.h"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

class SimulationState;

namespace openbus::rendering {

class MapSceneLoader;

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
    std::optional<double> terrainHeightAt(double worldX, double worldY) const;
    void draw(bool renderTypeFourOnly = false) const;

  private:
    friend class MapSceneLoader;

    void initializeActiveScripts();

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering