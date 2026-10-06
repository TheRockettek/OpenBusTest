#include "MapCollisionStreamer.h"

#include "BusSimulation.h"
#include "MapCollisionBuilder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace openbus::map {
namespace {

std::uint64_t tileKey(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
           static_cast<std::uint32_t>(y);
}

int worldTile(double coordinate) {
    if (!std::isfinite(coordinate)) {
        throw std::invalid_argument("Map collision streaming position must be finite");
    }
    const double tile = std::floor(coordinate / OMSI_TILE_SIZE_METERS);
    if (tile < static_cast<double>(std::numeric_limits<int>::min()) ||
        tile > static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::out_of_range("Map collision streaming position exceeds tile coordinates");
    }
    return static_cast<int>(tile);
}

bool withinRadius(int x, int y, int centerX, int centerY, int radius) {
    return std::abs(static_cast<long long>(x) - centerX) <= radius &&
           std::abs(static_cast<long long>(y) - centerY) <= radius;
}

} // namespace

MapCollisionStreamer::MapCollisionStreamer(const MapDefinition& map,
                                           const std::filesystem::path& omsiRoot,
                                           BusSimulation& simulation, int loadRadius,
                                           int unloadRadius)
    : map_(map), omsiRoot_(omsiRoot), simulation_(simulation), loadRadius_(loadRadius),
      unloadRadius_(unloadRadius) {
    if (loadRadius_ < 0 || unloadRadius_ < loadRadius_) {
        throw std::invalid_argument("Map collision unload radius must be at least the load radius");
    }
}

void MapCollisionStreamer::update(double worldX, double worldY) {
    const int centerX = worldTile(worldX);
    const int centerY = worldTile(worldY);
    if (lastCenterTile_ == std::pair{centerX, centerY}) {
        return;
    }

    for (const MapTileReference& tile : map_.tiles) {
        if (!withinRadius(tile.x, tile.y, centerX, centerY, loadRadius_)) {
            continue;
        }
        const std::uint64_t key = tileKey(tile.x, tile.y);
        if (loadedTiles_.find(key) != loadedTiles_.end()) {
            continue;
        }

        std::optional<TerrainCollisionGrid> terrain;
        if (tile.hasTerrainFile) {
            const TerrainGrid grid = loadTerrainGrid(tile.terrainPath);
            terrain = TerrainCollisionGrid{tile.x, tile.y, OMSI_TILE_SIZE_METERS, grid.intervals,
                                           grid.heights};
        }
        MapRoadCollisionResult roadCollision =
            buildMapRoadCollisionTile(map_, omsiRoot_, tile.x, tile.y);
        simulation_.addMapTileCollision(tile.x, tile.y, std::move(terrain),
                                        std::move(roadCollision.meshes));
        loadedTiles_.emplace(key, std::pair{tile.x, tile.y});
    }

    for (auto tile = loadedTiles_.begin(); tile != loadedTiles_.end();) {
        const auto [tileX, tileY] = tile->second;
        if (withinRadius(tileX, tileY, centerX, centerY, unloadRadius_)) {
            ++tile;
            continue;
        }
        simulation_.removeMapTileCollision(tileX, tileY);
        tile = loadedTiles_.erase(tile);
    }
    lastCenterTile_ = std::pair{centerX, centerY};
}

std::size_t MapCollisionStreamer::loadedTileCount() const {
    return loadedTiles_.size();
}

} // namespace openbus::map
