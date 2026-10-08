#pragma once

#include "MapConfigLoader.h"
#include "MapTileWindow.h"

#include <filesystem>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>

class BusSimulation;

namespace openbus::map {

// Keeps collision resident around the moving vehicle instead of building ODE meshes for the
// complete map during startup. A collision tile must have render geometry currently resident.
class MapCollisionStreamer {
  public:
    MapCollisionStreamer(const MapDefinition& map, const std::filesystem::path& omsiRoot,
                         BusSimulation& simulation, int loadRadius = 1, int unloadRadius = 2);
    ~MapCollisionStreamer();

    void update(double worldX, double worldY,
            const std::vector<MapTileCoordinate>& loadedGeometryTiles);
    void waitForPendingBuilds();
    std::size_t loadedTileCount() const;

  private:
    struct WorkerState;

    const MapDefinition& map_;
    std::filesystem::path omsiRoot_;
    BusSimulation& simulation_;
    int loadRadius_;
    int unloadRadius_;
    std::optional<std::pair<int, int>> lastCenterTile_;
    std::unordered_map<std::uint64_t, std::pair<int, int>> loadedTiles_;
    std::unordered_set<std::uint64_t> lastLoadedGeometryTileKeys_;
    std::unordered_set<std::uint64_t> failedTiles_;
    std::unique_ptr<WorkerState> worker_;
};

} // namespace openbus::map
