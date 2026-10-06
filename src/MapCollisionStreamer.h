#pragma once

#include "MapConfigLoader.h"

#include <filesystem>
#include <optional>
#include <unordered_map>
#include <utility>

class BusSimulation;

namespace openbus::map {

// Keeps collision resident around the moving vehicle instead of building ODE meshes for the
// complete map during startup. Tiles are loaded within loadRadius and retained until they are
// farther than unloadRadius, providing simple boundary hysteresis.
class MapCollisionStreamer {
  public:
    MapCollisionStreamer(const MapDefinition& map, const std::filesystem::path& omsiRoot,
                         BusSimulation& simulation, int loadRadius = 1, int unloadRadius = 2);

    void update(double worldX, double worldY);
    std::size_t loadedTileCount() const;

  private:
    const MapDefinition& map_;
    std::filesystem::path omsiRoot_;
    BusSimulation& simulation_;
    int loadRadius_;
    int unloadRadius_;
    std::optional<std::pair<int, int>> lastCenterTile_;
    std::unordered_map<std::uint64_t, std::pair<int, int>> loadedTiles_;
};

} // namespace openbus::map
