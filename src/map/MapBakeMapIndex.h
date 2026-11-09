#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace openbus::map {

class MapBakeArchiveReader;

struct MapBakeTileIndexEntry {
    std::int32_t tileX = 0;
    std::int32_t tileY = 0;
    std::string chunkKey;
    bool operator==(const MapBakeTileIndexEntry&) const = default;
};

struct MapBakeMapIndex {
    std::uint32_t schemaVersion = 1;
    std::vector<MapBakeTileIndexEntry> tiles;
    bool operator==(const MapBakeMapIndex&) const = default;
};

[[nodiscard]] std::vector<std::byte> serializeMapBakeMapIndex(const MapBakeMapIndex& index);
[[nodiscard]] MapBakeMapIndex deserializeMapBakeMapIndex(std::span<const std::byte> payload);
[[nodiscard]] MapBakeMapIndex readMapBakeMapIndex(MapBakeArchiveReader& archive);

} // namespace openbus::map
