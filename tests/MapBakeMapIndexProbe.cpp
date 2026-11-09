#include "map/MapBakeArchive.h"
#include "map/MapBakeMapIndex.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Callback>
void requireThrows(Callback&& callback, const char* message) {
    try {
        callback();
    } catch (const std::exception&) {
        return;
    }
    throw std::runtime_error(message);
}

void run() {
    using openbus::map::MapBakeChunk;
    using openbus::map::MapBakeChunkType;
    using openbus::map::MapBakeMapIndex;
    using openbus::map::MapBakeTileIndexEntry;

    const MapBakeMapIndex unsorted{
        1, {{-17, 42, "tile"}, {5, -12, "tile"}, {0, 0, "tile"}}};
    const auto payload = openbus::map::serializeMapBakeMapIndex(unsorted);
    const MapBakeMapIndex decoded = openbus::map::deserializeMapBakeMapIndex(payload);
    const MapBakeMapIndex expected{
        1, {{5, -12, "tile"}, {0, 0, "tile"}, {-17, 42, "tile"}}};
    require(decoded == expected, "map index round trip lost signed coordinates or canonical order");

    MapBakeMapIndex reversed = expected;
    std::reverse(reversed.tiles.begin(), reversed.tiles.end());
    require(openbus::map::serializeMapBakeMapIndex(reversed) == payload,
            "map-index serialization was not deterministic");

    std::vector<MapBakeChunk> chunks{
        {MapBakeChunkType::MapWideIndex, "map", 0, 0, payload}};
    for (const MapBakeTileIndexEntry& entry : expected.tiles) {
        chunks.push_back({MapBakeChunkType::TileData, entry.chunkKey, entry.tileX, entry.tileY,
                          {std::byte{0x01}}});
    }
    std::stringstream archive(std::ios::in | std::ios::out | std::ios::binary);
    openbus::map::writeMapBakeArchive(archive, chunks);
    archive.seekg(0);
    openbus::map::MapBakeArchiveReader reader(archive);
    require(openbus::map::readMapBakeMapIndex(reader) == expected,
            "archive map-index reader returned incorrect entries");

    std::stringstream missingTileArchive(std::ios::in | std::ios::out | std::ios::binary);
    openbus::map::writeMapBakeArchive(
        missingTileArchive, {{MapBakeChunkType::MapWideIndex, "map", 0, 0, payload}});
    missingTileArchive.seekg(0);
    openbus::map::MapBakeArchiveReader missingTileReader(missingTileArchive);
    requireThrows([&] { (void)openbus::map::readMapBakeMapIndex(missingTileReader); },
                  "map index referencing a missing tile-data chunk was accepted");

    std::stringstream missingIndexArchive(std::ios::in | std::ios::out | std::ios::binary);
    openbus::map::writeMapBakeArchive(missingIndexArchive, {});
    missingIndexArchive.seekg(0);
    openbus::map::MapBakeArchiveReader missingIndexReader(missingIndexArchive);
    requireThrows([&] { (void)openbus::map::readMapBakeMapIndex(missingIndexReader); },
                  "archive without a map-wide index was accepted");

    auto unsupported = payload;
    unsupported[4] = std::byte{2};
    requireThrows([&] { (void)openbus::map::deserializeMapBakeMapIndex(unsupported); },
                  "unsupported map-index version was accepted");
    auto truncated = payload;
    truncated.pop_back();
    requireThrows([&] { (void)openbus::map::deserializeMapBakeMapIndex(truncated); },
                  "truncated map index was accepted");
    requireThrows(
        [&] {
            (void)openbus::map::serializeMapBakeMapIndex(
                {1, {{0, 0, "tile"}, {0, 0, "other"}}});
        },
        "duplicate map tile coordinates were accepted");
    requireThrows(
        [&] { (void)openbus::map::serializeMapBakeMapIndex({1, {{0, 0, ""}}}); },
        "empty tile-data chunk key was accepted");
}

} // namespace

int main() {
    try {
        run();
    } catch (const std::exception& error) {
        std::cerr << "map bake map-index probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
