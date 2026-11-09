#include "map/MapBakeMapIndex.h"

#include "map/MapBakeArchive.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

namespace openbus::map {
namespace {

constexpr std::array<std::byte, 4> kIndexMagic{std::byte{'O'}, std::byte{'B'}, std::byte{'M'},
                                               std::byte{'I'}};
constexpr std::uint32_t kIndexVersion = 1;
constexpr std::uint32_t kMaximumKeySize = 1'048'576;
constexpr std::uint32_t kMaximumTileCount = 100'000;
constexpr std::size_t kMaximumPayloadSize = 64U * 1024U * 1024U;

void appendU32(std::vector<std::byte>& bytes, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8) {
        bytes.push_back(static_cast<std::byte>((value >> shift) & 0xffU));
    }
}

std::uint32_t readU32(std::span<const std::byte> bytes, std::size_t& cursor) {
    if (cursor > bytes.size() || bytes.size() - cursor < 4) {
        throw std::runtime_error("Truncated map bake map-index integer");
    }
    std::uint32_t value = 0;
    for (unsigned shift = 0; shift < 32; shift += 8) {
        value |= static_cast<std::uint32_t>(std::to_integer<unsigned char>(bytes[cursor++]))
                 << shift;
    }
    return value;
}

std::string readKey(std::span<const std::byte> bytes, std::size_t& cursor) {
    const std::uint32_t size = readU32(bytes, cursor);
    if (size == 0 || size > kMaximumKeySize || cursor > bytes.size() ||
        bytes.size() - cursor < size) {
        throw std::runtime_error("Invalid map bake tile chunk key");
    }
    std::string key(reinterpret_cast<const char*>(bytes.data() + cursor), size);
    cursor += size;
    if (key.find('\0') != std::string::npos) {
        throw std::runtime_error("Map bake tile chunk key contains a NUL byte");
    }
    return key;
}

bool precedes(const MapBakeTileIndexEntry& left, const MapBakeTileIndexEntry& right) {
    return std::tie(left.tileY, left.tileX) < std::tie(right.tileY, right.tileX);
}

void validateIndex(const MapBakeMapIndex& index) {
    if (index.schemaVersion != kIndexVersion) {
        throw std::invalid_argument("Unsupported map bake map-index version");
    }
    if (index.tiles.size() > kMaximumTileCount) {
        throw std::length_error("Map bake map index has too many tile entries");
    }
    std::set<std::pair<std::int32_t, std::int32_t>> coordinates;
    for (const MapBakeTileIndexEntry& entry : index.tiles) {
        if (entry.chunkKey.empty() || entry.chunkKey.size() > kMaximumKeySize ||
            entry.chunkKey.find('\0') != std::string::npos) {
            throw std::invalid_argument("Invalid map bake tile chunk key");
        }
        if (!coordinates.emplace(entry.tileX, entry.tileY).second) {
            throw std::invalid_argument("Duplicate tile coordinates in map bake map index");
        }
    }
}

} // namespace

std::vector<std::byte> serializeMapBakeMapIndex(const MapBakeMapIndex& index) {
    validateIndex(index);
    std::vector<MapBakeTileIndexEntry> tiles = index.tiles;
    std::sort(tiles.begin(), tiles.end(), precedes);

    std::uint64_t payloadSize = 12;
    for (const MapBakeTileIndexEntry& tile : tiles) {
        const std::uint64_t entrySize = 12ULL + tile.chunkKey.size();
        if (entrySize > kMaximumPayloadSize - payloadSize) {
            throw std::length_error("Map bake map index exceeds supported size");
        }
        payloadSize += entrySize;
    }

    std::vector<std::byte> payload(kIndexMagic.begin(), kIndexMagic.end());
    appendU32(payload, kIndexVersion);
    appendU32(payload, static_cast<std::uint32_t>(tiles.size()));
    for (const MapBakeTileIndexEntry& tile : tiles) {
        appendU32(payload, std::bit_cast<std::uint32_t>(tile.tileX));
        appendU32(payload, std::bit_cast<std::uint32_t>(tile.tileY));
        appendU32(payload, static_cast<std::uint32_t>(tile.chunkKey.size()));
        const auto* key = reinterpret_cast<const std::byte*>(tile.chunkKey.data());
        payload.insert(payload.end(), key, key + tile.chunkKey.size());
    }
    return payload;
}

MapBakeMapIndex deserializeMapBakeMapIndex(std::span<const std::byte> payload) {
    if (payload.size() > kMaximumPayloadSize || payload.size() < 12 ||
        !std::equal(kIndexMagic.begin(), kIndexMagic.end(), payload.begin())) {
        throw std::runtime_error("Invalid map bake map-index payload");
    }
    std::size_t cursor = kIndexMagic.size();
    const std::uint32_t version = readU32(payload, cursor);
    if (version != kIndexVersion) {
        throw std::runtime_error("Unsupported map bake map-index version");
    }
    const std::uint32_t tileCount = readU32(payload, cursor);
    if (tileCount > kMaximumTileCount || tileCount > (payload.size() - cursor) / 12) {
        throw std::runtime_error("Map bake map-index tile count exceeds payload bounds");
    }

    MapBakeMapIndex index;
    index.schemaVersion = version;
    index.tiles.reserve(tileCount);
    for (std::uint32_t tile = 0; tile < tileCount; ++tile) {
        MapBakeTileIndexEntry entry;
        entry.tileX = std::bit_cast<std::int32_t>(readU32(payload, cursor));
        entry.tileY = std::bit_cast<std::int32_t>(readU32(payload, cursor));
        entry.chunkKey = readKey(payload, cursor);
        if (!index.tiles.empty() && !precedes(index.tiles.back(), entry)) {
            throw std::runtime_error("Map bake tile index entries are not uniquely sorted");
        }
        index.tiles.push_back(std::move(entry));
    }
    if (cursor != payload.size()) {
        throw std::runtime_error("Unexpected trailing bytes in map bake map index");
    }
    return index;
}

MapBakeMapIndex readMapBakeMapIndex(MapBakeArchiveReader& archive) {
    const auto indexChunk = archive.find(MapBakeChunkType::MapWideIndex, "map");
    if (!indexChunk) {
        throw std::runtime_error("Map bake archive is missing its map-wide index chunk");
    }

    MapBakeMapIndex index = deserializeMapBakeMapIndex(archive.readChunk(*indexChunk));
    for (const MapBakeTileIndexEntry& tile : index.tiles) {
        if (!archive.find(MapBakeChunkType::TileData, tile.chunkKey, tile.tileX, tile.tileY)) {
            throw std::runtime_error("Map bake map index references missing tile-data chunk '" +
                                     tile.chunkKey + "' at (" + std::to_string(tile.tileX) + ", " +
                                     std::to_string(tile.tileY) + ")");
        }
    }
    return index;
}

} // namespace openbus::map
