#include "map/MapBakeArchive.h"

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
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

struct TemporaryDirectory {
    std::filesystem::path path;

    ~TemporaryDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
};

void run() {
    using openbus::map::MapBakeChunk;
    using openbus::map::MapBakeChunkType;
    using openbus::map::MapBakeArchiveReader;
    const std::vector<MapBakeChunk> chunks{
        {MapBakeChunkType::Manifest, "map", 0, 0,
         {std::byte{0x4d}, std::byte{0x41}, std::byte{0x50}}},
        {MapBakeChunkType::SharedAsset, "Sceneryobjects/bridge/model.o3d", 0, 0,
         {std::byte{0x00}, std::byte{0x10}, std::byte{0xff}}},
        {MapBakeChunkType::TileData, "tile", -17, 42,
         {std::byte{0x01}, std::byte{0x02}, std::byte{0x03}, std::byte{0x04}}}};

    std::stringstream archive(std::ios::in | std::ios::out | std::ios::binary);
    openbus::map::writeMapBakeArchive(archive, chunks);
    const std::string encoded = archive.str();
    std::stringstream streamedArchive(std::ios::in | std::ios::out | std::ios::binary);
    {
        openbus::map::MapBakeArchiveWriter writer(streamedArchive);
        for (const MapBakeChunk& chunk : chunks) {
            if (chunk.type == MapBakeChunkType::TileData) {
                const std::string content(reinterpret_cast<const char*>(chunk.payload.data()),
                                          chunk.payload.size());
                std::istringstream input(content, std::ios::in | std::ios::binary);
                writer.writeChunk(chunk.type, chunk.key, chunk.tileX, chunk.tileY, input,
                                  chunk.payload.size());
            } else {
                writer.writeChunk(chunk.type, chunk.key, chunk.tileX, chunk.tileY, chunk.payload);
            }
        }
        writer.finish();
    }
    require(streamedArchive.str() == encoded,
            "streaming chunk writer changed the archive encoding");
    archive.seekg(0);
    MapBakeArchiveReader reader(archive);
    require(reader.chunks().size() == chunks.size(), "archive chunk count changed");

    const auto manifest = reader.find(MapBakeChunkType::Manifest, "map");
    const auto sharedAsset =
        reader.find(MapBakeChunkType::SharedAsset, "Sceneryobjects/bridge/model.o3d");
    const auto tile = reader.find(MapBakeChunkType::TileData, "tile", -17, 42);
    require(manifest && sharedAsset && tile, "archive index lookup failed");
    require(reader.readChunk(*manifest) == chunks[*manifest].payload &&
                reader.readChunk(*sharedAsset) == chunks[*sharedAsset].payload &&
                reader.readChunk(*tile) == chunks[*tile].payload,
            "archive payload round trip failed");
    require(!reader.find(MapBakeChunkType::TileData, "tile", 17, 42),
            "signed tile coordinates were not indexed correctly");

    std::string corruptPayload = encoded;
    const std::uint64_t payloadOffset = reader.chunks()[*tile].payloadOffset;
    corruptPayload.at(static_cast<std::size_t>(payloadOffset)) ^= 1;
    std::istringstream corruptPayloadStream(corruptPayload,
                                            std::ios::in | std::ios::binary);
    MapBakeArchiveReader corruptPayloadReader(corruptPayloadStream);
    requireThrows([&] { (void)corruptPayloadReader.readChunk(*tile); },
                  "payload corruption was not detected");

    std::string unsupportedVersion = encoded;
    unsupportedVersion[8] = 2;
    std::istringstream unsupportedVersionStream(unsupportedVersion,
                                                std::ios::in | std::ios::binary);
    requireThrows([&] { MapBakeArchiveReader unsupported(unsupportedVersionStream); },
                  "unsupported archive version was accepted");

    std::istringstream truncatedStream(encoded.substr(0, 16),
                                       std::ios::in | std::ios::binary);
    requireThrows([&] { MapBakeArchiveReader truncated(truncatedStream); },
                  "truncated archive header was accepted");

    std::string corruptIndex = encoded;
    corruptIndex.back() ^= 1;
    std::istringstream corruptIndexStream(corruptIndex, std::ios::in | std::ios::binary);
    requireThrows([&] { MapBakeArchiveReader invalidIndex(corruptIndexStream); },
                  "index corruption was not detected");

    std::stringstream duplicateArchive(std::ios::in | std::ios::out | std::ios::binary);
    requireThrows(
        [&] { openbus::map::writeMapBakeArchive(duplicateArchive, {chunks.back(), chunks.back()}); },
        "duplicate chunk identity was accepted");

    std::stringstream emptyArchive(std::ios::in | std::ios::out | std::ios::binary);
    openbus::map::writeMapBakeArchive(emptyArchive, {});
    emptyArchive.seekg(0);
    MapBakeArchiveReader emptyReader(emptyArchive);
    require(emptyReader.chunks().empty(), "empty archive should have an empty index");

    const auto directory =
        std::filesystem::temp_directory_path() /
        ("openbus-map-bake-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    if (!std::filesystem::create_directory(directory)) {
        throw std::runtime_error("could not create isolated archive test directory");
    }
    const TemporaryDirectory cleanup{directory};
    const auto archivePath = directory / "map.obmap";
    {
        std::ofstream original(archivePath, std::ios::binary);
        original << "old archive";
    }
    openbus::map::writeMapBakeArchiveFile(
        archivePath, [&](openbus::map::MapBakeArchiveWriter& writer) {
            for (const MapBakeChunk& chunk : chunks) {
                if (chunk.type == MapBakeChunkType::TileData) {
                    const std::string content(reinterpret_cast<const char*>(chunk.payload.data()),
                                              chunk.payload.size());
                    std::istringstream input(content, std::ios::in | std::ios::binary);
                    writer.writeChunk(chunk.type, chunk.key, chunk.tileX, chunk.tileY, input,
                                      chunk.payload.size());
                } else {
                    writer.writeChunk(chunk.type, chunk.key, chunk.tileX, chunk.tileY,
                                      chunk.payload);
                }
            }
        });
    {
        std::ifstream file(archivePath, std::ios::binary);
        MapBakeArchiveReader fileReader(file);
        require(fileReader.chunks().size() == chunks.size(),
                "atomic archive writer did not replace the destination");
    }

    const auto readBytes = [](const std::filesystem::path& path) {
        std::ifstream file(path, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    };
    const std::string validArchive = readBytes(archivePath);
    requireThrows(
        [&] {
            openbus::map::writeMapBakeArchiveFile(archivePath, {chunks.back(), chunks.back()});
        },
        "invalid archive write unexpectedly succeeded");
    require(readBytes(archivePath) == validArchive,
            "failed atomic archive write replaced the previous valid archive");
    requireThrows(
        [&] {
            openbus::map::writeMapBakeArchiveFile(
                archivePath, [](openbus::map::MapBakeArchiveWriter& writer) {
                    std::istringstream incomplete("short", std::ios::in | std::ios::binary);
                    writer.writeChunk(MapBakeChunkType::TileData, "short", 0, 0, incomplete, 9);
                });
        },
        "short streamed chunk payload unexpectedly published");
    require(readBytes(archivePath) == validArchive,
            "failed streamed archive write replaced the previous valid archive");
    require(std::distance(std::filesystem::directory_iterator(directory),
                          std::filesystem::directory_iterator{}) == 1,
            "failed atomic archive write left a temporary file behind");
}

} // namespace

int main() {
    try {
        run();
    } catch (const std::exception& error) {
        std::cerr << "map bake archive probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
