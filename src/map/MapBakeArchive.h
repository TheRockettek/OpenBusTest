#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iosfwd>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace openbus::map {

enum class MapBakeChunkType : std::uint32_t {
    Manifest = 1,
    MapWideIndex = 2,
    SharedAsset = 3,
    TileData = 4,
};

struct MapBakeChunk {
    MapBakeChunkType type = MapBakeChunkType::Manifest;
    std::string key;
    std::int32_t tileX = 0;
    std::int32_t tileY = 0;
    std::vector<std::byte> payload;
};

struct MapBakeChunkInfo {
    MapBakeChunkType type = MapBakeChunkType::Manifest;
    std::string key;
    std::int32_t tileX = 0;
    std::int32_t tileY = 0;
    std::uint64_t payloadOffset = 0;
    std::uint64_t payloadSize = 0;
    std::uint32_t payloadChecksum = 0;
};

class MapBakeArchiveWriter;

void writeMapBakeArchive(std::ostream& output, const std::vector<MapBakeChunk>& chunks);
void writeMapBakeArchiveFile(const std::filesystem::path& path,
                             const std::vector<MapBakeChunk>& chunks);
void writeMapBakeArchiveFile(const std::filesystem::path& path,
                             const std::function<void(MapBakeArchiveWriter&)>& writeChunks);

class MapBakeArchiveWriter {
  public:
    explicit MapBakeArchiveWriter(std::ostream& output);

    void writeChunk(MapBakeChunkType type, const std::string& key, std::int32_t tileX,
                    std::int32_t tileY, std::span<const std::byte> payload);
    void writeChunk(MapBakeChunkType type, const std::string& key, std::int32_t tileX,
                    std::int32_t tileY, std::istream& payload, std::uint64_t payloadSize);
    void finish();

  private:
    using ChunkKey = std::tuple<std::uint32_t, std::int32_t, std::int32_t, std::string>;

    std::ostream& output_;
    std::set<ChunkKey> chunkLookup_;
    std::vector<MapBakeChunkInfo> chunks_;
    std::uint64_t indexSize_ = 0;
    bool finished_ = false;
    bool failed_ = false;
};

class MapBakeArchiveReader {
  public:
    explicit MapBakeArchiveReader(std::istream& input);

    [[nodiscard]] const std::vector<MapBakeChunkInfo>& chunks() const noexcept;
    [[nodiscard]] std::optional<std::size_t> find(MapBakeChunkType type, std::string_view key,
                                                  std::int32_t tileX = 0,
                                                  std::int32_t tileY = 0) const;
    [[nodiscard]] std::vector<std::byte> readChunk(std::size_t index);

  private:
    using ChunkKey = std::tuple<std::uint32_t, std::int32_t, std::int32_t, std::string>;

    std::istream& input_;
    std::vector<MapBakeChunkInfo> chunks_;
    std::map<ChunkKey, std::size_t> chunkLookup_;
};

} // namespace openbus::map
