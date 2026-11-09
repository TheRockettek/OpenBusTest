#include "map/MapBakeArchive.h"

#include "core/AtomicFile.h"

#include <algorithm>
#include <array>
#include <bit>
#include <istream>
#include <limits>
#include <ostream>
#include <set>
#include <span>
#include <stdexcept>
#include <utility>

namespace openbus::map {
namespace {

constexpr std::array<std::byte, 8> kMagic{std::byte{'O'}, std::byte{'B'}, std::byte{'M'},
                                          std::byte{'A'}, std::byte{'P'}, std::byte{'B'},
                                          std::byte{'K'}, std::byte{0}};
constexpr std::uint32_t kFormatVersion = 1;
constexpr std::uint32_t kHeaderSize = 32;
constexpr std::uint64_t kIndexEntryFixedSize = 40;
constexpr std::uint64_t kMaximumIndexSize = 256ULL * 1024ULL * 1024ULL;
constexpr std::uint32_t kMaximumChunkCount = 1'000'000;
constexpr std::uint32_t kMaximumKeySize = 1'048'576;
constexpr std::uint32_t kNoCompressionFlags = 0;
constexpr std::uint32_t kManifestType = static_cast<std::uint32_t>(MapBakeChunkType::Manifest);
constexpr std::uint32_t kMapWideIndexType =
    static_cast<std::uint32_t>(MapBakeChunkType::MapWideIndex);
constexpr std::uint32_t kSharedAssetType =
    static_cast<std::uint32_t>(MapBakeChunkType::SharedAsset);
constexpr std::uint32_t kTileDataType = static_cast<std::uint32_t>(MapBakeChunkType::TileData);

using ChunkKey = std::tuple<std::uint32_t, std::int32_t, std::int32_t, std::string>;

void appendU32(std::vector<std::byte>& bytes, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8) {
        bytes.push_back(static_cast<std::byte>((value >> shift) & 0xffU));
    }
}

void appendI32(std::vector<std::byte>& bytes, std::int32_t value) {
    appendU32(bytes, std::bit_cast<std::uint32_t>(value));
}

void appendU64(std::vector<std::byte>& bytes, std::uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8) {
        bytes.push_back(static_cast<std::byte>((value >> shift) & 0xffU));
    }
}

std::uint32_t readU32(std::span<const std::byte> bytes, std::size_t& cursor) {
    if (cursor > bytes.size() || bytes.size() - cursor < 4) {
        throw std::runtime_error("Truncated map bake archive integer");
    }
    std::uint32_t value = 0;
    for (unsigned shift = 0; shift < 32; shift += 8) {
        value |= static_cast<std::uint32_t>(std::to_integer<unsigned char>(bytes[cursor++]))
                 << shift;
    }
    return value;
}

std::int32_t readI32(std::span<const std::byte> bytes, std::size_t& cursor) {
    return std::bit_cast<std::int32_t>(readU32(bytes, cursor));
}

std::uint64_t readU64(std::span<const std::byte> bytes, std::size_t& cursor) {
    if (cursor > bytes.size() || bytes.size() - cursor < 8) {
        throw std::runtime_error("Truncated map bake archive integer");
    }
    std::uint64_t value = 0;
    for (unsigned shift = 0; shift < 64; shift += 8) {
        value |= static_cast<std::uint64_t>(std::to_integer<unsigned char>(bytes[cursor++]))
                 << shift;
    }
    return value;
}

std::uint32_t updateCrc32(std::uint32_t value, std::span<const std::byte> bytes) {
    for (const std::byte byte : bytes) {
        value ^= std::to_integer<unsigned char>(byte);
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t lowBitMask = 0U - (value & 1U);
            value = (value >> 1U) ^ (0xedb88320U & lowBitMask);
        }
    }
    return value;
}

std::uint32_t crc32(std::span<const std::byte> bytes) {
    return updateCrc32(0xffffffffU, bytes) ^ 0xffffffffU;
}

std::uint32_t chunkTypeValue(MapBakeChunkType type) {
    const auto value = static_cast<std::uint32_t>(type);
    if (value != kManifestType && value != kMapWideIndexType && value != kSharedAssetType &&
        value != kTileDataType) {
        throw std::invalid_argument("Unknown map bake chunk type");
    }
    return value;
}

std::uint64_t streamOffset(std::streampos position) {
    const std::streamoff offset = static_cast<std::streamoff>(position);
    if (offset < 0) {
        throw std::runtime_error("Map bake archive stream is not seekable");
    }
    return static_cast<std::uint64_t>(offset);
}

std::vector<std::byte> encodeHeader(std::uint64_t indexOffset, std::uint32_t chunkCount,
                                    std::uint32_t indexChecksum) {
    std::vector<std::byte> header(kMagic.begin(), kMagic.end());
    appendU32(header, kFormatVersion);
    appendU32(header, kHeaderSize);
    appendU64(header, indexOffset);
    appendU32(header, chunkCount);
    appendU32(header, indexChecksum);
    return header;
}

void writeBytes(std::ostream& output, std::span<const std::byte> bytes) {
    if (bytes.size() > static_cast<std::size_t>((std::numeric_limits<std::streamsize>::max)())) {
        throw std::length_error("Map bake archive write exceeds stream limits");
    }
    if (!bytes.empty()) {
        output.write(reinterpret_cast<const char*>(bytes.data()),
                     static_cast<std::streamsize>(bytes.size()));
    }
    if (!output) {
        throw std::runtime_error("Could not write map bake archive");
    }
}

void validateChunkBatch(const std::vector<MapBakeChunk>& chunks) {
    if (chunks.size() > kMaximumChunkCount) {
        throw std::length_error("Map bake archive has too many chunks");
    }

    std::set<ChunkKey> identities;
    std::uint64_t indexSize = 0;
    for (const MapBakeChunk& chunk : chunks) {
        const auto type = chunkTypeValue(chunk.type);
        if (chunk.key.empty() || chunk.key.size() > kMaximumKeySize ||
            chunk.key.find('\0') != std::string::npos) {
            throw std::invalid_argument("Map bake chunk key is empty or invalid");
        }
        if (!identities.emplace(type, chunk.tileX, chunk.tileY, chunk.key).second) {
            throw std::invalid_argument("Duplicate map bake chunk identity");
        }
        const std::uint64_t entrySize =
            kIndexEntryFixedSize + static_cast<std::uint64_t>(chunk.key.size());
        if (entrySize > kMaximumIndexSize - indexSize) {
            throw std::length_error("Map bake archive index exceeds supported size");
        }
        indexSize += entrySize;
        if (chunk.payload.size() >
            static_cast<std::size_t>((std::numeric_limits<std::streamsize>::max)())) {
            throw std::length_error("Map bake chunk payload exceeds stream limits");
        }
    }
}

} // namespace

MapBakeArchiveWriter::MapBakeArchiveWriter(std::ostream& output) : output_(output) {
    output.seekp(0, std::ios::end);
    if (output.tellp() != std::streampos(0)) {
        throw std::invalid_argument("Map bake archive output must start at offset zero");
    }
    output.seekp(0, std::ios::beg);
    if (!output) {
        throw std::runtime_error("Map bake archive output is not seekable");
    }
    const auto placeholder = encodeHeader(0, 0, 0);
    writeBytes(output, placeholder);
}

void MapBakeArchiveWriter::writeChunk(MapBakeChunkType type, const std::string& key,
                                      std::int32_t tileX, std::int32_t tileY,
                                      std::span<const std::byte> payload) {
    if (finished_ || failed_) {
        throw std::logic_error("Map bake archive writer is not active");
    }
    const std::uint32_t typeValue = chunkTypeValue(type);
    if (key.empty() || key.size() > kMaximumKeySize || key.find('\0') != std::string::npos) {
        throw std::invalid_argument("Map bake chunk key is empty or invalid");
    }
    if (chunks_.size() >= kMaximumChunkCount) {
        throw std::length_error("Map bake archive has too many chunks");
    }
    if (payload.size() > static_cast<std::size_t>((std::numeric_limits<std::streamsize>::max)())) {
        throw std::length_error("Map bake chunk payload exceeds stream limits");
    }
    const std::uint64_t entrySize = kIndexEntryFixedSize + static_cast<std::uint64_t>(key.size());
    if (entrySize > kMaximumIndexSize - indexSize_) {
        throw std::length_error("Map bake archive index exceeds supported size");
    }
    const ChunkKey identity{typeValue, tileX, tileY, key};
    if (!chunkLookup_.insert(identity).second) {
        throw std::invalid_argument("Duplicate map bake chunk identity");
    }
    indexSize_ += entrySize;

    try {
        const std::uint64_t offset = streamOffset(output_.tellp());
        writeBytes(output_, payload);
        chunks_.push_back({type, key, tileX, tileY, offset,
                           static_cast<std::uint64_t>(payload.size()), crc32(payload)});
    } catch (...) {
        failed_ = true;
        throw;
    }
}

void MapBakeArchiveWriter::writeChunk(MapBakeChunkType type, const std::string& key,
                                      std::int32_t tileX, std::int32_t tileY, std::istream& payload,
                                      std::uint64_t payloadSize) {
    if (finished_ || failed_) {
        throw std::logic_error("Map bake archive writer is not active");
    }
    const std::uint32_t typeValue = chunkTypeValue(type);
    if (key.empty() || key.size() > kMaximumKeySize || key.find('\0') != std::string::npos) {
        throw std::invalid_argument("Map bake chunk key is empty or invalid");
    }
    if (chunks_.size() >= kMaximumChunkCount) {
        throw std::length_error("Map bake archive has too many chunks");
    }
    if (payloadSize > static_cast<std::uint64_t>((std::numeric_limits<std::streamoff>::max)())) {
        throw std::length_error("Map bake chunk payload exceeds stream limits");
    }
    const std::uint64_t entrySize = kIndexEntryFixedSize + static_cast<std::uint64_t>(key.size());
    if (entrySize > kMaximumIndexSize - indexSize_) {
        throw std::length_error("Map bake archive index exceeds supported size");
    }
    const ChunkKey identity{typeValue, tileX, tileY, key};
    if (!chunkLookup_.insert(identity).second) {
        throw std::invalid_argument("Duplicate map bake chunk identity");
    }
    indexSize_ += entrySize;

    try {
        const std::uint64_t offset = streamOffset(output_.tellp());
        std::array<std::byte, 64U * 1024U> buffer{};
        std::uint64_t remaining = payloadSize;
        std::uint32_t checksum = 0xffffffffU;
        while (remaining > 0) {
            const auto amount =
                static_cast<std::size_t>(std::min<std::uint64_t>(remaining, buffer.size()));
            payload.read(reinterpret_cast<char*>(buffer.data()),
                         static_cast<std::streamsize>(amount));
            if (payload.gcount() != static_cast<std::streamsize>(amount)) {
                throw std::runtime_error("Could not read complete map bake chunk payload: " + key);
            }
            const std::span<const std::byte> part(buffer.data(), amount);
            writeBytes(output_, part);
            checksum = updateCrc32(checksum, part);
            remaining -= amount;
        }
        chunks_.push_back({type, key, tileX, tileY, offset, payloadSize, checksum ^ 0xffffffffU});
    } catch (...) {
        failed_ = true;
        throw;
    }
}

void MapBakeArchiveWriter::finish() {
    if (finished_ || failed_) {
        throw std::logic_error("Map bake archive writer is not active");
    }
    try {
        const std::uint64_t indexOffset = streamOffset(output_.tellp());
        std::vector<std::byte> index;
        index.reserve(static_cast<std::size_t>(indexSize_));
        for (const MapBakeChunkInfo& entry : chunks_) {
            appendU32(index, chunkTypeValue(entry.type));
            appendU32(index, static_cast<std::uint32_t>(entry.key.size()));
            appendI32(index, entry.tileX);
            appendI32(index, entry.tileY);
            appendU64(index, entry.payloadOffset);
            appendU64(index, entry.payloadSize);
            appendU32(index, entry.payloadChecksum);
            appendU32(index, kNoCompressionFlags);
            const auto* keyData = reinterpret_cast<const std::byte*>(entry.key.data());
            index.insert(index.end(), keyData, keyData + entry.key.size());
        }
        const std::uint32_t indexChecksum = crc32(index);
        writeBytes(output_, index);
        const std::streampos endPosition = output_.tellp();
        if (endPosition == std::streampos(-1)) {
            throw std::runtime_error("Could not determine map bake archive length");
        }
        output_.seekp(0, std::ios::beg);
        const auto header =
            encodeHeader(indexOffset, static_cast<std::uint32_t>(chunks_.size()), indexChecksum);
        writeBytes(output_, header);
        output_.seekp(endPosition);
        if (!output_) {
            throw std::runtime_error("Could not finalize map bake archive header");
        }
        finished_ = true;
    } catch (...) {
        failed_ = true;
        throw;
    }
}

void writeMapBakeArchive(std::ostream& output, const std::vector<MapBakeChunk>& chunks) {
    validateChunkBatch(chunks);
    MapBakeArchiveWriter writer(output);
    for (const MapBakeChunk& chunk : chunks) {
        writer.writeChunk(chunk.type, chunk.key, chunk.tileX, chunk.tileY, chunk.payload);
    }
    writer.finish();
}

void writeMapBakeArchiveFile(const std::filesystem::path& path,
                             const std::vector<MapBakeChunk>& chunks) {
    validateChunkBatch(chunks);
    writeMapBakeArchiveFile(path, [&](MapBakeArchiveWriter& writer) {
        for (const MapBakeChunk& chunk : chunks) {
            writer.writeChunk(chunk.type, chunk.key, chunk.tileX, chunk.tileY, chunk.payload);
        }
    });
}

void writeMapBakeArchiveFile(const std::filesystem::path& path,
                             const std::function<void(MapBakeArchiveWriter&)>& writeChunks) {
    if (!writeChunks) {
        throw std::invalid_argument("Map bake archive file writer callback is empty");
    }
    core::writeFileAtomically(path, [&](std::ostream& output) {
        MapBakeArchiveWriter writer(output);
        writeChunks(writer);
        writer.finish();
    });
}

MapBakeArchiveReader::MapBakeArchiveReader(std::istream& input) : input_(input) {
    if (input_.tellg() != std::streampos(0)) {
        throw std::invalid_argument("Map bake archive input must start at offset zero");
    }
    input_.seekg(0, std::ios::end);
    const std::streampos endPosition = input_.tellg();
    const std::uint64_t fileSize = streamOffset(endPosition);
    if (fileSize < kHeaderSize ||
        fileSize > static_cast<std::uint64_t>((std::numeric_limits<std::streamoff>::max)())) {
        throw std::runtime_error("Invalid map bake archive size");
    }
    input_.seekg(0, std::ios::beg);
    std::array<std::byte, kHeaderSize> header{};
    input_.read(reinterpret_cast<char*>(header.data()),
                static_cast<std::streamsize>(header.size()));
    if (!input_) {
        throw std::runtime_error("Truncated map bake archive header");
    }

    const std::span<const std::byte> headerBytes(header);
    if (!std::equal(kMagic.begin(), kMagic.end(), header.begin())) {
        throw std::runtime_error("Invalid map bake archive magic");
    }
    std::size_t cursor = kMagic.size();
    const std::uint32_t version = readU32(headerBytes, cursor);
    const std::uint32_t headerSize = readU32(headerBytes, cursor);
    const std::uint64_t indexOffset = readU64(headerBytes, cursor);
    const std::uint32_t chunkCount = readU32(headerBytes, cursor);
    const std::uint32_t indexChecksum = readU32(headerBytes, cursor);
    if (version != kFormatVersion) {
        throw std::runtime_error("Unsupported map bake archive version");
    }
    if (headerSize != kHeaderSize || chunkCount > kMaximumChunkCount || indexOffset < kHeaderSize ||
        indexOffset > fileSize) {
        throw std::runtime_error("Invalid map bake archive header");
    }

    const std::uint64_t indexSize = fileSize - indexOffset;
    if (indexSize > kMaximumIndexSize ||
        indexSize > static_cast<std::uint64_t>((std::numeric_limits<std::size_t>::max)()) ||
        chunkCount > indexSize / kIndexEntryFixedSize) {
        throw std::runtime_error("Invalid map bake archive index size");
    }
    std::vector<std::byte> index(static_cast<std::size_t>(indexSize));
    input_.seekg(static_cast<std::streamoff>(indexOffset), std::ios::beg);
    if (!index.empty()) {
        input_.read(reinterpret_cast<char*>(index.data()),
                    static_cast<std::streamsize>(index.size()));
    }
    if (!input_ || crc32(index) != indexChecksum) {
        throw std::runtime_error("Invalid or corrupt map bake archive index");
    }

    std::vector<std::pair<std::uint64_t, std::uint64_t>> payloadRanges;
    payloadRanges.reserve(chunkCount);
    chunks_.reserve(chunkCount);
    cursor = 0;
    for (std::uint32_t indexEntry = 0; indexEntry < chunkCount; ++indexEntry) {
        const std::uint32_t typeValue = readU32(index, cursor);
        const std::uint32_t keySize = readU32(index, cursor);
        const std::int32_t tileX = readI32(index, cursor);
        const std::int32_t tileY = readI32(index, cursor);
        const std::uint64_t payloadOffset = readU64(index, cursor);
        const std::uint64_t payloadSize = readU64(index, cursor);
        const std::uint32_t payloadChecksum = readU32(index, cursor);
        const std::uint32_t flags = readU32(index, cursor);
        if (keySize == 0 || keySize > kMaximumKeySize || cursor > index.size() ||
            index.size() - cursor < keySize) {
            throw std::runtime_error("Invalid map bake archive chunk key");
        }
        if (flags != kNoCompressionFlags) {
            throw std::runtime_error("Unsupported map bake archive chunk flags");
        }
        if (typeValue != kManifestType && typeValue != kMapWideIndexType &&
            typeValue != kSharedAssetType && typeValue != kTileDataType) {
            throw std::runtime_error("Unknown map bake archive chunk type");
        }
        std::string key(reinterpret_cast<const char*>(index.data() + cursor), keySize);
        cursor += keySize;
        if (key.find('\0') != std::string::npos || payloadOffset < kHeaderSize ||
            payloadOffset > indexOffset || payloadSize > indexOffset - payloadOffset) {
            throw std::runtime_error("Invalid map bake archive chunk bounds or key");
        }

        const auto type = static_cast<MapBakeChunkType>(typeValue);
        const std::size_t chunkIndex = chunks_.size();
        const ChunkKey identity{typeValue, tileX, tileY, key};
        if (!chunkLookup_.emplace(identity, chunkIndex).second) {
            throw std::runtime_error("Duplicate map bake archive chunk identity");
        }
        chunks_.push_back(
            {type, std::move(key), tileX, tileY, payloadOffset, payloadSize, payloadChecksum});
        if (payloadSize != 0) {
            payloadRanges.emplace_back(payloadOffset, payloadOffset + payloadSize);
        }
    }
    if (cursor != index.size()) {
        throw std::runtime_error("Unexpected trailing bytes in map bake archive index");
    }
    std::sort(payloadRanges.begin(), payloadRanges.end());
    for (std::size_t range = 1; range < payloadRanges.size(); ++range) {
        if (payloadRanges[range].first < payloadRanges[range - 1].second) {
            throw std::runtime_error("Overlapping map bake archive chunks");
        }
    }
}

const std::vector<MapBakeChunkInfo>& MapBakeArchiveReader::chunks() const noexcept {
    return chunks_;
}

std::optional<std::size_t> MapBakeArchiveReader::find(MapBakeChunkType type, std::string_view key,
                                                      std::int32_t tileX,
                                                      std::int32_t tileY) const {
    const auto found = chunkLookup_.find({chunkTypeValue(type), tileX, tileY, std::string(key)});
    if (found == chunkLookup_.end()) {
        return std::nullopt;
    }
    return found->second;
}

std::vector<std::byte> MapBakeArchiveReader::readChunk(std::size_t index) {
    if (index >= chunks_.size()) {
        throw std::out_of_range("Map bake archive chunk index is out of range");
    }
    const MapBakeChunkInfo& entry = chunks_[index];
    if (entry.payloadSize > static_cast<std::uint64_t>((std::numeric_limits<std::size_t>::max)()) ||
        entry.payloadSize >
            static_cast<std::uint64_t>((std::numeric_limits<std::streamsize>::max)())) {
        throw std::length_error("Map bake archive chunk exceeds read limits");
    }
    std::vector<std::byte> payload(static_cast<std::size_t>(entry.payloadSize));
    input_.clear();
    input_.seekg(static_cast<std::streamoff>(entry.payloadOffset), std::ios::beg);
    if (!payload.empty()) {
        input_.read(reinterpret_cast<char*>(payload.data()),
                    static_cast<std::streamsize>(payload.size()));
    }
    if (!input_) {
        throw std::runtime_error("Could not read map bake archive chunk: " + entry.key);
    }
    if (crc32(payload) != entry.payloadChecksum) {
        throw std::runtime_error("Map bake archive chunk checksum mismatch: " + entry.key);
    }
    return payload;
}

} // namespace openbus::map
