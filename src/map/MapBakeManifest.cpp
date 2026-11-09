#include "map/MapBakeManifest.h"

#include "core/Utf8Path.h"
#include "map/MapBakeArchive.h"

#include <algorithm>
#include <array>
#include <bit>
#include <fstream>
#include <limits>
#include <set>
#include <span>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <utility>

namespace openbus::map {
namespace {

constexpr std::array<std::byte, 4> kManifestMagic{std::byte{'O'}, std::byte{'B'}, std::byte{'M'},
                                                  std::byte{'F'}};
constexpr std::uint32_t kManifestVersion = 1;
constexpr std::uint32_t kMaximumStringSize = 1'048'576;
constexpr std::uint32_t kMaximumRecordCount = 100'000;
constexpr std::size_t kMaximumManifestSize = 64U * 1024U * 1024U;
constexpr std::array<std::uint32_t, 64> kSha256Constants{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U,
    0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU,
    0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU,
    0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
    0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
    0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U,
    0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U,
    0xc67178f2U};

void appendU32(std::vector<std::byte>& bytes, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8) {
        bytes.push_back(static_cast<std::byte>((value >> shift) & 0xffU));
    }
}

void appendU64(std::vector<std::byte>& bytes, std::uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8) {
        bytes.push_back(static_cast<std::byte>((value >> shift) & 0xffU));
    }
}

void appendString(std::vector<std::byte>& bytes, const std::string& value) {
    if (value.empty() || value.size() > kMaximumStringSize ||
        value.find('\0') != std::string::npos) {
        throw std::invalid_argument("Map bake manifest string is empty or invalid");
    }
    appendU32(bytes, static_cast<std::uint32_t>(value.size()));
    const auto* begin = reinterpret_cast<const std::byte*>(value.data());
    bytes.insert(bytes.end(), begin, begin + value.size());
}

std::uint32_t readU32(std::span<const std::byte> bytes, std::size_t& cursor) {
    if (cursor > bytes.size() || bytes.size() - cursor < 4) {
        throw std::runtime_error("Truncated map bake manifest integer");
    }
    std::uint32_t value = 0;
    for (unsigned shift = 0; shift < 32; shift += 8) {
        value |= static_cast<std::uint32_t>(std::to_integer<unsigned char>(bytes[cursor++]))
                 << shift;
    }
    return value;
}

std::uint64_t readU64(std::span<const std::byte> bytes, std::size_t& cursor) {
    if (cursor > bytes.size() || bytes.size() - cursor < 8) {
        throw std::runtime_error("Truncated map bake manifest integer");
    }
    std::uint64_t value = 0;
    for (unsigned shift = 0; shift < 64; shift += 8) {
        value |= static_cast<std::uint64_t>(std::to_integer<unsigned char>(bytes[cursor++]))
                 << shift;
    }
    return value;
}

std::string readString(std::span<const std::byte> bytes, std::size_t& cursor) {
    const std::uint32_t size = readU32(bytes, cursor);
    if (size == 0 || size > kMaximumStringSize || cursor > bytes.size() ||
        bytes.size() - cursor < size) {
        throw std::runtime_error("Invalid map bake manifest string");
    }
    std::string value(reinterpret_cast<const char*>(bytes.data() + cursor), size);
    cursor += size;
    if (value.find('\0') != std::string::npos) {
        throw std::runtime_error("Map bake manifest string contains a NUL byte");
    }
    return value;
}

class Sha256 {
  public:
    void update(std::span<const std::byte> input) {
        if (input.size() > (std::numeric_limits<std::uint64_t>::max)() - byteCount_) {
            throw std::length_error("Source file is too large to fingerprint");
        }
        byteCount_ += input.size();
        std::size_t cursor = 0;
        while (cursor < input.size()) {
            const std::size_t copySize =
                std::min(buffer_.size() - bufferedBytes_, input.size() - cursor);
            std::copy_n(input.begin() + static_cast<std::ptrdiff_t>(cursor), copySize,
                        buffer_.begin() + static_cast<std::ptrdiff_t>(bufferedBytes_));
            cursor += copySize;
            bufferedBytes_ += copySize;
            if (bufferedBytes_ == buffer_.size()) {
                transform(buffer_);
                bufferedBytes_ = 0;
            }
        }
    }

    std::array<std::byte, 32> finish() {
        if (byteCount_ > (std::numeric_limits<std::uint64_t>::max)() / 8U) {
            throw std::length_error("Source file is too large to fingerprint");
        }
        const std::uint64_t bitCount = byteCount_ * 8U;
        buffer_[bufferedBytes_++] = std::byte{0x80};
        if (bufferedBytes_ > 56) {
            std::fill(buffer_.begin() + static_cast<std::ptrdiff_t>(bufferedBytes_), buffer_.end(),
                      std::byte{0});
            transform(buffer_);
            bufferedBytes_ = 0;
        }
        std::fill(buffer_.begin() + static_cast<std::ptrdiff_t>(bufferedBytes_),
                  buffer_.begin() + 56, std::byte{0});
        for (unsigned byte = 0; byte < 8; ++byte) {
            buffer_[56 + byte] = static_cast<std::byte>((bitCount >> ((7U - byte) * 8U)) & 0xffU);
        }
        transform(buffer_);

        std::array<std::byte, 32> digest{};
        for (std::size_t word = 0; word < state_.size(); ++word) {
            for (unsigned byte = 0; byte < 4; ++byte) {
                digest[word * 4 + byte] =
                    static_cast<std::byte>((state_[word] >> ((3U - byte) * 8U)) & 0xffU);
            }
        }
        return digest;
    }

  private:
    void transform(const std::array<std::byte, 64>& block) {
        std::array<std::uint32_t, 64> words{};
        for (std::size_t word = 0; word < 16; ++word) {
            const std::size_t offset = word * 4;
            words[word] =
                (static_cast<std::uint32_t>(std::to_integer<unsigned char>(block[offset])) << 24U) |
                (static_cast<std::uint32_t>(std::to_integer<unsigned char>(block[offset + 1]))
                 << 16U) |
                (static_cast<std::uint32_t>(std::to_integer<unsigned char>(block[offset + 2]))
                 << 8U) |
                static_cast<std::uint32_t>(std::to_integer<unsigned char>(block[offset + 3]));
        }
        for (std::size_t word = 16; word < words.size(); ++word) {
            const std::uint32_t x = words[word - 15];
            const std::uint32_t y = words[word - 2];
            const std::uint32_t smallSigma0 = std::rotr(x, 7) ^ std::rotr(x, 18) ^ (x >> 3U);
            const std::uint32_t smallSigma1 = std::rotr(y, 17) ^ std::rotr(y, 19) ^ (y >> 10U);
            words[word] = words[word - 16] + smallSigma0 + words[word - 7] + smallSigma1;
        }

        auto working = state_;
        for (std::size_t word = 0; word < words.size(); ++word) {
            const std::uint32_t bigSigma1 =
                std::rotr(working[4], 6) ^ std::rotr(working[4], 11) ^ std::rotr(working[4], 25);
            const std::uint32_t choice = (working[4] & working[5]) ^ (~working[4] & working[6]);
            const std::uint32_t first =
                working[7] + bigSigma1 + choice + kSha256Constants[word] + words[word];
            const std::uint32_t bigSigma0 =
                std::rotr(working[0], 2) ^ std::rotr(working[0], 13) ^ std::rotr(working[0], 22);
            const std::uint32_t majority =
                (working[0] & working[1]) ^ (working[0] & working[2]) ^ (working[1] & working[2]);
            const std::uint32_t second = bigSigma0 + majority;
            working = {first + second,     working[0], working[1], working[2],
                       working[3] + first, working[4], working[5], working[6]};
        }
        for (std::size_t word = 0; word < state_.size(); ++word) {
            state_[word] += working[word];
        }
    }

    std::array<std::uint32_t, 8> state_{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                                        0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
    std::array<std::byte, 64> buffer_{};
    std::uint64_t byteCount_ = 0;
    std::size_t bufferedBytes_ = 0;
};

std::array<std::byte, 32> hashFile(const std::filesystem::path& path, std::uint64_t expectedSize) {
    const auto timestampBefore = std::filesystem::last_write_time(path);
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not open map bake source: " + path.string());
    }

    Sha256 hash;
    std::array<char, 64 * 1024> buffer{};
    std::uint64_t bytesRead = 0;
    while (true) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize count = input.gcount();
        if (count > 0) {
            const auto bytes =
                std::as_bytes(std::span(buffer.data(), static_cast<std::size_t>(count)));
            hash.update(bytes);
            bytesRead += static_cast<std::uint64_t>(count);
        }
        if (input.eof()) {
            break;
        }
        if (!input) {
            throw std::runtime_error("Could not read map bake source: " + path.string());
        }
    }
    const auto sizeAfter = std::filesystem::file_size(path);
    const auto timestampAfter = std::filesystem::last_write_time(path);
    if (bytesRead != expectedSize || sizeAfter != expectedSize ||
        timestampBefore != timestampAfter) {
        throw std::runtime_error("Map bake source changed while fingerprinting: " + path.string());
    }
    return hash.finish();
}

void validateManifest(const MapBakeManifest& manifest) {
    if (manifest.schemaVersion != kManifestVersion) {
        throw std::invalid_argument("Unsupported map bake manifest version");
    }
    if (manifest.mapKey.empty() || manifest.mapKey.size() > kMaximumStringSize ||
        manifest.mapKey.find('\0') != std::string::npos ||
        manifest.settings.size() > kMaximumRecordCount ||
        manifest.dependencies.size() > kMaximumRecordCount) {
        throw std::invalid_argument("Invalid map bake manifest");
    }
    for (const auto& [key, value] : manifest.settings) {
        if (key.empty() || key.size() > kMaximumStringSize || key.find('\0') != std::string::npos ||
            value.size() > kMaximumStringSize || value.find('\0') != std::string::npos) {
            throw std::invalid_argument("Invalid map bake manifest setting");
        }
    }
    std::string previousKey;
    for (const MapBakeDependency& dependency : manifest.dependencies) {
        if (dependency.key.empty() || dependency.key.size() > kMaximumStringSize ||
            dependency.key.find('\0') != std::string::npos ||
            (!previousKey.empty() && dependency.key <= previousKey)) {
            throw std::invalid_argument(
                "Map bake manifest dependencies must have unique sorted keys");
        }
        previousKey = dependency.key;
    }
}

bool isWithin(const std::filesystem::path& path, const std::filesystem::path& root,
              std::filesystem::path& relative) {
    relative = path.lexically_relative(root);
    if (relative.empty() || relative.is_absolute()) {
        return false;
    }
    return std::none_of(relative.begin(), relative.end(),
                        [](const auto& component) { return component == ".."; });
}

std::string sourceKey(std::string prefix, const std::filesystem::path& relative) {
    std::string key = std::move(prefix) + openbus::pathToUtf8(relative);
    std::replace(key.begin(), key.end(), '\\', '/');
#ifdef _WIN32
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char ch) {
        return static_cast<char>(ch >= 'A' && ch <= 'Z' ? ch + ('a' - 'A') : ch);
    });
#endif
    return key;
}

} // namespace

MapBakeManifest buildMapBakeManifest(std::string mapKey,
                                     std::map<std::string, std::string> settings,
                                     const std::vector<MapBakeSourceFile>& sourceFiles) {
    if (sourceFiles.size() > kMaximumRecordCount) {
        throw std::length_error("Map bake manifest has too many source files");
    }
    MapBakeManifest manifest;
    manifest.mapKey = std::move(mapKey);
    manifest.settings = std::move(settings);
    std::set<std::string> keys;
    for (const MapBakeSourceFile& source : sourceFiles) {
        if (source.key.empty() || source.key.size() > kMaximumStringSize ||
            source.key.find('\0') != std::string::npos || !keys.insert(source.key).second) {
            throw std::invalid_argument("Invalid or duplicate map bake source key");
        }
        if (!std::filesystem::is_regular_file(source.path)) {
            throw std::runtime_error("Map bake source is not a regular file: " +
                                     source.path.string());
        }
        const std::uint64_t size = std::filesystem::file_size(source.path);
        manifest.dependencies.push_back({source.key, size, hashFile(source.path, size)});
    }
    std::sort(manifest.dependencies.begin(), manifest.dependencies.end(),
              [](const auto& left, const auto& right) { return left.key < right.key; });
    validateManifest(manifest);
    return manifest;
}

std::vector<MapBakeSourceFile>
makeMapBakeSourceFiles(const std::vector<std::filesystem::path>& sourcePaths,
                       const std::filesystem::path& mapRoot,
                       const std::filesystem::path& omsiRoot) {
    if (!std::filesystem::is_directory(mapRoot) || !std::filesystem::is_directory(omsiRoot)) {
        throw std::invalid_argument("Map bake source roots must be existing directories");
    }
    const auto canonicalMapRoot = std::filesystem::weakly_canonical(mapRoot);
    const auto canonicalOmsiRoot = std::filesystem::weakly_canonical(omsiRoot);
    std::map<std::string, std::filesystem::path> keyedPaths;
    for (const auto& source : sourcePaths) {
        if (!std::filesystem::is_regular_file(source)) {
            throw std::runtime_error("Map bake source is not a regular file: " +
                                     openbus::pathToUtf8(source));
        }
        const auto canonicalSource = std::filesystem::weakly_canonical(source);
        std::filesystem::path relative;
        std::string key;
        if (isWithin(canonicalSource, canonicalMapRoot, relative)) {
            key = sourceKey("map/", relative);
        } else if (isWithin(canonicalSource, canonicalOmsiRoot, relative)) {
            key = sourceKey("omsi/", relative);
        } else {
            throw std::invalid_argument("Map bake source is outside the configured roots: " +
                                        openbus::pathToUtf8(source));
        }
        const auto [existing, inserted] = keyedPaths.emplace(key, canonicalSource);
        if (!inserted) {
            std::error_code error;
            if (!std::filesystem::equivalent(existing->second, canonicalSource, error) || error) {
                throw std::invalid_argument("Map bake source keys collide: " + key);
            }
        }
    }
    std::vector<MapBakeSourceFile> sourceFiles;
    sourceFiles.reserve(keyedPaths.size());
    for (const auto& [key, path] : keyedPaths) {
        sourceFiles.push_back({key, path});
    }
    return sourceFiles;
}

std::vector<std::byte> serializeMapBakeManifest(const MapBakeManifest& manifest) {
    validateManifest(manifest);
    std::uint64_t encodedSize = 16;
    const auto addSize = [&encodedSize](std::uint64_t size) {
        if (size > kMaximumManifestSize - encodedSize) {
            throw std::length_error("Map bake manifest exceeds supported size");
        }
        encodedSize += size;
    };
    addSize(4ULL + manifest.mapKey.size());
    for (const auto& [key, value] : manifest.settings) {
        addSize(8ULL + key.size() + value.size());
    }
    for (const MapBakeDependency& dependency : manifest.dependencies) {
        addSize(44ULL + dependency.key.size());
    }
    std::vector<std::byte> payload(kManifestMagic.begin(), kManifestMagic.end());
    appendU32(payload, kManifestVersion);
    appendString(payload, manifest.mapKey);
    appendU32(payload, static_cast<std::uint32_t>(manifest.settings.size()));
    appendU32(payload, static_cast<std::uint32_t>(manifest.dependencies.size()));
    for (const auto& [key, value] : manifest.settings) {
        appendString(payload, key);
        if (value.size() > kMaximumStringSize || value.find('\0') != std::string::npos) {
            throw std::invalid_argument("Invalid map bake manifest setting value");
        }
        appendU32(payload, static_cast<std::uint32_t>(value.size()));
        const auto* bytes = reinterpret_cast<const std::byte*>(value.data());
        payload.insert(payload.end(), bytes, bytes + value.size());
    }
    for (const MapBakeDependency& dependency : manifest.dependencies) {
        appendString(payload, dependency.key);
        appendU64(payload, dependency.size);
        payload.insert(payload.end(), dependency.sha256.begin(), dependency.sha256.end());
    }
    if (payload.size() > kMaximumManifestSize) {
        throw std::length_error("Map bake manifest exceeds supported size");
    }
    return payload;
}

MapBakeManifest deserializeMapBakeManifest(std::span<const std::byte> payload) {
    if (payload.size() > kMaximumManifestSize || payload.size() < 16 ||
        !std::equal(kManifestMagic.begin(), kManifestMagic.end(), payload.begin())) {
        throw std::runtime_error("Invalid map bake manifest payload");
    }
    std::size_t cursor = kManifestMagic.size();
    const std::uint32_t version = readU32(payload, cursor);
    if (version != kManifestVersion) {
        throw std::runtime_error("Unsupported map bake manifest version");
    }
    MapBakeManifest manifest;
    manifest.schemaVersion = version;
    manifest.mapKey = readString(payload, cursor);
    const std::uint32_t settingCount = readU32(payload, cursor);
    const std::uint32_t dependencyCount = readU32(payload, cursor);
    if (settingCount > kMaximumRecordCount || dependencyCount > kMaximumRecordCount) {
        throw std::runtime_error("Map bake manifest record count exceeds supported size");
    }
    for (std::uint32_t setting = 0; setting < settingCount; ++setting) {
        std::string key = readString(payload, cursor);
        const std::uint32_t valueSize = readU32(payload, cursor);
        if (valueSize > kMaximumStringSize || cursor > payload.size() ||
            payload.size() - cursor < valueSize) {
            throw std::runtime_error("Invalid map bake manifest setting value");
        }
        std::string value(reinterpret_cast<const char*>(payload.data() + cursor), valueSize);
        cursor += valueSize;
        if (value.find('\0') != std::string::npos ||
            !manifest.settings.emplace(std::move(key), std::move(value)).second) {
            throw std::runtime_error("Duplicate or invalid map bake manifest setting");
        }
    }
    manifest.dependencies.reserve(dependencyCount);
    for (std::uint32_t dependency = 0; dependency < dependencyCount; ++dependency) {
        MapBakeDependency entry;
        entry.key = readString(payload, cursor);
        entry.size = readU64(payload, cursor);
        if (cursor > payload.size() || payload.size() - cursor < entry.sha256.size()) {
            throw std::runtime_error("Truncated map bake manifest digest");
        }
        std::copy_n(payload.begin() + static_cast<std::ptrdiff_t>(cursor), entry.sha256.size(),
                    entry.sha256.begin());
        cursor += entry.sha256.size();
        manifest.dependencies.push_back(std::move(entry));
    }
    if (cursor != payload.size()) {
        throw std::runtime_error("Unexpected trailing bytes in map bake manifest");
    }
    try {
        validateManifest(manifest);
    } catch (const std::invalid_argument& error) {
        throw std::runtime_error(error.what());
    }
    return manifest;
}

std::vector<std::string> compareMapBakeManifests(const MapBakeManifest& expected,
                                                 const MapBakeManifest& current) {
    validateManifest(expected);
    validateManifest(current);
    std::vector<std::string> differences;
    if (expected.schemaVersion != current.schemaVersion) {
        differences.emplace_back("Manifest schema changed");
    }
    if (expected.mapKey != current.mapKey) {
        differences.emplace_back("Map identity changed");
    }
    for (const auto& [key, value] : expected.settings) {
        const auto found = current.settings.find(key);
        if (found == current.settings.end()) {
            differences.push_back("Setting removed: " + key);
        } else if (found->second != value) {
            differences.push_back("Setting changed: " + key);
        }
    }
    for (const auto& setting : current.settings) {
        if (!expected.settings.contains(setting.first)) {
            differences.push_back("Setting added: " + setting.first);
        }
    }

    std::map<std::string, const MapBakeDependency*> expectedDependencies;
    std::map<std::string, const MapBakeDependency*> currentDependencies;
    for (const MapBakeDependency& dependency : expected.dependencies) {
        expectedDependencies.emplace(dependency.key, &dependency);
    }
    for (const MapBakeDependency& dependency : current.dependencies) {
        currentDependencies.emplace(dependency.key, &dependency);
    }
    for (const auto& [key, expectedDependency] : expectedDependencies) {
        const auto found = currentDependencies.find(key);
        if (found == currentDependencies.end()) {
            differences.push_back("Source removed: " + key);
        } else if (found->second->size != expectedDependency->size ||
                   found->second->sha256 != expectedDependency->sha256) {
            differences.push_back("Source changed: " + key);
        }
    }
    for (const auto& [key, dependency] : currentDependencies) {
        if (!expectedDependencies.contains(key)) {
            differences.push_back("Source added: " + key);
        }
        (void)dependency;
    }
    return differences;
}

std::vector<std::string> compareMapBakeArchiveManifest(MapBakeArchiveReader& archive,
                                                       const MapBakeManifest& current) {
    validateManifest(current);
    const auto manifestChunk = archive.find(MapBakeChunkType::Manifest, "map");
    if (!manifestChunk) {
        return {"Manifest chunk missing"};
    }
    const MapBakeManifest baked = deserializeMapBakeManifest(archive.readChunk(*manifestChunk));
    return compareMapBakeManifests(baked, current);
}

} // namespace openbus::map
