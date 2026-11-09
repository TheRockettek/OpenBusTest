#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <vector>

namespace openbus::map {

class MapBakeArchiveReader;

struct MapBakeSourceFile {
    std::string key;
    std::filesystem::path path;
};

struct MapBakeDependency {
    std::string key;
    std::uint64_t size = 0;
    std::array<std::byte, 32> sha256{};
    bool operator==(const MapBakeDependency&) const = default;
};

struct MapBakeManifest {
    std::uint32_t schemaVersion = 1;
    std::string mapKey;
    std::map<std::string, std::string> settings;
    std::vector<MapBakeDependency> dependencies;
    bool operator==(const MapBakeManifest&) const = default;
};

[[nodiscard]] MapBakeManifest
buildMapBakeManifest(std::string mapKey, std::map<std::string, std::string> settings,
                     const std::vector<MapBakeSourceFile>& sourceFiles);
[[nodiscard]] std::vector<MapBakeSourceFile>
makeMapBakeSourceFiles(const std::vector<std::filesystem::path>& sourcePaths,
                       const std::filesystem::path& mapRoot, const std::filesystem::path& omsiRoot);
[[nodiscard]] std::vector<std::byte> serializeMapBakeManifest(const MapBakeManifest& manifest);
[[nodiscard]] MapBakeManifest deserializeMapBakeManifest(std::span<const std::byte> payload);
[[nodiscard]] std::vector<std::string> compareMapBakeManifests(const MapBakeManifest& expected,
                                                               const MapBakeManifest& current);
[[nodiscard]] std::vector<std::string>
compareMapBakeArchiveManifest(MapBakeArchiveReader& archive, const MapBakeManifest& current);

} // namespace openbus::map
