#include "map/MapBakeManifest.h"
#include "map/MapBakeArchive.h"
#include "ProbeTempDirectory.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
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

void requireThrows(const auto& callback, const char* message) {
    try {
        callback();
    } catch (const std::exception&) {
        return;
    }
    throw std::runtime_error(message);
}

void write(const std::filesystem::path& path, const std::string& value) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!(output << value)) {
        throw std::runtime_error("could not write manifest fixture");
    }
}

std::string toHex(const std::array<std::byte, 32>& digest) {
    std::ostringstream result;
    result << std::hex << std::setfill('0');
    for (const std::byte value : digest) {
        result << std::setw(2) << std::to_integer<unsigned int>(value);
    }
    return result.str();
}

void run() {
    const auto root = openbus::probe::makeProbeDirectory("openbus-map-bake-manifest", false);
    try {
        const auto mapConfig = root / "Map" / "global.cfg";
        const auto splineProfile = root / "OMSI" / "Splines" / "road.sli";
        write(mapConfig, "abc");
        write(splineProfile, "[texture]\nroad.bmp\n");

        const std::vector<openbus::map::MapBakeSourceFile> sourceFiles{
            {"omsi/Splines/road.sli", splineProfile},
            {"map/global.cfg", mapConfig}};
        const std::map<std::string, std::string> settings{
            {"format", "static-v1"}, {"map-parser", "14"}};
        const auto manifest =
            openbus::map::buildMapBakeManifest("Fixture", settings, sourceFiles);
        require(manifest.dependencies.size() == 2 &&
                    manifest.dependencies.front().key == "map/global.cfg" &&
                    toHex(manifest.dependencies.front().sha256) ==
                        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
                "source dependencies were not sorted or SHA-256 fingerprinted correctly");

        const auto payload = openbus::map::serializeMapBakeManifest(manifest);
        const auto decoded = openbus::map::deserializeMapBakeManifest(payload);
        require(decoded == manifest, "manifest serialization round trip failed");
        std::stringstream manifestArchive(std::ios::in | std::ios::out | std::ios::binary);
        openbus::map::writeMapBakeArchive(
            manifestArchive,
            {{openbus::map::MapBakeChunkType::Manifest, "map", 0, 0, payload}});
        const std::string encodedManifestArchive = manifestArchive.str();
        manifestArchive.seekg(0);
        openbus::map::MapBakeArchiveReader manifestArchiveReader(manifestArchive);
        const std::vector<openbus::map::MapBakeSourceFile> reversedSourceFiles(
            sourceFiles.rbegin(), sourceFiles.rend());
        const auto unchanged =
            openbus::map::buildMapBakeManifest("Fixture", settings, reversedSourceFiles);
        require(openbus::map::compareMapBakeManifests(manifest, unchanged).empty(),
                "source ordering changed the manifest fingerprint");
        require(openbus::map::compareMapBakeArchiveManifest(manifestArchiveReader, unchanged)
                    .empty(),
                "archived manifest did not match unchanged current sources");

        const auto mappedSources = openbus::map::makeMapBakeSourceFiles(
            {splineProfile, mapConfig, mapConfig}, root / "Map", root / "OMSI");
        const auto mappedKey = [&](std::size_t index) {
            std::string key = mappedSources[index].key;
            std::transform(key.begin(), key.end(), key.begin(),
                           [](unsigned char ch) {
                               return static_cast<char>(ch >= 'A' && ch <= 'Z'
                                                            ? ch + ('a' - 'A')
                                                            : ch);
                           });
            return key;
        };
        require(mappedSources.size() == 2 &&
                    mappedKey(0) == "map/global.cfg" &&
                    mappedKey(1) == "omsi/splines/road.sli",
                "source paths were not mapped to stable, sorted logical keys");

        write(splineProfile, "[texture]\nchanged.bmp\n");
        write(root / "Map" / "tile_0_0.map", "[version]\n14\n");
        const auto changed = openbus::map::buildMapBakeManifest(
            "Fixture", {{"format", "static-v2"}}, {sourceFiles[1], sourceFiles[0],
                                                   {"map/tile_0_0.map",
                                                    root / "Map" / "tile_0_0.map"}});
        const auto differences =
            openbus::map::compareMapBakeManifests(manifest, changed);
        require(std::find(differences.begin(), differences.end(),
                          "Source changed: omsi/Splines/road.sli") != differences.end() &&
                    std::find(differences.begin(), differences.end(),
                              "Source added: map/tile_0_0.map") != differences.end() &&
                    std::find(differences.begin(), differences.end(),
                              "Setting changed: format") != differences.end(),
                "manifest comparison did not identify stale sources and settings");
        const auto archiveDifferences =
            openbus::map::compareMapBakeArchiveManifest(manifestArchiveReader, changed);
        require(archiveDifferences == differences,
                "archive manifest comparison did not report stale sources and settings");

        std::stringstream emptyArchive(std::ios::in | std::ios::out | std::ios::binary);
        openbus::map::writeMapBakeArchive(emptyArchive, {});
        emptyArchive.seekg(0);
        openbus::map::MapBakeArchiveReader emptyArchiveReader(emptyArchive);
        require(openbus::map::compareMapBakeArchiveManifest(emptyArchiveReader, unchanged) ==
                    std::vector<std::string>{"Manifest chunk missing"},
                "archive without a manifest was not rejected as stale");

        std::string corruptedArchive = encodedManifestArchive;
        const std::size_t manifestOffset = static_cast<std::size_t>(
            manifestArchiveReader.chunks().front().payloadOffset);
        corruptedArchive.at(manifestOffset) ^= 1;
        std::istringstream corruptedInput(corruptedArchive,
                                          std::ios::in | std::ios::binary);
        openbus::map::MapBakeArchiveReader corruptedReader(corruptedInput);
        requireThrows(
            [&] {
                (void)openbus::map::compareMapBakeArchiveManifest(corruptedReader, unchanged);
            },
            "corrupt archived manifest was accepted");

        const auto removed =
            openbus::map::buildMapBakeManifest("Fixture", settings, {sourceFiles[0]});
        const auto removedDifferences =
            openbus::map::compareMapBakeManifests(manifest, removed);
        require(std::find(removedDifferences.begin(), removedDifferences.end(),
                          "Source removed: map/global.cfg") != removedDifferences.end(),
                "manifest comparison did not identify a removed source");

        requireThrows(
            [&] {
                (void)openbus::map::buildMapBakeManifest(
                    "Fixture", {}, {{"missing.cfg", root / "Map" / "missing.cfg"}});
            },
            "missing source dependency was silently accepted");
        requireThrows(
            [&] {
                (void)openbus::map::buildMapBakeManifest(
                    "Fixture", {}, {{ "duplicate", mapConfig }, {"duplicate", splineProfile}});
            },
            "duplicate source keys were silently accepted");
        write(root / "outside.cfg", "outside");
        requireThrows(
            [&] {
                (void)openbus::map::makeMapBakeSourceFiles(
                    {root / "outside.cfg"}, root / "Map", root / "OMSI");
            },
            "source outside the configured roots was silently accepted");

        auto unsupported = payload;
        unsupported[4] = std::byte{2};
        requireThrows([&] { (void)openbus::map::deserializeMapBakeManifest(unsupported); },
                      "unsupported manifest version was accepted");
        auto truncated = payload;
        truncated.pop_back();
        requireThrows([&] { (void)openbus::map::deserializeMapBakeManifest(truncated); },
                      "truncated manifest was accepted");
    } catch (...) {
        openbus::probe::removeProbeDirectory(root);
        throw;
    }
    openbus::probe::removeProbeDirectory(root);
}

} // namespace

int main() {
    try {
        run();
    } catch (const std::exception& error) {
        std::cerr << "map bake manifest probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
