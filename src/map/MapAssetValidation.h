#pragma once

#include <cstddef>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace openbus::map {

struct MapAssetFinding {
    std::string category;
    std::filesystem::path source;
    std::string reference;
    std::string detail;
    std::filesystem::path context = {};
    bool unsupported = false;
};

struct MapAssetValidationReport {
    std::vector<MapAssetFinding> findings;
    // Files reached by the current validation pass; this is not yet a complete bake dependency
    // set.
    std::vector<std::filesystem::path> sourceFiles;
    std::size_t tilesScanned = 0;
    // This first-stage scan does not yet cover all transitive or chrono dependencies.
    bool complete = false;
    std::size_t chronoTilesScanned = 0;
    std::size_t chronoCombinedTilesScanned = 0;
    std::size_t chronoTimetableDatesScanned = 0;
};

struct MapAssetValidationProgress {
    std::string phase;
    std::size_t completed = 0;
    std::size_t total = 0;
};

struct MapAssetValidationOptions {
    std::function<void(const MapAssetValidationProgress&)> onProgress;
};

MapAssetValidationReport validateMapAssets(const std::filesystem::path& mapDirectory,
                                           const std::filesystem::path& omsiRoot);
MapAssetValidationReport validateMapAssets(const std::filesystem::path& mapDirectory,
                                           const std::filesystem::path& omsiRoot,
                                           const MapAssetValidationOptions& options);

} // namespace openbus::map
