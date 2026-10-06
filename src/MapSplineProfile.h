#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace openbus::map {

struct MapSplineProfilePoint {
    double lateral = 0.0;
    double height = 0.0;
    double textureU = 0.0;
    double textureVPerMeter = 0.0;
};

struct MapSplineProfileSection {
    int textureIndex = -1;
    std::vector<MapSplineProfilePoint> points;
};

struct MapSplineProfile {
    std::vector<std::string> textures;
    std::vector<MapSplineProfileSection> sections;
    bool editorOnly = false;
};

// Reads the text-based StreetCreator/OMSI .sli sections used to define the
// spline cross-section, per-point UV coordinates, and texture references.
MapSplineProfile loadMapSplineProfile(const std::filesystem::path& path);

} // namespace openbus::map
