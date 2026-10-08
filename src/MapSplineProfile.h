#pragma once

#include <array>
#include <filesystem>
#include <optional>
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

struct MapSplineHeightProfile {
    // Source fields are retained in order; valid lateral extents can bound cant geometry.
    std::array<std::string, 4> rawFields;
    std::array<double, 4> values = {};
    bool valid = false;
};

struct MapSplineProfile {
    std::vector<std::string> textures;
    std::vector<int> textureAlphaModes;
    std::vector<MapSplineProfileSection> sections;
    std::vector<MapSplineHeightProfile> heightProfiles;
    bool editorOnly = false;
};

// Reads the text-based StreetCreator/OMSI .sli sections used to define the
// spline cross-section, per-point UV coordinates, height-profile records, and texture references.
MapSplineProfile loadMapSplineProfile(const std::filesystem::path& path);

// Returns the maximum absolute x extent across valid [heightprofile] records.
// A missing result means no usable lateral width was authored.
std::optional<double> mapSplineProfileCantHalfWidth(const MapSplineProfile& profile) noexcept;

} // namespace openbus::map
