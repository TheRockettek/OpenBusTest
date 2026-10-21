#pragma once

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace openbus::map {

inline constexpr int MAP_SPLINE_PATH_TYPE_ROAD = 0;
inline constexpr int MAP_SPLINE_PATH_TYPE_HUMAN = 1;

struct MapSplineProfilePoint {
    double lateral = 0.0;
    double height = 0.0;
    double textureU = 0.0;
    double textureVPerMeter = 0.0;
};

struct MapSplineProfileSection {
    int textureIndex = -1;
    bool scaleTextureByLength = false;
    std::vector<MapSplineProfilePoint> points;
};

struct MapSplineHeightProfile {
    // Source fields are retained in order; valid lateral extents can bound cant geometry.
    std::array<std::string, 4> rawFields;
    std::array<double, 4> values = {};
    bool valid = false;
};

struct MapSplinePathDefinition {
    // Path-field meaning is not fully established; retain source text alongside numeric values.
    std::vector<std::string> rawFields;
    std::vector<double> numericFields;
    bool numericFieldsValid = false;
    std::optional<int> pathTypeCode;
};

struct MapSplineProfile {
    std::vector<std::string> textures;
    std::vector<int> textureAlphaModes;
    std::vector<MapSplineProfileSection> sections;
    std::vector<MapSplineHeightProfile> heightProfiles;
    std::vector<MapSplinePathDefinition> paths;
    std::optional<double> explicitCantHalfWidth;
    bool editorOnly = false;
    // Preserve the spline for rendering; omit its profile strips from map collision.
    bool noCollision = false;
};

// Reads text-based StreetCreator/OMSI .sli cross-sections, UVs, height profiles,
// path records, texture references, [scaleTexByLength], [halfcantwidth], and [nocollision].
MapSplineProfile loadMapSplineProfile(const std::filesystem::path& path);

// Returns [halfcantwidth] when valid, otherwise the maximum absolute x extent
// across valid [heightprofile] records. A missing result means no usable width
// was authored.
std::optional<double> mapSplineProfileCantHalfWidth(const MapSplineProfile& profile) noexcept;

bool mapSplineProfileHasPathType(const MapSplineProfile& profile, int pathTypeCode) noexcept;

} // namespace openbus::map
