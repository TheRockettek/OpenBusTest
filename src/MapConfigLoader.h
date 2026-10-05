#pragma once

#include "BusTypes.h"
#include "ConfigurationTypes.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace openbus::map {

inline constexpr double OMSI_TILE_SIZE_METERS = 300.0;

struct GroundTextureDefinition {
    std::string texturePath;
    std::string detailTexturePath;
    std::array<std::string, 3> parameters;
};

struct MapTileReference {
    int x = 0;
    int y = 0;
    std::filesystem::path textPath;
    std::filesystem::path terrainPath;
    bool hasTerrainFile = false;
};

struct MapEntryPoint {
    std::array<std::string, 12> rawFields;
    int objectOnTileIndex = 0;
    int id = 0;
    int tileIndex = 0;
    std::string name;
    VehiclePlacement placement{};
};

struct MapSceneryPlacement {
    std::string label;
    std::string line1;
    std::string assetPath;
    int id = 0;
    std::array<std::string, 6> rawTransformFields;
    std::array<double, 3> localPosition = {};
    std::array<double, 3> rotationDegrees = {};
    bool transformValid = false;
    std::string trailingField;
};

struct MapSplinePlacement {
    std::array<std::string, 19> rawFields;
    std::string assetPath;
    double localX = 0.0;
    double elevation = 0.0;
    double localY = 0.0;
    double rotationDegrees = 0.0;
    double length = 0.0;
    double radius = 0.0;
    double gradientStart = 0.0;
    double gradientEnd = 0.0;
    bool geometryValid = false;
};

struct MapDefinition {
    std::filesystem::path rootPath;
    std::vector<GroundTextureDefinition> groundTextures;
    std::vector<MapEntryPoint> entryPoints;
    std::vector<MapTileReference> tiles;
    ConfigurationDiagnostics diagnostics;
};

struct MapTileData {
    MapTileReference reference;
    std::string version;
    bool hasTerrainMarker = false;
    std::size_t splineCount = 0;
    std::size_t elevatedSplineCount = 0;
    std::vector<MapSplinePlacement> splines;
    std::size_t objectCount = 0;
    std::vector<MapSceneryPlacement> sceneryObjects;
    std::size_t attachedObjectCount = 0;
    std::size_t splineAttachmentCount = 0;
    std::size_t splineRepeaterCount = 0;
};

struct TerrainGrid {
    // OMSI stores the number of intervals; each axis therefore has one extra
    // vertex (Grande Porto: 60 intervals, 61 x 61 float32 heights).
    std::size_t intervals = 0;
    std::vector<float> heights;
};

MapDefinition loadMapDefinition(const std::filesystem::path& mapDirectory);
MapTileData loadMapTile(const MapTileReference& tile);
TerrainGrid loadTerrainGrid(const std::filesystem::path& terrainPath);

} // namespace openbus::map
