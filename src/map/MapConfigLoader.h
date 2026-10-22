#pragma once

#include "core/BusTypes.h"
#include "config/ConfigurationTypes.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace openbus::map {

inline constexpr double OMSI_TILE_SIZE_METERS = 300.0;

struct GroundTextureDefinition {
    std::string texturePath;
    std::string detailTexturePath;
    std::array<std::string, 3> parameters;
};

struct MapGroundTextureTiling {
    double textureRepeatsPerTile = 1.0;
    double detailTextureRepeatsPerTile = 1.0;
};

MapGroundTextureTiling
mapGroundTextureTiling(const GroundTextureDefinition& groundTexture) noexcept;

std::optional<std::size_t>
resolveMapGroundTextureSidecarIndex(std::string_view ordinalSuffix,
                                    std::size_t groundTextureCount) noexcept;

struct MapSeasonDefinition {
    std::string label;
    std::array<std::string, 3> fields;
};

struct MapGlobalMetadata {
    std::string name;
    std::string friendlyName;
    std::string description;
    std::string version;
    std::string nextIdCode;
    std::array<std::string, 6> backgroundImage;
    std::array<std::string, 8> mapCamera;
    std::string moneySystem;
    std::string ticketPack;
    std::string repairTimeMin;
    std::array<std::string, 2> years;
    std::string realYearOffset;
    std::string standardDepot;
    std::vector<MapSeasonDefinition> seasons;
    std::vector<std::array<std::string, 2>> roadTrafficDensity;
    std::vector<std::array<std::string, 2>> passengerTrafficDensity;
    bool hasBackgroundImage = false;
    bool hasMapCamera = false;
    bool hasWorldCoordinates = false;
    bool hasDynamicHelperActive = false;
    bool hasRealRail = false;
    bool hasLeftHandTraffic = false;
};

struct MapTrafficDensityPoint {
    double hour = 0.0;
    double density = 0.0;
};

struct MapAiTrafficDensitySchedule {
    int dayOfWeekMask = 0;
    std::vector<MapTrafficDensityPoint> points;
};

struct MapAiTrafficDensityGroup {
    std::string name;
    double factor = 0.0;
    std::vector<MapAiTrafficDensitySchedule> schedules;
};

std::optional<double> mapTrafficDensityAtHour(const std::vector<MapTrafficDensityPoint>& schedule,
                                              double hour);
std::size_t mapTrafficPopulationForDensity(std::size_t basePopulation,
                                           std::optional<double> density,
                                           std::size_t populationCap);

struct MapAiVehicleEntry {
    std::string assetPath;
    std::string rawLine;
};

struct MapAiVehicleGroup {
    std::string name;
    std::vector<std::string> rawLines;
    std::vector<MapAiVehicleEntry> vehicles;
};

struct MapAiBusTypeDefinition {
    int type = 0;
    std::string assetPath;
    std::vector<std::string> rawLines;
    std::optional<std::size_t> depotIndex;
};

struct MapAiDepotDefinition {
    std::string name;
    std::string hofName;
    std::vector<std::string> rawLines;
    std::vector<std::size_t> busTypeDefinitionIndices;
};

struct MapAiTimetableRoute {
    // The .ttp filename stem referenced by [addtrip] in a .ttl file.
    std::string timetableKey;
    std::string tripName;
    std::string destinationName;
    std::string lineName;
    std::size_t stationCount = 0;
    std::vector<int> splineIds;
    // Parsed schedule values are minutes after midnight; dispatch is handled separately.
    std::vector<double> scheduledDepartureMinutes;
};

struct MapAiTrafficData {
    std::vector<MapTrafficDensityPoint> roadDensity;
    std::vector<MapTrafficDensityPoint> passengerDensity;
    std::vector<MapAiVehicleGroup> vehicleGroups;
    std::vector<std::string> unscheduledVehicleGroupNames;
    bool hasUnscheduledTrafficDensityFile = false;
    std::vector<MapAiTrafficDensityGroup> unscheduledTrafficDensityGroups;
    std::vector<MapAiBusTypeDefinition> busTypes;
    std::vector<MapAiDepotDefinition> depots;
    std::vector<MapAiTimetableRoute> timetableRoutes;
    // These paths are relative to the OMSI installation root.
    std::vector<std::string> driverModelPaths;
};

struct MapAiRoadVehicle {
    std::string assetPath;
    std::size_t groupIndex = 0;
    int defaultVehicleType = 0;
    std::string groupName;
    double selectionWeight = 1.0;
};

struct MapAiRoadTrafficGroupPopulation {
    std::size_t groupIndex = 0;
    std::size_t populationCount = 0;
};

std::vector<MapAiRoadVehicle> mapAiRoadVehicles(const MapAiTrafficData& traffic);
std::vector<MapAiRoadVehicle> mapAiRoadVehicles(const MapAiTrafficData& traffic,
                                                double hour, int dayOfWeekMask);
std::vector<MapAiRoadVehicle> mapAiRoadVehiclesForPool(const MapAiTrafficData& traffic);
std::optional<std::size_t> selectMapAiRoadVehicle(
    const std::vector<MapAiRoadVehicle>& vehicles, std::size_t selectionSeed);
std::vector<MapAiRoadTrafficGroupPopulation> mapAiRoadTrafficPopulationByGroup(
    const MapAiTrafficData& traffic, std::size_t populationCount, double hour,
    int dayOfWeekMask);
std::optional<double> mapAiRoadTrafficDensityAtHour(
    const MapAiTrafficData& traffic, const std::vector<MapAiRoadVehicle>& vehicles,
    double hour, int dayOfWeekMask);
std::optional<double> mapAiMaximumRoadTrafficDensity(
    const MapAiTrafficData& traffic, const std::vector<MapAiRoadVehicle>& vehicles);
std::vector<std::string> mapAiRoadVehicleAssetPaths(const MapAiTrafficData& traffic);
std::vector<std::string> mapAiBusAssetPaths(const MapAiTrafficData& traffic);

struct MapGroundTextureSidecar {
    std::string ordinalSuffix;
    std::filesystem::path path;
};

struct MapOpaqueSection {
    std::string keyword;
    std::size_t sectionLine = 0;
    std::vector<std::string> payloadLines;
};

struct MapTileReference {
    int x = 0;
    int y = 0;
    std::filesystem::path textPath;
    std::filesystem::path terrainPath;
    bool hasTerrainFile = false;
    std::filesystem::path waterPath;
    bool hasWaterFile = false;
    std::filesystem::path lightmapPath;
    bool hasLightmapFile = false;
    std::filesystem::path terrainReadinessPath;
    bool hasTerrainReadinessFile = false;
    std::vector<MapGroundTextureSidecar> groundTextureSidecars;
};

struct MapEntryPoint {
    std::array<std::string, 12> rawFields;
    int objectOnTileIndex = 0;
    int id = 0;
    int tileIndex = 0;
    std::string name;
    VehiclePlacement placement{};
};

struct MapTileRule {
    bool kill = false;
    std::array<std::string, 4> fields;
};

struct MapSceneryPlacement {
    std::string label;
    std::optional<int> editorObjectNumber;
    std::string line1;
    std::string assetPath;
    int id = 0;
    int detailLevel = 0;
    std::array<std::string, 6> rawTransformFields;
    std::array<double, 3> localPosition = {};
    std::array<double, 3> rotationDegrees = {};
    std::vector<std::string> labels;
    std::optional<int> variableParentId;
    bool splineTerrainAlign = false;
    std::vector<MapTileRule> rules;
    bool transformValid = false;
    bool splineAttachment = false;
    std::string trailingField;
};

struct MapSplinePlacement {
    std::array<std::string, 20> rawFields;
    std::string assetPath;
    int splineId = 0;
    int previousSplineId = 0;
    int nextSplineId = 0;
    double localX = 0.0;
    double elevation = 0.0;
    double localY = 0.0;
    double rotationDegrees = 0.0;
    double length = 0.0;
    double radius = 0.0;
    double gradientStart = 0.0;
    double gradientEnd = 0.0;
    // Percentage points, matching the values serialized by the map editor.
    double cantStart = 0.0;
    double cantEnd = 0.0;
    double skewStart = 0.0;
    double skewEnd = 0.0;
    double heightDelta = 0.0;
    double chainOffset = 0.0;
    std::optional<std::string> splineTerrainAlign2;
    std::vector<MapTileRule> rules;
    bool elevated = false;
    bool mirrored = false;
    bool geometryValid = false;
    bool chainOffsetValid = false;
};

struct MapSplineAttachment {
    std::string label;
    std::optional<int> editorObjectNumber;
    std::string assetPath;
    int id = 0;
    int splineIndex = -1;
    std::array<double, 3> offset = {}; // lateral, height, distance along the chain
    std::array<double, 3> rotationDegrees = {};
    double interval = 0.0;
    double range = 0.0;
    bool tilt = false;
    bool repeater = false;
    int repeaterMasterTileIndex = -1;
    std::size_t repeaterFirstObjectIndex = 0;
    std::optional<int> variableParentId;
    bool splineTerrainAlign = false;
    std::vector<MapTileRule> rules;
    bool transformValid = false;
};

struct MapAttachedObject {
    std::string label;
    std::optional<int> editorObjectNumber;
    std::string line1;
    std::string assetPath;
    int id = 0;
    int attachedToObjectId = 0;
    std::string line5;
    std::string attachPointIndex;
    int attachPointIndexValue = 0;
    std::array<std::string, 3> rotationFields;
    std::array<double, 3> rotationDegrees = {};
    std::string labelCount;
    std::vector<std::string> labels;
    std::vector<std::string> optionalLines;
    std::optional<int> variableParentId;
    bool splineTerrainAlign = false;
    std::vector<MapTileRule> rules;
    bool hasExplicitId = false;
    bool transformValid = false;
};

struct MapDefinition {
    std::filesystem::path rootPath;
    MapGlobalMetadata metadata;
    MapAiTrafficData aiTraffic;
    std::vector<GroundTextureDefinition> groundTextures;
    std::vector<MapEntryPoint> entryPoints;
    std::vector<MapTileReference> tiles;
    std::vector<MapOpaqueSection> unsupportedSections;
    ConfigurationDiagnostics diagnostics;
};

struct MapTileData {
    MapTileReference reference;
    std::string version;
    ConfigurationDiagnostics diagnostics;
    std::vector<MapOpaqueSection> unsupportedSections;
    bool hasTerrainMarker = false;
    bool hasWaterMarker = false;
    bool hasVariableTerrainMarker = false;
    bool hasVariableTerrainLightmapMarker = false;
    std::size_t splineCount = 0;
    std::size_t elevatedSplineCount = 0;
    std::vector<MapSplinePlacement> splines;
    std::vector<MapSplineAttachment> splineAttachments;
    std::size_t objectCount = 0;
    std::vector<MapSceneryPlacement> sceneryObjects;
    std::vector<MapAttachedObject> attachedObjects;
    std::size_t splineAttachmentCount = 0;
    std::size_t splineRepeaterCount = 0;
};

struct MapChronoSelector {
    // The value is "selobject" or "selspline"; selector payload semantics remain opaque.
    std::string keyword;
    std::string rawId;
    std::optional<int> id;
    std::size_t sectionLine = 0;
    std::size_t idLine = 0;
    std::vector<std::string> followingLines;
};

struct MapChronoTileData {
    std::string initialComment;
    std::string version;
    std::vector<std::string> rawLines;
    std::vector<MapChronoSelector> selectors;
    ConfigurationDiagnostics diagnostics;
};

struct TerrainGrid {
    // OMSI stores the number of intervals; each axis therefore has one extra
    // vertex (Grande Porto: 60 intervals, 61 x 61 float32 heights).
    std::size_t intervals = 0;
    std::vector<float> heights;
};

struct WaterSurface {
    std::array<float, 4> heights = {};
};

struct WaterData {
    std::vector<WaterSurface> surfaces;
};

MapDefinition loadMapDefinition(const std::filesystem::path& mapDirectory);
std::size_t selectMapSpawnPoint(const MapDefinition& map, std::string_view selector);
std::size_t selectMapEntryPoint(const MapDefinition& map, std::string_view selector);
std::size_t selectMapGroundTextureIndex(const MapDefinition& map, std::string_view selector);
MapTileData loadMapTile(const MapTileReference& tile);
MapChronoTileData loadChronoTile(const std::filesystem::path& chronoTilePath);
TerrainGrid loadTerrainGrid(const std::filesystem::path& terrainPath);
WaterData loadWaterData(const std::filesystem::path& waterPath);

} // namespace openbus::map
