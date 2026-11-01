#pragma once

#include "core/BusTypes.h"
#include "config/ConfigurationTypes.h"
#include "map/MapCoordinates.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace openbus::map {

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

struct MapTicketDefinition {
    std::string name;
    std::string displayValue;
    double value = 0.0;
    int type = 0;
};

struct MapTicketPackDefinition {
    std::vector<MapTicketDefinition> tickets;
    ConfigurationDiagnostics diagnostics;
};

struct MapCurrencyCoinDefinition {
    std::string modelPath;
    double value = 0.0;
};

struct MapCurrencyDefinition {
    std::string name;
    std::string mode;
    std::vector<MapCurrencyCoinDefinition> coins;
    ConfigurationDiagnostics diagnostics;
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
                                           std::optional<double> density);

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

struct MapAiTimetableProfile {
    std::string name;
    double durationMinutes = 0.0;
    std::vector<std::pair<int, double>> manualArrivalMinutes;
    std::vector<std::pair<int, double>> manualDepartureMinutes;
    std::vector<double> arrivalOffsetSeconds;
    std::vector<double> departureOffsetSeconds;
};

struct MapAiTimetableRoute {
    // The .ttp filename stem referenced by [addtrip] in a .ttl file.
    std::string timetableKey;
    std::string tripName;
    std::string destinationName;
    std::string lineName;
    std::size_t stationCount = 0;
    struct Station {
        int id = 0;
        int trackEntryIndex = 0;
        std::string name;
        int tileIndex = 0;
        // The four trailing [station] positions are retained verbatim; the
        // third is the station offset within its associated TTR track entry.
        std::array<std::string, 4> opaqueFields;
        std::optional<double> positionOnPreviousTrackEntryMeters;
        bool operator==(const Station&) const = default;
    };
    std::vector<Station> stations;
    // OMSI bus trips may list only stop IDs and use StnLinks rather than a .ttr.
    std::vector<int> stationIds;
    std::vector<int> splineIds;
    struct TrackEntry {
        int splineId = 0;
        int pathIndex = 0;
        int tileIndex = 0;
        double lengthMeters = 0.0;
        bool operator==(const TrackEntry&) const = default;
    };
    std::vector<TrackEntry> trackEntries;
    // Parsed schedule values are minutes after midnight; dispatch is handled separately.
    std::vector<double> scheduledDepartureMinutes;
    // Profile index from the corresponding .ttl [addtrip], kept aligned with departures.
    std::vector<int> scheduledDepartureProfileIndices;
    std::vector<MapAiTimetableProfile> profiles;
};

struct MapAiTimetableStationPosition {
    std::size_t trackEntryIndex = 0;
    double offsetMeters = 0.0;
};

std::optional<MapAiTimetableStationPosition>
mapAiTimetableStationPosition(const MapAiTimetableRoute& route,
                              const MapAiTimetableRoute::Station& station);

struct MapAiBusStop {
    std::string name;
    int tileIndex = 0;
    int id = 0;
    std::array<std::string, 4> remainingFields;
};

struct MapAiStationLink {
    struct Entry {
        int splineId = 0;
        int pathIndex = 0;
        int tileIndex = 0;
        double lengthMeters = 0.0;
        std::array<std::string, 3> remainingFields;
        bool operator==(const Entry&) const = default;
    };
    std::array<std::string, 9> fields;
    int fromStopId = 0;
    int toStopId = 0;
    std::vector<Entry> entries;
};

struct MapAiPassengerTimetable {
    std::string timetableKey;
    std::string tripName;
    std::string destinationName;
    std::string lineName;
    std::vector<int> stationIds;
    std::vector<std::optional<MapAiTimetableRoute::Station>> stationRecords;
    std::vector<double> scheduledDepartureMinutes;
    std::vector<int> scheduledDepartureProfileIndices;
    std::vector<MapAiTimetableProfile> profiles;
};

struct MapAiTimetableCourseTrip {
    std::string timetableKey;
    int profileIndex = 0;
    double departureMinutes = 0.0;
    bool operator==(const MapAiTimetableCourseTrip&) const = default;
};

struct MapAiTimetableCourse {
    std::string name;
    std::vector<MapAiTimetableCourseTrip> trips;
};

std::optional<std::size_t>
mapAiTimetableNextCourseTripIndex(const MapAiTimetableCourse& course, std::size_t currentTripIndex,
                                  std::span<const std::string> availableTimetableKeys);

struct MapAiPassengerStopCall {
    int stationId = 0;
    std::optional<std::size_t> busStopIndex;
    std::optional<std::size_t> outgoingStationLinkIndex;
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
    std::vector<MapAiTimetableCourse> timetableCourses;
    std::vector<MapAiBusStop> busStops;
    std::vector<MapAiStationLink> stationLinks;
    // Contains station-based bus trips, including trips without a driving .ttr.
    std::vector<MapAiPassengerTimetable> passengerTimetables;
    // Human models come from humans.txt; driver models are for AI buses.
    // These paths are relative to the OMSI installation root.
    std::vector<std::string> humanModelPaths;
    std::vector<std::string> driverModelPaths;
};

struct MapAiRoadVehicle {
    std::string assetPath;
    std::size_t groupIndex = 0;
    // Index from unsched_vehgroups.txt, also used by path traffic rules.
    std::size_t trafficRuleGroupIndex = 0;
    int defaultVehicleType = 0;
    std::string groupName;
    double selectionWeight = 1.0;
};

struct MapAiRoadTrafficGroupPopulation {
    std::size_t groupIndex = 0;
    std::size_t populationCount = 0;
};

std::vector<MapAiPassengerStopCall>
mapAiPassengerStopCalls(const MapAiTrafficData& traffic, const MapAiPassengerTimetable& timetable);
bool mapAiPassengerTimetableHasResolvedLinks(const MapAiTrafficData& traffic,
                                             const MapAiPassengerTimetable& timetable);
std::vector<MapAiRoadVehicle> mapAiRoadVehicles(const MapAiTrafficData& traffic);
std::vector<MapAiRoadVehicle> mapAiRoadVehicles(const MapAiTrafficData& traffic, double hour,
                                                int dayOfWeekMask);
std::vector<MapAiRoadVehicle> mapAiRoadVehiclesForPool(const MapAiTrafficData& traffic);
std::optional<std::size_t> selectMapAiRoadVehicle(const std::vector<MapAiRoadVehicle>& vehicles,
                                                  std::size_t selectionSeed);
std::vector<MapAiRoadTrafficGroupPopulation>
mapAiRoadTrafficPopulationByGroup(const MapAiTrafficData& traffic, std::size_t populationCount,
                                  double hour, int dayOfWeekMask);
std::vector<MapAiRoadTrafficGroupPopulation>
mapAiMaximumRoadTrafficPoolByGroup(const MapAiTrafficData& traffic,
                                   const std::vector<MapAiRoadVehicle>& vehicles,
                                   std::size_t basePopulation, double densityMultiplier);
std::optional<double> mapAiRoadTrafficDensityAtHour(const MapAiTrafficData& traffic,
                                                    const std::vector<MapAiRoadVehicle>& vehicles,
                                                    double hour, int dayOfWeekMask);
std::optional<double> mapAiMaximumRoadTrafficDensity(const MapAiTrafficData& traffic,
                                                     const std::vector<MapAiRoadVehicle>& vehicles);
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
    MapCoordinates coordinates;
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
std::size_t applyMapChronoTimetables(MapDefinition& map, int dateYyyyMmDd);
MapTicketPackDefinition loadMapTicketPackDefinition(const std::filesystem::path& ticketPackPath);
MapCurrencyDefinition loadMapCurrencyDefinition(const std::filesystem::path& currencyPath);
std::size_t applyMapChronoTiles(MapDefinition& map, int dateYyyyMmDd);
std::size_t selectMapSpawnPoint(const MapDefinition& map, std::string_view selector);
std::size_t selectMapEntryPoint(const MapDefinition& map, std::string_view selector);
std::size_t selectMapGroundTextureIndex(const MapDefinition& map, std::string_view selector);
MapTileData loadMapTile(const MapTileReference& tile);
MapChronoTileData loadChronoTile(const std::filesystem::path& chronoTilePath);
TerrainGrid loadTerrainGrid(const std::filesystem::path& terrainPath);
WaterData loadWaterData(const std::filesystem::path& waterPath);

} // namespace openbus::map
