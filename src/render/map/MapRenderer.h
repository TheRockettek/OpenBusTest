#pragma once

#include "map/MapConfigLoader.h"
#include "map/MapSplineProfile.h"
#include "map/MapTrafficRuntime.h"
#include "render/HumanSkinning.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

class SimulationState;
struct ModelConfig;

namespace openbus::rendering {

class MapSceneLoader;
class AssetRequestManager;
struct HumanGaitParameters;

struct MapRenderStats {
    std::size_t activeTiles = 0;
    std::size_t totalTiles = 0;
    std::size_t activeTerrainTiles = 0;
    std::size_t totalTerrainTiles = 0;
    std::size_t activeSceneryObjects = 0;
    std::size_t totalSceneryObjects = 0;
    std::size_t activeRoadBatches = 0;
    std::size_t totalRoadBatches = 0;
    std::size_t totalWaterBatches = 0;
    std::size_t totalTextures = 0;
};

struct TrafficSignalDebugStats {
    std::size_t activeSignalModels = 0;
    std::size_t controllerDrivenModels = 0;
    std::size_t modelsWithoutPhase = 0;
    std::array<std::size_t, 13> modelsByPhase = {};
};

struct TrafficSignalHeadDebugInfo {
    std::optional<int> objectId;
    std::optional<int> linkedJunctionObjectId;
    std::array<double, 3> position = {};
    int tileX = 0;
    int tileY = 0;
    std::optional<std::size_t> lightIndex;
    std::optional<int> phase;
};

struct TrafficPathDebugSegment {
    std::vector<std::array<double, 3>> points;
    double pathTypeCode = 0.0;
    double widthMeters = 0.0;
    std::optional<int> mapObjectId;
    std::size_t pathIndex = 0;
    openbus::map::MapTrafficPathRules pathRules;
    std::optional<openbus::map::MapTrafficSignalSchedule> trafficSignal;
    int tileX = 0;
    int tileY = 0;
};

struct EditorPassengerDebugObject {
    int tileX = 0;
    int tileY = 0;
    std::optional<int> objectId;
    std::string name;
    bool busStop = false;
    bool passengerCabin = false;
    std::array<double, 3> position = {};
    std::array<std::array<double, 3>, 8> bounds = {};
    std::vector<openbus::map::MapTrafficPose> passengerPositions;
};

class MapRenderer {
  public:
    MapRenderer(const openbus::map::MapDefinition& map, std::size_t centerTileIndex,
                double spawnWorldX, double spawnWorldY, std::size_t groundTextureIndex,
                const std::filesystem::path& omsiRoot, SimulationState& simulationState,
                AssetRequestManager& assetRequests, bool prepareInitialWindowAsync = false);
    ~MapRenderer();

    MapRenderer(const MapRenderer&) = delete;
    MapRenderer& operator=(const MapRenderer&) = delete;

    void updateScripts();
    void updateStreaming(double worldX, double worldY);
    void enableScripts();
    bool initialWindowReady() const noexcept;
    void updatePoleStates(const std::vector<MapPoleState>& states);
    bool leftHandTraffic() const noexcept;
    double tileSize() const noexcept;
    const std::vector<std::pair<int, int>>& activeTileCoordinates() const;
    std::vector<std::array<double, 3>> activeTileBoundaryVertices(double fallbackHeight) const;
    MapRenderStats renderStats() const;
    std::uint64_t trafficDebugGeometryRevision() const noexcept;
    TrafficSignalDebugStats trafficSignalDebugStats() const;
    std::vector<TrafficSignalHeadDebugInfo>
    trafficSignalHeadDebugInfo(double signalClockSeconds) const;
    std::optional<openbus::map::MapTrafficRoute> buildTrafficRoute(int splineId) const;
    std::optional<openbus::map::MapTrafficRoute> buildTrafficRoute(int splineId,
                                                                   int pathTypeCode) const;
    std::optional<openbus::map::MapTrafficRoute> buildDrivingRoute(
        int splineId, std::size_t laneSelector = 0, std::size_t routeSeed = 0,
        double minimumTurnRadiusMeters = 0.0, openbus::map::MapTrafficVehicleFilter vehicle = {},
        const std::vector<int>& requiredSplineSequence = {},
        const std::vector<std::optional<std::size_t>>& requiredSceneryPathSequence = {},
        bool stopAtRequiredSequenceEnd = false,
        bool allowIntermediarySegmentsBetweenRequired = false,
        bool allowReverseTraversalForRequired = false) const;
    std::vector<openbus::map::MapTrafficLaneStart>
    lanesForTimetableEntry(int splineId, std::optional<std::size_t> sceneryPathIndex) const;
    std::optional<openbus::map::MapTrafficPose>
    timetableStopPose(int splineId, std::size_t pathIndex, double offsetMeters) const;
    std::optional<openbus::map::MapTrafficLaneConnectionGap>
    nearestTrafficLaneConnectionGap(int fromSplineId, std::optional<std::size_t> fromPathIndex,
                                    int toSplineId, std::optional<std::size_t> toPathIndex) const;
    std::vector<openbus::map::MapTrafficLaneStart> trafficEntryLanes() const;
    std::optional<openbus::map::MapTrafficLaneStart>
    selectTrafficEntryLane(openbus::map::MapTrafficVehicleFilter vehicle,
                           std::size_t selectionSeed) const;
    std::vector<openbus::map::MapTrafficLaneStart> trafficSpawnLanes() const;
    std::optional<openbus::map::MapTrafficLaneStart>
    selectTrafficSpawnLane(openbus::map::MapTrafficVehicleFilter vehicle,
                           std::size_t selectionSeed) const;
    std::optional<openbus::map::MapTrafficLaneStart>
    selectTrafficSpawnLaneInTiles(openbus::map::MapTrafficVehicleFilter vehicle,
                                  std::size_t selectionSeed,
                                  const std::vector<std::pair<int, int>>& activeTiles) const;
    std::vector<int> trafficSplineIds(int pathTypeCode) const;
    std::vector<int> trafficSplineIdsInTiles(int pathTypeCode,
                                             const std::vector<std::pair<int, int>>& tiles) const;
    std::vector<int> trafficRouteStartSplineIds(int pathTypeCode) const;
    std::optional<openbus::map::MapTrafficSplineEndpointStatus>
    trafficSplineEndpointStatus(int splineId, int pathTypeCode) const;
    std::vector<openbus::map::MapSplinePathOffset> trafficSplinePathOffsets(int splineId,
                                                                            int pathTypeCode) const;
    const std::vector<TrafficPathDebugSegment>& sceneryTrafficPathSegments() const;
    const std::vector<EditorPassengerDebugObject>& editorPassengerDebugObjects() const;
    std::optional<std::array<double, 3>> editorOnlyBusStopPosition(int stopId) const;
    std::optional<openbus::map::MapTrafficActor> createTrafficActor(
        int splineId, std::string modelPath, double speedMetersPerSecond,
        double laneOffsetMagnitudeMeters, std::optional<std::string> driverModelPath = std::nullopt,
        std::optional<int> pathTypeCode = std::nullopt, std::size_t laneSelector = 0) const;
    void loadHumanTrafficModel(const std::filesystem::path& humanConfigPath);
    void drawHumanTrafficActor(const std::filesystem::path& humanConfigPath,
                               const openbus::map::MapTrafficPose& pose, double gaitPhaseRadians,
                               double gaitWeight) const;
    std::vector<std::shared_ptr<openbus::map::MapTrafficSignalController>>
    trafficSignalControllers() const;
    std::vector<openbus::map::MapTrafficSignalRequest>
    trafficSignalRequests(std::optional<std::array<double, 2>> playerBusPosition,
                          double playerSpeedMetersPerSecond) const;
    void updateTrafficSignalScripts(double signalClockSeconds) const;
    bool addTrafficActor(openbus::map::MapTrafficActor actor);
    std::size_t updateTrafficActors(double elapsedSeconds);
    const std::vector<openbus::map::MapTrafficActor>& trafficActors() const;
    std::optional<double> terrainHeightAt(double worldX, double worldY) const;
    void draw(bool renderTypeFourOnly = false, double minimumObjectViewportFraction = 0.0) const;

  private:
    friend class MapSceneLoader;

    void initializeActiveScripts();
    bool finishHumanTrafficModelLoad(const std::string& cacheKey) const;
    void loadHumanTrafficModelNow(const std::filesystem::path& humanConfigPath,
                                  const std::filesystem::path& modelConfigPath,
                                  const ModelConfig& configuration, const HumanGaitParameters& gait,
                                  const std::optional<HumanJoints>& joints) const;

    openbus::map::MapDefinition mapDefinition_;
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::unique_ptr<MapSceneLoader> sceneLoader_;
    AssetRequestManager* assetRequests_ = nullptr;
    bool scriptsEnabled_ = true;
    bool loadSummaryLogged_ = false;
    bool asyncInitialLoad_ = false;
};

} // namespace openbus::rendering