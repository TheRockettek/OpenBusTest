#pragma once

#include "map/MapConfigLoader.h"
#include "map/MapSplineProfile.h"
#include "map/MapTrafficRuntime.h"

#include <array>
#include <cstddef>
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

struct MapRenderStats {
    std::size_t activeTiles = 0;
    std::size_t totalTiles = 0;
    std::size_t activeTerrainTiles = 0;
    std::size_t totalTerrainTiles = 0;
    std::size_t activeSceneryObjects = 0;
    std::size_t totalSceneryObjects = 0;
    std::size_t activeRoadBatches = 0;
    std::size_t totalRoadBatches = 0;
};

struct TrafficPathDebugSegment {
    std::vector<std::array<double, 3>> points;
    double pathTypeCode = 0.0;
    int tileX = 0;
    int tileY = 0;
};

class MapRenderer {
  public:
    MapRenderer(const openbus::map::MapDefinition& map, std::size_t centerTileIndex,
                double spawnWorldX, double spawnWorldY, std::size_t groundTextureIndex,
                const std::filesystem::path& omsiRoot, SimulationState& simulationState,
                AssetRequestManager& assetRequests);
    ~MapRenderer();

    MapRenderer(const MapRenderer&) = delete;
    MapRenderer& operator=(const MapRenderer&) = delete;

    void updateScripts();
    void updateStreaming(double worldX, double worldY);
    const std::vector<std::pair<int, int>>& activeTileCoordinates() const;
    MapRenderStats renderStats() const;
    std::optional<openbus::map::MapTrafficRoute> buildTrafficRoute(int splineId) const;
    std::optional<openbus::map::MapTrafficRoute> buildTrafficRoute(int splineId,
                                                                   int pathTypeCode) const;
    std::optional<openbus::map::MapTrafficRoute> buildDrivingRoute(
        int splineId, std::size_t laneSelector = 0, std::size_t routeSeed = 0,
        double minimumTurnRadiusMeters = 0.0,
        openbus::map::MapTrafficVehicleFilter vehicle = {},
        const std::vector<int>& requiredSplineSequence = {}) const;
    std::vector<openbus::map::MapTrafficLaneStart> trafficEntryLanes() const;
    std::optional<openbus::map::MapTrafficLaneStart> selectTrafficEntryLane(
        openbus::map::MapTrafficVehicleFilter vehicle, std::size_t selectionSeed) const;
    std::vector<int> trafficSplineIds(int pathTypeCode) const;
    std::vector<int> trafficRouteStartSplineIds(int pathTypeCode) const;
    std::optional<openbus::map::MapTrafficSplineEndpointStatus> trafficSplineEndpointStatus(
        int splineId, int pathTypeCode) const;
    std::vector<openbus::map::MapSplinePathOffset> trafficSplinePathOffsets(
        int splineId, int pathTypeCode) const;
    const std::vector<TrafficPathDebugSegment>& sceneryTrafficPathSegments() const;
    std::optional<openbus::map::MapTrafficActor>
    createTrafficActor(int splineId, std::string modelPath, double speedMetersPerSecond,
                       double laneOffsetMagnitudeMeters,
                       std::optional<std::string> driverModelPath = std::nullopt,
                       std::optional<int> pathTypeCode = std::nullopt,
                       std::size_t laneSelector = 0) const;
    void loadHumanTrafficModel(const std::filesystem::path& humanConfigPath);
    void drawHumanTrafficActor(const std::filesystem::path& humanConfigPath,
                               const openbus::map::MapTrafficPose& pose) const;
    void updateTrafficSignalControllers(
        const std::vector<openbus::map::MapTrafficActor*>& actors,
        double elapsedSeconds, double initialClockSeconds) const;
    bool addTrafficActor(openbus::map::MapTrafficActor actor);
    std::size_t updateTrafficActors(double elapsedSeconds);
    const std::vector<openbus::map::MapTrafficActor>& trafficActors() const;
    std::optional<double> terrainHeightAt(double worldX, double worldY) const;
    void draw(bool renderTypeFourOnly = false) const;

  private:
    friend class MapSceneLoader;

    void initializeActiveScripts();
    bool finishHumanTrafficModelLoad(const std::string& cacheKey) const;
    void loadHumanTrafficModelNow(
        const std::filesystem::path& humanConfigPath,
        const std::filesystem::path& modelConfigPath,
        const ModelConfig& configuration,
        const std::array<double, 2>& geometry) const;

    struct Impl;
    std::unique_ptr<Impl> impl_;
    AssetRequestManager* assetRequests_ = nullptr;
};

} // namespace openbus::rendering