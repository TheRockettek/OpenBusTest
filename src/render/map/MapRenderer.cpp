// MapRenderer lifecycle, streaming and script updates. Loading lives in
// MapSceneLoader*.cpp / MapScene*.cpp and drawing in MapRendererDraw.cpp.

#include "render/map/MapRendererInternal.h"
#include "render/map/MapSceneLoader.h"
#include "render/AssetRequestManager.h"
#include "render/HumanRig.h"
#include "config/HumanConfigLoader.h"
#include "core/Utf8Path.h"

#include <algorithm>
#include <cmath>

namespace openbus::rendering {
namespace map_detail {

Matrix4 bentPoleTransform(const Matrix4& baseTransform, const MapPoleState& state) {
    if (!std::isfinite(state.angleRadians) || std::abs(state.angleRadians) <= 1.0e-8) {
        return baseTransform;
    }
    const double axisLength =
        std::hypot(state.rotationAxis[0], state.rotationAxis[1], state.rotationAxis[2]);
    if (!std::isfinite(axisLength) || axisLength <= 1.0e-8) {
        return baseTransform;
    }
    const std::array<double, 3> axis = {state.rotationAxis[0] / axisLength,
                                        state.rotationAxis[1] / axisLength,
                                        state.rotationAxis[2] / axisLength};
    const double cosine = std::cos(state.angleRadians);
    const double sine = std::sin(state.angleRadians);
    const auto rotate = [&](const std::array<double, 3>& vector) {
        const double dot = vector[0] * axis[0] + vector[1] * axis[1] + vector[2] * axis[2];
        const std::array<double, 3> cross = {axis[1] * vector[2] - axis[2] * vector[1],
                                             axis[2] * vector[0] - axis[0] * vector[2],
                                             axis[0] * vector[1] - axis[1] * vector[0]};
        return std::array<double, 3>{
            vector[0] * cosine + cross[0] * sine + axis[0] * dot * (1.0 - cosine),
            vector[1] * cosine + cross[1] * sine + axis[1] * dot * (1.0 - cosine),
            vector[2] * cosine + cross[2] * sine + axis[2] * dot * (1.0 - cosine)};
    };
    Matrix4 transformed = baseTransform;
    for (std::size_t column = 0; column < 3; ++column) {
        const std::array<double, 3> direction = {baseTransform[column * 4],
                                                 baseTransform[column * 4 + 1],
                                                 baseTransform[column * 4 + 2]};
        const auto bent = rotate(direction);
        for (std::size_t row = 0; row < 3; ++row) {
            transformed[column * 4 + row] = bent[row];
        }
    }
    const std::array<double, 3> origin = {baseTransform[12], baseTransform[13], baseTransform[14]};
    const auto& base = state.basePosition;
    const auto bentOffset = rotate({origin[0] - base[0], origin[1] - base[1], origin[2] - base[2]});
    transformed[12] = base[0] + bentOffset[0];
    transformed[13] = base[1] + bentOffset[1];
    transformed[14] = base[2] + bentOffset[2];
    return transformed;
}

} // namespace map_detail

MapRenderer::Impl::~Impl() {
    releaseStaticPrimitiveBuffer(occlusionProxyBuffer);
    if (!occlusionQueries.empty() && pglDeleteQueries != nullptr) {
        pglDeleteQueries(static_cast<GLsizei>(occlusionQueries.size()), occlusionQueries.data());
    }
    const auto deleteBuffer = [](GLuint buffer) {
        if (buffer != 0 && pglDeleteBuffers != nullptr) {
            deleteRendererBuffers(1, &buffer);
        }
    };
    const auto deleteBatchBuffers = [&](const auto& batches) {
        for (const auto& batch : batches) {
            deleteBuffer(batch.buffer);
            deleteBuffer(batch.indexBuffer);
        }
    };
    deleteBatchBuffers(terrainBuffers);
    deleteBatchBuffers(waterBatches);
    deleteBatchBuffers(sceneryBatches);
    deleteBatchBuffers(roadBatches);
    for (const auto& batch : sceneryBatches) {
        deleteBuffer(batch.skinWeightBuffer);
        deleteBuffer(batch.seatedSkinWeightBuffer);
    }
    for (const GLuint ownedTexture : ownedTextures) {
        if (ownedTexture != 0) {
            glDeleteTextures(1, &ownedTexture);
        }
    }
    if (texture != 0) {
        glDeleteTextures(1, &texture);
    }
    if (!ownedTextures.empty() || texture != 0)
        invalidateTextureBindings();
}

MapRenderer::MapRenderer(const openbus::map::MapDefinition& map, std::size_t centerTileIndex,
                         double spawnWorldX, double spawnWorldY, std::size_t groundTextureIndex,
                         const std::filesystem::path& omsiRoot, SimulationState& simulationState,
                         AssetRequestManager& assetRequests, bool prepareInitialWindowAsync)
    : mapDefinition_(map), impl_(std::make_unique<Impl>()),
      scriptsEnabled_(!prepareInitialWindowAsync), asyncInitialLoad_(prepareInitialWindowAsync) {
    TraceScope trace("map", "MapRenderer::MapRenderer");
    assetRequests_ = &assetRequests;
    impl_->simulationState = &simulationState;
    impl_->omsiRoot = omsiRoot;
    impl_->coordinates = mapDefinition_.coordinates;
    for (const auto& tile : mapDefinition_.tiles) {
        const auto transform = mapDefinition_.coordinates.tile(tile.x, tile.y);
        impl_->sceneryProjectionRadiusScale =
            std::max({impl_->sceneryProjectionRadiusScale, transform.scaleX, transform.scaleY});
    }
    impl_->leftHandTraffic = mapDefinition_.metadata.hasLeftHandTraffic;
    sceneLoader_ = std::make_unique<MapSceneLoader>(
        mapDefinition_, centerTileIndex, groundTextureIndex, omsiRoot, *impl_, assetRequests);
    {
        TraceScope loadTrace("map", "MapRenderer.startSceneLoad");
        sceneLoader_->load();
    }
    {
        TraceScope streamingTrace("map", "MapRenderer.initialStreaming");
        updateStreaming(spawnWorldX, spawnWorldY);
    }
}

MapRenderer::~MapRenderer() = default;

bool MapRenderer::leftHandTraffic() const noexcept {
    return impl_->leftHandTraffic;
}

double MapRenderer::tileSize() const noexcept {
    return impl_->coordinates.tileSize;
}

void MapRenderer::enableScripts() {
    scriptsEnabled_ = true;
    initializeActiveScripts();
}

bool MapRenderer::initialWindowReady() const noexcept {
    return impl_->activeCenterTile.has_value();
}

void MapRenderer::updateScripts() {
    TraceScope trace("script", "MapRenderer::updateScripts");
    initializeActiveScripts();
    for (const std::size_t stateIndex : impl_->activeSceneryScriptStateIndices) {
        Impl::SceneryScriptState& scriptState = *impl_->sceneryScriptStates[stateIndex];
        if (!scriptState.runtime || !scriptState.runtimeInitialized) {
            continue;
        }
        const std::size_t previousErrorCount = scriptState.runtime->errors().size();
        scriptState.runtime->update(false);
        const std::vector<std::string>& errors = scriptState.runtime->errors();
        for (std::size_t index = previousErrorCount; index < errors.size(); ++index) {
            gameLog.Error("Scenery script frame error: " + errors[index]);
        }
    }
}

void MapRenderer::initializeActiveScripts() {
    if (!scriptsEnabled_) {
        return;
    }
    for (const std::size_t stateIndex : impl_->activeSceneryScriptStateIndices) {
        Impl::SceneryScriptState& scriptState = *impl_->sceneryScriptStates[stateIndex];
        if (!scriptState.definition || !scriptState.definition->hasRuntime) {
            continue;
        }
        if (!scriptState.runtime) {
            scriptState.runtime =
                std::make_unique<ScriptRuntime>(scriptState.definition->configuration,
                                                scriptState.variables, *impl_->simulationState);
            scriptState.runtime->configureScriptTextures(scriptState.definition->scriptTextures);
            scriptState.runtime->configureTextTextures(scriptState.definition->textTextures,
                                                       scriptState.placementLabels);
            ++impl_->sceneryScriptRuntimeCount;
        }
        if (scriptState.runtimeInitialized) {
            continue;
        }
        const std::size_t previousErrorCount = scriptState.runtime->errors().size();
        scriptState.runtime->initialize();
        scriptState.runtimeInitialized = true;
        scriptState.runtime->update(false);
        const std::vector<std::string>& errors = scriptState.runtime->errors();
        for (std::size_t index = previousErrorCount; index < errors.size(); ++index) {
            gameLog.Error("Scenery script initialization error: " + errors[index]);
        }
    }
}

void MapRenderer::updateStreaming(double worldX, double worldY) {
    const bool initialStreaming = !impl_->activeCenterTile.has_value();
    std::optional<TraceScope> streamingTrace;
    if (initialStreaming) {
        streamingTrace.emplace("map", "MapRenderer.updateStreaming.initial");
    }
    const bool waitForInitialLoad = !impl_->activeCenterTile.has_value() && !asyncInitialLoad_;
    bool sceneReady = false;
    if (initialStreaming) {
        TraceScope finishTrace("map", "MapRenderer.finishSceneLoad.initial");
        sceneReady = sceneLoader_->finishLoad(waitForInitialLoad);
    } else {
        sceneReady = sceneLoader_->finishLoad(waitForInitialLoad);
    }
    if (!sceneReady) {
        return;
    }
    const int centerX = openbus::map::mapWorldTile(worldX, impl_->coordinates.tileSize);
    const int centerY = openbus::map::mapWorldTile(worldY, impl_->coordinates.tileSize);
    const openbus::map::MapTileCoordinate center{centerX, centerY};
    if (impl_->requestedCenterTile != center) {
        impl_->requestedTileCoordinates = openbus::map::selectMapTilesWithinRadius(
            impl_->mapTileCoordinates, worldX, worldY, MAP_ACTIVE_TILE_RADIUS,
            impl_->coordinates.tileSize);
        impl_->requestedCenterTile = center;
    }
    MapSceneLoader::TileLoadStatus loadStatus;
    if (initialStreaming) {
        TraceScope tileTrace("map", "MapRenderer.loadActiveTiles.initial");
        loadStatus = sceneLoader_->loadActiveTiles(
            impl_->requestedTileCoordinates, *impl_->requestedCenterTile, waitForInitialLoad,
            !impl_->activeCenterTile.has_value() && asyncInitialLoad_);
    } else {
        loadStatus = sceneLoader_->loadActiveTiles(
            impl_->requestedTileCoordinates, *impl_->requestedCenterTile, waitForInitialLoad,
            !impl_->activeCenterTile.has_value() && asyncInitialLoad_);
    }
    if (loadStatus == MapSceneLoader::TileLoadStatus::Pending) {
        return;
    }
    const bool centerChanged = impl_->activeCenterTile != impl_->requestedCenterTile;
    if (!centerChanged && loadStatus != MapSceneLoader::TileLoadStatus::Updated) {
        return;
    }
    ++impl_->trafficDebugGeometryRevision;
    if (centerChanged) {
        impl_->activeTileCoordinates = impl_->requestedTileCoordinates;
        impl_->activeTileKeys.clear();
        impl_->activeTileKeys.reserve(impl_->activeTileCoordinates.size());
        for (const auto& [tileX, tileY] : impl_->activeTileCoordinates) {
            impl_->activeTileKeys.insert(tileKey(tileX, tileY));
        }
        impl_->activeCenterTile = impl_->requestedCenterTile;
    }
    MapRenderStats stats;
    stats.activeTiles = impl_->activeTileCoordinates.size();
    stats.totalTiles = impl_->mapTileCoordinates.size();
    stats.totalTerrainTiles = impl_->terrainBuffers.size();
    stats.totalSceneryObjects = impl_->sceneryObjectCount;
    stats.totalRoadBatches = impl_->roadBatches.size();
    stats.totalWaterBatches = impl_->waterBatches.size();
    stats.totalTextures = impl_->ownedTextures.size();
    const auto countForTile = [](const auto& counts, std::uint64_t key) {
        const auto found = counts.find(key);
        return found == counts.end() ? std::size_t{0} : found->second;
    };
    for (const auto& [tileX, tileY] : impl_->activeTileCoordinates) {
        const std::uint64_t key = tileKey(tileX, tileY);
        stats.activeTerrainTiles += countForTile(impl_->terrainTilesByTile, key);
        stats.activeSceneryObjects += countForTile(impl_->sceneryObjectsByTile, key);
        stats.activeRoadBatches += countForTile(impl_->roadBatchesByTile, key);
    }
    impl_->renderStats = stats;
    for (auto& activeChunks : impl_->activeSceneryChunks) {
        activeChunks.clear();
    }
    std::unordered_map<std::uint64_t, std::array<Impl::ActiveSceneryChunk, 2>> activeByChunk;
    for (const auto& [tileX, tileY] : impl_->activeTileCoordinates) {
        const auto instances = impl_->sceneryInstancesBySourceTile.find(tileKey(tileX, tileY));
        if (instances == impl_->sceneryInstancesBySourceTile.end()) {
            continue;
        }
        for (const Impl::SceneryInstanceReference& reference : instances->second) {
            const auto chunk = impl_->sceneryChunks.find(reference.chunkKey);
            if (chunk == impl_->sceneryChunks.end() ||
                reference.instanceIndex >= chunk->second.instances.size()) {
                continue;
            }
            const Impl::SceneryInstance& instance =
                chunk->second.instances[reference.instanceIndex];
            if (instance.batchIndex >= impl_->sceneryBatches.size()) {
                continue;
            }
            auto& active = activeByChunk[reference.chunkKey];
            active[0].chunk = &chunk->second;
            active[1].chunk = &chunk->second;
            const bool typeFour = impl_->sceneryBatches[instance.batchIndex].renderType == 4;
            active[typeFour ? 1 : 0].instanceIndexes.push_back(reference.instanceIndex);
        }
    }
    for (auto& [key, active] : activeByChunk) {
        static_cast<void>(key);
        for (std::size_t pass = 0; pass < active.size(); ++pass) {
            if (!active[pass].instanceIndexes.empty()) {
                impl_->activeSceneryChunks[pass].push_back(std::move(active[pass]));
            }
        }
    }
    impl_->activeSceneryScriptStateIndices.clear();
    for (const auto& [tileX, tileY] : impl_->activeTileCoordinates) {
        const auto states = impl_->sceneryScriptStatesByTile.find(tileKey(tileX, tileY));
        if (states != impl_->sceneryScriptStatesByTile.end()) {
            impl_->activeSceneryScriptStateIndices.insert(
                impl_->activeSceneryScriptStateIndices.end(), states->second.begin(),
                states->second.end());
        }
    }
    if (asyncInitialLoad_ && !scriptsEnabled_ && impl_->activeCenterTile.has_value()) {
        scriptsEnabled_ = true;
    }
    if (scriptsEnabled_) {
        initializeActiveScripts();
    }
    if (!loadSummaryLogged_) {
        sceneLoader_->logSummary();
        loadSummaryLogged_ = true;
    }
    gameLog.Log("Map active render window: center=" + tileLabel(centerX, centerY) +
                ", active tiles=" + std::to_string(impl_->activeTileCoordinates.size()) +
                ", radius=" + std::to_string(MAP_ACTIVE_TILE_RADIUS) + " tiles.");
}

void MapRenderer::updatePoleStates(const std::vector<MapPoleState>& states) {
    std::map<std::tuple<int, int, int>, MapPoleState> statesByPlacement;
    for (const MapPoleState& state : states) {
        statesByPlacement.insert_or_assign(std::tuple{state.tileX, state.tileY, state.mapObjectId},
                                           state);
    }
    for (auto& [key, chunk] : impl_->sceneryChunks) {
        static_cast<void>(key);
        for (Impl::SceneryInstance& instance : chunk.instances) {
            if (!instance.poleCrash || !instance.mapObjectId.has_value()) {
                continue;
            }
            const auto state = statesByPlacement.find(
                std::tuple{instance.sourceTileX, instance.sourceTileY, *instance.mapObjectId});
            instance.transform = state == statesByPlacement.end()
                                     ? instance.baseTransform
                                     : bentPoleTransform(instance.baseTransform, state->second);
        }
    }
}

const std::vector<std::pair<int, int>>& MapRenderer::activeTileCoordinates() const {
    return impl_->activeTileCoordinates;
}

std::vector<std::array<double, 3>>
MapRenderer::activeTileBoundaryVertices(double fallbackHeight) const {
    std::vector<std::array<double, 3>> vertices;
    vertices.reserve(impl_->activeTileCoordinates.size() * 8);
    for (const auto& [tileX, tileY] : impl_->activeTileCoordinates) {
        const auto transform = impl_->coordinates.tile(tileX, tileY);
        const auto terrain = impl_->terrainByCoordinate.find(tileKey(tileX, tileY));
        const auto corner = [&](double localX, double localY) {
            const auto world = transform.toWorld(localX, localY);
            const double height =
                terrain == impl_->terrainByCoordinate.end()
                    ? fallbackHeight
                    : openbus::map::sampleMapTerrainHeight(terrain->second, localX, localY,
                                                           impl_->coordinates.tileSize)
                          .value_or(fallbackHeight);
            return std::array<double, 3>{world[0], world[1], height + 0.05};
        };
        const std::array<std::array<double, 3>, 4> corners = {
            corner(0.0, 0.0), corner(impl_->coordinates.tileSize, 0.0),
            corner(impl_->coordinates.tileSize, impl_->coordinates.tileSize),
            corner(0.0, impl_->coordinates.tileSize)};
        for (std::size_t edge = 0; edge < corners.size(); ++edge) {
            vertices.push_back(corners[edge]);
            vertices.push_back(corners[(edge + 1) % corners.size()]);
        }
    }
    return vertices;
}

MapRenderStats MapRenderer::renderStats() const {
    return impl_->renderStats;
}

std::uint64_t MapRenderer::trafficDebugGeometryRevision() const noexcept {
    return impl_->trafficDebugGeometryRevision;
}

TrafficSignalDebugStats MapRenderer::trafficSignalDebugStats() const {
    TrafficSignalDebugStats stats;
    for (const std::size_t stateIndex : impl_->activeSceneryScriptStateIndices) {
        const Impl::SceneryScriptState& scriptState = *impl_->sceneryScriptStates[stateIndex];
        if (!scriptState.trafficSignal.has_value()) {
            continue;
        }
        ++stats.activeSignalModels;
        stats.controllerDrivenModels +=
            scriptState.trafficSignal->hasConditionalOverrides ? 1U : 0U;
        const auto phase = scriptState.variables.numericValues().find("trafficlightphase");
        if (phase == scriptState.variables.numericValues().end() || !std::isfinite(phase->second) ||
            phase->second < 0.0F || phase->second > 12.0F ||
            std::floor(phase->second) != phase->second) {
            ++stats.modelsWithoutPhase;
            continue;
        }
        ++stats.modelsByPhase[static_cast<std::size_t>(phase->second)];
    }
    return stats;
}

std::vector<TrafficSignalHeadDebugInfo>
MapRenderer::trafficSignalHeadDebugInfo(double signalClockSeconds) const {
    std::vector<TrafficSignalHeadDebugInfo> result;
    result.reserve(impl_->activeSceneryScriptStateIndices.size());
    for (const std::size_t stateIndex : impl_->activeSceneryScriptStateIndices) {
        const Impl::SceneryScriptState& scriptState = *impl_->sceneryScriptStates[stateIndex];
        if (!scriptState.trafficLightModel &&
            (!scriptState.trafficSignal.has_value() ||
             !scriptState.resolvedSignalParentObjectId.has_value())) {
            continue;
        }
        TrafficSignalHeadDebugInfo info;
        info.objectId = scriptState.mapObjectId;
        info.linkedJunctionObjectId = scriptState.resolvedSignalParentObjectId.has_value()
                                          ? scriptState.resolvedSignalParentObjectId
                                          : scriptState.variableParentId;
        info.position = scriptState.worldPosition;
        info.tileX = scriptState.ownerTileX;
        info.tileY = scriptState.ownerTileY;
        if (scriptState.trafficSignal.has_value()) {
            info.lightIndex = scriptState.trafficSignal->lightIndex;
            info.phase = openbus::map::mapTrafficSignalStateAtTime(*scriptState.trafficSignal,
                                                                   signalClockSeconds);
        }
        result.push_back(std::move(info));
    }
    return result;
}

std::optional<openbus::map::MapTrafficRoute> MapRenderer::buildTrafficRoute(int splineId) const {
    return impl_->trafficRouteNetwork.buildRoute(splineId);
}

std::optional<openbus::map::MapTrafficRoute>
MapRenderer::buildTrafficRoute(int splineId, int pathTypeCode) const {
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD) {
        return impl_->roadTrafficRouteNetwork.buildRoute(splineId);
    }
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
        return impl_->humanTrafficRouteNetwork.buildRoute(splineId);
    }
    return std::nullopt;
}

std::vector<int> MapRenderer::trafficSplineIds(int pathTypeCode) const {
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD) {
        return impl_->roadTrafficRouteNetwork.splineIds();
    }
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
        return impl_->humanTrafficRouteNetwork.splineIds();
    }
    return {};
}

bool MapRenderer::humanTrafficLaneNetworkReady() const noexcept {
    return sceneLoader_ && sceneLoader_->humanTrafficLaneNetworkReady();
}

std::optional<openbus::map::MapPassengerPedestrianJourney>
MapRenderer::passengerPedestrianJourneyNear(const std::array<double, 3>& stopPosition,
                                            std::size_t seed) const {
    if (!humanTrafficLaneNetworkReady()) {
        throw std::logic_error("Pedestrian lane network is not ready");
    }
    constexpr double radius = 25.0;
    const int minX = openbus::map::mapWorldTile(stopPosition[0] - radius, impl_->coordinates.tileSize);
    const int maxX = openbus::map::mapWorldTile(stopPosition[0] + radius, impl_->coordinates.tileSize);
    const int minY = openbus::map::mapWorldTile(stopPosition[1] - radius, impl_->coordinates.tileSize);
    const int maxY = openbus::map::mapWorldTile(stopPosition[1] + radius, impl_->coordinates.tileSize);
    std::vector<std::pair<int, int>> tiles;
    for (int x = minX; x <= maxX; ++x) {
        for (int y = minY; y <= maxY; ++y) {
            tiles.emplace_back(x, y);
        }
    }
    return impl_->humanTrafficLaneNetwork.passengerJourneyNear(
        impl_->humanTrafficRouteNetwork.splineIdsInTiles(tiles), stopPosition, seed, radius);
}

std::optional<openbus::map::MapPassengerPedestrianJourney>
MapRenderer::passengerPedestrianJourneyBetween(
    const std::array<double, 3>& startPosition,
    const std::array<double, 3>& destinationPosition, std::size_t seed,
    double maximumConnectorMeters, double maximumRouteMeters) const {
    if (!humanTrafficLaneNetworkReady()) {
        throw std::logic_error("Pedestrian lane network is not ready");
    }
    if (!std::ranges::all_of(startPosition,
                             [](double value) { return std::isfinite(value); }) ||
        !std::ranges::all_of(destinationPosition,
                             [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Pedestrian journey positions must be finite");
    }
    constexpr double searchRadius = 25.0;
    const int minX =
        openbus::map::mapWorldTile(startPosition[0] - searchRadius, impl_->coordinates.tileSize);
    const int maxX =
        openbus::map::mapWorldTile(startPosition[0] + searchRadius, impl_->coordinates.tileSize);
    const int minY =
        openbus::map::mapWorldTile(startPosition[1] - searchRadius, impl_->coordinates.tileSize);
    const int maxY =
        openbus::map::mapWorldTile(startPosition[1] + searchRadius, impl_->coordinates.tileSize);
    std::vector<std::pair<int, int>> tiles;
    for (int x = minX; x <= maxX; ++x) {
        for (int y = minY; y <= maxY; ++y) {
            tiles.emplace_back(x, y);
        }
    }
    return impl_->humanTrafficLaneNetwork.passengerJourneyBetween(
        impl_->humanTrafficRouteNetwork.splineIdsInTiles(tiles), startPosition,
        destinationPosition, seed, maximumConnectorMeters, maximumRouteMeters);
}

std::vector<int>
MapRenderer::trafficSplineIdsInTiles(int pathTypeCode,
                                     const std::vector<std::pair<int, int>>& tiles) const {
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD) {
        return impl_->roadTrafficRouteNetwork.splineIdsInTiles(tiles);
    }
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
        return impl_->humanTrafficRouteNetwork.splineIdsInTiles(tiles);
    }
    return {};
}

std::vector<int> MapRenderer::trafficRouteStartSplineIds(int pathTypeCode) const {
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD) {
        std::vector<int> ids;
        for (const auto& entry : trafficEntryLanes()) {
            if (ids.empty() || ids.back() != entry.splineId) {
                ids.push_back(entry.splineId);
            }
        }
        return ids;
    }
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
        return impl_->humanTrafficRouteNetwork.routeStartSplineIds();
    }
    return {};
}

std::optional<openbus::map::MapTrafficSplineEndpointStatus>
MapRenderer::trafficSplineEndpointStatus(int splineId, int pathTypeCode) const {
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD) {
        return impl_->roadTrafficRouteNetwork.endpointStatus(splineId);
    }
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
        return impl_->humanTrafficRouteNetwork.endpointStatus(splineId);
    }
    return std::nullopt;
}

std::vector<openbus::map::MapSplinePathOffset>
MapRenderer::trafficSplinePathOffsets(int splineId, int pathTypeCode) const {
    const auto* pathOffsets = pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD
                                  ? &impl_->roadTrafficSplinePathOffsets
                              : pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN
                                  ? &impl_->humanTrafficSplinePathOffsets
                                  : nullptr;
    if (pathOffsets == nullptr) {
        return {};
    }
    const auto found = pathOffsets->find(splineId);
    return found == pathOffsets->end() ? std::vector<openbus::map::MapSplinePathOffset>{}
                                       : found->second;
}

const std::vector<TrafficPathDebugSegment>& MapRenderer::sceneryTrafficPathSegments() const {
    return impl_->sceneryTrafficPathSegments;
}

const std::vector<EditorPassengerDebugObject>& MapRenderer::editorPassengerDebugObjects() const {
    return impl_->editorPassengerDebugObjects;
}

std::optional<std::array<double, 3>> MapRenderer::editorOnlyBusStopPosition(int stopId) const {
    for (const EditorPassengerDebugObject& object : impl_->editorPassengerDebugObjects) {
        if (object.busStop && object.objectId == stopId) {
            return object.position;
        }
    }
    return std::nullopt;
}

std::optional<openbus::map::MapTrafficActor>
MapRenderer::createTrafficActor(int splineId, std::string modelPath, double speedMetersPerSecond,
                                double laneOffsetMagnitudeMeters,
                                std::optional<std::string> driverModelPath,
                                std::optional<int> pathTypeCode, std::size_t laneSelector) const {
    std::optional<openbus::map::MapTrafficRoute> route;
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD) {
        route = buildDrivingRoute(splineId, laneSelector);
    } else if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
        route = impl_->humanTrafficLaneNetwork.buildRoute(splineId, laneSelector);
    } else {
        route = impl_->trafficRouteNetwork.buildRoute(splineId);
    }
    if (!route.has_value()) {
        return std::nullopt;
    }
    return openbus::map::makeMapTrafficActor(std::move(*route), std::move(modelPath),
                                             speedMetersPerSecond, laneOffsetMagnitudeMeters,
                                             impl_->leftHandTraffic, std::move(driverModelPath));
}

void MapRenderer::loadHumanTrafficModel(const std::filesystem::path& humanConfigPath) {
    const std::string cacheKey = openbus::pathToUtf8(humanConfigPath.lexically_normal());
    if (impl_->humanTrafficModels.contains(cacheKey) ||
        impl_->pendingHumanTrafficModels.contains(cacheKey) ||
        impl_->failedHumanTrafficModels.contains(cacheKey)) {
        return;
    }
    if (assetRequests_ == nullptr) {
        throw std::logic_error("Human traffic model loading requires an asset request manager");
    }

    Impl::PendingHumanModel pending;
    pending.humanConfigPath = humanConfigPath;
    pending.configurationRequest = assetRequests_->submit([humanConfigPath] {
        if (!std::filesystem::is_regular_file(humanConfigPath)) {
            throw std::runtime_error("Human traffic configuration is missing: " +
                                     openbus::pathToUtf8(humanConfigPath));
        }
        const openbus::config::HumanConfig humanConfig =
            openbus::config::loadHumanConfig(humanConfigPath);
        if (humanConfig.diagnostics.hasErrors() || humanConfig.modelConfigPath.empty()) {
            throw std::runtime_error("Human traffic configuration is invalid: " +
                                     openbus::pathToUtf8(humanConfigPath));
        }
        const auto geometry =
            std::find_if(humanConfig.numericRecords.begin(), humanConfig.numericRecords.end(),
                         [](const openbus::config::HumanNumericRecord& record) {
                             return record.keyword == "humangeom" && record.valid &&
                                    record.values.size() == 2 && std::isfinite(record.values[0]) &&
                                    record.values[0] > 0.0 && std::isfinite(record.values[1]) &&
                                    record.values[1] > 0.0;
                         });
        if (geometry == humanConfig.numericRecords.end()) {
            throw std::runtime_error("Human traffic configuration has no valid [humangeom]: " +
                                     openbus::pathToUtf8(humanConfigPath));
        }
        const std::filesystem::path modelConfigPath =
            openbus::assets::resolveReferencedPath(humanConfigPath, humanConfig.modelConfigPath);
        if (!std::filesystem::is_regular_file(modelConfigPath)) {
            throw std::runtime_error("Human traffic model configuration is missing: " +
                                     openbus::pathToUtf8(modelConfigPath));
        }
        openbus::scripting::SceneryObject variables;
        ModelConfig configuration = loadModelConfig(modelConfigPath, modelConfigPath.parent_path(),
                                                    ModelConfigKind::SceneryObject, variables);
        if (configuration.diagnostics.hasErrors()) {
            std::string diagnosticDetails;
            for (const ConfigurationDiagnostic& diagnostic : configuration.diagnostics.entries) {
                if (diagnostic.severity != ConfigurationDiagnostic::Severity::Error) {
                    continue;
                }
                diagnosticDetails += " line=" + std::to_string(diagnostic.line) + " [" +
                                     diagnostic.keyword + "] " + diagnostic.message;
            }
            throw std::runtime_error("Human traffic model configuration is invalid: " +
                                     openbus::pathToUtf8(modelConfigPath) + diagnosticDetails);
        }
        for (ModelPart& part : configuration.parts) {
            std::string sourceExtension =
                std::filesystem::path(part.sourceMeshPath).extension().string();
            std::transform(sourceExtension.begin(), sourceExtension.end(), sourceExtension.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            if (part.boneBindings.empty() || sourceExtension != ".o3d")
                continue;
            const std::filesystem::path source = openbus::assets::resolveReferencedPath(
                modelConfigPath, openbus::pathToUtf8(part.sourceMeshPath));
            if (std::filesystem::is_regular_file(source)) {
                part.objPath = source;
                part.bundleEntry.clear();
            }
        }
        const bool hasFirstLodMesh = std::any_of(
            configuration.parts.begin(), configuration.parts.end(),
            [](const ModelPart& part) { return part.lodIndex <= 0 && !part.objPath.empty(); });
        if (!hasFirstLodMesh) {
            throw std::runtime_error("Human traffic model has no first-LOD meshes: " +
                                     openbus::pathToUtf8(modelConfigPath));
        }
        return Impl::HumanModelConfiguration{modelConfigPath, std::move(configuration),
                                             makeHumanGaitProfile(humanConfig, geometry->values[1]),
                                             makeHumanJoints(humanConfig)};
    });
    impl_->pendingHumanTrafficModels.emplace(cacheKey, std::move(pending));
}

bool MapRenderer::finishHumanTrafficModelLoad(const std::string& cacheKey) const {
    if (impl_->humanTrafficModels.contains(cacheKey)) {
        return true;
    }
    const auto pending = impl_->pendingHumanTrafficModels.find(cacheKey);
    if (pending == impl_->pendingHumanTrafficModels.end()) {
        return false;
    }
    Impl::PendingHumanModel& pendingModel = pending->second;
    if (!pendingModel.configuration.has_value()) {
        if (!pendingModel.configurationRequest.valid()) {
            throw std::logic_error("Human traffic configuration request is invalid");
        }
        if (pendingModel.configurationRequest.wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
            return false;
        }
        pendingModel.configuration = pendingModel.configurationRequest.get();
        for (const ModelPart& part : pendingModel.configuration->configuration.parts) {
            if (part.lodIndex <= 0 && !part.objPath.empty()) {
                pendingModel.meshRequests.push_back(
                    assetRequests_->requestObj(part.objPath, part.bundleEntry));
            }
        }
        if (pendingModel.meshRequests.empty()) {
            throw std::runtime_error(
                "Human traffic model has no first-LOD meshes: " +
                openbus::pathToUtf8(pendingModel.configuration->modelConfigPath));
        }
    }
    for (const auto& request : pendingModel.meshRequests) {
        if (request.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return false;
        }
    }
    const std::filesystem::path humanConfigPath = pendingModel.humanConfigPath;
    Impl::HumanModelConfiguration configuration = std::move(*pendingModel.configuration);
    impl_->pendingHumanTrafficModels.erase(pending);
    loadHumanTrafficModelNow(humanConfigPath, configuration.modelConfigPath,
                             configuration.configuration, configuration.gait, configuration.joints);
    return impl_->humanTrafficModels.contains(cacheKey);
}

void MapRenderer::loadHumanTrafficModelNow(const std::filesystem::path& humanConfigPath,
                                           const std::filesystem::path& modelConfigPath,
                                           const ModelConfig& configuration,
                                           const HumanGaitParameters& gait,
                                           const std::optional<HumanJoints>& joints) const {
    const std::string cacheKey = openbus::pathToUtf8(humanConfigPath.lexically_normal());
    if (impl_->humanTrafficModels.contains(cacheKey)) {
        return;
    }
    const std::filesystem::path modelRoot = modelConfigPath.parent_path();
    for (const ConfigurationDiagnostic& diagnostic : configuration.diagnostics.entries) {
        if (diagnostic.severity == ConfigurationDiagnostic::Severity::Error) {
            gameLog.Error(
                "Human model configuration error: " + openbus::pathToUtf8(modelConfigPath) +
                " line=" + std::to_string(diagnostic.line) + " [" + diagnostic.keyword + "] " +
                diagnostic.message);
        }
    }
    if (configuration.diagnostics.hasErrors()) {
        throw std::runtime_error("Human traffic model configuration is invalid: " +
                                 openbus::pathToUtf8(modelConfigPath));
    }

    struct ParsedHumanPart {
        const ModelPart* part = nullptr;
        std::shared_ptr<ParsedObj> mesh;
    };
    std::vector<ParsedHumanPart> parsedParts;
    std::array<double, 3> minimum = {std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::infinity()};
    std::array<double, 3> maximum = {-std::numeric_limits<double>::infinity(),
                                     -std::numeric_limits<double>::infinity(),
                                     -std::numeric_limits<double>::infinity()};
    for (const ModelPart& part : configuration.parts) {
        if (part.lodIndex > 0 || part.objPath.empty()) {
            continue;
        }
        try {
            std::string extension = part.objPath.extension().string();
            std::transform(
                extension.begin(), extension.end(), extension.begin(),
                [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
            std::shared_ptr<ParsedObj> mesh;
            if (extension == ".o3d") {
                mesh = assetRequests_->requestObj(part.objPath, part.bundleEntry).get();
            } else if (extension == ".x") {
                mesh = assetRequests_->requestObj(part.objPath, part.bundleEntry).get();
            } else if (extension == ".obj") {
                mesh = assetRequests_->requestObj(part.objPath, part.bundleEntry).get();
            }
            if (!mesh) {
                gameLog.Warning("Skipping unsupported or empty human model mesh: " +
                                openbus::pathToUtf8(part.objPath));
                continue;
            }
            for (const ObjPosition& position : mesh->positions) {
                const std::array<double, 3> values = {position.x, position.y, position.z};
                for (std::size_t axis = 0; axis < values.size(); ++axis) {
                    minimum[axis] = std::min(minimum[axis], values[axis]);
                    maximum[axis] = std::max(maximum[axis], values[axis]);
                }
            }
            parsedParts.push_back({&part, std::move(mesh)});
        } catch (const std::exception& error) {
            gameLog.Warning("Could not load human model mesh " + openbus::pathToUtf8(part.objPath) +
                            ": " + error.what());
        }
    }
    if (parsedParts.empty() || !std::isfinite(minimum[0]) || !std::isfinite(minimum[1]) ||
        !std::isfinite(minimum[2])) {
        throw std::runtime_error("Human traffic model has no loadable mesh: " +
                                 openbus::pathToUtf8(modelConfigPath));
    }
    if (pglGenBuffers == nullptr || pglBindBuffer == nullptr || pglBufferData == nullptr) {
        throw std::runtime_error("OpenGL buffer functions are unavailable for human models");
    }

    const std::array<double, 3> extent = {maximum[0] - minimum[0], maximum[1] - minimum[1],
                                          maximum[2] - minimum[2]};
    if (!std::isfinite(gait.heightMeters) || gait.heightMeters <= 0.0 || extent[2] <= 1.0e-6) {
        throw std::runtime_error("Human traffic model has invalid height geometry: " +
                                 openbus::pathToUtf8(modelConfigPath));
    }
    const double modelScale = gait.heightMeters / extent[2];
    const std::array<double, 2> horizontalCenter = {(minimum[0] + maximum[0]) * 0.5,
                                                    (minimum[1] + maximum[1]) * 0.5};
    std::vector<Impl::ModelBatch> batches;
    std::vector<std::filesystem::path> texturePaths;
    std::vector<std::shared_future<std::shared_ptr<Image>>> textureImages;
    bool standingSkinningReliable = joints.has_value();
    const auto drainGlErrors = [] {
        GLenum firstError = GL_NO_ERROR;
        for (;;) {
            const GLenum error = glGetError();
            if (error == GL_NO_ERROR) {
                break;
            }
            if (firstError == GL_NO_ERROR) {
                firstError = error;
            }
        }
        return firstError;
    };
    bool loggedPriorUploadError = false;
    const HumanJoints seatedJoints = joints.value_or(approximateHumanJoints(gait));
    for (const ParsedHumanPart& parsedPart : parsedParts) {
        const ModelPart& part = *parsedPart.part;
        std::unordered_map<int, std::vector<ModelVertex>> verticesByMaterial;
        std::unordered_map<int, std::vector<HumanVertexWeights>> weightsByMaterial;
        std::unordered_map<int, std::vector<HumanVertexWeights>> seatedWeightsByMaterial;
        HumanSkinWeights skin;
        if (joints && !parsedPart.mesh->bones.empty() && !part.boneBindings.empty()) {
            skin = resolveHumanSkinWeights(*parsedPart.mesh, part.boneBindings);
            if (skin.invalidRecords != 0 || skin.unboundBones != 0 ||
                skin.unweightedVertices != 0) {
                gameLog.Warning("Human skin weights for " + openbus::pathToUtf8(part.objPath) +
                                ": invalid records=" + std::to_string(skin.invalidRecords) +
                                ", unbound bones=" + std::to_string(skin.unboundBones) +
                                ", unweighted vertices=" + std::to_string(skin.unweightedVertices));
            }
            if (!hasCompleteHumanSkinWeights(skin)) {
                standingSkinningReliable = false;
            }
            if (skin.hasWeights && !hasCompleteHumanSkinWeights(skin)) {
                gameLog.Warning("Using procedural gait instead of incomplete bone skinning for " +
                                openbus::pathToUtf8(part.objPath));
                skin.hasWeights = false;
            }
        } else if (!parsedPart.mesh->bones.empty() || !part.boneBindings.empty()) {
            standingSkinningReliable = false;
            gameLog.Warning("Human mesh cannot use bone skinning without valid [links] and bone "
                            "bindings: " +
                            openbus::pathToUtf8(part.objPath));
        } else {
            standingSkinningReliable = false;
        }
        const bool inferSeatedWeights = !skin.hasWeights || skin.unweightedVertices != 0;
        if (inferSeatedWeights) {
            gameLog.Warning("Using approximate seated skin weights for human mesh: " +
                            openbus::pathToUtf8(part.objPath));
        }
        for (const ObjTriangle& triangle : parsedPart.mesh->triangles) {
            const auto sourceMaterial = parsedPart.mesh->materials.find(triangle.material);
            const int materialIndex = sourceMaterial == parsedPart.mesh->materials.end()
                                          ? 0
                                          : sourceMaterial->second.materialIndex;
            const auto validIndex = [](int index, std::size_t count) {
                return index > 0 && static_cast<std::size_t>(index) <= count;
            };
            for (const ObjIndex& index : triangle.indices) {
                if (!validIndex(index.position, parsedPart.mesh->positions.size())) {
                    continue;
                }
                const std::size_t positionIndex = static_cast<std::size_t>(index.position - 1);
                const ObjPosition& sourcePosition = parsedPart.mesh->positions[positionIndex];
                const std::array<double, 4> vertexColor =
                    positionIndex < parsedPart.mesh->vertexColors.size()
                        ? parsedPart.mesh->vertexColors[positionIndex]
                        : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};
                std::array<double, 3> normal = {0.0, 0.0, 1.0};
                if (validIndex(index.normal, parsedPart.mesh->normals.size())) {
                    const ObjNormal& sourceNormal =
                        parsedPart.mesh->normals[static_cast<std::size_t>(index.normal - 1)];
                    normal = {sourceNormal.x, sourceNormal.y, sourceNormal.z};
                    const double normalLength = std::hypot(normal[0], normal[1], normal[2]);
                    if (normalLength > 1.0e-8) {
                        for (double& component : normal) {
                            component /= normalLength;
                        }
                    }
                }
                float u = 0.0F;
                float v = 0.0F;
                if (validIndex(index.texCoord, parsedPart.mesh->texCoords.size())) {
                    const ObjTexCoord& coordinate =
                        parsedPart.mesh->texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                    u = static_cast<float>(coordinate.u);
                    v = static_cast<float>(1.0 - coordinate.v);
                }
                verticesByMaterial[materialIndex].push_back(
                    {static_cast<float>((sourcePosition.x - horizontalCenter[0]) * modelScale),
                     static_cast<float>((sourcePosition.y - horizontalCenter[1]) * modelScale),
                     static_cast<float>((sourcePosition.z - minimum[2]) * modelScale), u, v, 0.0F,
                     static_cast<float>(normal[0]), static_cast<float>(normal[1]),
                     static_cast<float>(normal[2]), static_cast<float>(vertexColor[0]),
                     static_cast<float>(vertexColor[1]), static_cast<float>(vertexColor[2]),
                     static_cast<float>(vertexColor[3])});
                if (skin.hasWeights) {
                    weightsByMaterial[materialIndex].push_back(skin.vertices[positionIndex]);
                }
                if (inferSeatedWeights) {
                    HumanVertexWeights seatedWeights = {};
                    if (skin.hasWeights) {
                        seatedWeights = skin.vertices[positionIndex];
                    }
                    const bool weighted = std::any_of(
                        seatedWeights.begin(), seatedWeights.begin() + HUMAN_BONE_COUNT,
                        [](float weight) { return weight > 0.0F; });
                    if (!weighted) {
                        seatedWeights = inferHumanSkinWeights(
                            {(sourcePosition.x - horizontalCenter[0]) * modelScale,
                             (sourcePosition.y - horizontalCenter[1]) * modelScale,
                             (sourcePosition.z - minimum[2]) * modelScale},
                            seatedJoints, gait.heightMeters);
                    }
                    seatedWeightsByMaterial[materialIndex].push_back(seatedWeights);
                }
            }
        }
        for (auto& [materialIndex, vertices] : verticesByMaterial) {
            if (vertices.empty()) {
                continue;
            }
            const auto sourceMaterial =
                std::find_if(parsedPart.mesh->materials.begin(), parsedPart.mesh->materials.end(),
                             [materialIndex](const auto& item) {
                                 return item.second.materialIndex == materialIndex;
                             });
            const ModelMaterialState* materialState = nullptr;
            const auto state =
                std::find_if(part.materialStatesInOrder.begin(), part.materialStatesInOrder.end(),
                             [materialIndex](const ModelMaterialState& item) {
                                 return item.materialIndex == materialIndex;
                             });
            if (state != part.materialStatesInOrder.end()) {
                materialState = &*state;
            }
            std::string textureName = sourceMaterial == parsedPart.mesh->materials.end()
                                          ? std::string{}
                                          : sourceMaterial->second.textureName;
            if (materialState != nullptr && !materialState->textureName.empty()) {
                textureName = materialState->textureName;
            }

            Impl::ModelBatch batch;
            batch.seatedSkinUsesFallback = inferSeatedWeights;
            batch.vertexCount = vertices.size();
            batch.color = sourceMaterial == parsedPart.mesh->materials.end()
                              ? part.color
                              : sourceMaterial->second.color;
            batch.alpha = sourceMaterial == parsedPart.mesh->materials.end()
                              ? 1.0
                              : std::clamp(sourceMaterial->second.alpha, 0.0, 1.0);
            batch.alphaMode = materialState == nullptr ? 0 : materialState->alphaMode;
            if (batch.alphaMode == 0 && batch.alpha < 1.0 - 1.0e-6) {
                batch.alphaMode = 2;
            }
            batch.cullBackFaces = parsedPart.mesh->backFaceCulling;
            if (materialState != nullptr) {
                batch.material.textureWrapS =
                    textureAddressModeToGl(materialState->textureAddressS);
                batch.material.textureWrapT =
                    textureAddressModeToGl(materialState->textureAddressT);
                batch.noZwrite = materialState->noZwrite;
                batch.noZcheck = materialState->noZcheck;
            }
            if (sourceMaterial != parsedPart.mesh->materials.end()) {
                for (std::size_t component = 0; component < 3; ++component) {
                    batch.material.specularColor[component] =
                        static_cast<float>(sourceMaterial->second.specular[component]);
                    batch.material.emissionColor[component] =
                        static_cast<float>(sourceMaterial->second.emission[component]);
                }
                batch.material.specularPower =
                    static_cast<float>(sourceMaterial->second.specularPower);
            }
            const std::filesystem::path texturePath =
                findSceneryTexture(modelRoot, modelRoot, impl_->omsiRoot, textureName);
            std::shared_future<std::shared_ptr<Image>> textureImage;
            if (!texturePath.empty()) {
                const std::string textureKey = openbus::pathToUtf8(texturePath.lexically_normal());
                auto texture = impl_->dynamicSceneryTextures.find(textureKey);
                if (texture != impl_->dynamicSceneryTextures.end()) {
                    batch.material.texture = texture->second;
                    batch.material.textured = batch.material.texture != 0;
                } else {
                    auto request = impl_->humanTextureRequests.find(textureKey);
                    if (request == impl_->humanTextureRequests.end()) {
                        textureImage = assetRequests_
                                           ->submit([texturePath] {
                                               auto image = std::make_shared<Image>();
                                               if (!TextureLoader::readImage(texturePath, *image)) {
                                                   throw std::runtime_error(
                                                       "Could not decode human model texture: " +
                                                       openbus::pathToUtf8(texturePath));
                                               }
                                               return image;
                                           })
                                           .share();
                        request =
                            impl_->humanTextureRequests.emplace(textureKey, std::move(textureImage))
                                .first;
                    }
                    textureImage = request->second;
                }
            } else if (!textureName.empty()) {
                gameLog.Warning("Human model texture was not found: " + textureName);
            }
            const GLenum priorGlError = drainGlErrors();
            if (priorGlError != GL_NO_ERROR && !loggedPriorUploadError) {
                gameLog.Warning("OpenGL error predates human mesh upload; error=" +
                                std::to_string(priorGlError));
                loggedPriorUploadError = true;
            }
            pglGenBuffers(1, &batch.buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(vertices.size() * sizeof(ModelVertex)),
                          vertices.data(), GL_STATIC_DRAW);
            const GLenum uploadError = drainGlErrors();
            if (batch.buffer == 0 || uploadError != GL_NO_ERROR) {
                if (batch.buffer != 0) {
                    deleteRendererBuffers(1, &batch.buffer);
                }
                gameLog.Warning("Could not upload human model mesh '" +
                                openbus::pathToUtf8(part.objPath) +
                                "', material=" + std::to_string(materialIndex) +
                                ", OpenGL error=" + std::to_string(uploadError));
                continue;
            }
            const auto uploadWeights = [&](GLuint& buffer,
                                           const std::vector<HumanVertexWeights>& weights) {
                pglGenBuffers(1, &buffer);
                pglBindBuffer(GL_ARRAY_BUFFER, buffer);
                pglBufferData(
                    GL_ARRAY_BUFFER,
                    static_cast<std::ptrdiff_t>(weights.size() * sizeof(HumanVertexWeights)),
                    weights.data(), GL_STATIC_DRAW);
                const GLenum skinUploadError = drainGlErrors();
                if (buffer == 0 || skinUploadError != GL_NO_ERROR) {
                    if (buffer != 0) {
                        deleteRendererBuffers(1, &buffer);
                        buffer = 0;
                    }
                    gameLog.Warning("Could not upload human skin weights for " +
                                    openbus::pathToUtf8(part.objPath) +
                                    ", OpenGL error=" + std::to_string(skinUploadError));
                }
            };
            if (skin.hasWeights) {
                uploadWeights(batch.skinWeightBuffer, weightsByMaterial.at(materialIndex));
            }
            if (inferSeatedWeights) {
                uploadWeights(batch.seatedSkinWeightBuffer,
                              seatedWeightsByMaterial.at(materialIndex));
            }
            texturePaths.push_back(texturePath);
            textureImages.push_back(std::move(textureImage));
            batches.push_back(std::move(batch));
        }
    }
    if (batches.empty()) {
        throw std::runtime_error("Human traffic model contains no renderable mesh: " +
                                 openbus::pathToUtf8(modelConfigPath));
    }

    Impl::HumanModelAsset asset;
    asset.batchIndexes.reserve(batches.size());
    asset.texturePaths = std::move(texturePaths);
    asset.textureImages = std::move(textureImages);
    asset.gait = gait;
    asset.joints = joints;
    asset.standingSkinningReliable = standingSkinningReliable;
    impl_->sceneryBatches.reserve(impl_->sceneryBatches.size() + batches.size());
    for (Impl::ModelBatch& batch : batches) {
        asset.batchIndexes.push_back(impl_->sceneryBatches.size());
        impl_->sceneryBatches.push_back(std::move(batch));
    }
    impl_->humanTrafficModels.emplace(cacheKey, std::move(asset));
}

std::vector<std::shared_ptr<openbus::map::MapTrafficSignalController>>
MapRenderer::trafficSignalControllers() const {
    return impl_->trafficSignalControllers;
}

std::vector<openbus::map::MapTrafficSignalRequest>
MapRenderer::trafficSignalRequests(std::optional<std::array<double, 2>> playerBusPosition,
                                   double playerSpeedMetersPerSecond) const {
    if (!std::isfinite(playerSpeedMetersPerSecond)) {
        throw std::invalid_argument("Player traffic-signal request speed must be finite");
    }
    if (playerBusPosition.has_value() &&
        (!std::isfinite((*playerBusPosition)[0]) || !std::isfinite((*playerBusPosition)[1]))) {
        throw std::invalid_argument("Player traffic-signal request position must be finite");
    }
    std::vector<openbus::map::MapTrafficSignalRequest> playerRequests;
    if (playerBusPosition.has_value()) {
        for (const TrafficPathDebugSegment& segment : impl_->sceneryTrafficPathSegments) {
            if (!segment.trafficSignal.has_value() ||
                !segment.trafficSignal->hasConditionalOverrides ||
                segment.trafficSignal->controller == nullptr ||
                segment.trafficSignal->program == nullptr || segment.points.size() < 2) {
                continue;
            }
            const auto& start = segment.points.front();
            const auto& next = segment.points[1];
            const double directionX = next[0] - start[0];
            const double directionY = next[1] - start[1];
            const double directionLength = std::hypot(directionX, directionY);
            if (directionLength <= 1.0e-9) {
                continue;
            }
            const double approachDistance =
                segment.trafficSignal->approachDistanceMeters.value_or(50.0);
            const double unitX = directionX / directionLength;
            const double unitY = directionY / directionLength;
            const double offsetX = (*playerBusPosition)[0] - start[0];
            const double offsetY = (*playerBusPosition)[1] - start[1];
            const double distanceToStart = -(offsetX * unitX + offsetY * unitY);
            const double lateralDistance = -offsetX * unitY + offsetY * unitX;
            const bool stoppedAtLine = std::abs(playerSpeedMetersPerSecond) < 0.5 &&
                                       distanceToStart >= -0.5 && distanceToStart <= 3.0;
            const bool withinApproachDistance = approachDistance > 0.0 && distanceToStart >= -0.5 &&
                                                distanceToStart <= approachDistance;
            if ((stoppedAtLine || withinApproachDistance) &&
                std::abs(lateralDistance) <= segment.widthMeters * 0.5 + 1.5) {
                playerRequests.push_back(
                    {segment.trafficSignal->controller, segment.trafficSignal->lightIndex});
            }
        }
    }
    return playerRequests;
}

void MapRenderer::updateTrafficSignalScripts(double signalClockSeconds) const {
    for (const std::size_t stateIndex : impl_->activeSceneryScriptStateIndices) {
        Impl::SceneryScriptState& scriptState = *impl_->sceneryScriptStates[stateIndex];
        if (!scriptState.trafficSignal.has_value()) {
            continue;
        }
        const std::optional<int> phase = openbus::map::mapTrafficSignalStateAtTime(
            *scriptState.trafficSignal, signalClockSeconds);
        if (!phase.has_value()) {
            throw std::runtime_error("Could not resolve a scenery traffic-light phase");
        }
        scriptState.variables.set("trafficlightphase", static_cast<float>(*phase));
    }
}

std::optional<openbus::map::MapTrafficRoute> MapRenderer::buildDrivingRoute(
    int splineId, std::size_t laneSelector, std::size_t routeSeed, double minimumTurnRadiusMeters,
    openbus::map::MapTrafficVehicleFilter vehicle, const std::vector<int>& requiredSplineSequence,
    const std::vector<std::optional<std::size_t>>& requiredSceneryPathSequence,
    bool stopAtRequiredSequenceEnd, bool allowIntermediarySegmentsBetweenRequired,
    bool allowReverseTraversalForRequired) const {
    if (!sceneLoader_->roadTrafficLaneNetworkReady()) {
        return std::nullopt;
    }
    return impl_->roadTrafficLaneNetwork.buildRoute(
        splineId, laneSelector, routeSeed, minimumTurnRadiusMeters, vehicle, requiredSplineSequence,
        requiredSceneryPathSequence, stopAtRequiredSequenceEnd,
        allowIntermediarySegmentsBetweenRequired, allowReverseTraversalForRequired);
}

std::vector<openbus::map::MapTrafficLaneStart>
MapRenderer::lanesForTimetableEntry(int splineId,
                                    std::optional<std::size_t> sceneryPathIndex) const {
    if (!sceneLoader_->roadTrafficLaneNetworkReady()) {
        return {};
    }
    return impl_->roadTrafficLaneNetwork.lanesForTimetableEntry(splineId, sceneryPathIndex);
}

std::optional<openbus::map::MapTrafficPose>
MapRenderer::timetableStopPose(int splineId, std::size_t pathIndex, double offsetMeters) const {
    if (!sceneLoader_->roadTrafficLaneNetworkReady()) {
        return std::nullopt;
    }
    return impl_->roadTrafficLaneNetwork.timetableStopPose(splineId, pathIndex, offsetMeters);
}

std::optional<openbus::map::MapTrafficLaneConnectionGap>
MapRenderer::nearestTrafficLaneConnectionGap(int fromSplineId,
                                             std::optional<std::size_t> fromPathIndex,
                                             int toSplineId,
                                             std::optional<std::size_t> toPathIndex) const {
    if (!sceneLoader_->roadTrafficLaneNetworkReady()) {
        return std::nullopt;
    }
    return impl_->roadTrafficLaneNetwork.nearestConnectionGap(fromSplineId, fromPathIndex,
                                                              toSplineId, toPathIndex);
}

std::vector<openbus::map::MapTrafficLaneStart> MapRenderer::trafficEntryLanes() const {
    if (!sceneLoader_->roadTrafficLaneNetworkReady()) {
        return {};
    }
    return impl_->roadTrafficLaneNetwork.entryLanes();
}

std::optional<openbus::map::MapTrafficLaneStart>
MapRenderer::selectTrafficEntryLane(openbus::map::MapTrafficVehicleFilter vehicle,
                                    std::size_t selectionSeed) const {
    if (!sceneLoader_->roadTrafficLaneNetworkReady()) {
        return std::nullopt;
    }
    return impl_->roadTrafficLaneNetwork.selectEntryLane(vehicle, selectionSeed);
}

std::vector<openbus::map::MapTrafficLaneStart> MapRenderer::trafficSpawnLanes() const {
    if (!sceneLoader_->roadTrafficLaneNetworkReady()) {
        return {};
    }
    return impl_->roadTrafficLaneNetwork.spawnLanes();
}

std::optional<openbus::map::MapTrafficLaneStart>
MapRenderer::selectTrafficSpawnLane(openbus::map::MapTrafficVehicleFilter vehicle,
                                    std::size_t selectionSeed) const {
    if (!sceneLoader_->roadTrafficLaneNetworkReady()) {
        return std::nullopt;
    }
    return impl_->roadTrafficLaneNetwork.selectSpawnLane(vehicle, selectionSeed);
}

std::optional<openbus::map::MapTrafficLaneStart> MapRenderer::selectTrafficSpawnLaneInTiles(
    openbus::map::MapTrafficVehicleFilter vehicle, std::size_t selectionSeed,
    const std::vector<std::pair<int, int>>& activeTiles) const {
    if (!sceneLoader_->roadTrafficLaneNetworkReady()) {
        return std::nullopt;
    }
    return impl_->roadTrafficLaneNetwork.selectSpawnLaneInTiles(vehicle, selectionSeed,
                                                                activeTiles);
}

bool MapRenderer::addTrafficActor(openbus::map::MapTrafficActor actor) {
    if (!openbus::map::advanceMapTrafficActor(actor, 0.0)) {
        return false;
    }
    impl_->trafficActors.push_back(std::move(actor));
    return true;
}

std::size_t MapRenderer::updateTrafficActors(double elapsedSeconds) {
    return openbus::map::advanceMapTrafficActors(impl_->trafficActors, elapsedSeconds);
}

const std::vector<openbus::map::MapTrafficActor>& MapRenderer::trafficActors() const {
    return impl_->trafficActors;
}

std::optional<double> MapRenderer::terrainHeightAt(double worldX, double worldY) const {
    if (!std::isfinite(worldX) || !std::isfinite(worldY)) {
        return std::nullopt;
    }
    const double tileXValue = std::floor(worldX / impl_->coordinates.tileSize);
    const double tileYValue = std::floor(worldY / impl_->coordinates.tileSize);
    if (tileXValue < static_cast<double>(std::numeric_limits<int>::min()) ||
        tileXValue > static_cast<double>(std::numeric_limits<int>::max()) ||
        tileYValue < static_cast<double>(std::numeric_limits<int>::min()) ||
        tileYValue > static_cast<double>(std::numeric_limits<int>::max())) {
        return std::nullopt;
    }
    const int tileX = static_cast<int>(tileXValue);
    const int tileY = static_cast<int>(tileYValue);
    const auto found = impl_->terrainByCoordinate.find(tileKey(tileX, tileY));
    if (found == impl_->terrainByCoordinate.end()) {
        return std::nullopt;
    }
    const double localX = worldX - tileXValue * impl_->coordinates.tileSize;
    const double localY = worldY - tileYValue * impl_->coordinates.tileSize;
    return openbus::map::sampleMapTerrainHeight(found->second, localX, localY,
                                                impl_->coordinates.tileSize);
}

} // namespace openbus::rendering
