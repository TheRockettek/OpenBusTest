// MapRenderer lifecycle, streaming and script updates. Loading lives in
// MapSceneLoader*.cpp / MapScene*.cpp and drawing in MapRendererDraw.cpp.

#include "render/map/MapRendererInternal.h"
#include "render/map/MapSceneLoader.h"

namespace openbus::rendering {

MapRenderer::Impl::~Impl() {
    const auto deleteBuffer = [](GLuint buffer) {
        if (buffer != 0 && pglDeleteBuffers != nullptr) {
            pglDeleteBuffers(1, &buffer);
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
    for (const GLuint ownedTexture : ownedTextures) {
        if (ownedTexture != 0) {
            glDeleteTextures(1, &ownedTexture);
        }
    }
    if (texture != 0) {
        glDeleteTextures(1, &texture);
    }
}

MapRenderer::MapRenderer(const openbus::map::MapDefinition& map, std::size_t centerTileIndex,
                         double spawnWorldX, double spawnWorldY, std::size_t groundTextureIndex,
                         const std::filesystem::path& omsiRoot, SimulationState& simulationState)
    : impl_(std::make_unique<Impl>()) {
    TraceScope trace("map", "MapRenderer::MapRenderer");
    impl_->simulationState = &simulationState;
    impl_->omsiRoot = omsiRoot;
    impl_->leftHandTraffic = map.metadata.hasLeftHandTraffic;
    MapSceneLoader loader(map, centerTileIndex, groundTextureIndex, omsiRoot, *impl_);
    loader.load();
    updateStreaming(spawnWorldX, spawnWorldY);
    loader.logSummary();
}

MapRenderer::~MapRenderer() = default;

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
            gameLog.Log("Scenery script frame error: " + errors[index]);
        }
    }
}

void MapRenderer::initializeActiveScripts() {
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
            gameLog.Log("Scenery script initialization error: " + errors[index]);
        }
    }
}

void MapRenderer::updateStreaming(double worldX, double worldY) {
    const int centerX = openbus::map::mapWorldTile(worldX);
    const int centerY = openbus::map::mapWorldTile(worldY);
    const openbus::map::MapTileCoordinate center{centerX, centerY};
    if (impl_->activeCenterTile == center) {
        return;
    }
    impl_->activeCenterTile = center;
    impl_->activeTileCoordinates = openbus::map::selectMapTilesWithinRadius(
        impl_->mapTileCoordinates, worldX, worldY, MAP_ACTIVE_TILE_RADIUS);
    impl_->activeTileKeys.clear();
    impl_->activeTileKeys.reserve(impl_->activeTileCoordinates.size());
    MapRenderStats stats;
    stats.activeTiles = impl_->activeTileCoordinates.size();
    stats.totalTiles = impl_->mapTileCoordinates.size();
    stats.totalTerrainTiles = impl_->terrainBuffers.size();
    stats.totalSceneryObjects = impl_->sceneryObjectCount;
    stats.totalRoadBatches = impl_->roadBatches.size();
    const auto countForTile = [](const auto& counts, std::uint64_t key) {
        const auto found = counts.find(key);
        return found == counts.end() ? std::size_t{0} : found->second;
    };
    for (const auto& [tileX, tileY] : impl_->activeTileCoordinates) {
        const std::uint64_t key = tileKey(tileX, tileY);
        impl_->activeTileKeys.insert(key);
        stats.activeTerrainTiles += countForTile(impl_->terrainTilesByTile, key);
        stats.activeSceneryObjects += countForTile(impl_->sceneryObjectsByTile, key);
        stats.activeRoadBatches += countForTile(impl_->roadBatchesByTile, key);
    }
    impl_->renderStats = stats;
    for (auto& activeChunks : impl_->activeSceneryChunks) {
        activeChunks.clear();
    }
    for (const auto& [key, chunk] : impl_->sceneryChunks) {
        static_cast<void>(key);
        std::array<Impl::ActiveSceneryChunk, 2> active = {Impl::ActiveSceneryChunk{&chunk, {}},
                                                          Impl::ActiveSceneryChunk{&chunk, {}}};
        for (const Impl::SceneryInstance& instance : chunk.instances) {
            if (instance.batchIndex >= impl_->sceneryBatches.size() ||
                !impl_->activeTileKeys.contains(
                    tileKey(instance.sourceTileX, instance.sourceTileY))) {
                continue;
            }
            const bool typeFour = impl_->sceneryBatches[instance.batchIndex].renderType == 4;
            active[typeFour ? 1 : 0].instances.push_back(&instance);
        }
        for (std::size_t pass = 0; pass < active.size(); ++pass) {
            if (!active[pass].instances.empty()) {
                impl_->activeSceneryChunks[pass].push_back(std::move(active[pass]));
            }
        }
    }
    impl_->activeSceneryScriptStateIndices.clear();
    impl_->activeSceneryScriptStateIndices.reserve(impl_->sceneryScriptStates.size());
    for (std::size_t stateIndex = 0; stateIndex < impl_->sceneryScriptStates.size(); ++stateIndex) {
        const Impl::SceneryScriptState& scriptState = *impl_->sceneryScriptStates[stateIndex];
        if (impl_->activeTileKeys.contains(
                tileKey(scriptState.ownerTileX, scriptState.ownerTileY))) {
            impl_->activeSceneryScriptStateIndices.push_back(stateIndex);
        }
    }
    initializeActiveScripts();
    gameLog.Log("Map active render window: center=" + tileLabel(centerX, centerY) +
                ", active tiles=" + std::to_string(impl_->activeTileCoordinates.size()) +
                ", radius=" + std::to_string(MAP_ACTIVE_TILE_RADIUS) + " tiles.");
}

const std::vector<std::pair<int, int>>& MapRenderer::activeTileCoordinates() const {
    return impl_->activeTileCoordinates;
}

MapRenderStats MapRenderer::renderStats() const {
    return impl_->renderStats;
}

std::optional<openbus::map::MapTrafficRoute> MapRenderer::buildTrafficRoute(int splineId) const {
    return impl_->trafficRouteNetwork.buildRoute(splineId);
}

std::optional<openbus::map::MapTrafficRoute> MapRenderer::buildTrafficRoute(
    int splineId, int pathTypeCode) const {
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

std::optional<openbus::map::MapTrafficActor>
MapRenderer::createTrafficActor(int splineId, std::string modelPath, double speedMetersPerSecond,
                                double laneOffsetMagnitudeMeters,
                                std::optional<std::string> driverModelPath,
                                std::optional<int> pathTypeCode) const {
    std::optional<openbus::map::MapTrafficRoute> route;
    if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD) {
        route = impl_->roadTrafficRouteNetwork.buildRoute(splineId);
    } else if (pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
        route = impl_->humanTrafficRouteNetwork.buildRoute(splineId);
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
    const double tileXValue = std::floor(worldX / openbus::map::OMSI_TILE_SIZE_METERS);
    const double tileYValue = std::floor(worldY / openbus::map::OMSI_TILE_SIZE_METERS);
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
    const double localX = worldX - tileXValue * openbus::map::OMSI_TILE_SIZE_METERS;
    const double localY = worldY - tileYValue * openbus::map::OMSI_TILE_SIZE_METERS;
    return openbus::map::sampleMapTerrainHeight(found->second, localX, localY);
}

} // namespace openbus::rendering
