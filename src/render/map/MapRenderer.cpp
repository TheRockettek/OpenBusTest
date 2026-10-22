// MapRenderer lifecycle, streaming and script updates. Loading lives in
// MapSceneLoader*.cpp / MapScene*.cpp and drawing in MapRendererDraw.cpp.

#include "render/map/MapRendererInternal.h"
#include "render/map/MapSceneLoader.h"
#include "render/AssetRequestManager.h"
#include "config/HumanConfigLoader.h"

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
                         const std::filesystem::path& omsiRoot, SimulationState& simulationState,
                         AssetRequestManager& assetRequests)
    : impl_(std::make_unique<Impl>()) {
    TraceScope trace("map", "MapRenderer::MapRenderer");
    assetRequests_ = &assetRequests;
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

std::vector<openbus::map::MapSplinePathOffset> MapRenderer::trafficSplinePathOffsets(
    int splineId, int pathTypeCode) const {
    const auto* pathOffsets =
        pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD
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

std::optional<openbus::map::MapTrafficActor>
MapRenderer::createTrafficActor(int splineId, std::string modelPath, double speedMetersPerSecond,
                                double laneOffsetMagnitudeMeters,
                                std::optional<std::string> driverModelPath,
                                std::optional<int> pathTypeCode,
                                std::size_t laneSelector) const {
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
    const std::string cacheKey =
        humanConfigPath.lexically_normal().generic_string();
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
                                     humanConfigPath.string());
        }
        const openbus::config::HumanConfig humanConfig =
            openbus::config::loadHumanConfig(humanConfigPath);
        if (humanConfig.diagnostics.hasErrors() || humanConfig.modelConfigPath.empty()) {
            throw std::runtime_error("Human traffic configuration is invalid: " +
                                     humanConfigPath.string());
        }
        const auto geometry = std::find_if(
            humanConfig.numericRecords.begin(), humanConfig.numericRecords.end(),
            [](const openbus::config::HumanNumericRecord& record) {
                return record.keyword == "humangeom" && record.valid &&
                       record.values.size() == 2 && std::isfinite(record.values[0]) &&
                       record.values[0] > 0.0 && std::isfinite(record.values[1]) &&
                       record.values[1] > 0.0;
            });
        if (geometry == humanConfig.numericRecords.end()) {
            throw std::runtime_error("Human traffic configuration has no valid [humangeom]: " +
                                     humanConfigPath.string());
        }
        const std::filesystem::path modelConfigPath =
            openbus::assets::resolveReferencedPath(humanConfigPath,
                                                   humanConfig.modelConfigPath);
        if (!std::filesystem::is_regular_file(modelConfigPath)) {
            throw std::runtime_error("Human traffic model configuration is missing: " +
                                     modelConfigPath.string());
        }
        openbus::scripting::SceneryObject variables;
        ModelConfig configuration = loadModelConfig(
            modelConfigPath, modelConfigPath.parent_path(), ModelConfigKind::SceneryObject,
            variables);
        if (configuration.diagnostics.hasErrors()) {
            throw std::runtime_error("Human traffic model configuration is invalid: " +
                                     modelConfigPath.string());
        }
        const bool hasFirstLodMesh = std::any_of(
            configuration.parts.begin(), configuration.parts.end(),
            [](const ModelPart& part) { return part.lodIndex == 0 && !part.objPath.empty(); });
        if (!hasFirstLodMesh) {
            throw std::runtime_error("Human traffic model has no first-LOD meshes: " +
                                     modelConfigPath.string());
        }
        return Impl::HumanModelConfiguration{
            modelConfigPath, std::move(configuration), {geometry->values[0], geometry->values[1]}};
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
            if (part.lodIndex == 0 && !part.objPath.empty()) {
                pendingModel.meshRequests.push_back(
                    assetRequests_->requestObj(part.objPath, part.bundleEntry));
            }
        }
        if (pendingModel.meshRequests.empty()) {
            throw std::runtime_error("Human traffic model has no first-LOD meshes: " +
                                     pendingModel.configuration->modelConfigPath.string());
        }
    }
    for (const auto& request : pendingModel.meshRequests) {
        if (request.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return false;
        }
    }
    const std::filesystem::path humanConfigPath = pendingModel.humanConfigPath;
    Impl::HumanModelConfiguration configuration =
        std::move(*pendingModel.configuration);
    impl_->pendingHumanTrafficModels.erase(pending);
    loadHumanTrafficModelNow(humanConfigPath, configuration.modelConfigPath,
                             configuration.configuration, configuration.geometry);
    return impl_->humanTrafficModels.contains(cacheKey);
}

void MapRenderer::loadHumanTrafficModelNow(
    const std::filesystem::path& humanConfigPath,
    const std::filesystem::path& modelConfigPath,
    const ModelConfig& configuration,
    const std::array<double, 2>& geometry) const {
    const std::string cacheKey =
        humanConfigPath.lexically_normal().generic_string();
    if (impl_->humanTrafficModels.contains(cacheKey)) {
        return;
    }
    const std::filesystem::path modelRoot = modelConfigPath.parent_path();
    for (const ConfigurationDiagnostic& diagnostic : configuration.diagnostics.entries) {
        if (diagnostic.severity == ConfigurationDiagnostic::Severity::Error) {
            gameLog.Log("Human model configuration error: " +
                        modelConfigPath.generic_string() + " line=" +
                        std::to_string(diagnostic.line) + " [" + diagnostic.keyword +
                        "] " + diagnostic.message);
        }
    }
    if (configuration.diagnostics.hasErrors()) {
        throw std::runtime_error("Human traffic model configuration is invalid: " +
                                 modelConfigPath.string());
    }

    struct ParsedHumanPart {
        const ModelPart* part = nullptr;
        std::shared_ptr<ParsedObj> mesh;
    };
    std::vector<ParsedHumanPart> parsedParts;
    std::array<double, 3> minimum = {
        std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
    std::array<double, 3> maximum = {
        -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
    for (const ModelPart& part : configuration.parts) {
        if (part.lodIndex > 0 || part.objPath.empty()) {
            continue;
        }
        try {
            std::string extension = part.objPath.extension().string();
            std::transform(extension.begin(), extension.end(), extension.begin(),
                           [](unsigned char value) {
                               return static_cast<char>(std::tolower(value));
                           });
            std::shared_ptr<ParsedObj> mesh;
            if (extension == ".o3d") {
                mesh = assetRequests_->requestObj(part.objPath, part.bundleEntry).get();
            } else if (extension == ".x") {
                mesh = assetRequests_->requestObj(part.objPath, part.bundleEntry).get();
            } else if (extension == ".obj") {
                mesh = assetRequests_->requestObj(part.objPath, part.bundleEntry).get();
            }
            if (!mesh) {
                gameLog.Log("Skipping unsupported or empty human model mesh: " +
                            part.objPath.generic_string());
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
            gameLog.Log("Could not load human model mesh " + part.objPath.generic_string() +
                        ": " + error.what());
        }
    }
    if (parsedParts.empty() || !std::isfinite(minimum[0]) ||
        !std::isfinite(minimum[1]) || !std::isfinite(minimum[2])) {
        throw std::runtime_error("Human traffic model has no loadable mesh: " +
                                 modelConfigPath.string());
    }
    if (pglGenBuffers == nullptr || pglBindBuffer == nullptr || pglBufferData == nullptr) {
        throw std::runtime_error("OpenGL buffer functions are unavailable for human models");
    }

    const std::array<double, 3> extent = {maximum[0] - minimum[0], maximum[1] - minimum[1],
                                          maximum[2] - minimum[2]};
    const double horizontalScale =
        extent[0] > 1.0e-6 ? geometry[0] / extent[0] : 1.0;
    const double verticalScale =
        extent[2] > 1.0e-6 ? geometry[1] / extent[2] : 1.0;
    const std::array<double, 2> horizontalCenter = {
        (minimum[0] + maximum[0]) * 0.5, (minimum[1] + maximum[1]) * 0.5};
    std::vector<Impl::ModelBatch> batches;
    std::vector<std::filesystem::path> texturePaths;
    std::vector<std::shared_future<std::shared_ptr<Image>>> textureImages;
    for (const ParsedHumanPart& parsedPart : parsedParts) {
        const ModelPart& part = *parsedPart.part;
        std::unordered_map<int, std::vector<ModelVertex>> verticesByMaterial;
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
                const std::size_t positionIndex =
                    static_cast<std::size_t>(index.position - 1);
                const ObjPosition& sourcePosition =
                    parsedPart.mesh->positions[positionIndex];
                const std::array<double, 4> vertexColor =
                    positionIndex < parsedPart.mesh->vertexColors.size()
                        ? parsedPart.mesh->vertexColors[positionIndex]
                        : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};
                std::array<double, 3> normal = {0.0, 0.0, 1.0};
                if (validIndex(index.normal, parsedPart.mesh->normals.size())) {
                    const ObjNormal& sourceNormal = parsedPart.mesh->normals[
                        static_cast<std::size_t>(index.normal - 1)];
                    normal = {sourceNormal.x, sourceNormal.y, sourceNormal.z};
                    const double normalLength =
                        std::hypot(normal[0], normal[1], normal[2]);
                    if (normalLength > 1.0e-8) {
                        for (double& component : normal) {
                            component /= normalLength;
                        }
                    }
                }
                float u = 0.0F;
                float v = 0.0F;
                if (validIndex(index.texCoord, parsedPart.mesh->texCoords.size())) {
                    const ObjTexCoord& coordinate = parsedPart.mesh->texCoords[
                        static_cast<std::size_t>(index.texCoord - 1)];
                    u = static_cast<float>(coordinate.u);
                    v = static_cast<float>(1.0 - coordinate.v);
                }
                verticesByMaterial[materialIndex].push_back(
                    {static_cast<float>((sourcePosition.x - horizontalCenter[0]) *
                                        horizontalScale),
                     static_cast<float>((sourcePosition.y - horizontalCenter[1]) *
                                        horizontalScale),
                     static_cast<float>((sourcePosition.z - minimum[2]) * verticalScale), u, v,
                     0.0F, static_cast<float>(normal[0]), static_cast<float>(normal[1]),
                     static_cast<float>(normal[2]), static_cast<float>(vertexColor[0]),
                     static_cast<float>(vertexColor[1]), static_cast<float>(vertexColor[2]),
                     static_cast<float>(vertexColor[3])});
            }
        }
        for (auto& [materialIndex, vertices] : verticesByMaterial) {
            if (vertices.empty()) {
                continue;
            }
            const auto sourceMaterial = std::find_if(
                parsedPart.mesh->materials.begin(), parsedPart.mesh->materials.end(),
                [materialIndex](const auto& item) {
                    return item.second.materialIndex == materialIndex;
                });
            const ModelMaterialState* materialState = nullptr;
            const auto state = std::find_if(
                part.materialStatesInOrder.begin(), part.materialStatesInOrder.end(),
                [materialIndex](const ModelMaterialState& item) {
                    return item.materialIndex == materialIndex;
                });
            if (state != part.materialStatesInOrder.end()) {
                materialState = &*state;
            }
            std::string textureName =
                sourceMaterial == parsedPart.mesh->materials.end()
                    ? std::string{}
                    : sourceMaterial->second.textureName;
            if (materialState != nullptr && !materialState->textureName.empty()) {
                textureName = materialState->textureName;
            }

            Impl::ModelBatch batch;
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
                const std::string textureKey =
                    texturePath.lexically_normal().generic_string();
                auto texture = impl_->dynamicSceneryTextures.find(textureKey);
                if (texture != impl_->dynamicSceneryTextures.end()) {
                    batch.material.texture = texture->second;
                    batch.material.textured = batch.material.texture != 0;
                } else {
                    auto request = impl_->humanTextureRequests.find(textureKey);
                    if (request == impl_->humanTextureRequests.end()) {
                        textureImage = assetRequests_->submit([texturePath] {
                            auto image = std::make_shared<Image>();
                            if (!TextureLoader::readImage(texturePath, *image)) {
                                throw std::runtime_error("Could not decode human model texture: " +
                                                         texturePath.string());
                            }
                            return image;
                        }).share();
                        request = impl_->humanTextureRequests
                                      .emplace(textureKey, std::move(textureImage))
                                      .first;
                    }
                    textureImage = request->second;
                }
            } else if (!textureName.empty()) {
                gameLog.Log("Human model texture was not found: " + textureName);
            }
            pglGenBuffers(1, &batch.buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(vertices.size() * sizeof(ModelVertex)),
                          vertices.data(), GL_STATIC_DRAW);
            if (batch.buffer == 0 || glGetError() != GL_NO_ERROR) {
                if (batch.buffer != 0) {
                    pglDeleteBuffers(1, &batch.buffer);
                }
                gameLog.Log("Could not upload human model mesh: " +
                            modelConfigPath.generic_string());
                continue;
            }
            texturePaths.push_back(texturePath);
            textureImages.push_back(std::move(textureImage));
            batches.push_back(std::move(batch));
        }
    }
    if (batches.empty()) {
        throw std::runtime_error("Human traffic model contains no renderable mesh: " +
                                 modelConfigPath.string());
    }

    Impl::HumanModelAsset asset;
    asset.batchIndexes.reserve(batches.size());
    asset.texturePaths = std::move(texturePaths);
    asset.textureImages = std::move(textureImages);
    impl_->sceneryBatches.reserve(impl_->sceneryBatches.size() + batches.size());
    for (Impl::ModelBatch& batch : batches) {
        asset.batchIndexes.push_back(impl_->sceneryBatches.size());
        impl_->sceneryBatches.push_back(std::move(batch));
    }
    impl_->humanTrafficModels.emplace(cacheKey, std::move(asset));
}

void MapRenderer::updateTrafficSignalControllers(
    const std::vector<openbus::map::MapTrafficActor*>& actors,
    double elapsedSeconds, double initialClockSeconds) const {
    openbus::map::advanceMapTrafficSignalControllers(
        impl_->trafficSignalControllers, actors, elapsedSeconds, initialClockSeconds);
}

std::optional<openbus::map::MapTrafficRoute>
MapRenderer::buildDrivingRoute(int splineId, std::size_t laneSelector, std::size_t routeSeed,
                               double minimumTurnRadiusMeters,
                               openbus::map::MapTrafficVehicleFilter vehicle,
                               const std::vector<int>& requiredSplineSequence) const {
    return impl_->roadTrafficLaneNetwork.buildRoute(splineId, laneSelector, routeSeed,
                                                    minimumTurnRadiusMeters, vehicle,
                                                    requiredSplineSequence);
}

std::vector<openbus::map::MapTrafficLaneStart> MapRenderer::trafficEntryLanes() const {
    return impl_->roadTrafficLaneNetwork.entryLanes();
}

std::optional<openbus::map::MapTrafficLaneStart> MapRenderer::selectTrafficEntryLane(
    openbus::map::MapTrafficVehicleFilter vehicle, std::size_t selectionSeed) const {
    return impl_->roadTrafficLaneNetwork.selectEntryLane(vehicle, selectionSeed);
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
