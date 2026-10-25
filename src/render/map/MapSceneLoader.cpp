// Map load orchestration: ground textures, terrain, map indexing and road upload.

#include "render/map/MapSceneLoader.h"

namespace openbus::rendering {

MapSceneLoader::MapSceneLoader(const openbus::map::MapDefinition& mapDefinition,
                               std::size_t centerTile, std::size_t groundTexture,
                               const std::filesystem::path& omsiRootDirectory,
                               MapRenderer::Impl& impl, AssetRequestManager& assetRequests)
    : map(mapDefinition), centerTileIndex(centerTile), groundTextureIndex(groundTexture),
      omsiRoot(omsiRootDirectory), impl_(&impl), assetRequests_(&assetRequests),
      terrainByCoordinate(impl.terrainByCoordinate),
      verboseAlphaMaterials(
          parseEnabledFlag(openbus::getEnvironment("OPENBUS_VERBOSE_TEXTURE_READ"))),
      roadConnections(mapDefinition.coordinates) {
    splineCantWidth =
        [this](const openbus::map::MapSplinePlacement& spline) -> std::optional<double> {
        const auto found = splineCantWidths.find(spline.assetPath);
        if (found != splineCantWidths.end()) {
            return found->second;
        }
        const auto* profile =
            cachedSplineProfile(resolveSplineProfile(map, omsiRoot, spline.assetPath));
        const std::optional<double> width =
            profile != nullptr ? openbus::map::mapSplineProfileCantHalfWidth(*profile)
                               : std::nullopt;
        splineCantWidths.emplace(spline.assetPath, width);
        return width;
    };
    mapSplineResolver =
        [this](int splineId) -> std::optional<openbus::map::MapSplinePlacementReference> {
        const auto found = mapSplinesById.find(splineId);
        if (found == mapSplinesById.end()) {
            return std::nullopt;
        }
        return found->second;
    };
}

void MapSceneLoader::load() {
    gameLog.Log("Map scene loading started: root=" + map.rootPath.string() +
                ", tiles=" + std::to_string(map.tiles.size()) + ".");
    gameLog.Log("Map phase: loading ground textures.");
    loadGroundTextures();
    if (std::none_of(
            map.tiles.begin(), map.tiles.end(),
            [](const openbus::map::MapTileReference& tile) { return tile.hasTerrainFile; })) {
        throw std::runtime_error("No terrain files found in the selected map");
    }
    gameLog.Log("Map phase: indexing tile placements and traffic paths.");
    assetRequests_->submit([this] { indexMapContent(); }).get();
}

std::optional<std::vector<std::string>>
MapSceneLoader::requestTileSceneryAssets(const openbus::map::MapTileData& tile,
                                         std::uint64_t tileKeyValue) {
    std::vector<std::string> assetKeys;
    std::vector<std::pair<std::string, std::filesystem::path>> configPaths;
    std::unordered_set<std::string> requestedKeys;
    std::unordered_set<int> visitedObjectIds;
    const auto addAsset = [&](const std::string& assetPath) {
        const std::filesystem::path configPath = resolveSceneryConfigPath(assetPath);
        if (configPath.empty()) {
            return;
        }
        const std::string key = configPath.lexically_normal().generic_string();
        if (requestedKeys.insert(key).second) {
            assetKeys.push_back(key);
            if (!sceneryAssetBatches.contains(key) && !preparedSceneryAssetConfigs.contains(key)) {
                configPaths.emplace_back(key, configPath);
            }
        }
    };
    const auto addParentChain = [&](int parentId) {
        while (parentId != 0 && visitedObjectIds.insert(parentId).second) {
            const auto asset = mapObjectAssetPathsById.find(parentId);
            if (asset != mapObjectAssetPathsById.end()) {
                addAsset(asset->second);
            }
            const auto attached = attachedObjectsById.find(parentId);
            if (attached == attachedObjectsById.end()) {
                break;
            }
            parentId = attached->second.parentId;
        }
    };
    for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
        addAsset(object.assetPath);
    }
    for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
        addAsset(object.assetPath);
        addParentChain(object.attachedToObjectId);
    }
    for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
        addAsset(attachment.assetPath);
    }
    std::sort(assetKeys.begin(), assetKeys.end());
    constexpr std::size_t maxPendingSceneryTilePreparations = 8;
    if (!configPaths.empty() && !pendingTileSceneryRequests.contains(tileKeyValue) &&
        pendingTileSceneryRequests.size() >= maxPendingSceneryTilePreparations) {
        return std::nullopt;
    }
    if (!configPaths.empty() && !pendingTileSceneryRequests.contains(tileKeyValue)) {
        pendingTileSceneryRequests.emplace(
            tileKeyValue, assetRequests_->submit([configPaths = std::move(configPaths)]() mutable {
                std::vector<std::pair<std::string, PreparedSceneryAsset>> preparedAssets;
                preparedAssets.reserve(configPaths.size());
                for (const auto& [key, configPath] : configPaths) {
                    PreparedSceneryAsset prepared;
                    try {
                        prepared = prepareSceneryAsset(configPath);
                    } catch (...) {
                        prepared.preparationError = std::current_exception();
                    }
                    preparedAssets.emplace_back(key, std::move(prepared));
                }
                return preparedAssets;
            }));
    }
    return assetKeys;
}

bool MapSceneLoader::completeSceneryAssetPreparation(std::uint64_t tileKeyValue,
                                                     bool waitForCompletion) {
    auto request = pendingTileSceneryRequests.find(tileKeyValue);
    if (request == pendingTileSceneryRequests.end()) {
        return true;
    }
    if (waitForCompletion) {
        request->second.wait();
    } else if (request->second.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return false;
    }
    std::vector<std::pair<std::string, PreparedSceneryAsset>> preparedAssets =
        request->second.get();
    pendingTileSceneryRequests.erase(request);
    if (pendingTileAssetKeys.contains(tileKeyValue)) {
        for (auto& [key, prepared] : preparedAssets) {
            if (!sceneryAssetBatches.contains(key) && !preparedSceneryAssetConfigs.contains(key)) {
                preparedSceneryAssetConfigs.emplace(key, std::move(prepared));
            }
        }
    }
    return true;
}

bool MapSceneLoader::sceneryAssetPreparationsReady(
    const std::vector<std::string>& assetKeys) const {
    return std::all_of(assetKeys.begin(), assetKeys.end(), [this](const std::string& key) {
        return sceneryAssetBatches.contains(key) || preparedSceneryAssetConfigs.contains(key);
    });
}

MapSceneLoader::PreparedSceneryAsset
MapSceneLoader::prepareSceneryAsset(const std::filesystem::path& configPath) {
    PreparedSceneryAsset prepared;
    prepared.objectConfiguration = loadSceneryObjectFile(configPath);
    prepared.modelConfigPath = resolveSceneryObjectModelConfigPath(prepared.objectConfiguration);
    prepared.usesExternalModelConfig = !prepared.objectConfiguration.modelPath.empty();
    prepared.externalModelConfigPath = prepared.modelConfigPath;
    if (prepared.usesExternalModelConfig &&
        !std::filesystem::is_regular_file(prepared.modelConfigPath)) {
        prepared.externalModelConfigMissing = true;
        prepared.modelConfigPath = configPath;
        prepared.usesExternalModelConfig = false;
    }
    prepared.modelRoot = prepared.usesExternalModelConfig ? prepared.modelConfigPath.parent_path()
                                                          : configPath.parent_path() / "model";
    if (!prepared.objectConfiguration.onlyEditor || !prepared.objectConfiguration.trees.empty()) {
        if (prepared.objectConfiguration.trees.empty()) {
            prepared.modelConfiguration =
                loadModelConfig(prepared.modelConfigPath, prepared.modelRoot,
                                ModelConfigKind::SceneryObject, prepared.variables);
            prepared.hasModelConfiguration = true;
            for (const ModelPart& part : prepared.modelConfiguration.parts) {
                if (part.objPath.empty()) {
                    continue;
                }
                std::string extension = part.objPath.extension().string();
                std::transform(
                    extension.begin(), extension.end(), extension.begin(),
                    [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
                if (extension == ".o3d" || extension == ".x") {
                    const std::string meshKey = part.objPath.lexically_normal().generic_string();
                    if (!prepared.meshes.contains(meshKey)) {
                        PreparedSceneryMesh mesh;
                        try {
                            mesh = prepareSceneryMesh(part.objPath);
                        } catch (...) {
                            mesh.preparationError = std::current_exception();
                        }
                        prepared.meshes.emplace(meshKey, std::move(mesh));
                    }
                }
            }
        }
    }
    return prepared;
}

MapSceneLoader::PreparedSceneryMesh
MapSceneLoader::prepareSceneryMesh(const std::filesystem::path& meshPath) {
    PreparedSceneryMesh prepared;
    std::string extension = meshPath.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    prepared.parsed = extension == ".x" ? XLoader::parse(meshPath) : O3DLoader::parse(meshPath);
    if (!prepared.parsed) {
        return prepared;
    }
    for (const ObjTriangle& triangle : prepared.parsed->triangles) {
        const auto sourceMaterial = prepared.parsed->materials.find(triangle.material);
        const int materialIndex = sourceMaterial == prepared.parsed->materials.end()
                                      ? 0
                                      : sourceMaterial->second.materialIndex;
        for (const ObjIndex& index : triangle.indices) {
            const auto validIndex = [](int value, std::size_t count) {
                return value > 0 && static_cast<std::size_t>(value) <= count;
            };
            if (!validIndex(index.position, prepared.parsed->positions.size())) {
                ++prepared.invalidPositionIndices;
                continue;
            }
            const std::size_t positionIndex = static_cast<std::size_t>(index.position - 1);
            const ObjPosition& sourcePosition = prepared.parsed->positions[positionIndex];
            const std::array<double, 4> vertexColor =
                positionIndex < prepared.parsed->vertexColors.size()
                    ? prepared.parsed->vertexColors[positionIndex]
                    : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};
            std::array<double, 3> normal = {0.0, 0.0, 1.0};
            if (validIndex(index.normal, prepared.parsed->normals.size())) {
                const ObjNormal& sourceNormal =
                    prepared.parsed->normals[static_cast<std::size_t>(index.normal - 1)];
                // O3DLoader already converts axes and handedness for map scenery.
                normal = {sourceNormal.x, sourceNormal.y, sourceNormal.z};
                const double length = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] +
                                                normal[2] * normal[2]);
                if (length > 1.0e-8) {
                    for (double& component : normal) {
                        component /= length;
                    }
                }
            }
            float u = 0.0F;
            float v = 0.0F;
            if (validIndex(index.texCoord, prepared.parsed->texCoords.size())) {
                const ObjTexCoord& coordinate =
                    prepared.parsed->texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                u = static_cast<float>(coordinate.u);
                v = static_cast<float>(1.0 - coordinate.v);
            }
            prepared.verticesByMaterial[materialIndex].push_back(
                {static_cast<float>(sourcePosition.x), static_cast<float>(sourcePosition.y),
                 static_cast<float>(sourcePosition.z), u, v, 0.0F, static_cast<float>(normal[0]),
                 static_cast<float>(normal[1]), static_cast<float>(normal[2]),
                 static_cast<float>(vertexColor[0]), static_cast<float>(vertexColor[1]),
                 static_cast<float>(vertexColor[2]), static_cast<float>(vertexColor[3])});
        }
    }
    return prepared;
}

MapSceneLoader::TileLoadStatus
MapSceneLoader::loadActiveTiles(const std::vector<std::pair<int, int>>& tileCoordinates,
                                const openbus::map::MapTileCoordinate& priorityCenter,
                                bool waitForCompletion) {
    std::unordered_set<std::uint64_t> activeTileKeys;
    activeTileKeys.reserve(tileCoordinates.size());
    for (const auto& [tileX, tileY] : tileCoordinates) {
        activeTileKeys.insert(tileKey(tileX, tileY));
    }

    std::unordered_set<std::uint64_t> terrainTileKeys = activeTileKeys;
    for (const auto& [tileX, tileY] : tileCoordinates) {
        for (int offsetY = -1; offsetY <= 1; ++offsetY) {
            for (int offsetX = -1; offsetX <= 1; ++offsetX) {
                terrainTileKeys.insert(tileKey(tileX + offsetX, tileY + offsetY));
            }
        }
    }
    for (auto request = pendingTileDataRequests.begin();
         request != pendingTileDataRequests.end();) {
        if (!activeTileKeys.contains(request->first)) {
            request = pendingTileDataRequests.erase(request);
        } else {
            ++request;
        }
    }
    for (auto request = pendingTileContentData.begin(); request != pendingTileContentData.end();) {
        if (!activeTileKeys.contains(request->first)) {
            pendingTileAssetKeys.erase(request->first);
            request = pendingTileContentData.erase(request);
        } else {
            ++request;
        }
    }
    std::vector<std::uint64_t> sceneryPreparations;
    sceneryPreparations.reserve(pendingTileSceneryRequests.size());
    for (const auto& [key, request] : pendingTileSceneryRequests) {
        static_cast<void>(request);
        sceneryPreparations.push_back(key);
    }
    for (const std::uint64_t key : sceneryPreparations) {
        completeSceneryAssetPreparation(key, false);
    }
    for (auto request = pendingTerrainRequests.begin(); request != pendingTerrainRequests.end();) {
        if (!terrainTileKeys.contains(request->first)) {
            request = pendingTerrainRequests.erase(request);
        } else {
            ++request;
        }
    }
    std::vector<const openbus::map::MapTileReference*> prioritizedActiveTiles;
    for (const openbus::map::MapTileReference& tileReference : map.tiles) {
        if (activeTileKeys.contains(tileKey(tileReference.x, tileReference.y)) &&
            !loadedContentTileKeys.contains(tileKey(tileReference.x, tileReference.y))) {
            prioritizedActiveTiles.push_back(&tileReference);
        }
    }
    const auto tileDistanceSquared = [&](const auto* tile) {
        const long long x = static_cast<long long>(tile->x) - priorityCenter.first;
        const long long y = static_cast<long long>(tile->y) - priorityCenter.second;
        return x * x + y * y;
    };
    std::sort(prioritizedActiveTiles.begin(), prioritizedActiveTiles.end(),
              [&](const auto* left, const auto* right) {
                  const long long leftDistance = tileDistanceSquared(left);
                  const long long rightDistance = tileDistanceSquared(right);
                  return leftDistance != rightDistance
                             ? leftDistance < rightDistance
                             : std::pair{left->x, left->y} < std::pair{right->x, right->y};
              });
    constexpr std::size_t maxPendingTileParses = 8;
    for (const openbus::map::MapTileReference* tileReference : prioritizedActiveTiles) {
        const std::uint64_t key = tileKey(tileReference->x, tileReference->y);
        if (pendingTileDataRequests.contains(key) ||
            (!waitForCompletion && pendingTileDataRequests.size() >= maxPendingTileParses)) {
            continue;
        }
        pendingTileDataRequests.emplace(key,
                                        assetRequests_->submit([tileReference = *tileReference] {
                                            return openbus::map::loadMapTile(tileReference);
                                        }));
    }

    gameLog.Log("Map phase: loading terrain for the active tile window.");
    bool contentChanged = loadTerrainTiles(terrainTileKeys, priorityCenter, waitForCompletion);
    const bool terrainReady =
        std::all_of(map.tiles.begin(), map.tiles.end(), [this, &terrainTileKeys](const auto& tile) {
            const std::uint64_t key = tileKey(tile.x, tile.y);
            return !terrainTileKeys.contains(key) || loadedTerrainTileKeys.contains(key);
        });
    if (!terrainReady) {
        return TileLoadStatus::Pending;
    }
    for (const openbus::map::MapTileReference* tileReference : prioritizedActiveTiles) {
        const std::uint64_t key = tileKey(tileReference->x, tileReference->y);
        if (!activeTileKeys.contains(key) || loadedContentTileKeys.contains(key)) {
            continue;
        }
        auto tileData = pendingTileContentData.find(key);
        if (tileData == pendingTileContentData.end()) {
            auto request = pendingTileDataRequests.find(key);
            if (request == pendingTileDataRequests.end()) {
                throw std::logic_error("Active map tile has no pending parse request");
            }
            if (waitForCompletion) {
                request->second.wait();
            } else if (request->second.wait_for(std::chrono::seconds(0)) !=
                       std::future_status::ready) {
                continue;
            }
            openbus::map::MapTileData parsedTile = request->second.get();
            pendingTileDataRequests.erase(request);
            tileData = pendingTileContentData.emplace(key, std::move(parsedTile)).first;
        }
        auto assetKeys = pendingTileAssetKeys.find(key);
        if (assetKeys == pendingTileAssetKeys.end()) {
            std::optional<std::vector<std::string>> requestedAssets =
                requestTileSceneryAssets(tileData->second, key);
            if (!requestedAssets.has_value()) {
                continue;
            }
            assetKeys = pendingTileAssetKeys.emplace(key, std::move(*requestedAssets)).first;
        }
        if (assetKeys != pendingTileAssetKeys.end() &&
            !sceneryAssetPreparationsReady(assetKeys->second)) {
            if (waitForCompletion) {
                completeSceneryAssetPreparation(key, true);
            } else {
                continue;
            }
        }
        gameLog.Log("Map phase: loading active tile " +
                    tileLabel(tileReference->x, tileReference->y) + ".");
        buildTileContent(*tileReference, tileData->second);
        loadedContentTileKeys.insert(key);
        pendingTileContentData.erase(tileData);
        pendingTileAssetKeys.erase(key);
        contentChanged = true;
        uploadRoadBatches();
        if (!waitForCompletion) {
            break;
        }
    }
    if (!contentChanged) {
        const bool activeTilesReady = std::all_of(
            tileCoordinates.begin(), tileCoordinates.end(), [this](const auto& coordinate) {
                return loadedContentTileKeys.contains(tileKey(coordinate.first, coordinate.second));
            });
        return activeTilesReady ? TileLoadStatus::Ready : TileLoadStatus::Pending;
    }
    if (roadTrafficLaneNetworkDirty || humanTrafficLaneNetworkDirty) {
        gameLog.Log("Map phase: finalizing traffic lane networks.");
        if (roadTrafficLaneNetworkDirty) {
            impl_->roadTrafficLaneNetwork.finalize();
            roadTrafficLaneNetworkDirty = false;
        }
        if (humanTrafficLaneNetworkDirty) {
            impl_->humanTrafficLaneNetwork.finalize();
            humanTrafficLaneNetworkDirty = false;
        }
        const openbus::map::MapTrafficLaneNetworkStats laneStats =
            impl_->roadTrafficLaneNetwork.stats();
        gameLog.Log("Map road lane network: lanes=" + std::to_string(laneStats.lanes) +
                    ", unconnected lane ends=" + std::to_string(laneStats.deadEnds) +
                    ", unconnected ends within 3 m of an aligned lane start=" +
                    std::to_string(laneStats.nearMissDeadEnds) + ".");
    }
    resolveTrafficLightVariableParents();
    const bool activeTilesReady =
        std::all_of(tileCoordinates.begin(), tileCoordinates.end(), [this](const auto& coordinate) {
            return loadedContentTileKeys.contains(tileKey(coordinate.first, coordinate.second));
        });
    return activeTilesReady ? TileLoadStatus::Updated : TileLoadStatus::Pending;
}

void MapSceneLoader::resolveTrafficLightVariableParents() {
    std::unordered_map<int, const Impl::SceneryScriptState*> statesByObjectId;
    std::unordered_set<int> ambiguousStateObjectIds = ambiguousMapObjectIds;
    for (const std::unique_ptr<Impl::SceneryScriptState>& scriptState :
         impl_->sceneryScriptStates) {
        if (!scriptState->mapObjectId.has_value() ||
            ambiguousStateObjectIds.contains(*scriptState->mapObjectId)) {
            continue;
        }
        const auto [existing, inserted] =
            statesByObjectId.try_emplace(*scriptState->mapObjectId, scriptState.get());
        if (!inserted) {
            statesByObjectId.erase(existing);
            ambiguousStateObjectIds.insert(*scriptState->mapObjectId);
        }
    }

    for (const std::unique_ptr<Impl::SceneryScriptState>& scriptState :
         impl_->sceneryScriptStates) {
        if (!scriptState->trafficLightModel || scriptState->trafficSignal.has_value() ||
            !scriptState->variableParentId.has_value()) {
            continue;
        }
        const auto parentState = statesByObjectId.find(*scriptState->variableParentId);
        if (!ambiguousStateObjectIds.contains(*scriptState->variableParentId) &&
            parentState != statesByObjectId.end()) {
            const auto& parentSchedules = parentState->second->trafficSignalsByLightIndex;
            if (parentSchedules.size() == 1) {
                scriptState->trafficSignal = parentSchedules.begin()->second;
                continue;
            }
            for (const std::string& label : scriptState->placementLabels) {
                int lightIndex = -1;
                const auto [end, error] =
                    std::from_chars(label.data(), label.data() + label.size(), lightIndex);
                if (error != std::errc{} || end != label.data() + label.size() || lightIndex < 0) {
                    continue;
                }
                const auto schedule = parentSchedules.find(static_cast<std::size_t>(lightIndex));
                if (schedule != parentSchedules.end()) {
                    scriptState->trafficSignal = schedule->second;
                    break;
                }
            }
        }
        if (!scriptState->trafficSignal.has_value()) {
            const auto parentTile = mapObjectTileKeysById.find(*scriptState->variableParentId);
            if (parentTile != mapObjectTileKeysById.end() &&
                !ambiguousStateObjectIds.contains(*scriptState->variableParentId) &&
                !loadedContentTileKeys.contains(parentTile->second)) {
                continue;
            }
            const std::string selection =
                scriptState->placementLabels.empty()
                    ? "no light-index label"
                    : "light-index labels=" + scriptState->placementLabels.front();
            recordSkipped(skipped, "[trafficlight] variable parents without a matching schedule",
                          "object ID=" +
                              (scriptState->mapObjectId.has_value()
                                   ? std::to_string(*scriptState->mapObjectId)
                                   : std::string("unknown")) +
                              " parentObjectId=" + std::to_string(*scriptState->variableParentId) +
                              " " + selection);
        }
    }
}

void MapSceneLoader::loadGroundTextures() {
    if (map.groundTextures.empty()) {
        throw std::runtime_error("Map has no [groundtex] records");
    }
    if (centerTileIndex >= map.tiles.size()) {
        throw std::runtime_error("Map spawn references an invalid tile index");
    }
    if (groundTextureIndex >= map.groundTextures.size()) {
        throw std::runtime_error("Map ground-texture preview references an invalid [groundtex] "
                                 "index");
    }
    if (pglGenBuffers == nullptr || pglBindBuffer == nullptr || pglBufferData == nullptr) {
        throw std::runtime_error("OpenGL buffer functions are unavailable for map terrain");
    }
    impl_->mapTileCoordinates.reserve(map.tiles.size());
    for (const openbus::map::MapTileReference& tile : map.tiles) {
        impl_->mapTileCoordinates.emplace_back(tile.x, tile.y);
    }

    const char* groundTextureOverride = openbus::getEnvironment("OPENBUS_MAP_GROUNDTEX");
    uniformGroundTexturePreview =
        groundTextureOverride != nullptr && *groundTextureOverride != '\0';
    baseGroundTextureIndex = uniformGroundTexturePreview ? groundTextureIndex : 0;
    baseTextureRepeatsPerTile =
        openbus::map::mapGroundTextureTiling(map.groundTextures[baseGroundTextureIndex])
            .textureRepeatsPerTile;
    const std::filesystem::path texturePath =
        groundTexturePath(map, omsiRoot, map.groundTextures[baseGroundTextureIndex].texturePath);
    if (uniformGroundTexturePreview) {
        gameLog.Log("Map base-ground-texture preview override: [groundtex] index=" +
                    std::to_string(groundTextureIndex) + ", path=" + texturePath.generic_string() +
                    "; applied uniformly.");
    } else {
        gameLog.Log("Map base-ground-texture: [groundtex] index=0, path=" +
                    texturePath.generic_string() + "; per-tile ground-texture masks enabled.");
    }
    {
        TraceScope phase("map", "MapRenderer.loadGroundTexture");
        impl_->texture = loadTexture(texturePath);
    }

    terrainByCoordinate.reserve(map.tiles.size());

    if (!uniformGroundTexturePreview) {
        const std::string& detailTextureName =
            map.groundTextures[baseGroundTextureIndex].detailTexturePath;
        if (!detailTextureName.empty()) {
            try {
                impl_->baseDetailTexture =
                    getTexture(groundTexturePath(map, omsiRoot, detailTextureName));
            } catch (const std::exception& error) {
                recordSkipped(skipped, "base ground-texture detail textures that failed to load",
                              detailTextureName + ": " + error.what());
            }
        }
    }
    const openbus::map::MapTileReference& center = map.tiles[centerTileIndex];
    gameLog.Log("Map loading started: root=" + map.rootPath.generic_string() +
                ", tiles=" + std::to_string(map.tiles.size()) +
                ", render tiles load on demand, render visibility=" +
                std::to_string(static_cast<int>(MAP_VISIBILITY_RADIUS_METERS)) +
                " m, spawn=" + tileLabel(center.x, center.y) + ".");
}

bool MapSceneLoader::loadTerrainTiles(const std::unordered_set<std::uint64_t>& tileKeys,
                                      const openbus::map::MapTileCoordinate& priorityCenter,
                                      bool waitForCompletion) {
    const auto terrainLoadStart = std::chrono::steady_clock::now();
    std::size_t terrainTilesVisited = 0;
    std::size_t terrainUploads = 0;
    bool terrainChanged = false;
    {
        TraceScope phase("map", "MapRenderer.loadActiveTerrainTiles");
        std::vector<const openbus::map::MapTileReference*> prioritizedTerrainTiles;
        for (const openbus::map::MapTileReference& tile : map.tiles) {
            if (tileKeys.contains(tileKey(tile.x, tile.y)) &&
                !loadedTerrainTileKeys.contains(tileKey(tile.x, tile.y))) {
                prioritizedTerrainTiles.push_back(&tile);
            }
        }
        const auto tileDistanceSquared = [&](const auto* tile) {
            const long long x = static_cast<long long>(tile->x) - priorityCenter.first;
            const long long y = static_cast<long long>(tile->y) - priorityCenter.second;
            return x * x + y * y;
        };
        std::sort(prioritizedTerrainTiles.begin(), prioritizedTerrainTiles.end(),
                  [&](const auto* left, const auto* right) {
                      const long long leftDistance = tileDistanceSquared(left);
                      const long long rightDistance = tileDistanceSquared(right);
                      return leftDistance != rightDistance
                                 ? leftDistance < rightDistance
                                 : std::pair{left->x, left->y} < std::pair{right->x, right->y};
                  });
        constexpr std::size_t maxPendingTerrainPreparations = 8;
        for (const openbus::map::MapTileReference* tileReference : prioritizedTerrainTiles) {
            const openbus::map::MapTileReference& tile = *tileReference;
            const std::uint64_t key = tileKey(tile.x, tile.y);
            ++terrainTilesVisited;
            if (!tile.hasTerrainFile) {
                loadedTerrainTileKeys.insert(key);
                recordSkipped(skipped, "listed map tiles without a .terrain sidecar",
                              tileLabel(tile.x, tile.y) + " " + tile.textPath.filename().string());
                terrainChanged = true;
                continue;
            }
            auto request = pendingTerrainRequests.find(key);
            if (request == pendingTerrainRequests.end() &&
                (waitForCompletion ||
                 pendingTerrainRequests.size() < maxPendingTerrainPreparations)) {
                const std::filesystem::path terrainPath = tile.terrainPath;
                const double tileSize = map.coordinates.tileSize;
                request =
                    pendingTerrainRequests
                        .emplace(key, assetRequests_->submit([terrainPath, tileSize] {
                            PreparedTerrainTile prepared;
                            prepared.grid = openbus::map::loadTerrainGrid(terrainPath);
                            const auto& grid = prepared.grid;
                            const std::size_t side = grid.intervals + 1;
                            const double step = tileSize / grid.intervals;
                            prepared.vertices.reserve(side * side);
                            for (std::size_t row = 0; row < side; ++row) {
                                for (std::size_t column = 0; column < side; ++column) {
                                    prepared.vertices.push_back({static_cast<float>(column * step),
                                                                 static_cast<float>(row * step),
                                                                 grid.heights[row * side + column],
                                                                 static_cast<float>(column),
                                                                 static_cast<float>(row), 0.0F,
                                                                 0.0F, 0.0F, 1.0F});
                                }
                            }
                            prepared.indices.reserve(grid.intervals * grid.intervals * 6);
                            for (std::size_t row = 0; row < grid.intervals; ++row) {
                                for (std::size_t column = 0; column < grid.intervals; ++column) {
                                    const auto topLeft =
                                        static_cast<std::uint32_t>(row * side + column);
                                    const auto topRight =
                                        static_cast<std::uint32_t>(row * side + column + 1);
                                    const auto bottomRight =
                                        static_cast<std::uint32_t>((row + 1) * side + column + 1);
                                    const auto bottomLeft =
                                        static_cast<std::uint32_t>((row + 1) * side + column);
                                    // Match the renderer's GL_CW terrain winding.
                                    prepared.indices.insert(prepared.indices.end(),
                                                            {topLeft, bottomRight, topRight,
                                                             topLeft, bottomLeft, bottomRight});
                                }
                            }
                            const auto [minimum, maximum] =
                                std::minmax_element(grid.heights.begin(), grid.heights.end());
                            prepared.minimumHeight = *minimum;
                            prepared.maximumHeight = *maximum;
                            return prepared;
                        }))
                        .first;
            }
            if (request == pendingTerrainRequests.end()) {
                continue;
            }
            if (waitForCompletion) {
                request->second.wait();
            } else if (request->second.wait_for(std::chrono::seconds(0)) !=
                       std::future_status::ready) {
                continue;
            }
            if (!waitForCompletion && terrainUploads >= 1) {
                continue;
            }
            PreparedTerrainTile prepared = request->second.get();
            pendingTerrainRequests.erase(request);
            gameLog.Log("Map terrain tile " + std::to_string(terrainTilesVisited) + ": " +
                        tile.terrainPath.string() +
                        "; meshes uploaded=" + std::to_string(impl_->terrainBuffers.size()) +
                        ", elapsed=" + std::to_string(elapsedMilliseconds(terrainLoadStart)) +
                        " ms.");
            const auto [terrain, inserted] =
                terrainByCoordinate.emplace(key, std::move(prepared.grid));
            static_cast<void>(inserted);
            const openbus::map::TerrainGrid& grid = terrain->second;
            Impl::TileBuffer buffer;
            buffer.indexCount = prepared.indices.size();
            buffer.tileX = tile.x;
            buffer.tileY = tile.y;
            const float inverseIntervals = 1.0F / static_cast<float>(grid.intervals);
            const float baseTextureScale =
                static_cast<float>(baseTextureRepeatsPerTile) * inverseIntervals;
            buffer.baseTextureScale = {baseTextureScale, baseTextureScale};
            if (impl_->baseDetailTexture != 0) {
                const double baseDetailRepeatsPerTile =
                    openbus::map::mapGroundTextureTiling(map.groundTextures[baseGroundTextureIndex])
                        .detailTextureRepeatsPerTile;
                const float baseDetailScale =
                    static_cast<float>(baseDetailRepeatsPerTile) * inverseIntervals;
                buffer.baseDetailTextureScale = {baseDetailScale, baseDetailScale};
            }
            if (!uniformGroundTexturePreview) {
                std::map<std::size_t, const openbus::map::MapGroundTextureSidecar*> layersByIndex;
                for (const openbus::map::MapGroundTextureSidecar& sidecar :
                     tile.groundTextureSidecars) {
                    const std::optional<std::size_t> resolvedIndex =
                        openbus::map::resolveMapGroundTextureSidecarIndex(
                            sidecar.ordinalSuffix, map.groundTextures.size());
                    if (!resolvedIndex) {
                        recordSkipped(skipped,
                                      "ground-texture sidecars with invalid groundtex indices",
                                      tile.textPath.filename().string() +
                                          " sidecar=" + sidecar.path.filename().string());
                        continue;
                    }
                    const std::size_t textureIndex = *resolvedIndex;
                    if (!layersByIndex.emplace(textureIndex, &sidecar).second) {
                        recordSkipped(skipped, "duplicate ground-texture sidecar indices",
                                      tile.textPath.filename().string() +
                                          " index=" + std::to_string(textureIndex));
                    }
                }
                for (const auto& [textureIndex, sidecar] : layersByIndex) {
                    try {
                        ModelMaterial layer;
                        const std::filesystem::path layerTexturePath = groundTexturePath(
                            map, omsiRoot, map.groundTextures[textureIndex].texturePath);
                        layer.texture = getTexture(layerTexturePath);
                        layer.textured = true;
                        const openbus::map::MapGroundTextureTiling tiling =
                            openbus::map::mapGroundTextureTiling(map.groundTextures[textureIndex]);
                        const double layerRepeatsPerTile = tiling.textureRepeatsPerTile;
                        const float layerTextureScale =
                            static_cast<float>(layerRepeatsPerTile) * inverseIntervals;
                        layer.textureScale = {layerTextureScale, layerTextureScale};
                        const std::string& detailTextureName =
                            map.groundTextures[textureIndex].detailTexturePath;
                        if (!detailTextureName.empty()) {
                            try {
                                const std::filesystem::path detailTexturePath =
                                    groundTexturePath(map, omsiRoot, detailTextureName);
                                layer.detailTexture = getTexture(detailTexturePath);
                                layer.useDetailTexture = true;
                                const float detailTextureScale =
                                    static_cast<float>(tiling.detailTextureRepeatsPerTile) *
                                    inverseIntervals;
                                layer.detailTextureScale = {detailTextureScale, detailTextureScale};
                            } catch (const std::exception& error) {
                                recordSkipped(
                                    skipped, "ground-texture detail textures that failed to load",
                                    sidecar->path.filename().string() +
                                        " detail=" + detailTextureName + ": " + error.what());
                            }
                        }
                        layer.transmap = getTexture(sidecar->path, false);
                        layer.useTransmap = true;
                        // OMSI ground-texture DDS rows run opposite the terrain mesh's map
                        // northing.
                        layer.flipTransmapY = true;
                        layer.transmapScale = {inverseIntervals, inverseIntervals};
                        buffer.groundTextureLayers.push_back(layer);
                    } catch (const std::exception& error) {
                        recordSkipped(skipped, "ground-texture layers that failed to load",
                                      sidecar->path.filename().string() + ": " + error.what());
                    }
                }
            }
            buffer.minZ = prepared.minimumHeight;
            buffer.maxZ = prepared.maximumHeight;
            pglGenBuffers(1, &buffer.buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, buffer.buffer);
            pglBufferData(
                GL_ARRAY_BUFFER,
                static_cast<std::ptrdiff_t>(prepared.vertices.size() * sizeof(TerrainVertex)),
                prepared.vertices.data(), GL_STATIC_DRAW);
            pglGenBuffers(1, &buffer.indexBuffer);
            pglBindBuffer(GL_ARRAY_BUFFER, buffer.indexBuffer);
            pglBufferData(
                GL_ARRAY_BUFFER,
                static_cast<std::ptrdiff_t>(prepared.indices.size() * sizeof(std::uint32_t)),
                prepared.indices.data(), GL_STATIC_DRAW);
            if (buffer.buffer == 0 || buffer.indexBuffer == 0 || glGetError() != GL_NO_ERROR) {
                if (buffer.buffer != 0) {
                    pglDeleteBuffers(1, &buffer.buffer);
                }
                if (buffer.indexBuffer != 0) {
                    pglDeleteBuffers(1, &buffer.indexBuffer);
                }
                throw std::runtime_error("Could not upload terrain mesh for " +
                                         tile.textPath.filename().string());
            }
            impl_->terrainBuffers.push_back(buffer);
            ++impl_->terrainTilesByTile[tileKey(buffer.tileX, buffer.tileY)];
            loadedTerrainTileKeys.insert(key);
            terrainChanged = true;
            ++terrainUploads;
        }
    }
    gameLog.Log("Map active terrain load complete: " + std::to_string(terrainTilesVisited) +
                " tiles visited, " + std::to_string(impl_->terrainBuffers.size()) +
                " total meshes in " + std::to_string(elapsedMilliseconds(terrainLoadStart)) +
                " ms.");
    return terrainChanged;
}

void MapSceneLoader::indexMapContent() {
    const auto indexStart = std::chrono::steady_clock::now();
    const auto registerObjectId = [&](int objectId, MapObjectRecordKind kind,
                                      const std::string& tileName, int tileX, int tileY) {
        mapObjectTileKeysById.try_emplace(objectId, tileKey(tileX, tileY));
        const auto [existing, inserted] = mapObjectRecordKindsById.try_emplace(objectId, kind);
        if (!inserted) {
            ambiguousMapObjectIds.insert(objectId);
            if (existing->second != kind) {
                recordSkipped(skipped, "map object IDs reused across record types",
                              tileName + " object ID=" + std::to_string(objectId));
            }
        }
    };
    std::size_t submergedTrafficLaneCount = 0;
    for (std::size_t tileReferenceIndex = 0; tileReferenceIndex < map.tiles.size();
         ++tileReferenceIndex) {
        const openbus::map::MapTileReference& tileReference = map.tiles[tileReferenceIndex];
        gameLog.Log("Map index tile " + std::to_string(tileReferenceIndex + 1) + "/" +
                    std::to_string(map.tiles.size()) + ": " + tileReference.textPath.string() +
                    "; elapsed=" + std::to_string(elapsedMilliseconds(indexStart)) + " ms.");
        const openbus::map::MapTileData tile = openbus::map::loadMapTile(tileReference);
        const std::string tileName = tileLabel(tileReference.x, tileReference.y);
        std::optional<openbus::map::WaterData> trafficWater;
        if (tile.hasWaterMarker && tileReference.hasWaterFile) {
            try {
                trafficWater = openbus::map::loadWaterData(tileReference.waterPath);
            } catch (const std::exception& error) {
                recordSkipped(skipped, "water sidecars unavailable for traffic clearance",
                              tileName + ": " + error.what());
            }
        }
        impl_->trafficRouteNetwork.addTileSplines(tile.splines, tileReference.x, tileReference.y,
                                                  map.coordinates);
        std::vector<openbus::map::MapSplinePlacement> roadTrafficSplines;
        std::vector<openbus::map::MapSplinePlacement> humanTrafficSplines;
        for (const auto& spline : tile.splines) {
            const auto* profile =
                cachedSplineProfile(resolveSplineProfile(map, omsiRoot, spline.assetPath));
            if (profile != nullptr) {
                if (openbus::map::mapSplineProfileHasPathType(
                        *profile, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD)) {
                    roadTrafficSplines.push_back(spline);
                    impl_->roadTrafficSplinePathOffsets[spline.splineId] =
                        openbus::map::mapSplineProfilePathOffsets(
                            *profile, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, spline.mirrored);
                    if (spline.geometryValid && std::isfinite(spline.length) &&
                        spline.length > 0.0) {
                        for (const auto& lane :
                             impl_->roadTrafficSplinePathOffsets[spline.splineId]) {
                            openbus::map::MapTrafficRouteSegment segment;
                            segment.spline = {spline, tileReference.x, tileReference.y,
                                              map.coordinates};
                            segment.reversed = lane.reversed;
                            segment.pathLateralMeters = lane.lateralMeters;
                            segment.pathHeightMeters = lane.heightMeters;
                            segment.pathRules =
                                openbus::map::mapTrafficRulesForPath(spline.rules, lane.pathIndex);
                            if (trafficWater &&
                                !openbus::map::mapTrafficRouteSegmentClearsWater(
                                    segment, *trafficWater, map.coordinates.tileSize)) {
                                ++submergedTrafficLaneCount;
                                continue;
                            }
                            if (lane.bidirectional) {
                                impl_->roadTrafficLaneNetwork.addSegment(segment);
                                impl_->roadTrafficLaneNetwork.addSegment(
                                    openbus::map::reverseMapTrafficRouteSegment(segment,
                                                                                spline.length));
                            } else {
                                impl_->roadTrafficLaneNetwork.addSegment(std::move(segment));
                            }
                        }
                    }
                }
                if (openbus::map::mapSplineProfileHasPathType(
                        *profile, openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN)) {
                    humanTrafficSplines.push_back(spline);
                    auto pathOffsets = openbus::map::mapSplineProfilePathOffsets(
                        *profile, openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN, spline.mirrored);
                    impl_->humanTrafficSplinePathOffsets[spline.splineId] = pathOffsets;
                    if (spline.geometryValid && std::isfinite(spline.length) &&
                        spline.length > 0.0) {
                        for (const auto& pathOffset : pathOffsets) {
                            openbus::map::MapTrafficRouteSegment segment;
                            segment.spline = {spline, tileReference.x, tileReference.y,
                                              map.coordinates};
                            segment.reversed = pathOffset.reversed;
                            segment.pathLateralMeters = pathOffset.lateralMeters;
                            segment.pathHeightMeters = pathOffset.heightMeters;
                            impl_->humanTrafficLaneNetwork.addSegment(segment);
                            if (pathOffset.bidirectional) {
                                impl_->humanTrafficLaneNetwork.addSegment(
                                    openbus::map::reverseMapTrafficRouteSegment(std::move(segment),
                                                                                spline.length));
                            }
                        }
                    }
                }
            }
            roadConnections.addSpline(spline, tileReference.x, tileReference.y,
                                      spline.geometryValid ? splineCantWidth(spline)
                                                           : std::nullopt);
            if (spline.splineId != 0 && !ambiguousMapSplineIds.contains(spline.splineId)) {
                const auto [existing, inserted] = mapSplinesById.try_emplace(
                    spline.splineId,
                    openbus::map::MapSplinePlacementReference{spline, tileReference.x,
                                                              tileReference.y, map.coordinates});
                if (!inserted) {
                    mapSplinesById.erase(existing);
                    ambiguousMapSplineIds.insert(spline.splineId);
                }
            }
        }
        impl_->roadTrafficRouteNetwork.addTileSplines(roadTrafficSplines, tileReference.x,
                                                      tileReference.y, map.coordinates);
        impl_->humanTrafficRouteNetwork.addTileSplines(humanTrafficSplines, tileReference.x,
                                                       tileReference.y, map.coordinates);
        for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
            registerObjectId(object.id, MapObjectRecordKind::Scenery, tileName, tileReference.x,
                             tileReference.y);
            mapObjectAssetPathsById.try_emplace(object.id, object.assetPath);
            IndexedSceneryObject indexed;
            indexed.assetPath = object.assetPath;
            indexed.transformValid = object.transformValid;
            indexed.splineTerrainAlign = object.splineTerrainAlign;
            if (const auto pose = openbus::map::placeMapSceneryObject(
                    object, tileReference.x, tileReference.y, map.coordinates)) {
                indexed.pose = *pose;
            }
            if (!sceneryObjectsById.try_emplace(object.id, std::move(indexed)).second) {
                recordSkipped(skipped, "map placements with duplicate object IDs",
                              tileLabel(tileReference.x, tileReference.y) +
                                  " object ID=" + std::to_string(object.id));
            }
        }
        for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
            registerObjectId(object.id, MapObjectRecordKind::Attached, tileName, tileReference.x,
                             tileReference.y);
            mapObjectAssetPathsById.try_emplace(object.id, object.assetPath);
            IndexedAttachedObject indexed;
            indexed.assetPath = object.assetPath;
            indexed.parentId = object.attachedToObjectId;
            indexed.attachmentIndex = object.attachPointIndexValue;
            indexed.rotationDegrees = object.rotationDegrees;
            indexed.transformValid = object.transformValid;
            indexed.splineTerrainAlign = object.splineTerrainAlign;
            if (!attachedObjectsById.try_emplace(object.id, std::move(indexed)).second) {
                recordSkipped(skipped, "map placements with duplicate object IDs",
                              tileLabel(tileReference.x, tileReference.y) +
                                  " attached object ID=" + std::to_string(object.id));
            }
        }
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
            registerObjectId(attachment.id, MapObjectRecordKind::SplineAttachment, tileName,
                             tileReference.x, tileReference.y);
            mapObjectAssetPathsById.try_emplace(attachment.id, attachment.assetPath);
            if (!splineAttachmentTileById.try_emplace(attachment.id, tileReferenceIndex).second) {
                recordSkipped(skipped, "map placements with duplicate object IDs",
                              tileLabel(tileReference.x, tileReference.y) +
                                  " spline attachment ID=" + std::to_string(attachment.id));
            }
        }
    }
    gameLog.Log("Map tile index parsed; connecting road and traffic route networks.");
    roadConnections.finalize();
    impl_->trafficRouteNetwork.finalize();
    impl_->roadTrafficRouteNetwork.finalize();
    impl_->humanTrafficRouteNetwork.finalize();
    roadTrafficLaneNetworkDirty = true;
    humanTrafficLaneNetworkDirty = true;
    if (submergedTrafficLaneCount > 0) {
        gameLog.Log("Map traffic excluded " + std::to_string(submergedTrafficLaneCount) +
                    " road lanes without 0.25 m of clearance above water.");
    }
    gameLog.Log("Map index phase complete: " + std::to_string(map.tiles.size()) + " tiles in " +
                std::to_string(elapsedMilliseconds(indexStart)) + " ms.");
}

void MapSceneLoader::uploadRoadBatches() {
    const auto roadUploadStart = std::chrono::steady_clock::now();
    TraceScope roadUploadPhase("map", "MapRenderer.uploadRoadBatches");
    for (auto& [batchKey, geometry] : roadVertices) {
        if (geometry.vertices.empty()) {
            continue;
        }
        Impl::ModelBatch batch;
        batch.vertexCount = geometry.vertices.size();
        batch.indexCount = geometry.indices.size();
        batch.hasVisibilityBounds = true;
        batch.minX = geometry.minX;
        batch.minY = geometry.minY;
        batch.maxX = geometry.maxX;
        batch.maxY = geometry.maxY;
        batch.minZ = geometry.minZ;
        batch.maxZ = geometry.maxZ;
        batch.tileX = geometry.tileX;
        batch.tileY = geometry.tileY;
        batch.alphaMode = geometry.alphaMode;
        batch.cullBackFaces = true;
        if (geometry.textured) {
            try {
                batch.material.texture = getTexture(geometry.texturePath);
                batch.material.textured = true;
            } catch (const std::exception& error) {
                recordSkipped(skipped, "spline profile textures that failed to load",
                              geometry.texturePath.generic_string() + ": " + error.what());
                batch.color = {0.45, 0.45, 0.45};
            }
        } else {
            batch.color = {0.45, 0.45, 0.45};
        }
        pglGenBuffers(1, &batch.buffer);
        pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
        pglBufferData(GL_ARRAY_BUFFER,
                      static_cast<std::ptrdiff_t>(geometry.vertices.size() * sizeof(ModelVertex)),
                      geometry.vertices.data(), GL_STATIC_DRAW);
        pglGenBuffers(1, &batch.indexBuffer);
        pglBindBuffer(GL_ARRAY_BUFFER, batch.indexBuffer);
        pglBufferData(GL_ARRAY_BUFFER,
                      static_cast<std::ptrdiff_t>(geometry.indices.size() * sizeof(std::uint32_t)),
                      geometry.indices.data(), GL_STATIC_DRAW);
        if (batch.buffer == 0 || batch.indexBuffer == 0 || glGetError() != GL_NO_ERROR) {
            if (batch.buffer != 0) {
                pglDeleteBuffers(1, &batch.buffer);
            }
            if (batch.indexBuffer != 0) {
                pglDeleteBuffers(1, &batch.indexBuffer);
            }
            recordSkipped(skipped, "road batches rejected by OpenGL upload", batchKey);
            continue;
        }
        impl_->roadBatches.push_back(batch);
        ++impl_->roadBatchesByTile[tileKey(batch.tileX, batch.tileY)];
    }
    gameLog.Log("Map road upload complete: geometry groups=" + std::to_string(roadVertices.size()) +
                ", uploaded batches=" + std::to_string(impl_->roadBatches.size()) + " in " +
                std::to_string(elapsedMilliseconds(roadUploadStart)) + " ms.");
    roadVertices.clear();
}

void MapSceneLoader::logSummary() const {
    gameLog.Log("Map render result: terrain tiles=" + std::to_string(impl_->terrainBuffers.size()) +
                ", scenery instances=" + std::to_string(impl_->sceneryInstanceCount) +
                ", scenery batches=" + std::to_string(impl_->sceneryBatches.size()) +
                ", scenery variable states=" + std::to_string(impl_->sceneryScriptStates.size()) +
                ", scenery script runtimes=" + std::to_string(impl_->sceneryScriptRuntimeCount) +
                ", road batches=" + std::to_string(impl_->roadBatches.size()) +
                ", water batches=" + std::to_string(impl_->waterBatches.size()) +
                ", cached textures=" + std::to_string(textureCache.size()) + ".");
    logSkipped(skipped);
    gameLog.Log("Map loading complete in " + std::to_string(elapsedMilliseconds(mapLoadStart)) +
                " ms.");
}

GLuint MapSceneLoader::getTexture(const std::filesystem::path& path, bool repeat) {
    const std::string key =
        path.lexically_normal().generic_string() + (repeat ? "|repeat" : "|clamp");
    const auto found = textureCache.find(key);
    if (found != textureCache.end()) {
        return found->second;
    }
    const GLuint texture = loadTexture(path, repeat);
    textureCache.emplace(key, texture);
    impl_->ownedTextures.push_back(texture);
    return texture;
}

std::optional<double> MapSceneLoader::sampleTerrainHeight(int tileX, int tileY, double localX,
                                                          double localY) const {
    const auto found = terrainByCoordinate.find(tileKey(tileX, tileY));
    if (found == terrainByCoordinate.end()) {
        return std::nullopt;
    }
    return openbus::map::sampleMapTerrainHeight(found->second, localX, localY,
                                                map.coordinates.tileSize);
}

std::optional<std::array<double, 3>> MapSceneLoader::sampleTerrainNormal(double worldX,
                                                                         double worldY) const {
    const int tileX = static_cast<int>(std::floor(worldX / map.coordinates.tileSize));
    const int tileY = static_cast<int>(std::floor(worldY / map.coordinates.tileSize));
    const auto found = terrainByCoordinate.find(tileKey(tileX, tileY));
    if (found == terrainByCoordinate.end()) {
        return std::nullopt;
    }
    const double localX = worldX - static_cast<double>(tileX) * map.coordinates.tileSize;
    const double localY = worldY - static_cast<double>(tileY) * map.coordinates.tileSize;
    return openbus::map::sampleMapTerrainNormal(found->second, localX, localY,
                                                map.coordinates.tileSize);
}

} // namespace openbus::rendering
