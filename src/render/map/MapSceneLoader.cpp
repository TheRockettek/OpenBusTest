// Map load orchestration: ground textures, terrain, map indexing and road upload.

#include "render/map/MapSceneLoader.h"

#include "core/Utf8Path.h"

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

MapSceneLoader::~MapSceneLoader() {
    if (mapIndexRequest_.valid()) {
        mapIndexRequest_.wait();
    }
    if (pendingRoadTrafficLaneNetworkFinalization_.valid()) {
        pendingRoadTrafficLaneNetworkFinalization_.wait();
    }
    if (pendingHumanTrafficLaneNetworkFinalization_.valid()) {
        pendingHumanTrafficLaneNetworkFinalization_.wait();
    }
}

void MapSceneLoader::load() {
    TraceScope trace("map", "MapSceneLoader::load");
    if (loadStarted_) {
        throw std::logic_error("Map scene loading was already started");
    }
    loadStarted_ = true;
    gameLog.Log("Map scene loading started: root=" + openbus::pathToUtf8(map.rootPath) +
                ", tiles=" + std::to_string(map.tiles.size()) + ".");
    {
        TraceScope textureConfigTrace("map", "MapSceneLoader.configureGroundTextures");
        configureGroundTextures();
    }
    gameLog.Log("Map phase: loading ground textures.");
    if (std::none_of(
            map.tiles.begin(), map.tiles.end(),
            [](const openbus::map::MapTileReference& tile) { return tile.hasTerrainFile; })) {
        throw std::runtime_error("No terrain files found in the selected map");
    }
    gameLog.Log("Map phase: indexing tile placements and traffic paths.");
    baseTexturePath_ =
        groundTexturePath(map, omsiRoot, map.groundTextures[baseGroundTextureIndex].texturePath);
    baseTextureRequest_ = assetRequests_->submit([baseTexturePath = baseTexturePath_] {
        TraceScope textureReadTrace("map", "MapSceneLoader.readBaseGroundTexture");
        Image image;
        if (!TextureLoader::readImage(baseTexturePath, image)) {
            throw std::runtime_error("Could not decode map ground texture: " +
                                     openbus::pathToUtf8(baseTexturePath));
        }
        return image;
    });
    mapIndexRequest_ = assetRequests_->submit([this] {
        TraceScope indexTrace("map", "MapSceneLoader.indexMapContent");
        indexMapContent();
    });
}

bool MapSceneLoader::finishLoad(bool waitForCompletion) {
    std::optional<TraceScope> trace;
    if (!loadFinished_) {
        trace.emplace("map", "MapSceneLoader::finishLoad");
    }
    if (loadFinished_) {
        return true;
    }
    if (!loadStarted_) {
        throw std::logic_error("Map scene loading was not started");
    }
    if (waitForCompletion) {
        TraceScope waitTrace("map", "MapSceneLoader.finishLoad.waitForPreparation");
        baseTextureRequest_.wait();
        mapIndexRequest_.wait();
    } else if (baseTextureRequest_.wait_for(std::chrono::seconds(0)) != std::future_status::ready ||
               mapIndexRequest_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return false;
    }

    std::exception_ptr preparationError;
    try {
        Image image = baseTextureRequest_.get();
        preparedTextureImages.emplace(openbus::pathToUtf8(baseTexturePath_.lexically_normal()),
                                      std::make_shared<Image>(std::move(image)));
    } catch (...) {
        preparationError = std::current_exception();
    }
    try {
        mapIndexRequest_.get();
    } catch (...) {
        if (!preparationError) {
            preparationError = std::current_exception();
        }
    }
    if (preparationError) {
        loadFinished_ = true;
        std::rethrow_exception(preparationError);
    }
    {
        TraceScope textureLoadTrace("map", "MapSceneLoader.finishLoad.loadGroundTextures");
        loadGroundTextures();
    }
    loadFinished_ = true;
    return true;
}

bool MapSceneLoader::finishTrafficLaneNetworkPreparation(bool waitForCompletion) {
    const auto finishNetwork = [this, waitForCompletion](
                                   bool& dirty,
                                   std::future<openbus::map::MapTrafficLaneNetwork>& request,
                                   openbus::map::MapTrafficLaneNetwork& network,
                                   const char* networkName, bool roadNetwork) {
        if (dirty) {
            if (request.valid()) {
                throw std::logic_error(
                    "Traffic lane network changed during background finalization");
            }
            dirty = false;
            auto networkToFinalize = std::move(network);
            request = assetRequests_->submit([network = std::move(networkToFinalize)]() mutable {
                network.finalize();
                return network;
            });
            gameLog.Log("Map phase: preparing " + std::string(networkName) +
                        " traffic lane network asynchronously.");
        }
        if (!request.valid()) {
            return true;
        }
        if (waitForCompletion) {
            request.wait();
        } else if (request.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return false;
        }
        network = request.get();
        if (roadNetwork) {
            const openbus::map::MapTrafficLaneNetworkStats laneStats = network.stats();
            gameLog.Log("Map road lane network: lanes=" + std::to_string(laneStats.lanes) +
                        ", unconnected lane ends=" + std::to_string(laneStats.deadEnds) +
                        ", unconnected ends within 3 m of an aligned lane start=" +
                        std::to_string(laneStats.nearMissDeadEnds) + ".");
        }
        return true;
    };

    const bool roadReady =
        finishNetwork(roadTrafficLaneNetworkDirty, pendingRoadTrafficLaneNetworkFinalization_,
                      impl_->roadTrafficLaneNetwork, "road", true);
    const bool humanReady =
        finishNetwork(humanTrafficLaneNetworkDirty, pendingHumanTrafficLaneNetworkFinalization_,
                      impl_->humanTrafficLaneNetwork, "human", false);
    return roadReady && humanReady;
}

std::vector<std::string>
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
        const std::string key = openbus::pathToUtf8(configPath.lexically_normal());
        if (requestedKeys.insert(key).second) {
            assetKeys.push_back(key);
            if (!sceneryAssetBatches.contains(key) && !preparedSceneryAssetConfigs.contains(key) &&
                !pendingSceneryAssetRequests.contains(key)) {
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
    constexpr std::size_t maxPendingSceneryAssetPreparations = 8;
    std::size_t availablePreparations =
        pendingSceneryAssetRequests.size() < maxPendingSceneryAssetPreparations
            ? maxPendingSceneryAssetPreparations - pendingSceneryAssetRequests.size()
            : 0;
    std::vector<PendingSceneryAsset> requests;
    for (const std::string& key : assetKeys) {
        if (sceneryAssetBatches.contains(key) || preparedSceneryAssetConfigs.contains(key)) {
            continue;
        }
        auto request = pendingSceneryAssetRequests.find(key);
        if (request == pendingSceneryAssetRequests.end()) {
            const auto configPath =
                std::find_if(configPaths.begin(), configPaths.end(),
                             [&key](const auto& entry) { return entry.first == key; });
            if (configPath == configPaths.end()) {
                throw std::logic_error("Scenery asset preparation path was not captured");
            }
            if (availablePreparations == 0) {
                continue;
            }
            const std::filesystem::path path = configPath->second;
            const std::filesystem::path omsiRootPath = omsiRoot;
            auto future = assetRequests_
                              ->submit([path, omsiRootPath] {
                                  TraceScope preparationTrace(
                                      "map", "MapSceneLoader.prepareSceneryAsset.work");
                                  auto prepared = std::make_shared<PreparedSceneryAsset>();
                                  try {
                                      *prepared = prepareSceneryAsset(path, omsiRootPath);
                                  } catch (...) {
                                      prepared->preparationError = std::current_exception();
                                  }
                                  return prepared;
                              })
                              .share();
            request = pendingSceneryAssetRequests.emplace(key, std::move(future)).first;
            --availablePreparations;
        }
        requests.push_back({key, request->second});
    }
    if (!requests.empty()) {
        pendingTileSceneryRequests.insert_or_assign(tileKeyValue, std::move(requests));
    }
    return assetKeys;
}

bool MapSceneLoader::completeSceneryAssetPreparation(std::uint64_t tileKeyValue,
                                                     bool waitForCompletion) {
    auto request = pendingTileSceneryRequests.find(tileKeyValue);
    if (request == pendingTileSceneryRequests.end()) {
        return true;
    }
    for (const PendingSceneryAsset& assetRequest : request->second) {
        if (waitForCompletion) {
            assetRequest.result.wait();
        } else if (assetRequest.result.wait_for(std::chrono::seconds(0)) !=
                   std::future_status::ready) {
            return false;
        }
    }
    const bool requestedByActiveTile = pendingTileAssetKeys.contains(tileKeyValue);
    bool preparationSucceeded = true;
    std::exception_ptr batchPreparationError;
    std::filesystem::path failedAssetPath;
    std::vector<std::string> completedAssetKeys;
    completedAssetKeys.reserve(request->second.size());
    for (const PendingSceneryAsset& assetRequest : request->second) {
        std::shared_ptr<PreparedSceneryAsset> prepared;
        std::exception_ptr preparationError;
        try {
            prepared = assetRequest.result.get();
            if (!prepared) {
                throw std::runtime_error("worker returned no prepared asset");
            }
            preparationError = prepared->preparationError;
        } catch (...) {
            preparationError = std::current_exception();
        }
        if (preparationError) {
            preparationSucceeded = false;
            if (!batchPreparationError) {
                batchPreparationError = preparationError;
                failedAssetPath = assetRequest.key;
            }
        } else if (requestedByActiveTile && prepared &&
                   !sceneryAssetBatches.contains(assetRequest.key) &&
                   !preparedSceneryAssetConfigs.contains(assetRequest.key)) {
            preparedSceneryAssetConfigs.emplace(assetRequest.key, std::move(prepared));
        }
        completedAssetKeys.push_back(assetRequest.key);
    }
    if (batchPreparationError && requestedByActiveTile) {
        recordTilePreparationFailure(tileKeyValue, false, "scenery/model asset preparation",
                                     failedAssetPath, batchPreparationError, waitForCompletion);
    }
    pendingTileSceneryRequests.erase(request);
    for (const std::string& key : completedAssetKeys) {
        if (sceneryAssetBatches.contains(key) || preparedSceneryAssetConfigs.contains(key)) {
            pendingSceneryAssetRequests.erase(key);
            continue;
        }
        const bool stillRequested =
            std::any_of(pendingTileSceneryRequests.begin(), pendingTileSceneryRequests.end(),
                        [&key](const auto& tileRequest) {
                            return std::any_of(tileRequest.second.begin(), tileRequest.second.end(),
                                               [&key](const PendingSceneryAsset& assetRequest) {
                                                   return assetRequest.key == key;
                                               });
                        });
        if (!stillRequested) {
            pendingSceneryAssetRequests.erase(key);
        }
    }
    if (!preparationSucceeded && requestedByActiveTile) {
        pendingTileAssetKeys.erase(tileKeyValue);
    }
    return preparationSucceeded;
}

bool MapSceneLoader::sceneryAssetPreparationsReady(
    const std::vector<std::string>& assetKeys) const {
    return std::all_of(assetKeys.begin(), assetKeys.end(), [this](const std::string& key) {
        return sceneryAssetBatches.contains(key) || preparedSceneryAssetConfigs.contains(key);
    });
}

bool MapSceneLoader::tilePreparationRetryReady(std::uint64_t tileKeyValue) const {
    const auto failure = tilePreparationFailures.find(tileKeyValue);
    return failure == tilePreparationFailures.end() ||
           (!failure->second.exhausted &&
            std::chrono::steady_clock::now() >= failure->second.retryAfter);
}

void MapSceneLoader::recordTilePreparationFailure(std::uint64_t tileKeyValue,
                                                  bool terrainPreparation, const std::string& stage,
                                                  const std::filesystem::path& path,
                                                  std::exception_ptr error,
                                                  bool waitForCompletion) {
    std::string detail = "unknown error";
    try {
        if (error) {
            std::rethrow_exception(error);
        }
    } catch (const std::exception& exception) {
        detail = exception.what();
    } catch (...) {
    }

    if (waitForCompletion) {
        gameLog.Error("Map " + stage + " failed for " + openbus::pathToUtf8(path) + ": " + detail);
        std::rethrow_exception(error);
    }

    TilePreparationFailure& failure = tilePreparationFailures[tileKeyValue];
    if (failure.attempts == 0) {
        failure.terrainPreparation = terrainPreparation;
    }
    ++failure.attempts;
    failure.exhausted = failure.attempts >= 3;
    failure.diagnostic = stage + " failed for " + openbus::pathToUtf8(path) + ": " + detail;
    if (!failure.exhausted) {
        const auto retryDelay = std::chrono::milliseconds(250 * (1U << (failure.attempts - 1U)));
        failure.retryAfter = std::chrono::steady_clock::now() + retryDelay;
    }
    gameLog.Log(
        failure.exhausted ? LogLevel::Error : LogLevel::Warning,
        "Map " + stage + " failed for " + openbus::pathToUtf8(path) + ": " + detail +
            (failure.exhausted
                 ? "; automatic retries exhausted, retry after the tile leaves the active "
                   "window."
                 : "; retrying in " +
                       std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                                          failure.retryAfter - std::chrono::steady_clock::now())
                                          .count()) +
                       " ms (attempt " + std::to_string(failure.attempts) + " of 3)."));
}

MapSceneLoader::PreparedSceneryAsset
MapSceneLoader::prepareSceneryAsset(const std::filesystem::path& configPath,
                                    const std::filesystem::path& omsiRootPath) {
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
                    const std::string meshKey =
                        openbus::pathToUtf8(part.objPath.lexically_normal());
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
    const auto decodeTexture = [&](const std::string& textureName) {
        if (textureName.empty()) {
            return;
        }
        const std::filesystem::path texturePath = map_detail::findSceneryTexture(
            configPath.parent_path(), prepared.modelRoot, omsiRootPath, textureName);
        prepared.resolvedTexturePaths.try_emplace(textureName, texturePath);
        if (texturePath.empty()) {
            return;
        }
        const std::string key = openbus::pathToUtf8(texturePath.lexically_normal());
        if (prepared.decodedTextures.contains(key)) {
            return;
        }
        Image image;
        if (!TextureLoader::readImage(texturePath, image)) {
            throw std::runtime_error("Could not decode scenery texture: " +
                                     openbus::pathToUtf8(texturePath));
        }
        prepared.decodedTextures.emplace(key, std::make_shared<Image>(std::move(image)));
    };
    for (const SceneryTreeDefinition& tree : prepared.objectConfiguration.trees) {
        decodeTexture(tree.texturePath);
    }
    for (const auto& [meshKey, mesh] : prepared.meshes) {
        static_cast<void>(meshKey);
        if (!mesh.parsed) {
            continue;
        }
        for (const auto& [materialName, material] : mesh.parsed->materials) {
            static_cast<void>(materialName);
            decodeTexture(material.textureName);
        }
    }
    for (const ModelPart& part : prepared.modelConfiguration.parts) {
        for (const ModelMaterialState& state : part.materialStatesInOrder) {
            decodeTexture(state.textureName);
            decodeTexture(state.transmapTextureName);
            decodeTexture(state.freeTextureName);
        }
    }
    const auto resolvedTexture = [&prepared](const std::string& textureName) {
        if (textureName.empty()) {
            return std::filesystem::path{};
        }
        const auto texture = prepared.resolvedTexturePaths.find(textureName);
        if (texture == prepared.resolvedTexturePaths.end()) {
            throw std::logic_error("Scenery texture path was not prepared: " + textureName);
        }
        return texture->second;
    };
    for (const SceneryTreeDefinition& tree : prepared.objectConfiguration.trees) {
        const std::filesystem::path texturePath = resolvedTexture(tree.texturePath);
        if (texturePath.empty()) {
            prepared.treeBatches.push_back({{}, tree.texturePath, {}, {}});
            continue;
        }
        const double height = (tree.minimumHeight + tree.maximumHeight) * 0.5;
        const double ratio = (tree.minimumRatio + tree.maximumRatio) * 0.5;
        PreparedSceneryTreeBatch batch;
        batch.vertices = makeTreePreview(height, height * 0.55 * ratio);
        batch.textureName = tree.texturePath;
        batch.texturePath = texturePath;
        batch.batch.vertexCount = batch.vertices.size();
        batch.batch.boundingRadius = vertexBoundingRadius(batch.vertices);
        batch.batch.renderType = prepared.objectConfiguration.renderType;
        batch.batch.material.textured = true;
        batch.batch.material.flipTextureY = true;
        batch.batch.alphaMode = 1;
        prepared.treeBatches.push_back(std::move(batch));
    }
    for (const ModelPart& part : prepared.modelConfiguration.parts) {
        if (part.objPath.empty()) {
            continue;
        }
        std::string meshExtension = part.objPath.extension().string();
        std::transform(meshExtension.begin(), meshExtension.end(), meshExtension.begin(),
                       [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
        if (meshExtension != ".o3d" && meshExtension != ".x") {
            continue;
        }
        const std::string meshKey = openbus::pathToUtf8(part.objPath.lexically_normal());
        const auto preparedMesh = prepared.meshes.find(meshKey);
        if (preparedMesh == prepared.meshes.end() || !preparedMesh->second.parsed ||
            preparedMesh->second.preparationError) {
            continue;
        }
        const PreparedSceneryMesh& mesh = preparedMesh->second;
        const ParsedObj& parsed = *mesh.parsed;
        const auto statesByMaterial = openbus::map::resolveSceneryMaterialStates(parsed, part);
        for (const auto& [materialIndex, vertices] : mesh.verticesByMaterial) {
            if (vertices.empty()) {
                continue;
            }
            const auto sourceMaterial =
                parsed.materials.find("matl_" + std::to_string(materialIndex));
            const auto stateFound = statesByMaterial.find(materialIndex);
            const ModelMaterialState* state =
                stateFound == statesByMaterial.end() ? nullptr : stateFound->second;
            std::string textureName = sourceMaterial == parsed.materials.end()
                                          ? std::string{}
                                          : sourceMaterial->second.textureName;
            std::array<double, 3> color = sourceMaterial == parsed.materials.end()
                                              ? std::array<double, 3>{1.0, 1.0, 1.0}
                                              : sourceMaterial->second.color;
            if (state != nullptr && !state->textureName.empty()) {
                textureName = state->textureName;
                const std::string stateTexture =
                    openbus::pathToUtf8(std::filesystem::path(state->textureName).filename());
                const auto materialForTexture = std::find_if(
                    parsed.materials.begin(), parsed.materials.end(), [&](const auto& entry) {
                        return std::filesystem::path(entry.second.textureName).filename() ==
                               stateTexture;
                    });
                if (materialForTexture != parsed.materials.end()) {
                    color = materialForTexture->second.color;
                }
            }
            PreparedSceneryBatch preparedBatch;
            preparedBatch.meshKey = meshKey;
            preparedBatch.materialIndex = materialIndex;
            preparedBatch.textureName = textureName;
            preparedBatch.texturePath = resolvedTexture(textureName);
            if (state != nullptr) {
                preparedBatch.transmapTextureName = state->transmapTextureName;
                preparedBatch.transmapTexturePath = resolvedTexture(state->transmapTextureName);
                preparedBatch.freeTextureName = state->freeTextureName;
                preparedBatch.freeTexturePath = resolvedTexture(state->freeTextureName);
                preparedBatch.hasMaterialState = true;
                preparedBatch.stateAlphaMode = state->alphaMode;
            }
            Impl::ModelBatch& batch = preparedBatch.batch;
            batch.cullBackFaces = true;
            batch.vertexCount = vertices.size();
            batch.boundingRadius = vertexBoundingRadius(vertices);
            batch.sceneryLodIndex =
                part.lodIndex < 0 ? 0U : static_cast<std::size_t>(part.lodIndex);
            if (sourceMaterial != parsed.materials.end()) {
                for (std::size_t index = 0; index < 3; ++index) {
                    batch.material.specularColor[index] =
                        static_cast<float>(sourceMaterial->second.specular[index]);
                    batch.material.emissionColor[index] =
                        static_cast<float>(sourceMaterial->second.emission[index]);
                }
                batch.material.specularPower =
                    static_cast<float>(sourceMaterial->second.specularPower);
                batch.alpha = std::clamp(sourceMaterial->second.alpha, 0.0, 1.0);
            }
            batch.color = color;
            batch.renderType =
                resolveSceneryRenderType(part, prepared.objectConfiguration.renderType);
            batch.visibilityVariable = part.visibleVariable;
            batch.visibilityValue = part.visibleValue;
            if (state != nullptr) {
                batch.scriptTextureIndex = state->scriptTextureIndex;
                batch.textTextureIndex = state->textTextureIndex;
                batch.freeTextureName = state->freeTextureName;
                batch.freeTextureVariable = state->freeTextureVariable;
                batch.textureObjectRoot = configPath.parent_path();
                batch.textureModelRoot = prepared.modelRoot;
                batch.material.textureWrapS = textureAddressModeToGl(state->textureAddressS);
                batch.material.textureWrapT = textureAddressModeToGl(state->textureAddressT);
                batch.alphaMode = state->alphaMode;
                batch.noZwrite = state->noZwrite;
                batch.noZcheck = state->noZcheck;
            }
            if (batch.alphaMode == 0 && batch.alpha < 1.0 - 1.0e-6) {
                batch.alphaMode = 2;
            }
            prepared.batches.push_back(std::move(preparedBatch));
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

bool MapSceneLoader::requestTileRoadPreparation(const openbus::map::MapTileData& tile,
                                                const openbus::map::MapTileReference& tileReference,
                                                std::uint64_t tileKeyValue) {
    if (pendingTileRoadRequests.contains(tileKeyValue) ||
        preparedTileRoads.contains(tileKeyValue)) {
        return true;
    }
    constexpr std::size_t maxPendingRoadPreparations = 8;
    if (pendingTileRoadRequests.size() >= maxPendingRoadPreparations) {
        return false;
    }
    std::vector<RoadSplineInput> splines;
    splines.reserve(tile.splines.size());
    for (const openbus::map::MapSplinePlacement& spline : tile.splines) {
        RoadSplineInput input;
        input.spline = spline;
        input.joins = roadConnections.joinsFor(spline, tile.reference.x, tile.reference.y);
        input.profilePath = resolveSplineProfile(map, omsiRoot, spline.assetPath);
        const std::string profileKey = openbus::pathToUtf8(input.profilePath.lexically_normal());
        const auto profile = splineProfiles.find(profileKey);
        if (profile != splineProfiles.end()) {
            input.profile = profile->second;
        }
        splines.push_back(std::move(input));
    }
    const int tileX = tile.reference.x;
    const int tileY = tile.reference.y;
    const std::filesystem::path omsiRootPath = omsiRoot;
    const openbus::map::MapCoordinates coordinates = map.coordinates;
    const bool hasWaterMarker = tile.hasWaterMarker;
    const std::filesystem::path waterPath = tileReference.waterPath;
    const bool hasWaterFile = tileReference.hasWaterFile;
    pendingTileRoadRequests.emplace(
        tileKeyValue,
        assetRequests_->submit([splines = std::move(splines), omsiRootPath, tileX, tileY,
                                coordinates, hasWaterMarker, waterPath, hasWaterFile] {
            TraceScope preparationTrace("map", "MapSceneLoader.prepareRoadTile.work");
            return prepareRoadTile(splines, omsiRootPath, tileX, tileY, coordinates,
                                   hasWaterMarker && hasWaterFile, waterPath);
        }));
    return true;
}

MapSceneLoader::PreparedRoadTile
MapSceneLoader::prepareRoadTile(const std::vector<RoadSplineInput>& splines,
                                const std::filesystem::path& omsiRootPath, int tileX, int tileY,
                                const openbus::map::MapCoordinates& coordinates, bool loadWater,
                                const std::filesystem::path& waterPath) {
    PreparedRoadTile prepared;
    for (const RoadSplineInput& input : splines) {
        const openbus::map::MapSplinePlacement& spline = input.spline;
        const std::string description =
            map_detail::tileLabel(tileX, tileY) +
            (spline.elevated ? " [spline_h] asset=" : " [spline] asset=") + spline.assetPath;
        if (!spline.geometryValid) {
            prepared.skipped.emplace_back("[spline] sections with invalid numeric geometry",
                                          description, 1);
            continue;
        }
        const std::filesystem::path& profilePath = input.profilePath;
        if (profilePath.empty()) {
            prepared.skipped.emplace_back("spline profile files not found", description, 1);
            continue;
        }
        if (!input.profile.has_value()) {
            continue;
        }
        const openbus::map::MapSplineProfile& profile = *input.profile;
        const std::string profileKey = openbus::pathToUtf8(profilePath.lexically_normal());
        if (profile.editorOnly) {
            prepared.editorOnlyProfileKeys.insert(profileKey);
            continue;
        }
        if (profile.sections.empty()) {
            prepared.skipped.emplace_back("spline profiles with no graphical cross-section strips",
                                          openbus::pathToUtf8(profilePath.filename()), 1);
            continue;
        }
        const double startX = static_cast<double>(tileX) * coordinates.tileSize + spline.localX;
        const double startY = static_cast<double>(tileY) * coordinates.tileSize + spline.localY;
        const std::vector<openbus::map::MapSplineSample> centerline =
            openbus::map::tessellateMapSpline(startX, startY, spline.rotationDegrees, spline.length,
                                              spline.radius);
        if (centerline.size() < 2) {
            prepared.skipped.emplace_back("[spline] sections that failed geometry tessellation",
                                          description, 1);
            continue;
        }
        const std::optional<double> cantHalfWidth =
            openbus::map::mapSplineProfileCantHalfWidth(profile);
        for (const openbus::map::MapSplineProfileSection& section : profile.sections) {
            if (section.points.size() < 2) {
                prepared.skipped.emplace_back("spline profile strips with fewer than two points",
                                              openbus::pathToUtf8(profilePath.filename()), 1);
                continue;
            }
            if (section.textureIndex < 0 ||
                static_cast<std::size_t>(section.textureIndex) >= profile.textures.size()) {
                prepared.skipped.emplace_back("spline profile strips with invalid texture indices",
                                              openbus::pathToUtf8(profilePath.filename()) +
                                                  " index=" + std::to_string(section.textureIndex),
                                              1);
                continue;
            }
            const std::string& textureName =
                profile.textures[static_cast<std::size_t>(section.textureIndex)];
            const std::filesystem::path texturePath =
                map_detail::findSplineTexture(profilePath, omsiRootPath, textureName);
            const int alphaMode =
                static_cast<std::size_t>(section.textureIndex) < profile.textureAlphaModes.size()
                    ? profile.textureAlphaModes[static_cast<std::size_t>(section.textureIndex)]
                    : 0;
            const bool textured = !texturePath.empty();
            if (!textured) {
                prepared.skipped.emplace_back(
                    "spline profile textures not found (geometry rendered untextured)",
                    openbus::pathToUtf8(profilePath.filename()) + " texture=" + textureName, 1);
            } else {
                const std::string textureKey = openbus::pathToUtf8(texturePath.lexically_normal());
                if (!prepared.decodedTextures.contains(textureKey) &&
                    !prepared.textureErrors.contains(textureKey)) {
                    Image image;
                    try {
                        if (!TextureLoader::readImage(texturePath, image)) {
                            prepared.textureErrors.emplace(textureKey, "image decode failed");
                        } else {
                            prepared.decodedTextures.emplace(
                                textureKey, std::make_shared<Image>(std::move(image)));
                        }
                    } catch (const std::exception& error) {
                        prepared.textureErrors.emplace(textureKey, error.what());
                    } catch (...) {
                        prepared.textureErrors.emplace(textureKey, "unknown error");
                    }
                }
            }
            const std::string batchKey =
                std::to_string(tileX) + "," + std::to_string(tileY) + "|" +
                (textured
                     ? openbus::pathToUtf8(texturePath.lexically_normal())
                     : "untextured:" + profileKey + "#" + std::to_string(section.textureIndex)) +
                "|alpha:" + std::to_string(alphaMode);
            RoadMeshGeometry& road = prepared.geometryByBatch[batchKey];
            if (road.vertices.empty()) {
                road.texturePath = texturePath;
                road.textured = textured;
                road.tileX = tileX;
                road.tileY = tileY;
                road.alphaMode = alphaMode;
            }
            map_detail::appendRoadProfile(road, spline, centerline, section, input.joins,
                                          cantHalfWidth, coordinates);
        }
    }
    if (loadWater) {
        try {
            prepared.waterData = openbus::map::loadWaterData(waterPath);
            const openbus::map::WaterData& water = *prepared.waterData;
            prepared.waterVertices.reserve(water.surfaces.size() * 4);
            prepared.waterIndices.reserve(water.surfaces.size() * 6);
            double minimumHeight = std::numeric_limits<double>::infinity();
            double maximumHeight = -std::numeric_limits<double>::infinity();
            double heightSum = 0.0;
            std::size_t heightCount = 0;
            const double tileOriginX = static_cast<double>(tileX) * coordinates.tileSize;
            const double tileOriginY = static_cast<double>(tileY) * coordinates.tileSize;
            for (const openbus::map::WaterSurface& surface : water.surfaces) {
                const auto corners = openbus::map::buildMapWaterSurfaceVertices(
                    tileX, tileY, surface, coordinates.tileSize);
                const std::uint32_t base =
                    static_cast<std::uint32_t>(prepared.waterVertices.size());
                for (std::size_t corner = 0; corner < corners.size(); ++corner) {
                    const openbus::map::MapWaterVertex& vertex = corners[corner];
                    prepared.waterVertices.push_back(
                        {static_cast<float>(vertex.x - tileOriginX),
                         static_cast<float>(vertex.y - tileOriginY), static_cast<float>(vertex.z),
                         corner == 1 || corner == 2 ? 1.0F : 0.0F, corner >= 2 ? 1.0F : 0.0F, 0.0F,
                         0.0F, 0.0F, 1.0F});
                    minimumHeight = (std::min)(minimumHeight, vertex.z);
                    maximumHeight = (std::max)(maximumHeight, vertex.z);
                    heightSum += vertex.z;
                    ++heightCount;
                }
                prepared.waterIndices.insert(prepared.waterIndices.end(),
                                             {base, base + 1, base + 2, base, base + 2, base + 3});
            }
            if (heightCount > 0) {
                prepared.waterMinimumHeight = minimumHeight;
                prepared.waterMaximumHeight = maximumHeight;
                prepared.waterMeanHeight = heightSum / static_cast<double>(heightCount);
            }
        } catch (const std::exception& error) {
            prepared.waterError = error.what();
        } catch (...) {
            prepared.waterError = "unknown error";
        }
    }
    return prepared;
}

MapSceneLoader::TileLoadStatus
MapSceneLoader::loadActiveTiles(const std::vector<std::pair<int, int>>& tileCoordinates,
                                const openbus::map::MapTileCoordinate& priorityCenter,
                                bool waitForCompletion, bool failOnExhaustedRetries) {
    const bool initialLoad = waitForCompletion || failOnExhaustedRetries;
    std::optional<TraceScope> trace;
    if (initialLoad) {
        trace.emplace("map", "MapSceneLoader::loadActiveTiles.initial");
    }
    std::unordered_set<std::uint64_t> activeTileKeys;
    activeTileKeys.reserve(tileCoordinates.size());
    for (const auto& [tileX, tileY] : tileCoordinates) {
        activeTileKeys.insert(tileKey(tileX, tileY));
    }
    const auto activeContentReady = [&] {
        return std::all_of(
            tileCoordinates.begin(), tileCoordinates.end(), [this](const auto& coordinate) {
                return loadedContentTileKeys.contains(tileKey(coordinate.first, coordinate.second));
            });
    };
    std::unordered_set<std::uint64_t> terrainTileKeys = activeTileKeys;
    for (const auto& [tileX, tileY] : tileCoordinates) {
        for (int offsetY = -1; offsetY <= 1; ++offsetY) {
            for (int offsetX = -1; offsetX <= 1; ++offsetX) {
                terrainTileKeys.insert(tileKey(tileX + offsetX, tileY + offsetY));
            }
        }
    }
    for (auto failure = tilePreparationFailures.begin();
         failure != tilePreparationFailures.end();) {
        const bool stillRequested = failure->second.terrainPreparation
                                        ? terrainTileKeys.contains(failure->first)
                                        : activeTileKeys.contains(failure->first);
        if (!stillRequested) {
            failure = tilePreparationFailures.erase(failure);
        } else {
            ++failure;
        }
    }
    if (failOnExhaustedRetries) {
        for (const auto& [key, failure] : tilePreparationFailures) {
            const bool stillRequested = failure.terrainPreparation ? terrainTileKeys.contains(key)
                                                                   : activeTileKeys.contains(key);
            if (stillRequested && failure.exhausted) {
                throw std::runtime_error("Initial map tile loading failed: " + failure.diagnostic);
            }
        }
    }
    for (auto request = pendingTileDataRequests.begin();
         request != pendingTileDataRequests.end();) {
        if (!activeTileKeys.contains(request->first) ||
            loadedContentTileKeys.contains(request->first) ||
            pendingTileContentData.contains(request->first)) {
            request = pendingTileDataRequests.erase(request);
        } else {
            ++request;
        }
    }
    for (auto request = pendingTileContentData.begin(); request != pendingTileContentData.end();) {
        if (!activeTileKeys.contains(request->first)) {
            pendingTileAssetKeys.erase(request->first);
            preparedTileRoads.erase(request->first);
            request = pendingTileContentData.erase(request);
        } else {
            ++request;
        }
    }
    for (auto request = preparedTileRoads.begin(); request != preparedTileRoads.end();) {
        if (!activeTileKeys.contains(request->first)) {
            request = preparedTileRoads.erase(request);
        } else {
            ++request;
        }
    }
    for (auto request = pendingTileRoadRequests.begin();
         request != pendingTileRoadRequests.end();) {
        if (!activeTileKeys.contains(request->first)) {
            request = pendingTileRoadRequests.erase(request);
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
        completeSceneryAssetPreparation(key, waitForCompletion);
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
    std::optional<TraceScope> parseSubmissionTrace;
    if (initialLoad) {
        parseSubmissionTrace.emplace("map", "MapSceneLoader.submitActiveTileParses");
    }
    for (const openbus::map::MapTileReference* tileReference : prioritizedActiveTiles) {
        const std::uint64_t key = tileKey(tileReference->x, tileReference->y);
        if (!tilePreparationRetryReady(key) || pendingTileDataRequests.contains(key) ||
            pendingTileContentData.contains(key) ||
            (!waitForCompletion && pendingTileDataRequests.size() >= maxPendingTileParses)) {
            continue;
        }
        pendingTileDataRequests.emplace(
            key, assetRequests_->submit([tileReference = *tileReference] {
                TraceScope parseTrace("map", "MapSceneLoader.parseActiveTile.work");
                return openbus::map::loadMapTile(tileReference);
            }));
    }
    if (parseSubmissionTrace.has_value()) {
        parseSubmissionTrace->finish();
    }

    if (!terrainLoadPhaseLogged) {
        gameLog.Log("Map phase: loading terrain for the active tile window.");
        terrainLoadPhaseLogged = true;
    }
    bool contentChanged = false;
    if (initialLoad) {
        TraceScope terrainTrace("map", "MapSceneLoader.loadActiveTiles.terrain");
        contentChanged = loadTerrainTiles(terrainTileKeys, priorityCenter, waitForCompletion);
    } else {
        contentChanged = loadTerrainTiles(terrainTileKeys, priorityCenter, waitForCompletion);
    }
    const bool terrainReady =
        std::all_of(map.tiles.begin(), map.tiles.end(), [this, &terrainTileKeys](const auto& tile) {
            const std::uint64_t key = tileKey(tile.x, tile.y);
            return !terrainTileKeys.contains(key) || loadedTerrainTileKeys.contains(key);
        });
    const auto reportTileProgress = [&](const char* phase) {
        const auto now = std::chrono::steady_clock::now();
        if (now - lastTileProgressLog < std::chrono::seconds(1)) {
            return;
        }
        lastTileProgressLog = now;
        const auto countActive = [&activeTileKeys](const auto& requests) {
            return static_cast<std::size_t>(std::count_if(
                requests.begin(), requests.end(), [&activeTileKeys](const auto& entry) {
                    return activeTileKeys.contains(entry.first);
                }));
        };
        const AssetRequestManager::QueueStats workerStats = assetRequests_->queueStats();
        const std::size_t contentReady = static_cast<std::size_t>(std::count_if(
            tileCoordinates.begin(), tileCoordinates.end(), [this](const auto& coordinate) {
                return loadedContentTileKeys.contains(tileKey(coordinate.first, coordinate.second));
            }));
        std::size_t parseReady = 0;
        for (const auto& [key, request] : pendingTileDataRequests) {
            if (activeTileKeys.contains(key) &&
                request.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                ++parseReady;
            }
        }
        std::size_t terrainReadyCount = 0;
        for (const std::uint64_t key : terrainTileKeys) {
            terrainReadyCount += loadedTerrainTileKeys.contains(key) ? 1U : 0U;
        }
        gameLog.Debug(
            "Map active tile progress: phase=" + std::string(phase) +
            ", terrain=" + std::to_string(terrainReadyCount) + "/" +
            std::to_string(terrainTileKeys.size()) + ", content=" + std::to_string(contentReady) +
            "/" + std::to_string(tileCoordinates.size()) +
            ", parse futures=" + std::to_string(countActive(pendingTileDataRequests)) + " (" +
            std::to_string(parseReady) + " ready), parsed waiting for assets=" +
            std::to_string(countActive(pendingTileContentData)) +
            ", scenery batches=" + std::to_string(countActive(pendingTileSceneryRequests)) +
            ", road preparations=" + std::to_string(countActive(pendingTileRoadRequests)) +
            ", prepared roads=" + std::to_string(countActive(preparedTileRoads)) +
            ", asset workers active/total=" + std::to_string(workerStats.activeWorkers) + "/" +
            std::to_string(workerStats.workerCount) +
            ", queued=" + std::to_string(workerStats.queuedTasks) +
            ", peak active/queued=" + std::to_string(workerStats.peakActiveWorkers) + "/" +
            std::to_string(workerStats.peakQueuedTasks) + ".");
    };
    bool laneNetworksReady = false;
    if (initialLoad) {
        TraceScope laneTrace("map", "MapSceneLoader.loadActiveTiles.finishLaneNetworks");
        laneNetworksReady = finishTrafficLaneNetworkPreparation(waitForCompletion);
    } else {
        laneNetworksReady = finishTrafficLaneNetworkPreparation(waitForCompletion);
    }
    if (!laneNetworksReady) {
        reportTileProgress("traffic lane preparation");
        return TileLoadStatus::Pending;
    }
    if (!terrainReady) {
        reportTileProgress("terrain");
        return TileLoadStatus::Pending;
    }
    if (!roadVertices.empty()) {
        static_cast<void>(uploadRoadBatches(waitForCompletion));
        if (!waitForCompletion) {
            if (!roadVertices.empty() || !activeContentReady()) {
                if (!finishTrafficLaneNetworkPreparation(waitForCompletion)) {
                    reportTileProgress("traffic lane preparation");
                    return TileLoadStatus::Pending;
                }
                reportTileProgress("road upload");
                return TileLoadStatus::Pending;
            }
            if (!finishTrafficLaneNetworkPreparation(waitForCompletion)) {
                reportTileProgress("traffic lane preparation");
                return TileLoadStatus::Pending;
            }
            return TileLoadStatus::Updated;
        }
    }
    std::optional<TraceScope> contentTrace;
    if (initialLoad) {
        contentTrace.emplace("map", "MapSceneLoader.prepareActiveTileContent");
    }
    for (const openbus::map::MapTileReference* tileReference : prioritizedActiveTiles) {
        const std::uint64_t key = tileKey(tileReference->x, tileReference->y);
        if (!activeTileKeys.contains(key) || loadedContentTileKeys.contains(key)) {
            continue;
        }
        if (!tilePreparationRetryReady(key)) {
            continue;
        }
        auto tileData = pendingTileContentData.find(key);
        if (tileData == pendingTileContentData.end()) {
            auto request = pendingTileDataRequests.find(key);
            if (request == pendingTileDataRequests.end()) {
                continue;
            }
            if (waitForCompletion) {
                request->second.wait();
            } else if (request->second.wait_for(std::chrono::seconds(0)) !=
                       std::future_status::ready) {
                continue;
            }
            openbus::map::MapTileData parsedTile;
            try {
                parsedTile = request->second.get();
            } catch (...) {
                const std::exception_ptr error = std::current_exception();
                pendingTileDataRequests.erase(request);
                recordTilePreparationFailure(key, false, "tile parsing", tileReference->textPath,
                                             error, waitForCompletion);
                continue;
            }
            pendingTileDataRequests.erase(request);
            tilePreparationFailures.erase(key);
            tileData = pendingTileContentData.emplace(key, std::move(parsedTile)).first;
        }
        auto assetKeys = pendingTileAssetKeys.find(key);
        if (assetKeys == pendingTileAssetKeys.end()) {
            assetKeys =
                pendingTileAssetKeys.emplace(key, requestTileSceneryAssets(tileData->second, key))
                    .first;
        }
        if (assetKeys != pendingTileAssetKeys.end() &&
            !sceneryAssetPreparationsReady(assetKeys->second)) {
            if (waitForCompletion) {
                while (!sceneryAssetPreparationsReady(assetKeys->second)) {
                    reportTileProgress("scenery/model preparation");
                    if (!pendingTileSceneryRequests.contains(key)) {
                        static_cast<void>(requestTileSceneryAssets(tileData->second, key));
                    }
                    if (pendingTileSceneryRequests.contains(key)) {
                        static_cast<void>(completeSceneryAssetPreparation(key, true));
                    }
                }
            } else {
                if (!pendingTileSceneryRequests.contains(key)) {
                    static_cast<void>(requestTileSceneryAssets(tileData->second, key));
                }
                continue;
            }
        }
        if (!requestTileRoadPreparation(tileData->second, *tileReference, key)) {
            continue;
        }
        if (!preparedTileRoads.contains(key)) {
            auto roadRequest = pendingTileRoadRequests.find(key);
            if (roadRequest == pendingTileRoadRequests.end()) {
                throw std::logic_error("Active map tile has no road preparation request");
            }
            if (waitForCompletion) {
                roadRequest->second.wait();
            } else if (roadRequest->second.wait_for(std::chrono::seconds(0)) !=
                       std::future_status::ready) {
                continue;
            }
            try {
                preparedTileRoads.emplace(key, roadRequest->second.get());
            } catch (...) {
                const std::exception_ptr error = std::current_exception();
                pendingTileRoadRequests.erase(roadRequest);
                recordTilePreparationFailure(key, false, "road geometry preparation",
                                             tileReference->textPath, error, waitForCompletion);
                continue;
            }
            pendingTileRoadRequests.erase(roadRequest);
        }
        tilePreparationFailures.erase(key);
        gameLog.Debug("Map phase: loading active tile " +
                      tileLabel(tileReference->x, tileReference->y) + ".");
        const auto roadData = preparedTileRoads.find(key);
        if (roadData == preparedTileRoads.end()) {
            throw std::logic_error("Active map tile road data was not prepared");
        }
        buildTileContent(*tileReference, tileData->second, roadData->second);
        loadedContentTileKeys.insert(key);
        pendingTileContentData.erase(tileData);
        pendingTileAssetKeys.erase(key);
        preparedTileRoads.erase(roadData);
        contentChanged = true;
        resolveTrafficLightVariableParents();
        static_cast<void>(uploadRoadBatches(waitForCompletion));
        if (!waitForCompletion && !roadVertices.empty()) {
            if (!finishTrafficLaneNetworkPreparation(waitForCompletion)) {
                reportTileProgress("traffic lane preparation");
                return TileLoadStatus::Pending;
            }
            reportTileProgress("road upload");
            return TileLoadStatus::Pending;
        }
        if (!waitForCompletion) {
            break;
        }
    }
    if (contentTrace.has_value()) {
        contentTrace->finish();
    }
    if (!contentChanged) {
        if (!finishTrafficLaneNetworkPreparation(waitForCompletion)) {
            reportTileProgress("traffic lane preparation");
            return TileLoadStatus::Pending;
        }
        const bool activeTilesReady = std::all_of(
            tileCoordinates.begin(), tileCoordinates.end(), [this](const auto& coordinate) {
                return loadedContentTileKeys.contains(tileKey(coordinate.first, coordinate.second));
            });
        if (!activeTilesReady) {
            reportTileProgress("tile content");
            return TileLoadStatus::Pending;
        }
        return TileLoadStatus::Ready;
    }
    if (!finishTrafficLaneNetworkPreparation(waitForCompletion)) {
        reportTileProgress("traffic lane preparation");
        return TileLoadStatus::Pending;
    }
    const bool activeTilesReady =
        std::all_of(tileCoordinates.begin(), tileCoordinates.end(), [this](const auto& coordinate) {
            return loadedContentTileKeys.contains(tileKey(coordinate.first, coordinate.second));
        });
    if (!activeTilesReady) {
        reportTileProgress("tile content");
        return TileLoadStatus::Pending;
    }
    return TileLoadStatus::Updated;
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
        const bool resolveInheritedTrafficHead = scriptState->trafficLightModel &&
                                                 scriptState->variableParentId.has_value() &&
                                                 scriptState->trafficSignalsByLightIndex.empty();
        if (resolveInheritedTrafficHead) {
            // SCO paths can arrive in later tile-content updates, so earlier matches are
            // provisional.
            scriptState->trafficSignal.reset();
            scriptState->resolvedSignalParentObjectId = scriptState->variableParentId;
        } else if (scriptState->trafficSignal.has_value() ||
                   !scriptState->variableParentId.has_value()) {
            continue;
        }
        std::optional<int> parentObjectId = scriptState->variableParentId;
        std::vector<int> parentChain;
        std::unordered_set<int> visitedParentIds;
        while (parentObjectId.has_value() && visitedParentIds.insert(*parentObjectId).second &&
               !ambiguousStateObjectIds.contains(*parentObjectId)) {
            parentChain.push_back(*parentObjectId);
            const auto parentState = statesByObjectId.find(*parentObjectId);
            if (parentState != statesByObjectId.end()) {
                const auto& parentSchedules = parentState->second->trafficSignalsByLightIndex;
                if (parentSchedules.empty() && parentState->second->trafficSignal.has_value()) {
                    scriptState->trafficSignal = parentState->second->trafficSignal;
                } else if (parentSchedules.size() == 1) {
                    scriptState->trafficSignal = parentSchedules.begin()->second;
                } else {
                    for (const std::string& label : scriptState->placementLabels) {
                        int lightIndex = -1;
                        const auto [end, error] =
                            std::from_chars(label.data(), label.data() + label.size(), lightIndex);
                        if (error != std::errc{} || end != label.data() + label.size() ||
                            lightIndex < 0) {
                            continue;
                        }
                        const auto schedule =
                            parentSchedules.find(static_cast<std::size_t>(lightIndex));
                        if (schedule != parentSchedules.end()) {
                            scriptState->trafficSignal = schedule->second;
                            break;
                        }
                    }
                }
                if (scriptState->trafficSignal.has_value()) {
                    scriptState->resolvedSignalParentObjectId = *parentObjectId;
                    break;
                }
                if (!parentSchedules.empty()) {
                    break;
                }
            }

            const auto attachedParent = attachedObjectsById.find(*parentObjectId);
            if (attachedParent != attachedObjectsById.end()) {
                parentObjectId = attachedParent->second.parentId;
            } else {
                const auto variableParent = mapObjectVariableParentsById.find(*parentObjectId);
                parentObjectId = variableParent == mapObjectVariableParentsById.end()
                                     ? std::nullopt
                                     : std::optional<int>(variableParent->second);
            }
        }
        if (!scriptState->trafficSignal.has_value() &&
            (scriptState->trafficLightModel || scriptState->variableParentId.has_value())) {
            constexpr double maxSignalHeadPathDistanceMeters = 30.0;
            constexpr double ambiguousSignalPathDistanceMeters = 1.0;
            struct NearbySignalPath {
                const TrafficPathDebugSegment* segment = nullptr;
                double distanceSquared = std::numeric_limits<double>::infinity();
            };
            std::vector<NearbySignalPath> nearbySignalPaths;
            for (const TrafficPathDebugSegment& segment : impl_->sceneryTrafficPathSegments) {
                if (!segment.trafficSignal.has_value() ||
                    segment.pathTypeCode != openbus::map::MAP_SPLINE_PATH_TYPE_ROAD ||
                    segment.points.size() < 2) {
                    continue;
                }
                double distanceSquared = std::numeric_limits<double>::infinity();
                for (std::size_t pointIndex = 1; pointIndex < segment.points.size(); ++pointIndex) {
                    const auto& start = segment.points[pointIndex - 1];
                    const auto& end = segment.points[pointIndex];
                    const double dx = end[0] - start[0];
                    const double dy = end[1] - start[1];
                    const double lengthSquared = dx * dx + dy * dy;
                    const double projection =
                        lengthSquared > 1.0e-12
                            ? std::clamp(((scriptState->worldPosition[0] - start[0]) * dx +
                                          (scriptState->worldPosition[1] - start[1]) * dy) /
                                             lengthSquared,
                                         0.0, 1.0)
                            : 0.0;
                    const double offsetX =
                        scriptState->worldPosition[0] - start[0] - projection * dx;
                    const double offsetY =
                        scriptState->worldPosition[1] - start[1] - projection * dy;
                    distanceSquared =
                        std::min(distanceSquared, offsetX * offsetX + offsetY * offsetY);
                }
                nearbySignalPaths.push_back({&segment, distanceSquared});
            }
            const auto belongsToParentChain = [&](const TrafficPathDebugSegment& segment) {
                return segment.mapObjectId.has_value() &&
                       std::find(parentChain.begin(), parentChain.end(),
                                 *segment.mapObjectId) != parentChain.end();
            };
            const bool hasParentOwnedSignalPaths =
                std::any_of(nearbySignalPaths.begin(), nearbySignalPaths.end(),
                            [&](const NearbySignalPath& candidate) {
                                return belongsToParentChain(*candidate.segment);
                            });
            if (hasParentOwnedSignalPaths) {
                std::erase_if(nearbySignalPaths, [&](const NearbySignalPath& candidate) {
                    return !belongsToParentChain(*candidate.segment);
                });
            }
            std::sort(nearbySignalPaths.begin(), nearbySignalPaths.end(),
                      [](const NearbySignalPath& first, const NearbySignalPath& second) {
                          return first.distanceSquared < second.distanceSquared;
                      });
            if (!nearbySignalPaths.empty() &&
                (hasParentOwnedSignalPaths ||
                 nearbySignalPaths.front().distanceSquared <=
                     maxSignalHeadPathDistanceMeters * maxSignalHeadPathDistanceMeters)) {
                const TrafficPathDebugSegment& nearest = *nearbySignalPaths.front().segment;
                const auto sameSignal = [&](const TrafficPathDebugSegment& candidate) {
                    const auto& nearestSignal = *nearest.trafficSignal;
                    const auto& candidateSignal = *candidate.trafficSignal;
                    return nearestSignal.lightIndex == candidateSignal.lightIndex &&
                           ((nearestSignal.controller != nullptr &&
                             nearestSignal.controller == candidateSignal.controller) ||
                            (nearestSignal.controller == nullptr &&
                             candidateSignal.controller == nullptr &&
                             nearest.mapObjectId == candidate.mapObjectId));
                };
                const double nearestDistance = std::sqrt(nearbySignalPaths.front().distanceSquared);
                bool ambiguous = false;
                for (std::size_t index = 1; index < nearbySignalPaths.size(); ++index) {
                    const NearbySignalPath& candidate = nearbySignalPaths[index];
                    if (std::sqrt(candidate.distanceSquared) - nearestDistance >=
                        ambiguousSignalPathDistanceMeters) {
                        break;
                    }
                    if (!sameSignal(*candidate.segment)) {
                        ambiguous = true;
                        break;
                    }
                }
                if (!ambiguous) {
                    scriptState->trafficSignal = nearest.trafficSignal;
                    scriptState->resolvedSignalParentObjectId = nearest.mapObjectId;
                }
            }
        }
        if (!scriptState->trafficSignal.has_value()) {
            const int unresolvedParentId = parentChain.empty() ? 0 : parentChain.back();
            const auto parentTile = mapObjectTileKeysById.find(unresolvedParentId);
            if (parentTile != mapObjectTileKeysById.end() &&
                !ambiguousStateObjectIds.contains(unresolvedParentId) &&
                !loadedContentTileKeys.contains(parentTile->second)) {
                continue;
            }
            std::ostringstream chain;
            for (const int id : parentChain) {
                if (chain.tellp() > 0) {
                    chain << " -> ";
                }
                chain << id;
            }
            if (scriptState->trafficLightModel) {
                recordSkipped(skipped,
                              "[trafficlight] variable parents without a matching schedule",
                              "object ID=" +
                                  (scriptState->mapObjectId.has_value()
                                       ? std::to_string(*scriptState->mapObjectId)
                                       : std::string("unknown")) +
                                  " parent chain=" + chain.str());
            }
        }
    }
}

void MapSceneLoader::configureGroundTextures() {
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
}

void MapSceneLoader::loadGroundTextures() {
    const std::filesystem::path texturePath =
        groundTexturePath(map, omsiRoot, map.groundTextures[baseGroundTextureIndex].texturePath);
    if (uniformGroundTexturePreview) {
        gameLog.Log("Map base-ground-texture preview override: [groundtex] index=" +
                    std::to_string(groundTextureIndex) +
                    ", path=" + openbus::pathToUtf8(texturePath) + "; applied uniformly.");
    } else {
        gameLog.Log("Map base-ground-texture: [groundtex] index=0, path=" +
                    openbus::pathToUtf8(texturePath) + "; per-tile ground-texture masks enabled.");
    }
    {
        TraceScope phase("map", "MapRenderer.loadGroundTexture");
        impl_->texture = uploadPreparedTexture(texturePath, true);
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
    gameLog.Log("Map loading started: root=" + openbus::pathToUtf8(map.rootPath) +
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
                              tileLabel(tile.x, tile.y) + " " +
                                  openbus::pathToUtf8(tile.textPath.filename()));
                terrainChanged = true;
                continue;
            }
            if (!tilePreparationRetryReady(key)) {
                continue;
            }
            auto request = pendingTerrainRequests.find(key);
            if (request == pendingTerrainRequests.end() &&
                (waitForCompletion ||
                 pendingTerrainRequests.size() < maxPendingTerrainPreparations)) {
                const std::filesystem::path terrainPath = tile.terrainPath;
                const double tileSize = map.coordinates.tileSize;
                const std::filesystem::path mapRootPath = map.rootPath;
                const std::filesystem::path omsiRootPath = omsiRoot;
                const std::vector<openbus::map::GroundTextureDefinition> groundTextures =
                    map.groundTextures;
                const std::vector<openbus::map::MapGroundTextureSidecar> groundTextureSidecars =
                    tile.groundTextureSidecars;
                request =
                    pendingTerrainRequests
                        .emplace(key, assetRequests_->submit([terrainPath, tileSize, mapRootPath,
                                                              omsiRootPath, groundTextures,
                                                              groundTextureSidecars] {
                            TraceScope preparationTrace("map",
                                                        "MapSceneLoader.prepareTerrainTile.work");
                            PreparedTerrainTile prepared;
                            prepared.grid = openbus::map::loadTerrainGrid(terrainPath);
                            if (!groundTextureSidecars.empty()) {
                                openbus::map::MapDefinition textureMap;
                                textureMap.rootPath = mapRootPath;
                                std::map<std::size_t, const openbus::map::MapGroundTextureSidecar*>
                                    layersByIndex;
                                for (const auto& sidecar : groundTextureSidecars) {
                                    const auto textureIndex =
                                        openbus::map::resolveMapGroundTextureSidecarIndex(
                                            sidecar.ordinalSuffix, groundTextures.size());
                                    if (textureIndex) {
                                        layersByIndex.emplace(*textureIndex, &sidecar);
                                    }
                                }
                                std::unordered_set<std::string> requestedTexturePaths;
                                const auto decodeTexture = [&](const std::filesystem::path& path) {
                                    const std::string imageKey =
                                        openbus::pathToUtf8(path.lexically_normal());
                                    if (!requestedTexturePaths.insert(imageKey).second) {
                                        return;
                                    }
                                    Image image;
                                    try {
                                        if (!TextureLoader::readImage(path, image)) {
                                            prepared.textureErrors.emplace(imageKey,
                                                                           "image decode failed");
                                        } else {
                                            prepared.decodedTextures.emplace(
                                                imageKey,
                                                std::make_shared<Image>(std::move(image)));
                                        }
                                    } catch (const std::exception& error) {
                                        prepared.textureErrors.emplace(imageKey, error.what());
                                    } catch (...) {
                                        prepared.textureErrors.emplace(
                                            imageKey, "unknown image decode error");
                                    }
                                };
                                for (const auto& [textureIndex, sidecar] : layersByIndex) {
                                    const auto& groundTexture = groundTextures[textureIndex];
                                    try {
                                        decodeTexture(groundTexturePath(textureMap, omsiRootPath,
                                                                        groundTexture.texturePath));
                                    } catch (const std::exception&) {
                                        // The renderer reports unresolved paths with tile context.
                                    }
                                    if (!groundTexture.detailTexturePath.empty()) {
                                        try {
                                            decodeTexture(
                                                groundTexturePath(textureMap, omsiRootPath,
                                                                  groundTexture.detailTexturePath));
                                        } catch (const std::exception&) {
                                            // Detail paths are optional and logged at publication.
                                        }
                                    }
                                    decodeTexture(sidecar->path);
                                }
                            }
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
            PreparedTerrainTile prepared;
            try {
                prepared = request->second.get();
            } catch (...) {
                const std::exception_ptr error = std::current_exception();
                pendingTerrainRequests.erase(request);
                recordTilePreparationFailure(key, true, "terrain preparation", tile.terrainPath,
                                             error, waitForCompletion);
                continue;
            }
            pendingTerrainRequests.erase(request);
            tilePreparationFailures.erase(key);
            for (auto& [imageKey, image] : prepared.decodedTextures) {
                preparedTextureImages.try_emplace(imageKey, std::move(image));
            }
            for (auto& [imageKey, error] : prepared.textureErrors) {
                if (!preparedTextureImages.contains(imageKey)) {
                    preparedTextureErrors.try_emplace(imageKey, std::move(error));
                }
            }
            gameLog.Debug("Map terrain tile " + std::to_string(terrainTilesVisited) + ": " +
                          openbus::pathToUtf8(tile.terrainPath) +
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
                        recordSkipped(
                            skipped, "ground-texture sidecars with invalid groundtex indices",
                            openbus::pathToUtf8(tile.textPath.filename()) +
                                " sidecar=" + openbus::pathToUtf8(sidecar.path.filename()));
                        continue;
                    }
                    const std::size_t textureIndex = *resolvedIndex;
                    if (!layersByIndex.emplace(textureIndex, &sidecar).second) {
                        recordSkipped(skipped, "duplicate ground-texture sidecar indices",
                                      openbus::pathToUtf8(tile.textPath.filename()) +
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
                                    openbus::pathToUtf8(sidecar->path.filename()) +
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
                                      openbus::pathToUtf8(sidecar->path.filename()) + ": " +
                                          error.what());
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
                    deleteRendererBuffers(1, &buffer.buffer);
                }
                if (buffer.indexBuffer != 0) {
                    deleteRendererBuffers(1, &buffer.indexBuffer);
                }
                throw std::runtime_error("Could not upload terrain mesh for " +
                                         openbus::pathToUtf8(tile.textPath.filename()));
            }
            impl_->terrainBuffers.push_back(buffer);
            ++impl_->terrainTilesByTile[tileKey(buffer.tileX, buffer.tileY)];
            loadedTerrainTileKeys.insert(key);
            terrainChanged = true;
            ++terrainUploads;
        }
    }
    if (terrainTilesVisited > 0 || terrainUploads > 0) {
        gameLog.Debug("Map active terrain load progress: " + std::to_string(terrainTilesVisited) +
                      " tiles visited, " + std::to_string(terrainUploads) + " uploaded, " +
                      std::to_string(impl_->terrainBuffers.size()) + " total meshes in " +
                      std::to_string(elapsedMilliseconds(terrainLoadStart)) + " ms.");
    }
    return terrainChanged;
}

void MapSceneLoader::indexMapContent() {
    TraceScope trace("map", "MapSceneLoader::indexMapContent.work");
    const auto indexStart = std::chrono::steady_clock::now();
    const char* configuredDirectionPolicy =
        openbus::getEnvironment("OPENBUS_MIRRORED_ROAD_DIRECTIONS");
    const auto mirroredRoadPolicy = openbus::map::parseMirroredRoadDirectionPolicy(
        configuredDirectionPolicy == nullptr ? "" : configuredDirectionPolicy);
    if (!mirroredRoadPolicy) {
        throw std::runtime_error(
            "OPENBUS_MIRRORED_ROAD_DIRECTIONS must be 'correct' or 'authored'");
    }
    if (*mirroredRoadPolicy == openbus::map::MirroredRoadDirectionPolicy::Authored) {
        gameLog.Log("Mirrored road directions use authored values "
                    "(OPENBUS_MIRRORED_ROAD_DIRECTIONS=authored).");
    }
    std::unordered_set<int> referencedSplineAttachmentIds;
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
        gameLog.Debug("Map index tile " + std::to_string(tileReferenceIndex + 1) + "/" +
                      std::to_string(map.tiles.size()) + ": " +
                      openbus::pathToUtf8(tileReference.textPath) +
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
                            *profile, openbus::map::MAP_SPLINE_PATH_TYPE_ROAD, spline.mirrored,
                            *mirroredRoadPolicy);
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
            if (object.variableParentId.has_value()) {
                mapObjectVariableParentsById.try_emplace(object.id, *object.variableParentId);
            }
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
            referencedSplineAttachmentIds.insert(object.attachedToObjectId);
            registerObjectId(object.id, MapObjectRecordKind::Attached, tileName, tileReference.x,
                             tileReference.y);
            mapObjectAssetPathsById.try_emplace(object.id, object.assetPath);
            if (object.variableParentId.has_value()) {
                mapObjectVariableParentsById.try_emplace(object.id, *object.variableParentId);
            }
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
            if (attachment.variableParentId.has_value()) {
                mapObjectVariableParentsById.try_emplace(attachment.id,
                                                         *attachment.variableParentId);
            }
            if (!splineAttachmentTileById.try_emplace(attachment.id, tileReferenceIndex).second) {
                recordSkipped(skipped, "map placements with duplicate object IDs",
                              tileLabel(tileReference.x, tileReference.y) +
                                  " spline attachment ID=" + std::to_string(attachment.id));
            }
        }
        if (!tile.splineAttachments.empty()) {
            openbus::map::MapTileData parentData;
            parentData.reference = tile.reference;
            parentData.splines = tile.splines;
            parentData.splineAttachments = tile.splineAttachments;
            splineAttachmentParentTiles.emplace(tileReferenceIndex, std::move(parentData));
        }
    }
    for (auto tile = splineAttachmentParentTiles.begin();
         tile != splineAttachmentParentTiles.end();) {
        auto& attachments = tile->second.splineAttachments;
        std::erase_if(attachments, [&referencedSplineAttachmentIds](
                                       const openbus::map::MapSplineAttachment& attachment) {
            return !referencedSplineAttachmentIds.contains(attachment.id);
        });
        if (attachments.empty()) {
            tile = splineAttachmentParentTiles.erase(tile);
        } else {
            ++tile;
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

bool MapSceneLoader::uploadRoadBatches(bool waitForCompletion) {
    const auto roadUploadStart = std::chrono::steady_clock::now();
    TraceScope roadUploadPhase("map", "MapRenderer.uploadRoadBatches");
    constexpr std::size_t maxRoadBatchesPerFrame = 4;
    std::size_t processedBatches = 0;
    for (auto geometryIt = roadVertices.begin(); geometryIt != roadVertices.end();) {
        auto& [batchKey, geometry] = *geometryIt;
        if (geometry.vertices.empty()) {
            geometryIt = roadVertices.erase(geometryIt);
            continue;
        }
        if (!waitForCompletion && processedBatches >= maxRoadBatchesPerFrame) {
            break;
        }
        ++processedBatches;
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
                              openbus::pathToUtf8(geometry.texturePath) + ": " + error.what());
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
                deleteRendererBuffers(1, &batch.buffer);
            }
            if (batch.indexBuffer != 0) {
                deleteRendererBuffers(1, &batch.indexBuffer);
            }
            recordSkipped(skipped, "road batches rejected by OpenGL upload", batchKey);
            geometryIt = roadVertices.erase(geometryIt);
            continue;
        }
        impl_->roadBatches.push_back(batch);
        ++impl_->roadBatchesByTile[tileKey(batch.tileX, batch.tileY)];
        geometryIt = roadVertices.erase(geometryIt);
    }
    if (roadVertices.empty()) {
        gameLog.Log("Map road upload complete: uploaded batches=" +
                    std::to_string(impl_->roadBatches.size()) + " in " +
                    std::to_string(elapsedMilliseconds(roadUploadStart)) + " ms.");
    }
    return processedBatches != 0;
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

GLuint MapSceneLoader::uploadPreparedTexture(const std::filesystem::path& path, bool repeat) const {
    const std::string imageKey = openbus::pathToUtf8(path.lexically_normal());
    const auto textureError = preparedTextureErrors.find(imageKey);
    if (textureError != preparedTextureErrors.end()) {
        throw std::runtime_error("Could not decode map texture " + openbus::pathToUtf8(path) +
                                 ": " + textureError->second);
    }
    const auto preparedImage = preparedTextureImages.find(imageKey);
    if (preparedImage != preparedTextureImages.end() && preparedImage->second) {
        try {
            return uploadTextureImage(*preparedImage->second, repeat);
        } catch (const std::exception& error) {
            throw std::runtime_error("Could not upload map texture " + openbus::pathToUtf8(path) +
                                     ": " + error.what());
        }
    }
    return loadTexture(path, repeat);
}

GLuint MapSceneLoader::getTexture(const std::filesystem::path& path, bool repeat) {
    const std::string key =
        openbus::pathToUtf8(path.lexically_normal()) + (repeat ? "|repeat" : "|clamp");
    const auto found = textureCache.find(key);
    if (found != textureCache.end()) {
        return found->second;
    }
    const GLuint texture = uploadPreparedTexture(path, repeat);
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
