// Scenery object (.sco) and spline profile (.sli) asset loading for map content.

#include "render/map/MapSceneLoader.h"

#include "core/Utf8Path.h"

#include <fstream>

namespace openbus::rendering {

const std::vector<std::string>& MapSceneLoader::loadCarparkVehicleList(int listIndex) {
    const auto [entry, inserted] = carparkVehicleLists_.try_emplace(listIndex);
    if (!inserted) {
        return entry->second;
    }

    const std::string filename =
        listIndex == 0 ? "parklist_p.txt" : "parklist_p_" + std::to_string(listIndex) + ".txt";
    const std::filesystem::path requestedPath = map.rootPath / filename;
    const std::filesystem::path listPath =
        openbus::assets::resolveCaseInsensitivePath(requestedPath);
    std::ifstream input(listPath);
    if (!input) {
        recordSkipped(skipped, "[carpark_p] park lists that could not be read",
                      openbus::pathToUtf8(requestedPath), 1, LogLevel::Warning);
        return entry->second;
    }

    std::vector<std::string> availableEntries;
    std::string line;
    while (std::getline(input, line)) {
        if (availableEntries.empty() && line.size() >= 3 &&
            static_cast<unsigned char>(line[0]) == 0xEF &&
            static_cast<unsigned char>(line[1]) == 0xBB &&
            static_cast<unsigned char>(line[2]) == 0xBF) {
            line.erase(0, 3);
        }
        const std::size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos || line[first] == ';' || line[first] == '#') {
            continue;
        }
        const std::size_t last = line.find_last_not_of(" \t\r\n");
        const std::string assetPath = line.substr(first, last - first + 1);
        if (resolveSceneryConfigPath(assetPath).empty()) {
            recordSkipped(skipped, "[carpark_p] park-list entries with missing scenery configs",
                          assetPath, 1, LogLevel::Warning);
            continue;
        }
        availableEntries.push_back(assetPath);
    }
    if (input.bad()) {
        recordSkipped(skipped, "[carpark_p] park lists that could not be read completely",
                      openbus::pathToUtf8(listPath), 1, LogLevel::Warning);
    }
    if (availableEntries.empty()) {
        recordSkipped(skipped, "[carpark_p] park lists with no available vehicle configs",
                      openbus::pathToUtf8(listPath), 1, LogLevel::Warning);
    }
    entry->second = std::move(availableEntries);
    return entry->second;
}

std::filesystem::path MapSceneLoader::resolveSceneryConfigPath(const std::string& assetPath) const {
    const std::filesystem::path relative = normalizedAssetPath(assetPath);
    const std::array<std::filesystem::path, 2> candidates = {omsiRoot / relative,
                                                             map.rootPath / relative};
    for (const std::filesystem::path& candidate : candidates) {
        const std::filesystem::path resolved =
            openbus::assets::resolveCaseInsensitivePath(candidate);
        if (std::filesystem::is_regular_file(resolved)) {
            return resolved;
        }
    }
    return std::filesystem::path{};
}

const SceneryObjectConfig*
MapSceneLoader::loadSceneryObjectMetadata(const std::string& assetPath) {
    const std::filesystem::path configPath = resolveSceneryConfigPath(assetPath);
    if (configPath.empty()) {
        return nullptr;
    }
    const std::string cacheKey = openbus::pathToUtf8(configPath.lexically_normal());
    const auto cached = sceneryObjectMetadata.find(cacheKey);
    if (cached != sceneryObjectMetadata.end()) {
        return &cached->second;
    }
    try {
        const auto [entry, inserted] =
            sceneryObjectMetadata.emplace(cacheKey, loadSceneryObjectFile(configPath));
        if (!inserted) {
            throw std::logic_error("Scenery metadata cache insertion failed");
        }
        return &entry->second;
    } catch (const std::exception& error) {
        recordSkipped(skipped, "scenery object metadata unavailable for map indexing",
                      openbus::pathToUtf8(configPath) + ": " + error.what(), 1,
                      LogLevel::Warning);
        return nullptr;
    }
}

const std::vector<std::size_t>&
MapSceneLoader::loadSceneryAsset(const std::filesystem::path& configPath) {
    const std::string cacheKey = openbus::pathToUtf8(configPath.lexically_normal());
    const auto cached = sceneryAssetBatches.find(cacheKey);
    if (cached != sceneryAssetBatches.end()) {
        return cached->second;
    }
    TraceScope trace("map", "MapRenderer.loadSceneryAsset");
    std::vector<std::size_t> batchIndexes;
    try {
        PreparedSceneryAsset prepared;
        auto preparation = preparedSceneryAssetConfigs.find(cacheKey);
        if (preparation == preparedSceneryAssetConfigs.end()) {
            prepared = prepareSceneryAsset(configPath, omsiRoot, *sharedSceneryTextureImages_);
        } else {
            prepared = std::move(*preparation->second);
            preparedSceneryAssetConfigs.erase(preparation);
        }
        if (prepared.preparationError) {
            std::rethrow_exception(prepared.preparationError);
        }
        sceneryTextureLookups += prepared.textureLookups;
        sceneryTextureResolutions += prepared.textureResolutions;
        for (auto& [path, image] : prepared.decodedTextures) {
            rememberPreparedTexture(path, std::move(image));
        }
        const SceneryObjectConfig& objectConfiguration = prepared.objectConfiguration;
        sceneryAttachmentPoints[cacheKey] = objectConfiguration.attachmentPoints;
        sceneryTrafficPaths[cacheKey] = objectConfiguration.trafficPaths;
        sceneryTrafficLightSchedules[cacheKey] = objectConfiguration.trafficLightSchedules;
        sceneryScriptConfigurations[cacheKey] = objectConfiguration.scriptConfiguration;
        if (objectConfiguration.trafficLight) {
            sceneryTrafficLightAssets.insert(cacheKey);
        }
        if (objectConfiguration.poleCrash.has_value()) {
            poleCrashSceneryAssets.insert(cacheKey);
        }
        if (objectConfiguration.onlyEditor &&
            (objectConfiguration.busStop || objectConfiguration.hasPassengerCabin)) {
            editorPassengerAssets.emplace(
                cacheKey, EditorPassengerAsset{
                              objectConfiguration.busStop, objectConfiguration.hasPassengerCabin,
                              objectConfiguration.hasBoundingBox, objectConfiguration.boundingBox,
                              objectConfiguration.passengerPositions});
        }
        // Editor-only assets can still contribute path debug geometry.
        if (objectConfiguration.absoluteHeight) {
            absoluteHeightSceneryAssets.insert(cacheKey);
        }
        if (!objectConfiguration.scriptConfiguration.scripts.empty()) {
            sceneryAssetsNeedingVariables.insert(cacheKey);
        }
        const std::filesystem::path& modelConfigPath = prepared.modelConfigPath;
        if (prepared.externalModelConfigMissing) {
            recordSkipped(skipped, "scenery external model configs not found",
                          openbus::pathToUtf8(configPath.filename()) +
                              " model=" + openbus::pathToUtf8(prepared.externalModelConfigPath));
        }
        for (const ConfigurationDiagnostic& diagnostic : objectConfiguration.diagnostics.entries) {
            const std::string severity =
                diagnostic.severity == ConfigurationDiagnostic::Severity::Error ? "error"
                                                                                : "warning";
            recordSkipped(skipped, "scenery config diagnostics",
                          openbus::pathToUtf8(configPath.filename()) +
                              " line=" + std::to_string(diagnostic.line) + " [" +
                              diagnostic.keyword + "] " + severity + ": " + diagnostic.message,
                          1,
                          diagnostic.severity == ConfigurationDiagnostic::Severity::Error
                              ? LogLevel::Error
                              : LogLevel::Warning);
        }
        // Tree SCOs may mark their helper mesh [onlyeditor] while their [tree]
        // definition is runtime scenery. The tree preview below replaces that mesh.
        if (objectConfiguration.onlyEditor && objectConfiguration.trees.empty()) {
            editorOnlySceneryAssets.insert(cacheKey);
            return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes)).first->second;
        }
        if (!objectConfiguration.trees.empty()) {
            for (const PreparedSceneryTreeBatch& preparedTree : prepared.treeBatches) {
                if (preparedTree.texturePath.empty()) {
                    recordSkipped(skipped, "tree scenery definitions with missing textures",
                                  openbus::pathToUtf8(configPath.filename()) +
                                      " texture=" + preparedTree.textureName);
                    continue;
                }
                Impl::ModelBatch batch = preparedTree.batch;
                batch.material.texture = getTexture(preparedTree.texturePath);
                batch.material.textured = true;
                pglGenBuffers(1, &batch.buffer);
                pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                pglBufferData(
                    GL_ARRAY_BUFFER,
                    static_cast<std::ptrdiff_t>(preparedTree.vertices.size() * sizeof(ModelVertex)),
                    preparedTree.vertices.data(), GL_STATIC_DRAW);
                if (batch.buffer == 0 || glGetError() != GL_NO_ERROR) {
                    if (batch.buffer != 0) {
                        deleteRendererBuffers(1, &batch.buffer);
                    }
                    recordSkipped(skipped, "tree scenery batches rejected by OpenGL upload",
                                  openbus::pathToUtf8(configPath.filename()));
                    continue;
                }
                batchIndexes.push_back(impl_->sceneryBatches.size());
                impl_->sceneryBatches.push_back(batch);
            }
            return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes)).first->second;
        }
        const openbus::scripting::SceneryObject& variables = prepared.variables;
        const ModelConfig& configuration = prepared.modelConfiguration;
        const std::size_t lodThresholdSetIndex = impl_->sceneryLodThresholdSets.size();
        impl_->sceneryLodThresholdSets.push_back(configuration.lodThresholds);
        sceneryLodThresholdSetByAsset.emplace(cacheKey, lodThresholdSetIndex);
        for (const auto& [name, value] : variables.numericValues()) {
            static_cast<void>(value);
            sceneryModelFloatVariables[cacheKey].push_back(name);
        }
        for (const auto& [name, value] : variables.stringValues()) {
            static_cast<void>(value);
            sceneryModelStringVariables[cacheKey].push_back(name);
        }
        sceneryModelScriptTextures[cacheKey] = configuration.scriptTextures;
        sceneryModelTextTextures[cacheKey] = configuration.textTextures;
        if (!configuration.scriptTextures.empty() || !configuration.textTextures.empty()) {
            sceneryAssetsNeedingVariables.insert(cacheKey);
        }
        if (sceneryAttachmentPoints[cacheKey].empty()) {
            sceneryAttachmentPoints[cacheKey] = configuration.attachmentPoints;
        }
        if (configuration.absoluteHeight) {
            absoluteHeightSceneryAssets.insert(cacheKey);
        }
        for (const ConfigurationDiagnostic& diagnostic : configuration.diagnostics.entries) {
            const std::string severity =
                diagnostic.severity == ConfigurationDiagnostic::Severity::Error ? "error"
                                                                                : "warning";
            recordSkipped(skipped, "scenery model config diagnostics",
                          openbus::pathToUtf8(modelConfigPath.filename()) +
                              " line=" + std::to_string(diagnostic.line) + " [" +
                              diagnostic.keyword + "] " + severity + ": " + diagnostic.message,
                          1,
                          diagnostic.severity == ConfigurationDiagnostic::Severity::Error
                              ? LogLevel::Error
                              : LogLevel::Warning);
        }
        for (const ModelPart& part : configuration.parts) {
            if (!part.visibleVariable.empty()) {
                sceneryAssetsNeedingVariables.insert(cacheKey);
            }
            for (const ModelMaterialState& state : part.materialStatesInOrder) {
                if (state.scriptTextureIndex >= 0 || state.textTextureIndex >= 0 ||
                    !state.texcoordTransXVariable.empty() ||
                    !state.texcoordTransYVariable.empty() ||
                    !state.lightmapStrengthVariable.empty() || !state.nightmapTextureName.empty() ||
                    !state.alphaScaleVariable.empty()) {
                    sceneryAssetsNeedingVariables.insert(cacheKey);
                }
                for (const auto& change : state.textureChanges) {
                    for (const auto& item : change.items) {
                        if (item && (!item->lightmapTextureName.empty() ||
                                     !item->nightmapTextureName.empty())) {
                            sceneryAssetsNeedingVariables.insert(cacheKey);
                        }
                    }
                }
                if (!state.freeTextureVariable.empty()) {
                    sceneryAssetsNeedingVariables.insert(cacheKey);
                    auto& stringVariables = sceneryModelStringVariables[cacheKey];
                    if (std::find(stringVariables.begin(), stringVariables.end(),
                                  state.freeTextureVariable) == stringVariables.end()) {
                        stringVariables.push_back(state.freeTextureVariable);
                    }
                    sceneryFreeTextureVariables[cacheKey].push_back(state.freeTextureVariable);
                }
            }
            if (part.objPath.empty()) {
                recordSkipped(skipped, "scenery model parts with no resolved mesh path",
                              openbus::pathToUtf8(modelConfigPath.filename()));
                continue;
            }
            std::string meshExtension = part.objPath.extension().string();
            std::transform(
                meshExtension.begin(), meshExtension.end(), meshExtension.begin(),
                [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
            if (meshExtension != ".o3d" && meshExtension != ".x") {
                recordSkipped(skipped, "scenery model parts with unsupported mesh formats",
                              openbus::pathToUtf8(modelConfigPath.filename()) + " " +
                                  openbus::pathToUtf8(part.objPath));
                continue;
            }
            const std::string meshKey = openbus::pathToUtf8(part.objPath.lexically_normal());
            const auto preparedMesh = prepared.meshes.find(meshKey);
            if (preparedMesh != prepared.meshes.end() && preparedMesh->second.preparationError) {
                try {
                    std::rethrow_exception(preparedMesh->second.preparationError);
                } catch (const std::exception& error) {
                    recordSkipped(skipped, "scenery O3D meshes that failed to prepare",
                                  openbus::pathToUtf8(part.objPath.filename()) + ": " +
                                      error.what());
                } catch (...) {
                    recordSkipped(skipped, "scenery O3D meshes that failed to prepare",
                                  openbus::pathToUtf8(part.objPath.filename()) + ": unknown error");
                }
                continue;
            }
            if (preparedMesh == prepared.meshes.end() || !preparedMesh->second.parsed) {
                recordSkipped(skipped, "scenery O3D meshes that failed to parse",
                              openbus::pathToUtf8(configPath.filename()) + " " +
                                  openbus::pathToUtf8(part.objPath.filename()));
                continue;
            }
            const PreparedSceneryMesh& mesh = preparedMesh->second;
            if (mesh.invalidPositionIndices > 0) {
                recordSkipped(skipped, "scenery mesh vertices with invalid position indices",
                              openbus::pathToUtf8(part.objPath.filename()),
                              mesh.invalidPositionIndices);
            }
            for (const auto& [materialIndex, vertices] : mesh.verticesByMaterial) {
                static_cast<void>(materialIndex);
                if (vertices.empty()) {
                    recordSkipped(skipped, "empty scenery material batches",
                                  openbus::pathToUtf8(part.objPath.filename()));
                }
            }
        }
        for (const PreparedSceneryBatch& preparedBatch : prepared.batches) {
            const auto mesh = prepared.meshes.find(preparedBatch.meshKey);
            if (mesh == prepared.meshes.end()) {
                throw std::logic_error("Prepared scenery batch mesh was not retained");
            }
            const auto materialVertices =
                mesh->second.verticesByMaterial.find(preparedBatch.materialIndex);
            if (materialVertices == mesh->second.verticesByMaterial.end()) {
                throw std::logic_error("Prepared scenery batch vertices were not retained");
            }
            const std::vector<ModelVertex>& vertices = materialVertices->second;
            Impl::ModelBatch batch = preparedBatch.batch;
            if (!preparedBatch.texturePath.empty()) {
                batch.material.texture = getTexture(preparedBatch.texturePath);
                batch.material.textured = true;
            } else if (!preparedBatch.textureName.empty()) {
                recordSkipped(skipped, "scenery material textures not found",
                              openbus::pathToUtf8(modelConfigPath.filename()) + " mesh=" +
                                  openbus::pathToUtf8(
                                      openbus::pathFromUtf8(preparedBatch.meshKey).filename()) +
                                  " texture=" + preparedBatch.textureName);
            }
            if (!preparedBatch.transmapTextureName.empty()) {
                if (preparedBatch.transmapTexturePath.empty()) {
                    recordSkipped(skipped, "scenery transmap textures not found",
                                  openbus::pathToUtf8(modelConfigPath.filename()) + " mesh=" +
                                      openbus::pathToUtf8(
                                          openbus::pathFromUtf8(preparedBatch.meshKey).filename()) +
                                      " transmap=" + preparedBatch.transmapTextureName);
                } else {
                    batch.material.transmap = getTexture(preparedBatch.transmapTexturePath);
                    batch.material.useTransmap = batch.material.transmap != 0;
                }
            }
            const auto uploadAuxTexture = [&](const PreparedSceneryAuxTexture& texture,
                                              const std::string& type, GLuint& handle,
                                              bool& enabled) {
                if (texture.name.empty()) {
                    return;
                }
                if (!texture.preparationError.empty()) {
                    recordSkipped(skipped, "scenery " + type + " textures that failed to decode",
                                  openbus::pathToUtf8(modelConfigPath.filename()) + " " + type +
                                      "=" + texture.name + ": " + texture.preparationError);
                } else if (texture.path.empty()) {
                    recordSkipped(skipped, "scenery " + type + " textures not found",
                                  openbus::pathToUtf8(modelConfigPath.filename()) + " mesh=" +
                                      openbus::pathToUtf8(
                                          openbus::pathFromUtf8(preparedBatch.meshKey).filename()) +
                                      " " + type + "=" + texture.name);
                } else {
                    handle = getTexture(texture.path);
                    enabled = handle != 0;
                }
            };
            uploadAuxTexture(preparedBatch.lightmap, "lightmap", batch.material.lightmap,
                             batch.material.useLightmap);
            uploadAuxTexture(preparedBatch.nightmap, "nightmap", batch.material.nightmap,
                             batch.material.useNightmap);
            const auto uploadItemTextures =
                [&](const std::vector<PreparedSceneryAuxTexture>& textures, const std::string& type,
                    std::unordered_map<std::string, GLuint>& handles) {
                    for (const PreparedSceneryAuxTexture& texture : textures) {
                        if (handles.contains(texture.name)) {
                            continue;
                        }
                        GLuint handle = 0;
                        bool enabled = false;
                        uploadAuxTexture(texture, type, handle, enabled);
                        handles.emplace(texture.name, handle);
                    }
                };
            uploadItemTextures(preparedBatch.itemLightmaps, "lightmap", batch.itemLightmaps);
            uploadItemTextures(preparedBatch.itemNightmaps, "nightmap", batch.itemNightmaps);
            uploadAuxTexture(preparedBatch.bumpmap, "bumpmap", batch.material.bumpmap,
                             batch.material.useBumpmap);
            uploadAuxTexture(preparedBatch.environment, "environment map",
                             batch.material.environmentMap, batch.material.useEnvironmentMap);
            if (!preparedBatch.freeTextureName.empty()) {
                if (preparedBatch.freeTexturePath.empty()) {
                    recordSkipped(skipped, "scenery free textures not found",
                                  openbus::pathToUtf8(modelConfigPath.filename()) + " mesh=" +
                                      openbus::pathToUtf8(
                                          openbus::pathFromUtf8(preparedBatch.meshKey).filename()) +
                                      " texture=" + preparedBatch.freeTextureName);
                } else {
                    batch.material.freeTexture = getTexture(preparedBatch.freeTexturePath);
                    batch.material.useFreeTexture = batch.material.freeTexture != 0;
                }
            }
            if (verboseAlphaMaterials &&
                (batch.alphaMode != 0 || batch.noZcheck || batch.noZwrite)) {
                gameLog.Debug(
                    "Map scenery alpha material: asset=" +
                    openbus::pathToUtf8(modelConfigPath.filename()) + " mesh=" +
                    openbus::pathToUtf8(openbus::pathFromUtf8(preparedBatch.meshKey).filename()) +
                    " index=" + std::to_string(preparedBatch.materialIndex) +
                    " mode=" + std::to_string(batch.alphaMode) + " state=" +
                    (preparedBatch.hasMaterialState ? std::to_string(preparedBatch.stateAlphaMode)
                                                    : "none") +
                    " noZcheck=" + (batch.noZcheck ? "true" : "false") +
                    " noZwrite=" + (batch.noZwrite ? "true" : "false") +
                    " texture=" + openbus::pathToUtf8(preparedBatch.texturePath));
            }
            pglGenBuffers(1, &batch.buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(vertices.size() * sizeof(ModelVertex)),
                          vertices.data(), GL_STATIC_DRAW);
            if (batch.buffer == 0 || glGetError() != GL_NO_ERROR) {
                if (batch.buffer != 0) {
                    deleteRendererBuffers(1, &batch.buffer);
                }
                recordSkipped(
                    skipped, "scenery model batches rejected by OpenGL upload",
                    openbus::pathToUtf8(openbus::pathFromUtf8(preparedBatch.meshKey).filename()));
                continue;
            }
            batchIndexes.push_back(impl_->sceneryBatches.size());
            impl_->sceneryBatches.push_back(batch);
        }
    } catch (const std::exception& error) {
        // Unsupported scenery configuration syntax/assets are skipped; they must not prevent
        // the map terrain and supported static scenery from rendering.
        recordSkipped(skipped, "scenery assets rejected while loading",
                      openbus::pathToUtf8(configPath.filename()) + ": " + error.what());
    }
    return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes)).first->second;
}

const openbus::map::MapSplineProfile*
MapSceneLoader::cachedSplineProfile(const std::filesystem::path& path) {
    if (path.empty()) {
        return nullptr;
    }
    const std::string key = openbus::pathToUtf8(path.lexically_normal());
    if (failedSplineProfiles.contains(key)) {
        return nullptr;
    }
    auto found = splineProfiles.find(key);
    if (found == splineProfiles.end()) {
        try {
            found = splineProfiles.emplace(key, openbus::map::loadMapSplineProfile(path)).first;
        } catch (const std::exception& error) {
            failedSplineProfiles.insert(key);
            recordSkipped(skipped, "spline profile files that failed to parse",
                          openbus::pathToUtf8(path.filename()) + ": " + error.what());
            return nullptr;
        }
    }
    return &found->second;
}

} // namespace openbus::rendering
