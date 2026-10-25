// Scenery object (.sco) and spline profile (.sli) asset loading for map content.

#include "render/map/MapSceneLoader.h"

namespace openbus::rendering {

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

const std::vector<std::size_t>&
MapSceneLoader::loadSceneryAsset(const std::filesystem::path& configPath) {
    const std::string cacheKey = configPath.lexically_normal().generic_string();
    const auto cached = sceneryAssetBatches.find(cacheKey);
    if (cached != sceneryAssetBatches.end()) {
        return cached->second;
    }
    TraceScope trace("map", "MapRenderer.loadSceneryAsset");
    std::vector<std::size_t> batchIndexes;
    try {
        const std::filesystem::path objectRoot = configPath.parent_path();
        PreparedSceneryAsset prepared;
        auto preparation = preparedSceneryAssetConfigs.find(cacheKey);
        if (preparation == preparedSceneryAssetConfigs.end()) {
            prepared = prepareSceneryAsset(configPath);
        } else {
            prepared = std::move(preparation->second);
            preparedSceneryAssetConfigs.erase(preparation);
        }
        if (prepared.preparationError) {
            std::rethrow_exception(prepared.preparationError);
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
        // Editor-only assets can still contribute path debug geometry.
        if (objectConfiguration.absoluteHeight) {
            absoluteHeightSceneryAssets.insert(cacheKey);
        }
        if (!objectConfiguration.scriptConfiguration.scripts.empty()) {
            sceneryAssetsNeedingVariables.insert(cacheKey);
        }
        // Tree SCOs may mark their helper mesh [onlyeditor] while their [tree]
        // definition is runtime scenery. The tree preview below replaces that mesh.
        if (objectConfiguration.onlyEditor && objectConfiguration.trees.empty()) {
            editorOnlySceneryAssets.insert(cacheKey);
            return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes)).first->second;
        }
        const std::filesystem::path& modelConfigPath = prepared.modelConfigPath;
        const std::filesystem::path& modelRoot = prepared.modelRoot;
        if (prepared.externalModelConfigMissing) {
            recordSkipped(skipped, "scenery external model configs not found",
                          configPath.filename().string() +
                              " model=" + prepared.externalModelConfigPath.generic_string());
        }
        for (const ConfigurationDiagnostic& diagnostic : objectConfiguration.diagnostics.entries) {
            const std::string severity =
                diagnostic.severity == ConfigurationDiagnostic::Severity::Error ? "error"
                                                                                : "warning";
            recordSkipped(skipped, "scenery config diagnostics",
                          configPath.filename().string() +
                              " line=" + std::to_string(diagnostic.line) + " [" +
                              diagnostic.keyword + "] " + severity + ": " + diagnostic.message);
        }
        if (!objectConfiguration.trees.empty()) {
            for (const SceneryTreeDefinition& tree : objectConfiguration.trees) {
                const std::filesystem::path treeTexture =
                    findSceneryTexture(objectRoot, modelRoot, omsiRoot, tree.texturePath);
                if (treeTexture.empty()) {
                    recordSkipped(skipped, "tree scenery definitions with missing textures",
                                  configPath.filename().string() + " texture=" + tree.texturePath);
                    continue;
                }
                const double height = (tree.minimumHeight + tree.maximumHeight) * 0.5;
                const double ratio = (tree.minimumRatio + tree.maximumRatio) * 0.5;
                std::vector<ModelVertex> vertices = makeTreePreview(height, height * 0.55 * ratio);
                Impl::ModelBatch batch;
                batch.vertexCount = vertices.size();
                batch.boundingRadius = vertexBoundingRadius(vertices);
                batch.renderType = objectConfiguration.renderType;
                batch.material.texture = getTexture(treeTexture);
                batch.material.textured = true;
                batch.material.flipTextureY = true;
                batch.alphaMode = 1;
                pglGenBuffers(1, &batch.buffer);
                pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                pglBufferData(GL_ARRAY_BUFFER,
                              static_cast<std::ptrdiff_t>(vertices.size() * sizeof(ModelVertex)),
                              vertices.data(), GL_STATIC_DRAW);
                if (batch.buffer == 0 || glGetError() != GL_NO_ERROR) {
                    if (batch.buffer != 0) {
                        pglDeleteBuffers(1, &batch.buffer);
                    }
                    recordSkipped(skipped, "tree scenery batches rejected by OpenGL upload",
                                  configPath.filename().string());
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
                          modelConfigPath.filename().string() +
                              " line=" + std::to_string(diagnostic.line) + " [" +
                              diagnostic.keyword + "] " + severity + ": " + diagnostic.message);
        }
        for (const ModelPart& part : configuration.parts) {
            if (!part.visibleVariable.empty()) {
                sceneryAssetsNeedingVariables.insert(cacheKey);
            }
            for (const ModelMaterialState& state : part.materialStatesInOrder) {
                if (state.scriptTextureIndex >= 0 || state.textTextureIndex >= 0) {
                    sceneryAssetsNeedingVariables.insert(cacheKey);
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
                              modelConfigPath.filename().string());
                continue;
            }
            std::string meshExtension = part.objPath.extension().string();
            std::transform(
                meshExtension.begin(), meshExtension.end(), meshExtension.begin(),
                [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
            if (meshExtension != ".o3d" && meshExtension != ".x") {
                recordSkipped(skipped, "scenery model parts with unsupported mesh formats",
                              modelConfigPath.filename().string() + " " +
                                  part.objPath.generic_string());
                continue;
            }
            const std::string meshKey = part.objPath.lexically_normal().generic_string();
            const auto preparedMesh = prepared.meshes.find(meshKey);
            if (preparedMesh != prepared.meshes.end() && preparedMesh->second.preparationError) {
                try {
                    std::rethrow_exception(preparedMesh->second.preparationError);
                } catch (const std::exception& error) {
                    recordSkipped(skipped, "scenery O3D meshes that failed to prepare",
                                  part.objPath.filename().string() + ": " + error.what());
                } catch (...) {
                    recordSkipped(skipped, "scenery O3D meshes that failed to prepare",
                                  part.objPath.filename().string() + ": unknown error");
                }
                continue;
            }
            if (preparedMesh == prepared.meshes.end() || !preparedMesh->second.parsed) {
                recordSkipped(skipped, "scenery O3D meshes that failed to parse",
                              configPath.filename().string() + " " +
                                  part.objPath.filename().string());
                continue;
            }
            const PreparedSceneryMesh& mesh = preparedMesh->second;
            const std::shared_ptr<ParsedObj>& parsed = mesh.parsed;
            if (mesh.invalidPositionIndices > 0) {
                recordSkipped(skipped, "scenery mesh vertices with invalid position indices",
                              part.objPath.filename().string(), mesh.invalidPositionIndices);
            }
            const auto statesByMaterial = openbus::map::resolveSceneryMaterialStates(*parsed, part);
            for (const auto& [materialIndex, vertices] : mesh.verticesByMaterial) {
                if (vertices.empty()) {
                    recordSkipped(skipped, "empty scenery material batches",
                                  part.objPath.filename().string());
                    continue;
                }
                const auto sourceMaterial =
                    parsed->materials.find("matl_" + std::to_string(materialIndex));
                const auto stateFound = statesByMaterial.find(materialIndex);
                const ModelMaterialState* state =
                    stateFound == statesByMaterial.end() ? nullptr : stateFound->second;
                std::string textureName = sourceMaterial == parsed->materials.end()
                                              ? std::string{}
                                              : sourceMaterial->second.textureName;
                std::array<double, 3> color = sourceMaterial == parsed->materials.end()
                                                  ? std::array<double, 3>{1.0, 1.0, 1.0}
                                                  : sourceMaterial->second.color;
                if (state != nullptr && !state->textureName.empty()) {
                    textureName = state->textureName;
                }
                if (state != nullptr && !state->textureName.empty()) {
                    const std::string stateTexture =
                        std::filesystem::path(state->textureName).filename().string();
                    const auto materialForTexture = std::find_if(
                        parsed->materials.begin(), parsed->materials.end(), [&](const auto& entry) {
                            return std::filesystem::path(entry.second.textureName).filename() ==
                                   stateTexture;
                        });
                    if (materialForTexture != parsed->materials.end()) {
                        color = materialForTexture->second.color;
                    }
                }
                ModelMaterial material;
                if (sourceMaterial != parsed->materials.end()) {
                    for (std::size_t index = 0; index < 3; ++index) {
                        material.specularColor[index] =
                            static_cast<float>(sourceMaterial->second.specular[index]);
                        material.emissionColor[index] =
                            static_cast<float>(sourceMaterial->second.emission[index]);
                    }
                    material.specularPower =
                        static_cast<float>(sourceMaterial->second.specularPower);
                }
                const std::filesystem::path materialTexture =
                    findSceneryTexture(objectRoot, modelRoot, omsiRoot, textureName);
                if (!materialTexture.empty()) {
                    material.texture = getTexture(materialTexture);
                    material.textured = true;
                } else if (!textureName.empty()) {
                    recordSkipped(skipped, "scenery material textures not found",
                                  modelConfigPath.filename().string() + " mesh=" +
                                      part.objPath.filename().string() + " texture=" + textureName);
                }
                if (state != nullptr && !state->transmapTextureName.empty()) {
                    const std::filesystem::path transmapTexture = findSceneryTexture(
                        objectRoot, modelRoot, omsiRoot, state->transmapTextureName);
                    if (transmapTexture.empty()) {
                        recordSkipped(skipped, "scenery transmap textures not found",
                                      modelConfigPath.filename().string() +
                                          " mesh=" + part.objPath.filename().string() +
                                          " transmap=" + state->transmapTextureName);
                    } else {
                        material.transmap = getTexture(transmapTexture);
                        material.useTransmap = material.transmap != 0;
                    }
                }
                if (state != nullptr && !state->freeTextureName.empty()) {
                    const std::filesystem::path freeTexture =
                        findSceneryTexture(objectRoot, modelRoot, omsiRoot, state->freeTextureName);
                    if (freeTexture.empty()) {
                        recordSkipped(skipped, "scenery free textures not found",
                                      modelConfigPath.filename().string() +
                                          " mesh=" + part.objPath.filename().string() +
                                          " texture=" + state->freeTextureName);
                    } else {
                        material.freeTexture = getTexture(freeTexture);
                        material.useFreeTexture = material.freeTexture != 0;
                    }
                }
                Impl::ModelBatch batch;
                batch.cullBackFaces = true;
                batch.vertexCount = vertices.size();
                batch.boundingRadius = vertexBoundingRadius(vertices);
                batch.sceneryLodIndex =
                    part.lodIndex < 0 ? 0U : static_cast<std::size_t>(part.lodIndex);
                batch.material = material;
                if (state != nullptr) {
                    batch.scriptTextureIndex = state->scriptTextureIndex;
                    batch.textTextureIndex = state->textTextureIndex;
                    batch.freeTextureName = state->freeTextureName;
                    batch.freeTextureVariable = state->freeTextureVariable;
                    batch.textureObjectRoot = objectRoot;
                    batch.textureModelRoot = modelRoot;
                    batch.material.textureWrapS = textureAddressModeToGl(state->textureAddressS);
                    batch.material.textureWrapT = textureAddressModeToGl(state->textureAddressT);
                }
                batch.color = color;
                batch.alpha = sourceMaterial == parsed->materials.end()
                                  ? 1.0
                                  : std::clamp(sourceMaterial->second.alpha, 0.0, 1.0);
                batch.alphaMode = state == nullptr ? 0 : state->alphaMode;
                batch.noZwrite = state != nullptr && state->noZwrite;
                batch.noZcheck = state != nullptr && state->noZcheck;
                batch.renderType = resolveSceneryRenderType(part, objectConfiguration.renderType);
                batch.visibilityVariable = part.visibleVariable;
                batch.visibilityValue = part.visibleValue;
                if (batch.alphaMode == 0 && batch.alpha < 1.0 - 1.0e-6) {
                    batch.alphaMode = 2;
                }
                if (verboseAlphaMaterials && batch.alphaMode != 0) {
                    gameLog.Log(
                        "Map scenery alpha material: asset=" + modelConfigPath.filename().string() +
                        " mesh=" + part.objPath.filename().string() + " index=" +
                        std::to_string(materialIndex) + " mode=" + std::to_string(batch.alphaMode) +
                        " state=" + (state == nullptr ? "none" : std::to_string(state->alphaMode)) +
                        " texture=" + materialTexture.generic_string());
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
                    recordSkipped(skipped, "scenery model batches rejected by OpenGL upload",
                                  part.objPath.filename().string());
                    continue;
                }
                batchIndexes.push_back(impl_->sceneryBatches.size());
                impl_->sceneryBatches.push_back(batch);
            }
        }
    } catch (const std::exception& error) {
        // Unsupported scenery configuration syntax/assets are skipped; they must not prevent
        // the map terrain and supported static scenery from rendering.
        recordSkipped(skipped, "scenery assets rejected while loading",
                      configPath.filename().string() + ": " + error.what());
    }
    return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes)).first->second;
}

const openbus::map::MapSplineProfile*
MapSceneLoader::cachedSplineProfile(const std::filesystem::path& path) {
    if (path.empty()) {
        return nullptr;
    }
    const std::string key = path.lexically_normal().generic_string();
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
                          path.filename().string() + ": " + error.what());
            return nullptr;
        }
    }
    return &found->second;
}

} // namespace openbus::rendering
