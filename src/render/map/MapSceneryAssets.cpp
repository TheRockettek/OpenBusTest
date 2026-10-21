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
        const SceneryObjectConfig objectConfiguration = loadSceneryObjectFile(configPath);
        sceneryAttachmentPoints[cacheKey] = objectConfiguration.attachmentPoints;
        sceneryScriptConfigurations[cacheKey] = objectConfiguration.scriptConfiguration;
        if (!objectConfiguration.scriptConfiguration.scripts.empty()) {
            sceneryAssetsNeedingVariables.insert(cacheKey);
        }
        // Tree SCOs may mark their helper mesh [onlyeditor] while their [tree]
        // definition is runtime scenery. The tree preview below replaces that mesh.
        if (objectConfiguration.onlyEditor && objectConfiguration.trees.empty()) {
            editorOnlySceneryAssets.insert(cacheKey);
            return sceneryAssetBatches.emplace(cacheKey, std::move(batchIndexes)).first->second;
        }
        std::filesystem::path modelConfigPath =
            resolveSceneryObjectModelConfigPath(objectConfiguration);
        bool usesExternalModelConfig = !objectConfiguration.modelPath.empty();
        if (usesExternalModelConfig && !std::filesystem::is_regular_file(modelConfigPath)) {
            recordSkipped(skipped, "scenery external model configs not found",
                          configPath.filename().string() +
                              " model=" + modelConfigPath.generic_string());
            modelConfigPath = configPath;
            usesExternalModelConfig = false;
        }
        const std::filesystem::path modelRoot =
            usesExternalModelConfig ? modelConfigPath.parent_path() : objectRoot / "model";
        if (objectConfiguration.absoluteHeight) {
            absoluteHeightSceneryAssets.insert(cacheKey);
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
        openbus::scripting::SceneryObject variables;
        const ModelConfig configuration =
            loadModelConfig(modelConfigPath, modelRoot, ModelConfigKind::SceneryObject, variables);
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
            const std::shared_ptr<ParsedObj> parsed = meshExtension == ".x"
                                                          ? XLoader::parse(part.objPath)
                                                          : O3DLoader::parse(part.objPath);
            if (!parsed) {
                recordSkipped(skipped, "scenery O3D meshes that failed to parse",
                              configPath.filename().string() + " " +
                                  part.objPath.filename().string());
                continue;
            }
            std::unordered_map<int, std::vector<ModelVertex>> verticesByMaterial;
            const auto statesByMaterial = openbus::map::resolveSceneryMaterialStates(*parsed, part);
            for (const ObjTriangle& triangle : parsed->triangles) {
                const auto sourceMaterial = parsed->materials.find(triangle.material);
                const int materialIndex = sourceMaterial == parsed->materials.end()
                                              ? 0
                                              : sourceMaterial->second.materialIndex;
                const auto validIndex = [](int index, std::size_t count) {
                    return index > 0 && static_cast<std::size_t>(index) <= count;
                };
                for (const ObjIndex& index : triangle.indices) {
                    if (!validIndex(index.position, parsed->positions.size())) {
                        recordSkipped(skipped,
                                      "scenery mesh vertices with invalid position indices",
                                      part.objPath.filename().string());
                        continue;
                    }
                    const std::size_t positionIndex = static_cast<std::size_t>(index.position - 1);
                    const ObjPosition& sourcePosition = parsed->positions[positionIndex];
                    const std::array<double, 4> vertexColor =
                        positionIndex < parsed->vertexColors.size()
                            ? parsed->vertexColors[positionIndex]
                            : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};
                    std::array<double, 3> normal = {0.0, 0.0, 1.0};
                    if (validIndex(index.normal, parsed->normals.size())) {
                        const ObjNormal& sourceNormal =
                            parsed->normals[static_cast<std::size_t>(index.normal - 1)];
                        // O3DLoader already swaps the file's up/forward axes and
                        // adjusts handedness. Do not apply the OBJ vehicle-frame
                        // conversion a second time to map scenery normals.
                        normal = {sourceNormal.x, sourceNormal.y, sourceNormal.z};
                        const double length = std::sqrt(
                            normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
                        if (length > 1.0e-8) {
                            for (double& component : normal) {
                                component /= length;
                            }
                        }
                    }
                    float u = 0.0F;
                    float v = 0.0F;
                    if (validIndex(index.texCoord, parsed->texCoords.size())) {
                        const ObjTexCoord& coordinate =
                            parsed->texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                        u = static_cast<float>(coordinate.u);
                        v = static_cast<float>(1.0 - coordinate.v);
                    }
                    verticesByMaterial[materialIndex].push_back(
                        {static_cast<float>(sourcePosition.x), static_cast<float>(sourcePosition.y),
                         static_cast<float>(sourcePosition.z), u, v, 0.0F,
                         static_cast<float>(normal[0]), static_cast<float>(normal[1]),
                         static_cast<float>(normal[2]), static_cast<float>(vertexColor[0]),
                         static_cast<float>(vertexColor[1]), static_cast<float>(vertexColor[2]),
                         static_cast<float>(vertexColor[3])});
                }
            }

            for (auto& [materialIndex, vertices] : verticesByMaterial) {
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
