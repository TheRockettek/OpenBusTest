// Mesh parsing, display-list construction and the shared display cache.

#include "app/vehicle/Vehicle.h"
#include "config/ScriptTextureLimits.h"

bool Vehicle::areObjectsLoaded() const {
    return loaded && pendingParts.empty();
}

std::mutex& Vehicle::sharedDisplayCacheMutex() {
    static std::mutex mutex;
    return mutex;
}

std::unordered_map<std::string, std::vector<Vehicle::DisplayPart>>& Vehicle::sharedDisplayCache() {
    static std::unordered_map<std::string, std::vector<DisplayPart>> cache;
    return cache;
}

bool Vehicle::loadSharedDisplayLists() {
    std::lock_guard<std::mutex> lock(sharedDisplayCacheMutex());
    const auto cached = sharedDisplayCache().find(modelCacheKey);
    if (cached == sharedDisplayCache().end()) {
        return false;
    }
    displayLists = cached->second;
    for (DisplayPart& part : displayLists) {
        part.animationStates.assign(part.animations.size(), {});
        part.animationCacheGeneration = 0;
        part.cachedAnimationTransform = identityMatrix();
        part.viewDepthCacheGeneration = 0;
        part.cachedViewDepth = 0.0;
        for (Batch& batch : part.batches) {
            batch.sharedGeometryBuffer = true;
            batch.dynamicTexture = 0;
            batch.freeTexture = 0;
            batch.freeTextureIndex = -1;
            batch.freeTextureWidth = 0;
            batch.freeTextureHeight = 0;
            batch.freeTextureRevision = 0;
            batch.freeTextureAsset = {};
            for (AuxiliaryTexture* auxiliary :
                 {&batch.lightmap, &batch.nightmap, &batch.transmap, &batch.bumpmap}) {
                if (openbus::scripting::scriptTextureReferenceIndex(auxiliary->name) >= 0) {
                    auxiliary->texture = 0;
                    auxiliary->width = 0;
                    auxiliary->height = 0;
                    auxiliary->revision = 0;
                }
            }
            batch.selectedMaterialItem = false;
            batch.materialSelectionGeneration = 0;
        }
    }
    loaded = !displayLists.empty();
    hasSharedGeometry = loaded;
    hasLoadedInitialView = loaded;
    loggedAllObjectsLoaded = loaded;
    if (loaded) {
        rebuildDisplayOrder();
        preloadTextures();
    }
    return loaded;
}

void Vehicle::cacheSharedDisplayLists() {
    if (displayLists.empty()) {
        return;
    }
    for (DisplayPart& part : displayLists) {
        for (Batch& batch : part.batches) {
            batch.sharedGeometryBuffer = batch.buffer != 0;
        }
    }
    std::lock_guard<std::mutex> lock(sharedDisplayCacheMutex());
    sharedDisplayCache().try_emplace(modelCacheKey, displayLists);
}

void Vehicle::clearSharedDisplayCache() {
    std::lock_guard<std::mutex> lock(sharedDisplayCacheMutex());
    sharedDisplayCache().clear();
}

std::shared_future<std::shared_ptr<ParsedObj>>
Vehicle::parsedObjFuture(const std::filesystem::path& path, const std::string& bundleEntry) {
    return assets->requestObj(path, bundleEntry);
}

void Vehicle::loadObj(const Part& part, const std::shared_ptr<ParsedObj>& parsed) {
    TraceScope trace("obj", "loadObj");
    static const bool verboseObjLoadLogs =
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_VERBOSE_OBJ_LOAD"));
    static const bool materialBatchingEnabled =
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_MATERIAL_BATCHING"));
    if (verboseObjLoadLogs) {
        gameLog.Log("Loading OBJ model from path: " + part.objPath.generic_string());
    }
    if (!parsed) {
        return;
    }
    const auto& positions = parsed->positions;
    const auto& normals = parsed->normals;
    const auto& texCoords = parsed->texCoords;
    const auto& triangles = parsed->triangles;
    const auto& materials = parsed->materials;
    const auto& boundsCenter = parsed->boundsCenter;
    const auto& boundsSize = parsed->boundsSize;
    const double boundsRadius = parsed->boundsRadius;

    std::vector<ModelAnimation> animations = part.animations;
    for (ModelAnimation& animation : animations) {
        animation.variable = lower(animation.variable);
        if (animation.originFromMesh) {
            if (parsed->hasTransform) {
                // The converter preserves OBJ vertices and stores this transform as
                // metadata. Only its source-space translation supplies the mesh pivot;
                // applying its orientation again would double-transform the mesh.
                if (!animation.hasOrigin) {
                    animation.origin = {parsed->transform[13], -parsed->transform[12],
                                        parsed->transform[14]};
                    animation.hasOrigin = true;
                }
                animation.meshRotation = {
                    parsed->transform[0], parsed->transform[1], parsed->transform[2],
                    parsed->transform[4], parsed->transform[5], parsed->transform[6],
                    parsed->transform[8], parsed->transform[9], parsed->transform[10]};
                animation.hasMeshRotation = true;
                const std::array<double, 3> meshOrigin = {
                    parsed->transform[13], -parsed->transform[12], parsed->transform[14]};
                animation.meshTransform = makeMeshTransform(animation.meshRotation, meshOrigin);
                animation.hasMeshTransform = true;
            } else if (!animation.hasOrigin) {
                animation.origin = boundsCenter;
                animation.hasOrigin = true;
            }
        }
    }
    if (verboseObjLoadLogs && !animations.empty()) {
        for (const ModelAnimation& animation : animations) {
            gameLog.Log(
                "Model animation: " + part.objPath.filename().string() + " type=" + animation.type +
                " variable=" + animation.variable + " scale=" + std::to_string(animation.scale) +
                " origin=(" + std::to_string(animation.origin[0]) + ',' +
                std::to_string(animation.origin[1]) + ',' + std::to_string(animation.origin[2]) +
                ") hasOrigin=" + (animation.hasOrigin ? "true" : "false") +
                " value=" + std::to_string(variables.get(animation.variable)));
        }
    }

    std::vector<DisplayPart>* destination = &displayLists;
    auto makeBatch = [&](const std::vector<const ObjTriangle*>& source,
                         const std::filesystem::path& texturePath, const std::string& textureName,
                         const std::array<double, 3>& color,
                         const std::string& environmentTextureName, double environmentStrength,
                         int alphaMode, bool noZwrite, bool noZcheck,
                         const std::string& alphaScaleVariable,
                         const std::vector<MaterialState::TextureChange>& textureChanges,
                         const MaterialState& materialState) {
        TraceScope batchTrace("obj", "loadObj.makeBatch");
        std::vector<Vertex> vertices;
        if (source.size() > std::numeric_limits<std::size_t>::max() / 3) {
            throw std::runtime_error("triangle vertex count overflow");
        }
        vertices.reserve(source.size() * 3);
        const auto convertPosition = [](const ObjPosition& position) {
            return std::array<double, 3>{position.y, -position.x, position.z};
        };
        const auto normalizeVector = [](std::array<double, 3> value) {
            const double length =
                std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
            if (length <= 1.0e-8) {
                return std::array<double, 3>{0.0, 0.0, 1.0};
            }
            for (double& component : value) {
                component /= length;
            }
            return value;
        };
        const auto validIndex = [](int index, std::size_t count) {
            return index > 0 && static_cast<std::size_t>(index) <= count;
        };
        {
            TraceScope vertexTrace("obj", "loadObj.buildVertices");
            for (const ObjTriangle* triangle : source) {
                std::array<double, 3> fallbackNormal = {0.0, 0.0, 1.0};
                if (validIndex(triangle->indices[0].position, positions.size()) &&
                    validIndex(triangle->indices[1].position, positions.size()) &&
                    validIndex(triangle->indices[2].position, positions.size())) {
                    const std::array<double, 3> first = convertPosition(
                        positions[static_cast<std::size_t>(triangle->indices[0].position - 1)]);
                    const std::array<double, 3> second = convertPosition(
                        positions[static_cast<std::size_t>(triangle->indices[1].position - 1)]);
                    const std::array<double, 3> third = convertPosition(
                        positions[static_cast<std::size_t>(triangle->indices[2].position - 1)]);
                    const std::array<double, 3> edgeA = {second[0] - first[0], second[1] - first[1],
                                                         second[2] - first[2]};
                    const std::array<double, 3> edgeB = {third[0] - first[0], third[1] - first[1],
                                                         third[2] - first[2]};
                    fallbackNormal = normalizeVector({edgeA[1] * edgeB[2] - edgeA[2] * edgeB[1],
                                                      edgeA[2] * edgeB[0] - edgeA[0] * edgeB[2],
                                                      edgeA[0] * edgeB[1] - edgeA[1] * edgeB[0]});
                }
                for (const ObjIndex& index : triangle->indices) {
                    if (!validIndex(index.position, positions.size())) {
                        continue;
                    }
                    const ObjPosition& position =
                        positions[static_cast<std::size_t>(index.position - 1)];
                    std::array<double, 3> normal = fallbackNormal;
                    if (validIndex(index.normal, normals.size())) {
                        const ObjNormal& sourceNormal =
                            normals[static_cast<std::size_t>(index.normal - 1)];
                        normal = normalizeVector({sourceNormal.y, -sourceNormal.x, sourceNormal.z});
                    }
                    const ObjTexCoord* texCoord = nullptr;
                    if (validIndex(index.texCoord, texCoords.size())) {
                        texCoord = &texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                    }
                    vertices.push_back(
                        {static_cast<float>(position.y), static_cast<float>(-position.x),
                         static_cast<float>(position.z),
                         texCoord ? static_cast<float>(texCoord->u) : 0.0f,
                         texCoord ? static_cast<float>(1.0 - texCoord->v) : 0.0f, 0.0f,
                         static_cast<float>(normal[0]), static_cast<float>(normal[1]),
                         static_cast<float>(normal[2])});
                }
            }
        }
        if (vertices.empty())
            return;
        Batch batch;
        batch.vertices = std::move(vertices);
        batch.baseTexturePath = texturePath;
        batch.baseTextureName = resolveCtcTextureName(textureName);
        batch.texturePath = texturePath;
        batch.textureRoot = part.objPath.parent_path();
        batch.textureName = batch.baseTextureName;
        batch.environmentTextureName = environmentTextureName;
        batch.environmentStrength = environmentStrength;
        batch.lightmap.name = materialState.lightmapTextureName;
        batch.nightmap.name = materialState.nightmapTextureName;
        batch.transmap.name = materialState.transmapTextureName;
        batch.bumpmap.name = materialState.bumpmapTextureName;
        batch.bumpmapStrength = materialState.bumpmapStrength;
        batch.interiorLightIndexes = part.interiorLightIndexes;
        batch.lightmapStrengthVariable = lower(materialState.lightmapStrengthVariable);
        batch.freeTextureName = materialState.freeTextureName;
        batch.freeTextureVariable = lower(materialState.freeTextureVariable);
        batch.scriptTextureIndex = materialState.scriptTextureIndex;
        batch.textTextureIndex = materialState.textTextureIndex;
        batch.texcoordTransXVariable = lower(materialState.texcoordTransXVariable);
        batch.texcoordTransYVariable = lower(materialState.texcoordTransYVariable);
        batch.color = color;
        batch.hasNormals = true;
        batch.textureWrapS = textureAddressModeToGl(materialState.textureAddressS);
        batch.textureWrapT = textureAddressModeToGl(materialState.textureAddressT);
        batch.alphaMode = alphaMode;
        batch.noZwrite = noZwrite;
        batch.noZcheck = noZcheck;
        batch.alphaScaleVariable = lower(alphaScaleVariable);
        batch.textureChanges = textureChanges;
        batch.baseMaterial = materialState;
        for (MaterialState::TextureChange& change : batch.textureChanges) {
            change.activationVariable = lower(change.activationVariable);
        }
        batch.vertexCount = batch.vertices.size();

        const auto canMergeOpaqueBatches = [](const Batch& first, const Batch& second) {
            return first.alphaMode == 0 && second.alphaMode == 0 && !first.noZwrite &&
                   !second.noZwrite && !first.noZcheck && !second.noZcheck &&
                   first.texturePath == second.texturePath &&
                   first.textureRoot == second.textureRoot &&
                   first.textureName == second.textureName &&
                   first.environmentTextureName == second.environmentTextureName &&
                   first.environmentStrength == second.environmentStrength &&
                   first.color == second.color && first.textured == second.textured &&
                   first.hasNormals == second.hasNormals &&
                   first.textureWrapS == second.textureWrapS &&
                   first.textureWrapT == second.textureWrapT &&
                   first.lightmap.name == second.lightmap.name &&
                   first.nightmap.name == second.nightmap.name &&
                   first.transmap.name == second.transmap.name &&
                   first.bumpmap.name == second.bumpmap.name &&
                   first.bumpmapStrength == second.bumpmapStrength &&
                   first.interiorLightIndexes == second.interiorLightIndexes &&
                   first.lightmapStrengthVariable == second.lightmapStrengthVariable &&
                   first.freeTextureName == second.freeTextureName &&
                   first.freeTextureVariable == second.freeTextureVariable &&
                   first.scriptTextureIndex == second.scriptTextureIndex &&
                   first.textTextureIndex == second.textTextureIndex &&
                   first.texcoordTransXVariable == second.texcoordTransXVariable &&
                   first.texcoordTransYVariable == second.texcoordTransYVariable &&
                   first.alphaScaleVariable == second.alphaScaleVariable &&
                   first.baseTextureLayer == second.baseTextureLayer &&
                   first.textureLayer == second.textureLayer && first.textureChanges.empty() &&
                   second.textureChanges.empty();
        };
        std::vector<Batch>& batches = destination->back().batches;
        const auto compatibleBatch =
            std::find_if(batches.begin(), batches.end(), [&](const Batch& existing) {
                return canMergeOpaqueBatches(existing, batch);
            });
        if (compatibleBatch != batches.end()) {
            Batch& merged = *compatibleBatch;
            merged.vertices.insert(merged.vertices.end(), batch.vertices.begin(),
                                   batch.vertices.end());
            merged.vertexCount = merged.vertices.size();
            TraceScope uploadTrace("obj", "loadObj.mergeBatch");
            pglBindBuffer(GL_ARRAY_BUFFER, merged.buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(merged.vertices.size() * sizeof(Vertex)),
                          merged.vertices.data(), GL_STATIC_DRAW);
            return;
        }
        {
            TraceScope uploadTrace("obj", "loadObj.uploadVbo");
            pglGenBuffers(1, &batch.buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(batch.vertices.size() * sizeof(Vertex)),
                          batch.vertices.data(), GL_STATIC_DRAW);
        }
        batches.push_back(std::move(batch));
    };

    const auto canUseMaterialBatch = [](const Batch& batch) {
        return batch.alphaMode == 0 && !batch.noZwrite && !batch.noZcheck && !batch.textureArray &&
               batch.textureChanges.empty() && batch.textureWrapS == GL_REPEAT &&
               batch.textureWrapT == GL_REPEAT && batch.lightmap.name.empty() &&
               batch.nightmap.name.empty() && batch.transmap.name.empty() &&
               batch.freeTextureVariable.empty() && batch.bumpmap.name.empty() &&
               std::all_of(batch.interiorLightIndexes.begin(), batch.interiorLightIndexes.end(),
                           [](int index) { return index < 0; }) &&
               batch.freeTextureName.empty() && batch.scriptTextureIndex < 0 &&
               batch.textTextureIndex < 0 && batch.texcoordTransXVariable.empty() &&
               batch.texcoordTransYVariable.empty() && batch.alphaScaleVariable.empty();
    };
    const auto makeMaterialTexture = [](const Batch& source) {
        MaterialTextureSource result;
        result.texturePath = source.texturePath;
        result.textureRoot = source.textureRoot;
        result.textureName = source.textureName;
        return result;
    };
    const auto sameMaterialBatch = [&](const Batch& first, const Batch& second) {
        return canUseMaterialBatch(first) && canUseMaterialBatch(second) &&
               first.environmentTextureName == second.environmentTextureName &&
               first.environmentStrength == second.environmentStrength &&
               first.textured == second.textured && first.hasNormals == second.hasNormals &&
               first.baseTextureLayer == second.baseTextureLayer &&
               first.textureLayer == second.textureLayer && second.materialColors.empty() &&
               second.materialTextures.empty();
    };
    const auto consolidateMaterialBatches = [&](std::vector<Batch>& batches) {
        if (!materialBatchingEnabled) {
            return;
        }
        std::vector<Batch> consolidated;
        consolidated.reserve(batches.size());
        for (Batch& source : batches) {
            auto target =
                std::find_if(consolidated.begin(), consolidated.end(), [&](const Batch& candidate) {
                    return candidate.materialTextures.size() < 8 &&
                           sameMaterialBatch(candidate, source);
                });
            if (target == consolidated.end()) {
                if (canUseMaterialBatch(source)) {
                    for (Vertex& vertex : source.vertices) {
                        vertex.layer = 0.0f;
                    }
                    source.materialColors.push_back({static_cast<float>(source.color[0]),
                                                     static_cast<float>(source.color[1]),
                                                     static_cast<float>(source.color[2]), 1.0f});
                    source.materialTextures.push_back(makeMaterialTexture(source));
                }
                consolidated.push_back(std::move(source));
                continue;
            }

            const std::size_t materialIndex = target->materialColors.size();
            for (Vertex& vertex : source.vertices) {
                vertex.layer = static_cast<float>(materialIndex);
            }
            target->vertices.insert(target->vertices.end(), source.vertices.begin(),
                                    source.vertices.end());
            target->vertexCount = target->vertices.size();
            target->materialColors.push_back({static_cast<float>(source.color[0]),
                                              static_cast<float>(source.color[1]),
                                              static_cast<float>(source.color[2]), 1.0f});
            target->materialTextures.push_back(makeMaterialTexture(source));
            if (source.buffer != 0) {
                pglDeleteBuffers(1, &source.buffer);
                source.buffer = 0;
            }
            pglBindBuffer(GL_ARRAY_BUFFER, target->buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(target->vertices.size() * sizeof(Vertex)),
                          target->vertices.data(), GL_STATIC_DRAW);
        }
        batches = std::move(consolidated);
        for (Batch& batch : batches) {
            if (batch.materialColors.size() < 2) {
                batch.materialColors.clear();
                batch.materialTextures.clear();
            }
        }
        const std::size_t materialBatchCount = static_cast<std::size_t>(
            std::count_if(batches.begin(), batches.end(),
                          [](const Batch& batch) { return batch.materialColors.size() >= 2; }));
        if (verboseObjLoadLogs && materialBatchCount != 0) {
            gameLog.Log("Material batches: " + std::to_string(materialBatchCount) + " from " +
                        std::to_string(batches.size()) + " total batches");
        }
    };

    std::unordered_map<std::string, std::vector<const ObjTriangle*>> groups;
    std::unordered_map<std::string, std::filesystem::path> groupTextures;
    std::unordered_map<std::string, std::string> groupTextureNames;
    std::unordered_map<std::string, std::string> groupEnvironmentNames;
    std::unordered_map<std::string, double> groupEnvironmentStrengths;
    std::unordered_map<std::string, std::array<double, 3>> groupColors;
    std::unordered_map<std::string, MaterialState> groupStates;
    std::vector<std::string> groupOrder;
    std::unordered_map<std::string, const MaterialState*> materialStatesByFilename;
    std::unordered_map<std::string, const MaterialState*> materialStatesByStem;
    std::unordered_map<std::string, std::vector<const MaterialState*>>
        materialStatesByStemOccurrence;
    std::unordered_map<int, const MaterialState*> implicitMaterialStatesByIndex;
    materialStatesByFilename.reserve(part.materialStates.size());
    materialStatesByStem.reserve(part.materialStates.size());
    for (const auto& entry : part.materialStates) {
        materialStatesByFilename.emplace(
            lower(std::filesystem::path(entry.first).filename().string()), &entry.second);
        materialStatesByStem.emplace(lower(std::filesystem::path(entry.first).stem().string()),
                                     &entry.second);
    }
    for (const MaterialState& state : part.materialStatesInOrder) {
        const std::filesystem::path statePath = state.textureName.empty()
                                                    ? state.texturePath
                                                    : std::filesystem::path(state.textureName);
        if (state.textureName.empty() && state.texturePath.empty() && state.materialIndex >= 0) {
            implicitMaterialStatesByIndex.emplace(state.materialIndex, &state);
        }
        std::vector<const MaterialState*>& states =
            materialStatesByStemOccurrence[lower(statePath.stem().string())];
        if (state.materialIndex > 10000) {
            gameLog.Log("Ignoring out-of-range material index " +
                        std::to_string(state.materialIndex));
            continue;
        }
        if (state.materialIndex >= 0) {
            if (states.size() <= static_cast<std::size_t>(state.materialIndex)) {
                states.resize(static_cast<std::size_t>(state.materialIndex) + 1, nullptr);
            }
            states[static_cast<std::size_t>(state.materialIndex)] = &state;
        } else {
            states.push_back(&state);
        }
    }
    std::vector<std::pair<int, std::string>> indexedMaterials;
    indexedMaterials.reserve(materials.size());
    for (const auto& entry : materials) {
        if (entry.second.materialIndex >= 0) {
            const std::filesystem::path materialPath =
                entry.second.textureName.empty() ? entry.second.texturePath
                                                 : std::filesystem::path(entry.second.textureName);
            indexedMaterials.push_back(
                {entry.second.materialIndex, lower(materialPath.stem().string())});
        }
    }
    std::sort(indexedMaterials.begin(), indexedMaterials.end(),
              [](const auto& first, const auto& second) { return first.first < second.first; });
    std::unordered_map<std::string, int> textureOccurrences;
    std::unordered_map<int, int> materialOccurrenceByIndex;
    for (const auto& indexedMaterial : indexedMaterials) {
        const int occurrence = textureOccurrences[indexedMaterial.second]++;
        materialOccurrenceByIndex[indexedMaterial.first] = occurrence;
    }
    const std::filesystem::path fallbackTexturePath = part.texturePath;
    const std::string fallbackTextureName = part.textureName;
    bool hasTransparentMaterial = false;
    {
        TraceScope materialTrace("obj", "loadObj.groupMaterials");
        for (const auto& triangle : triangles) {
            const auto material = materials.find(triangle.material);
            std::filesystem::path texturePath =
                material != materials.end() && !material->second.texturePath.empty()
                    ? material->second.texturePath
                    : fallbackTexturePath;
            std::string textureName =
                material != materials.end() && !material->second.textureName.empty()
                    ? material->second.textureName
                    : fallbackTextureName;
            std::array<double, 3> color =
                material != materials.end() ? material->second.color : part.color;
            const std::string key = triangle.material.empty() ? "__fallback__" : triangle.material;
            if (groups.find(key) == groups.end()) {
                groupOrder.push_back(key);
            }
            MaterialState state;
            bool matchedMaterialState = false;
            if (material != materials.end() && !part.materialStatesInOrder.empty() &&
                material->second.materialIndex >= 0) {
                const auto occurrence =
                    materialOccurrenceByIndex.find(material->second.materialIndex);
                const std::filesystem::path materialPath =
                    material->second.textureName.empty()
                        ? material->second.texturePath
                        : std::filesystem::path(material->second.textureName);
                const auto states =
                    materialStatesByStemOccurrence.find(lower(materialPath.stem().string()));
                if (occurrence != materialOccurrenceByIndex.end() &&
                    states != materialStatesByStemOccurrence.end() && occurrence->second >= 0 &&
                    static_cast<std::size_t>(occurrence->second) < states->second.size() &&
                    states->second[static_cast<std::size_t>(occurrence->second)] != nullptr) {
                    state = *states->second[static_cast<std::size_t>(occurrence->second)];
                } else {
                    const auto implicitState =
                        implicitMaterialStatesByIndex.find(material->second.materialIndex);
                    if (implicitState != implicitMaterialStatesByIndex.end()) {
                        state = *implicitState->second;
                    }
                }
                matchedMaterialState = true;
            }
            if (!textureName.empty() || !texturePath.empty()) {
                const std::filesystem::path statePath =
                    textureName.empty() ? texturePath : std::filesystem::path(textureName);
                const std::string stateKey = lower(statePath.filename().string());
                if (!matchedMaterialState) {
                    const auto exactState = materialStatesByFilename.find(stateKey);
                    if (exactState != materialStatesByFilename.end()) {
                        state = *exactState->second;
                    } else {
                        const std::string stateStem = lower(statePath.stem().string());
                        const auto matchingState = materialStatesByStem.find(stateStem);
                        if (matchingState != materialStatesByStem.end()) {
                            state = *matchingState->second;
                        } else {
                            const auto matchingMaterial =
                                materialStatesByStem.find(lower(triangle.material));
                            if (matchingMaterial != materialStatesByStem.end()) {
                                state = *matchingMaterial->second;
                            }
                        }
                    }
                }
            }
            if (part.isShadow && state.alphaMode == 0) {
                state.alphaMode = 2;
            }
            if (!state.textureName.empty()) {
                textureName = state.textureName;
            }
            groups[key].push_back(&triangle);
            groupTextures[key] = texturePath;
            groupTextureNames[key] = textureName;
            groupEnvironmentNames[key] = state.environmentTextureName;
            groupEnvironmentStrengths[key] = state.environmentStrength;
            groupColors[key] = color;
            groupStates[key] = state;
            hasTransparentMaterial =
                hasTransparentMaterial || state.alphaMode != 0 || state.noZwrite || state.noZcheck;
        }
    }
    DisplayPart displayPart;
    displayPart.viewpoint = part.viewpoint;
    displayPart.renderType = part.renderType;
    displayPart.fixed = part.fixed;
    displayPart.isShadow = part.isShadow;
    displayPart.transparent = hasTransparentMaterial;
    displayPart.lodIndex = part.lodIndex;
    displayPart.meshName =
        part.bundleEntry.empty() ? part.objPath.filename().string() : part.bundleEntry;
    displayPart.visibleVariable = lower(part.visibleVariable);
    displayPart.visibleValue = part.visibleValue;
    displayPart.meshIdentifier = part.meshIdentifier;
    displayPart.animationParent = part.animationParent;
    displayPart.mouseEvent = part.mouseEvent;
    displayPart.backFaceCulling = parsed->backFaceCulling;
    displayPart.center = boundsCenter;
    displayPart.size = boundsSize;
    displayPart.radius = boundsRadius;
    displayPart.animations = std::move(animations);
    displayPart.interiorLightIndexes = part.interiorLightIndexes;
    const std::string wheelVariable = lower(part.wheelAnimation.rotationVariable);
    if (wheelVariable.rfind("wheel_rotation_", 0) == 0) {
        const std::size_t axleStart = std::string("wheel_rotation_").size();
        const std::size_t sideSeparator = wheelVariable.find('_', axleStart);
        if (sideSeparator != std::string::npos) {
            try {
                const int axleIndex =
                    std::stoi(wheelVariable.substr(axleStart, sideSeparator - axleStart));
                const std::string side = wheelVariable.substr(sideSeparator + 1);
                if (axleIndex >= 0 && (side == "l" || side == "r")) {
                    displayPart.odeWheelIndex = axleIndex * 2 + (side == "r" ? 1 : 0);
                }
            } catch (const std::exception&) {
                displayPart.odeWheelIndex = -1;
            }
        }
    }
    destination->push_back(std::move(displayPart));
    for (const std::string& key : groupOrder) {
        const MaterialState& state = groupStates[key];
        makeBatch(groups[key], groupTextures[key], groupTextureNames[key], groupColors[key],
                  groupEnvironmentNames[key], groupEnvironmentStrengths[key], state.alphaMode,
                  state.noZwrite, state.noZcheck, state.alphaScaleVariable, state.textureChanges,
                  state);
    }
    consolidateMaterialBatches(destination->back().batches);
    for (Batch& batch : destination->back().batches) {
        batch.visibilitySwitched = !destination->back().visibleVariable.empty();
        if (batch.buffer != 0 && !batch.bufferTracked) {
            assets->trackBuffer(batch.buffer);
            batch.bufferTracked = true;
        }
    }
    cacheReflectionTextureIndices(destination->back());
    if (verboseObjLoadLogs) {
        for (const Batch& batch : destination->back().batches) {
            gameLog.Log("OBJ batch: " + part.objPath.filename().string() +
                        " vertices=" + std::to_string(batch.vertexCount) +
                        " alpha=" + std::to_string(batch.alphaMode) +
                        " noZwrite=" + (batch.noZwrite ? "true" : "false") +
                        " texture=" + batch.textureName + " transmap=" + batch.transmap.name);
        }
    }
}

void Vehicle::ensureLoaded(RenderViewContext context, double viewDistance) {
    TraceScope trace("obj", "ensureLoaded");
    const int selectedLod = lodForDistance(viewDistance);
    activeLod = selectedLod;
    bool loadedPart = false;
    const bool immediateLoad =
        !hasLoadedInitialView || loadingPolicy.modelMode == AssetLoadingMode::Eager;
    const auto loadStart = std::chrono::steady_clock::now();
    constexpr auto loadBudget = std::chrono::milliseconds(4);
    static const std::size_t maxObjWorkers = [] {
        if (const char* value = openbus::getEnvironment("OPENBUS_OBJ_WORKERS")) {
            try {
                return static_cast<std::size_t>(std::clamp(std::stoi(value), 1, 64));
            } catch (const std::exception&) {
            }
        }
        const unsigned int concurrency = std::thread::hardware_concurrency();
        if (concurrency == 0) {
            return static_cast<std::size_t>(8);
        }
        return static_cast<std::size_t>(std::clamp(static_cast<int>(concurrency), 4, 32));
    }();
    static const std::size_t maxObjLoadsPerFrame = [] {
        if (const char* value = openbus::getEnvironment("OPENBUS_OBJ_LOADS_PER_FRAME")) {
            try {
                return static_cast<std::size_t>(std::clamp(std::stoi(value), 1, 128));
            } catch (const std::exception&) {
            }
        }
        return static_cast<std::size_t>(8);
    }();
    const std::size_t maxLoadsThisFrame = immediateLoad ? pendingParts.size() : maxObjLoadsPerFrame;
    const auto budgetReached = [&] {
        return !immediateLoad && std::chrono::steady_clock::now() - loadStart >= loadBudget;
    };

    std::size_t activeObjWorkers = 0;
    for (const Part& part : pendingParts) {
        if (!part.objRequest) {
            continue;
        }
        if (part.objRequest->future.wait_for(std::chrono::milliseconds(0)) !=
            std::future_status::ready) {
            ++activeObjWorkers;
        }
    }

    for (Part& part : pendingParts) {
        const bool viewpointMatches = openbus::rendering::viewpointMatches(part.viewpoint, context);
        const bool lodMatches = selectedLod < 0 || part.lodIndex == selectedLod;
        if (part.objRequest || (viewpointMatches && lodMatches)) {
            continue;
        }
        if (activeObjWorkers >= maxObjWorkers) {
            continue;
        }
        part.objRequest = std::make_shared<ObjRequest>();
        part.objRequest->future = parsedObjFuture(part.objPath, part.bundleEntry);
        ++activeObjWorkers;
    }

    std::size_t loadedThisFrame = 0;
    auto part = pendingParts.begin();
    while (part != pendingParts.end() && loadedThisFrame < maxLoadsThisFrame) {
        if (budgetReached() && loadedThisFrame > 0) {
            break;
        }
        const bool viewpointMatches =
            openbus::rendering::viewpointMatches(part->viewpoint, context);
        const bool shouldLoad =
            viewpointMatches && (selectedLod < 0 || part->lodIndex == selectedLod);
        if (!shouldLoad) {
            ++part;
            continue;
        }
        std::shared_ptr<ParsedObj> parsed;
        if (!part->objRequest) {
            parsed = parsedObjFuture(part->objPath, part->bundleEntry).get();
        } else {
            parsed = part->objRequest->future.get();
        }
        loadObj(*part, parsed);
        part = pendingParts.erase(part);
        loadedPart = true;
        ++loadedThisFrame;
    }

    auto backgroundPart = pendingParts.begin();
    while (backgroundPart != pendingParts.end() && loadedThisFrame < maxLoadsThisFrame) {
        if (budgetReached() && loadedThisFrame > 0) {
            break;
        }
        const bool viewpointMatches =
            openbus::rendering::viewpointMatches(backgroundPart->viewpoint, context);
        const bool lodMatches = selectedLod < 0 || backgroundPart->lodIndex == selectedLod;
        if ((viewpointMatches && lodMatches) || !backgroundPart->objRequest ||
            backgroundPart->objRequest->future.wait_for(std::chrono::milliseconds(0)) !=
                std::future_status::ready) {
            ++backgroundPart;
            continue;
        }
        const std::shared_ptr<ParsedObj> parsed = backgroundPart->objRequest->future.get();
        loadObj(*backgroundPart, parsed);
        backgroundPart = pendingParts.erase(backgroundPart);
        loadedPart = true;
        ++loadedThisFrame;
    }
    hasLoadedInitialView = true;
    if (loadedPart) {
        rebuildDisplayOrder();
    }
    loaded = !displayLists.empty() || !pendingParts.empty();
    if (!loggedAllObjectsLoaded && pendingParts.empty() && loaded) {
        gameLog.Log("All objects loaded. bodyParts=" + std::to_string(displayLists.size()) +
                    " wheelParts=generic");
        loggedAllObjectsLoaded = true;
    }
}
