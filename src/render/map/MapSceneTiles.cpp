// Per-tile map content: scenery placement resolution, instances, splines and water.

#include "render/map/MapSceneLoader.h"

namespace openbus::rendering {

void MapSceneLoader::applyTerrainAlignment(openbus::map::MapSceneryPose& pose, bool requested,
                                           const std::string& description) {
    if (!requested) {
        return;
    }
    const std::optional<std::array<double, 3>> normal = sampleTerrainNormal(pose.x, pose.y);
    if (!normal || !openbus::map::alignMapSceneryPoseToTerrainNormal(pose, *normal)) {
        recordSkipped(skipped, "[spline_terrain_align] placements without a usable terrain normal",
                      description);
    }
}

std::optional<MapSceneLoader::ResolvedSceneryPlacement>
MapSceneLoader::resolveMapSceneryPlacement(int objectId) {
    const auto cached = resolvedPlacements.find(objectId);
    if (cached != resolvedPlacements.end()) {
        return cached->second;
    }
    if (!resolvingPlacementIds.insert(objectId).second) {
        recordSkipped(skipped, "[attachObj] parent cycles",
                      "object ID=" + std::to_string(objectId));
        return std::nullopt;
    }
    struct ResolveGuard {
        std::unordered_set<int>& active;
        int id;
        ~ResolveGuard() {
            active.erase(id);
        }
    } guard{resolvingPlacementIds, objectId};

    const auto ordinary = sceneryObjectsById.find(objectId);
    if (ordinary != sceneryObjectsById.end()) {
        const IndexedSceneryObject& object = ordinary->second;
        if (!object.transformValid) {
            return std::nullopt;
        }
        openbus::map::MapSceneryPose pose = object.pose;
        const int terrainTileX =
            static_cast<int>(std::floor(pose.x / openbus::map::OMSI_TILE_SIZE_METERS));
        const int terrainTileY =
            static_cast<int>(std::floor(pose.y / openbus::map::OMSI_TILE_SIZE_METERS));
        const std::optional<double> ground = sampleTerrainHeight(
            terrainTileX, terrainTileY,
            pose.x - static_cast<double>(terrainTileX) * openbus::map::OMSI_TILE_SIZE_METERS,
            pose.y - static_cast<double>(terrainTileY) * openbus::map::OMSI_TILE_SIZE_METERS);
        const std::filesystem::path configPath = resolveSceneryConfigPath(object.assetPath);
        bool absoluteHeight = false;
        if (!configPath.empty()) {
            static_cast<void>(loadSceneryAsset(configPath));
            absoluteHeight = absoluteHeightSceneryAssets.contains(
                configPath.lexically_normal().generic_string());
        }
        pose.z = openbus::map::mapSceneryWorldHeight(pose.z, ground.value_or(0.0), absoluteHeight);
        applyTerrainAlignment(pose, object.splineTerrainAlign,
                              "object ID=" + std::to_string(objectId));
        ResolvedSceneryPlacement resolved{object.assetPath, pose};
        resolvedPlacements.emplace(objectId, resolved);
        return resolved;
    }

    const auto attached = attachedObjectsById.find(objectId);
    if (attached != attachedObjectsById.end()) {
        if (!attached->second.transformValid) {
            return std::nullopt;
        }
        const IndexedAttachedObject& object = attached->second;
        const std::optional<ResolvedSceneryPlacement> parent =
            resolveMapSceneryPlacement(object.parentId);
        if (!parent.has_value()) {
            return std::nullopt;
        }
        ModelAttachmentPoint attachment;
        if (object.attachmentIndex < 0) {
            return std::nullopt;
        }
        const std::filesystem::path parentConfig = resolveSceneryConfigPath(parent->assetPath);
        if (!parentConfig.empty()) {
            static_cast<void>(loadSceneryAsset(parentConfig));
            const auto attachmentDefinitions =
                sceneryAttachmentPoints.find(parentConfig.lexically_normal().generic_string());
            if (attachmentDefinitions != sceneryAttachmentPoints.end() &&
                !attachmentDefinitions->second.empty()) {
                const std::optional<std::size_t> attachmentOffset =
                    openbus::map::mapSceneryAttachmentPointOffset(
                        object.attachmentIndex, attachmentDefinitions->second.size());
                if (!attachmentOffset.has_value()) {
                    return std::nullopt;
                }
                attachment = attachmentDefinitions->second[*attachmentOffset];
            } else if (object.attachmentIndex != 0) {
                return std::nullopt;
            }
        } else if (object.attachmentIndex != 0) {
            return std::nullopt;
        }
        ResolvedSceneryPlacement resolved;
        resolved.assetPath = object.assetPath;
        resolved.pose = openbus::map::composeMapAttachedObjectPose(
            parent->pose, attachment.translation, attachment.rotationDegrees,
            object.rotationDegrees);
        applyTerrainAlignment(resolved.pose, object.splineTerrainAlign,
                              "attached object ID=" + std::to_string(objectId));
        resolvedPlacements.emplace(objectId, resolved);
        return resolved;
    }

    const auto splineAttachment = splineAttachmentTileById.find(objectId);
    if (splineAttachment == splineAttachmentTileById.end() ||
        splineAttachment->second >= map.tiles.size()) {
        return std::nullopt;
    }
    auto parentTile = splineAttachmentParentTiles.find(splineAttachment->second);
    if (parentTile == splineAttachmentParentTiles.end()) {
        const std::size_t tileIndex = splineAttachment->second;
        parentTile = splineAttachmentParentTiles
                         .emplace(tileIndex, openbus::map::loadMapTile(map.tiles[tileIndex]))
                         .first;
    }
    const auto& splineAttachmentRecord = parentTile->second.splineAttachments;
    const auto record =
        std::find_if(splineAttachmentRecord.begin(), splineAttachmentRecord.end(),
                     [objectId](const openbus::map::MapSplineAttachment& candidate) {
                         return candidate.id == objectId;
                     });
    if (record == splineAttachmentRecord.end()) {
        return std::nullopt;
    }
    const std::optional<openbus::map::MapSceneryPose> parentPose =
        openbus::map::placeMapSplineAttachmentParent(
            *record, parentTile->second.splines, parentTile->second.reference.x,
            parentTile->second.reference.y, mapSplineResolver, splineCantWidth);
    if (!parentPose.has_value()) {
        return std::nullopt;
    }
    ResolvedSceneryPlacement resolved{record->assetPath, *parentPose};
    applyTerrainAlignment(resolved.pose, record->splineTerrainAlign,
                          "spline attachment parent ID=" + std::to_string(objectId));
    resolvedPlacements.emplace(objectId, resolved);
    return resolved;
}

void MapSceneLoader::buildTileContent() {
    for (const openbus::map::MapTileReference& tileReference : map.tiles) {
        const int tileX = tileReference.x;
        const int tileY = tileReference.y;
        TraceScope tileTrace("map", "MapRenderer.processTile");
        const openbus::map::MapTileData tile = openbus::map::loadMapTile(tileReference);
        ++tilesProcessed;
        sceneryPlacementsParsed += tile.sceneryObjects.size() + tile.attachedObjects.size() +
                                   tile.splineAttachments.size();
        splinePlacementsParsed += tile.splines.size();
        const std::string tileName = tileLabel(tileX, tileY);
        for (const ConfigurationDiagnostic& diagnostic : tile.diagnostics.entries) {
            recordSkipped(skipped, "unimplemented map tile sections [" + diagnostic.keyword + "]",
                          tileName + " line=" + std::to_string(diagnostic.line));
        }
        const auto reportVariableParent = [&](int objectId, int objectTileX, int objectTileY,
                                              const std::optional<int>& parentNumber) {
            if (parentNumber.has_value()) {
                const auto tileObjects =
                    editorObjectNumbersByTile.find(tileKey(objectTileX, objectTileY));
                const bool resolved = tileObjects != editorObjectNumbersByTile.end() &&
                                      tileObjects->second.contains(*parentNumber);
                recordSkipped(
                    skipped,
                    resolved ? "[varparent] Object Nr. links resolved; scenery script-variable "
                               "inheritance is unsupported"
                             : "[varparent] references to missing same-tile Object Nr. records",
                    tileName + " id=" + std::to_string(objectId) +
                        " parentObjectNumber=" + std::to_string(*parentNumber));
            }
        };
        for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
            reportVariableParent(object.id, tileX, tileY, object.variableParentId);
        }
        for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
            reportVariableParent(object.id, tileX, tileY, object.variableParentId);
        }
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
            reportVariableParent(attachment.id, tileX, tileY, attachment.variableParentId);
        }
        const auto reportRules = [&](int objectId,
                                     const std::vector<openbus::map::MapTileRule>& rules) {
            for (const openbus::map::MapTileRule& rule : rules) {
                recordSkipped(skipped,
                              rule.kill ? "[kill_rule] records parsed but not applied"
                                        : "[rule] records parsed but not applied",
                              tileName + " id=" + std::to_string(objectId) +
                                  " first-field=" + rule.fields[0]);
            }
        };
        for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
            reportRules(object.id, object.rules);
        }
        for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
            reportRules(object.id, object.rules);
        }
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
            reportRules(attachment.id, attachment.rules);
        }
        for (const openbus::map::MapSplinePlacement& spline : tile.splines) {
            if (spline.splineTerrainAlign2.has_value()) {
                recordSkipped(skipped,
                              "[spline_terrain_align_2] values parsed but alignment not applied",
                              tileName + " spline=" + std::to_string(spline.splineId));
            }
            reportRules(spline.splineId, spline.rules);
        }
        if (tile.hasWaterMarker) {
            if (!tileReference.hasWaterFile) {
                recordSkipped(skipped, "[water] markers without a .map.water sidecar",
                              tileName + " " + tileReference.textPath.filename().string());
            } else {
                try {
                    const openbus::map::WaterData water =
                        openbus::map::loadWaterData(tileReference.waterPath);
                    std::vector<ModelVertex> vertices;
                    std::vector<std::uint32_t> indices;
                    vertices.reserve(water.surfaces.size() * 4);
                    indices.reserve(water.surfaces.size() * 6);
                    double minimumHeight = std::numeric_limits<double>::infinity();
                    double maximumHeight = -std::numeric_limits<double>::infinity();
                    double heightSum = 0.0;
                    std::size_t heightCount = 0;
                    for (const openbus::map::WaterSurface& surface : water.surfaces) {
                        const auto corners =
                            openbus::map::buildMapWaterSurfaceVertices(tileX, tileY, surface);
                        const std::uint32_t base = static_cast<std::uint32_t>(vertices.size());
                        for (std::size_t corner = 0; corner < corners.size(); ++corner) {
                            const openbus::map::MapWaterVertex& vertex = corners[corner];
                            vertices.push_back({static_cast<float>(vertex.x),
                                                static_cast<float>(vertex.y),
                                                static_cast<float>(vertex.z),
                                                corner == 1 || corner == 2 ? 1.0F : 0.0F,
                                                corner >= 2 ? 1.0F : 0.0F, 0.0F, 0.0F, 0.0F, 1.0F});
                            minimumHeight = (std::min)(minimumHeight, vertex.z);
                            maximumHeight = (std::max)(maximumHeight, vertex.z);
                            heightSum += vertex.z;
                            ++heightCount;
                        }
                        // SW, SE, NE, NW gives upward-facing triangles in the map XY plane.
                        indices.insert(indices.end(),
                                       {base, base + 1, base + 2, base, base + 2, base + 3});
                    }
                    if (!vertices.empty()) {
                        Impl::WaterBatch batch;
                        batch.indexCount = indices.size();
                        batch.tileX = tileX;
                        batch.tileY = tileY;
                        batch.minZ = minimumHeight;
                        batch.maxZ = maximumHeight;
                        batch.meanZ = heightSum / static_cast<double>(heightCount);
                        pglGenBuffers(1, &batch.buffer);
                        pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                        pglBufferData(
                            GL_ARRAY_BUFFER,
                            static_cast<std::ptrdiff_t>(vertices.size() * sizeof(ModelVertex)),
                            vertices.data(), GL_STATIC_DRAW);
                        pglGenBuffers(1, &batch.indexBuffer);
                        pglBindBuffer(GL_ARRAY_BUFFER, batch.indexBuffer);
                        pglBufferData(
                            GL_ARRAY_BUFFER,
                            static_cast<std::ptrdiff_t>(indices.size() * sizeof(std::uint32_t)),
                            indices.data(), GL_STATIC_DRAW);
                        if (batch.buffer == 0 || batch.indexBuffer == 0 ||
                            glGetError() != GL_NO_ERROR) {
                            if (batch.buffer != 0) {
                                pglDeleteBuffers(1, &batch.buffer);
                            }
                            if (batch.indexBuffer != 0) {
                                pglDeleteBuffers(1, &batch.indexBuffer);
                            }
                            recordSkipped(skipped, "water surfaces rejected by OpenGL upload",
                                          tileName);
                        } else {
                            impl_->waterBatches.push_back(batch);
                        }
                    }
                } catch (const std::exception& error) {
                    recordSkipped(skipped, "invalid .map.water sidecars",
                                  tileName + ": " + error.what());
                }
            }
        }
        if (tile.hasVariableTerrainMarker) {
            recordSkipped(skipped, "[variable_terrain] tiles (dynamic terrain not rendered)",
                          tileName + " " + tileReference.textPath.filename().string());
        }
        if (tile.hasVariableTerrainLightmapMarker) {
            recordSkipped(
                skipped,
                "[variable_terrainlightmap] tiles (dynamic terrain lightmaps not rendered)",
                tileName + " " + tileReference.textPath.filename().string());
        }
        const auto addSceneryInstance = [&](const std::string& assetPath,
                                            const std::string& description, double worldX,
                                            double worldY, double localZ,
                                            double terrainHeightOffset,
                                            const std::array<double, 6>& rotationDegrees,
                                            std::size_t rotationCount,
                                            const std::array<double, 9>* orientationMatrix =
                                                nullptr,
                                            std::vector<std::string> placementLabels = {}) {
            const std::filesystem::path configPath = resolveSceneryConfigPath(assetPath);
            if (configPath.empty()) {
                recordSkipped(skipped, "scenery placements whose .sco config was not found",
                              description);
                return;
            }
            const std::vector<std::size_t>& batchIndexes = loadSceneryAsset(configPath);
            if (editorOnlySceneryAssets.contains(configPath.lexically_normal().generic_string())) {
                recordSkipped(skipped, "map scenery placements marked [onlyeditor]",
                              description + " config=" + configPath.filename().string());
                return;
            }
            const std::string assetKey = configPath.lexically_normal().generic_string();
            auto lodSet = sceneryLodThresholdSetByAsset.find(assetKey);
            if (lodSet == sceneryLodThresholdSetByAsset.end()) {
                const std::size_t index = impl_->sceneryLodThresholdSets.size();
                impl_->sceneryLodThresholdSets.emplace_back();
                lodSet = sceneryLodThresholdSetByAsset.emplace(assetKey, index).first;
            }
            const std::size_t lodThresholdSetIndex = lodSet->second;
            std::size_t scriptStateIndex = std::numeric_limits<std::size_t>::max();
            if (sceneryAssetsNeedingVariables.contains(assetKey)) {
                auto scriptState = std::make_unique<Impl::SceneryScriptState>();
                scriptState->ownerTileX = tileX;
                scriptState->ownerTileY = tileY;
                scriptState->placementLabels = std::move(placementLabels);
                for (const std::string& variable : sceneryModelFloatVariables[assetKey]) {
                    scriptState->variables.declare(variable);
                }
                for (const std::string& variable : sceneryModelStringVariables[assetKey]) {
                    scriptState->variables.declareString(variable);
                }
                const auto scriptConfiguration = sceneryScriptConfigurations.find(assetKey);
                auto definition = sceneryRuntimeDefinitions.find(assetKey);
                if (definition == sceneryRuntimeDefinitions.end()) {
                    auto sharedDefinition = std::make_shared<Impl::SceneryRuntimeDefinition>();
                    if (scriptConfiguration != sceneryScriptConfigurations.end()) {
                        sharedDefinition->configuration = scriptConfiguration->second;
                    }
                    sharedDefinition->scriptTextures = sceneryModelScriptTextures[assetKey];
                    sharedDefinition->textTextures = sceneryModelTextTextures[assetKey];
                    sharedDefinition->floatVariables = sceneryModelFloatVariables[assetKey];
                    sharedDefinition->stringVariables = sceneryModelStringVariables[assetKey];
                    sharedDefinition->hasRuntime =
                        (scriptConfiguration != sceneryScriptConfigurations.end() &&
                         !scriptConfiguration->second.scripts.empty()) ||
                        !sharedDefinition->scriptTextures.empty() ||
                        !sharedDefinition->textTextures.empty() ||
                        !sceneryFreeTextureVariables[assetKey].empty();
                    definition =
                        sceneryRuntimeDefinitions.emplace(assetKey, std::move(sharedDefinition))
                            .first;
                }
                scriptState->definition = definition->second;
                scriptStateIndex = impl_->sceneryScriptStates.size();
                impl_->sceneryScriptStates.push_back(std::move(scriptState));
            }
            const bool absoluteHeight = absoluteHeightSceneryAssets.contains(assetKey);
            const double worldZ =
                openbus::map::mapSceneryWorldHeight(localZ, terrainHeightOffset, absoluteHeight);
            if (batchIndexes.empty()) {
                recordSkipped(skipped, "scenery placements with no renderable asset batches",
                              description + " config=" + configPath.filename().string());
            } else {
                ++impl_->sceneryObjectCount;
                ++impl_->sceneryObjectsByTile[tileKey(tileX, tileY)];
            }
            for (const std::size_t batchIndex : batchIndexes) {
                if (batchIndex >= impl_->sceneryBatches.size()) {
                    continue;
                }
                const double radius = impl_->sceneryBatches[batchIndex].boundingRadius;
                Impl::SceneryInstance instance;
                instance.batchIndex = batchIndex;
                instance.x = worldX;
                instance.y = worldY;
                instance.z = worldZ;
                instance.boundingRadius = radius;
                instance.transform = makeSceneryInstanceTransform(
                    worldX, worldY, worldZ, rotationDegrees, rotationCount, orientationMatrix);
                instance.scriptStateIndex = scriptStateIndex;
                instance.lodThresholdSetIndex = lodThresholdSetIndex;
                instance.sourceTileX = tileX;
                instance.sourceTileY = tileY;
                const int chunkX =
                    static_cast<int>(std::floor(worldX / openbus::map::OMSI_TILE_SIZE_METERS));
                const int chunkY =
                    static_cast<int>(std::floor(worldY / openbus::map::OMSI_TILE_SIZE_METERS));
                Impl::SceneryChunk& chunk = impl_->sceneryChunks[tileKey(chunkX, chunkY)];
                chunk.instances.push_back(instance);
                chunk.minX = std::min(chunk.minX, worldX - radius);
                chunk.minY = std::min(chunk.minY, worldY - radius);
                chunk.minZ = std::min(chunk.minZ, worldZ - radius);
                chunk.maxX = std::max(chunk.maxX, worldX + radius);
                chunk.maxY = std::max(chunk.maxY, worldY + radius);
                chunk.maxZ = std::max(chunk.maxZ, worldZ + radius);
                ++impl_->sceneryInstanceCount;
            }
        };

        for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
            const std::string objectDescription = tileName + " object=" + object.label +
                                                  " asset=" + object.assetPath + " rotationDeg=(" +
                                                  std::to_string(object.rotationDegrees[0]) + "," +
                                                  std::to_string(object.rotationDegrees[1]) + "," +
                                                  std::to_string(object.rotationDegrees[2]) + ")";
            if (!object.transformValid) {
                recordSkipped(
                    skipped, "[object] sections with invalid transforms",
                    objectDescription + " rawTransform=(" + object.rawTransformFields[0] + "," +
                        object.rawTransformFields[1] + "," + object.rawTransformFields[2] + "," +
                        object.rawTransformFields[3] + "," + object.rawTransformFields[4] + "," +
                        object.rawTransformFields[5] + ")");
                continue;
            }
            const std::optional<ResolvedSceneryPlacement> placement =
                resolveMapSceneryPlacement(object.id);
            if (!placement.has_value()) {
                recordSkipped(skipped, "[object] sections with invalid world placement",
                              objectDescription);
                continue;
            }
            const openbus::map::MapSceneryPose& pose = placement->pose;
            addSceneryInstance(placement->assetPath, objectDescription, pose.x, pose.y, pose.z, 0.0,
                               pose.rotationDegrees, pose.rotationCount,
                               pose.hasOrientationMatrix ? &pose.orientationMatrix : nullptr,
                               object.labels);
        }
        for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
            const std::string description = tileName +
                                            " [attachObj] id=" + std::to_string(object.id) +
                                            " parent=" + std::to_string(object.attachedToObjectId) +
                                            " asset=" + object.assetPath;
            if (!object.transformValid) {
                recordSkipped(skipped, "[attachObj] records with invalid rotation or anchor index",
                              description);
                continue;
            }
            const std::optional<ResolvedSceneryPlacement> placement =
                resolveMapSceneryPlacement(object.id);
            if (!placement.has_value()) {
                recordSkipped(
                    skipped,
                    "[attachObj] records with missing parents, attachment points, or cycles",
                    description + " anchor=" + std::to_string(object.attachPointIndexValue));
                continue;
            }
            addSceneryInstance(placement->assetPath, description, placement->pose.x,
                               placement->pose.y, placement->pose.z, 0.0,
                               placement->pose.rotationDegrees, placement->pose.rotationCount,
                               &placement->pose.orientationMatrix, object.labels);
        }
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
            const std::string description =
                tileName + (attachment.repeater ? " splineRepeater=" : " splineAttachment=") +
                attachment.label + " asset=" + attachment.assetPath +
                " splineIndex=" + std::to_string(attachment.splineIndex);
            if (!attachment.transformValid) {
                recordSkipped(skipped, "spline attachment records with invalid transforms",
                              description);
                continue;
            }
            const std::vector<openbus::map::MapSceneryPose> poses =
                openbus::map::placeMapSplineAttachment(attachment, tile.splines, tileX, tileY,
                                                       mapSplineResolver, splineCantWidth);
            if (poses.empty()) {
                recordSkipped(skipped, "spline attachment records with no local placement",
                              description);
                continue;
            }
            for (const openbus::map::MapSceneryPose& pose : poses) {
                openbus::map::MapSceneryPose renderedPose = pose;
                applyTerrainAlignment(renderedPose, attachment.splineTerrainAlign, description);
                addSceneryInstance(
                    attachment.assetPath, description, renderedPose.x, renderedPose.y,
                    renderedPose.z, 0.0, renderedPose.rotationDegrees, renderedPose.rotationCount,
                    renderedPose.hasOrientationMatrix ? &renderedPose.orientationMatrix : nullptr);
            }
        }
        for (const openbus::map::MapSplinePlacement& spline : tile.splines) {
            const std::string splineDescription =
                tileName + (spline.elevated ? " [spline_h] asset=" : " [spline] asset=") +
                spline.assetPath;
            if (!spline.geometryValid) {
                recordSkipped(skipped, "[spline] sections with invalid numeric geometry",
                              splineDescription);
                continue;
            }
            const std::filesystem::path profilePath =
                resolveSplineProfile(map, omsiRoot, spline.assetPath);
            if (profilePath.empty()) {
                recordSkipped(skipped, "spline profile files not found", splineDescription);
                continue;
            }
            const std::string profileKey = profilePath.lexically_normal().generic_string();
            const auto* loadedProfile = cachedSplineProfile(profilePath);
            if (loadedProfile == nullptr) {
                continue;
            }
            const openbus::map::MapSplineProfile& profile = *loadedProfile;
            if (profile.editorOnly) {
                if (editorOnlySplineProfiles.insert(profileKey).second) {
                    recordSkipped(skipped, "editor-only spline profiles",
                                  profilePath.filename().string());
                }
                continue;
            }
            if (profile.sections.empty()) {
                recordSkipped(skipped, "spline profiles with no graphical cross-section strips",
                              profilePath.filename().string());
                continue;
            }
            const double startX =
                static_cast<double>(tileX) * openbus::map::OMSI_TILE_SIZE_METERS + spline.localX;
            const double startY =
                static_cast<double>(tileY) * openbus::map::OMSI_TILE_SIZE_METERS + spline.localY;
            const std::vector<openbus::map::MapSplineSample> centerline =
                openbus::map::tessellateMapSpline(startX, startY, spline.rotationDegrees,
                                                  spline.length, spline.radius);
            if (centerline.size() < 2) {
                recordSkipped(skipped, "[spline] sections that failed geometry tessellation",
                              splineDescription);
                continue;
            }
            const std::optional<double> cantHalfWidth =
                openbus::map::mapSplineProfileCantHalfWidth(profile);
            for (const openbus::map::MapSplineProfileSection& section : profile.sections) {
                if (section.points.size() < 2) {
                    recordSkipped(skipped, "spline profile strips with fewer than two points",
                                  profilePath.filename().string());
                    continue;
                }
                if (section.textureIndex < 0 ||
                    static_cast<std::size_t>(section.textureIndex) >= profile.textures.size()) {
                    recordSkipped(skipped, "spline profile strips with invalid texture indices",
                                  profilePath.filename().string() +
                                      " index=" + std::to_string(section.textureIndex));
                    continue;
                }
                const std::string& textureName =
                    profile.textures[static_cast<std::size_t>(section.textureIndex)];
                const std::filesystem::path roadTexture =
                    findSplineTexture(profilePath, omsiRoot, textureName);
                const int alphaMode =
                    static_cast<std::size_t>(section.textureIndex) <
                            profile.textureAlphaModes.size()
                        ? profile.textureAlphaModes[static_cast<std::size_t>(section.textureIndex)]
                        : 0;
                const bool textured = !roadTexture.empty();
                if (!textured) {
                    recordSkipped(
                        skipped, "spline profile textures not found (geometry rendered untextured)",
                        profilePath.filename().string() + " texture=" + textureName);
                }
                const std::string tileBatchPrefix =
                    std::to_string(tileX) + "," + std::to_string(tileY) + "|";
                const std::string batchKey =
                    tileBatchPrefix +
                    (textured ? roadTexture.lexically_normal().generic_string()
                              : "untextured:" + profileKey + "#" +
                                    std::to_string(section.textureIndex)) +
                    "|alpha:" + std::to_string(alphaMode);
                RoadMeshGeometry& road = roadVertices[batchKey];
                if (road.vertices.empty()) {
                    road.texturePath = roadTexture;
                    road.textured = textured;
                    road.tileX = tileX;
                    road.tileY = tileY;
                    road.alphaMode = alphaMode;
                }
                appendRoadProfile(road, spline, centerline, section,
                                  roadConnections.joinsFor(spline, tileX, tileY), cantHalfWidth);
            }
        }
        if (tilesProcessed % 100 == 0 || tilesProcessed == map.tiles.size()) {
            gameLog.Log("Map content progress: " + std::to_string(tilesProcessed) + "/" +
                        std::to_string(map.tiles.size()) +
                        " tiles processed; scenery=" + std::to_string(sceneryPlacementsParsed) +
                        ", splines=" + std::to_string(splinePlacementsParsed) +
                        ", cached spline profiles=" + std::to_string(splineProfiles.size()) + ".");
        }
    }
    gameLog.Log("Map content phase complete: " + std::to_string(tilesProcessed) + " tiles, " +
                std::to_string(sceneryPlacementsParsed) + " scenery placements, " +
                std::to_string(splinePlacementsParsed) + " splines, " +
                std::to_string(splineProfiles.size()) + " unique profiles in " +
                std::to_string(elapsedMilliseconds(mapContentStart)) + " ms.");
}

} // namespace openbus::rendering
