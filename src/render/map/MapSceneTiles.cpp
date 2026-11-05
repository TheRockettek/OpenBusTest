// Per-tile map content: scenery placement resolution, instances, splines and water.

#include "render/map/MapSceneLoader.h"

#include "core/Utf8Path.h"

namespace openbus::rendering {

void MapSceneLoader::indexSceneryTrafficPathsMapWide() {
    const auto addPaths = [&](int objectId, int tileX, int tileY, const std::string& assetPath,
                              const openbus::map::MapSceneryPose& pose,
                              const std::vector<openbus::map::MapTileRule>& rules) {
        if (ambiguousMapObjectIds.contains(objectId)) {
            return;
        }
        const SceneryObjectConfig* metadata = loadSceneryObjectMetadata(assetPath);
        if (metadata == nullptr) {
            return;
        }
        for (const SceneryObjectTrafficPath& path : metadata->trafficPaths) {
            const std::vector<openbus::map::MapSplineSample> samples =
                openbus::map::tessellateMapSpline(path.startPosition[0], path.startPosition[1],
                                                  path.headingDegrees, path.length, path.radius);
            if (samples.size() < 2) {
                continue;
            }
            openbus::map::MapTrafficRouteSegment segment;
            segment.points.reserve(samples.size());
            const auto toWorldPosition = [&](const std::array<double, 3>& localPosition) {
                const auto offset = openbus::map::transformMapSceneryVector(pose, localPosition);
                return std::array<double, 3>{pose.x + offset[0], pose.y + offset[1],
                                             pose.z + offset[2]};
            };
            for (const openbus::map::MapSplineSample& sample : samples) {
                const double localZ = openbus::map::mapSplineElevation(
                    path.startPosition[2], path.length, path.gradientStartPercent,
                    path.gradientEndPercent, std::nullopt, sample.distance);
                segment.points.push_back(toWorldPosition({sample.x, sample.y, localZ}));
            }
            segment.spline.tileX = tileX;
            segment.spline.tileY = tileY;
            segment.spline.coordinates = map.coordinates;
            segment.spline.placement.splineId = objectId;
            segment.pathRules = openbus::map::mapTrafficRulesForPath(rules, path.pathIndex);
            if (path.trafficSignal) {
                segment.trafficSignal = path.trafficSignal;
                auto& signal = *segment.trafficSignal;
                if (signal.hasConditionalOverrides && signal.program != nullptr) {
                    signal.controller =
                        std::make_shared<openbus::map::MapTrafficSignalController>(signal.program);
                    impl_->trafficSignalControllers.push_back(signal.controller);
                }
            }
            if (path.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD) {
                segment.sceneryPathIndex = path.pathIndex;
                const auto water =
                    indexedTrafficWaterByTile.find(tileKey(tileX, tileY));
                if (water != indexedTrafficWaterByTile.end() &&
                    !openbus::map::mapTrafficRouteSegmentClearsWater(
                        segment, water->second, map.coordinates.tileSize)) {
                    recordSkipped(skipped, "map-wide scenery road paths without water clearance",
                                  "object ID=" + std::to_string(objectId) +
                                      " path=" + std::to_string(path.pathIndex));
                    continue;
                }
                impl_->roadTrafficLaneNetwork.addSegment(std::move(segment));
                mapWideRoadSceneryPathIndexes[objectId].insert(path.pathIndex);
                roadTrafficLaneNetworkDirty = true;
            } else if (path.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
                segment.pathWidthMeters =
                    std::isfinite(path.widthMeters) ? std::max(1.0, path.widthMeters) : 1.0;
                impl_->humanTrafficLaneNetwork.addSegment(std::move(segment));
                mapWideHumanSceneryPathIndexes[objectId].insert(path.pathIndex);
                humanTrafficLaneNetworkDirty = true;
            }
        }
    };

    for (const auto& [objectId, object] : sceneryObjectsById) {
        if (const auto placement = resolveMapSceneryPlacement(objectId, false)) {
            addPaths(objectId, object.tileX, object.tileY, placement->assetPath, placement->pose,
                     object.rules);
        }
    }
    for (const auto& [objectId, object] : attachedObjectsById) {
        if (const auto placement = resolveMapSceneryPlacement(objectId, false)) {
            addPaths(objectId, object.tileX, object.tileY, placement->assetPath, placement->pose,
                     object.rules);
        }
    }
    for (const auto& [tileIndex, tile] : splineAttachmentParentTiles) {
        for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
            if (ambiguousMapObjectIds.contains(attachment.id)) {
                continue;
            }
            const auto poses = openbus::map::placeMapSplineAttachment(
                attachment, tile.splines, tile.reference.x, tile.reference.y, mapSplineResolver,
                splineCantWidth, map.coordinates);
            for (const openbus::map::MapSceneryPose& authoredPose : poses) {
                openbus::map::MapSceneryPose pose = authoredPose;
                applyTerrainAlignment(pose, attachment.splineTerrainAlign,
                                      "spline attachment ID=" +
                                          std::to_string(attachment.id));
                addPaths(attachment.id, tile.reference.x, tile.reference.y, attachment.assetPath,
                         pose, attachment.rules);
            }
        }
        (void)tileIndex;
    }
}

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
MapSceneLoader::resolveMapSceneryPlacement(int objectId, bool useCache) {
    if (ambiguousMapObjectIds.contains(objectId)) {
        recordSkipped(skipped, "map placements with ambiguous object IDs",
                      "object ID=" + std::to_string(objectId));
        return std::nullopt;
    }
    if (useCache) {
        const auto cached = resolvedPlacements.find(objectId);
        if (cached != resolvedPlacements.end()) {
            return cached->second;
        }
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
        const int terrainTileX = static_cast<int>(std::floor(pose.x / map.coordinates.tileSize));
        const int terrainTileY = static_cast<int>(std::floor(pose.y / map.coordinates.tileSize));
        const std::optional<double> ground = sampleTerrainHeight(
            terrainTileX, terrainTileY,
            pose.x - static_cast<double>(terrainTileX) * map.coordinates.tileSize,
            pose.y - static_cast<double>(terrainTileY) * map.coordinates.tileSize);
        const SceneryObjectConfig* metadata = loadSceneryObjectMetadata(object.assetPath);
        const bool absoluteHeight = metadata != nullptr && metadata->absoluteHeight;
        pose.z = openbus::map::mapSceneryWorldHeight(pose.z, ground.value_or(0.0), absoluteHeight);
        applyTerrainAlignment(pose, object.splineTerrainAlign,
                              "object ID=" + std::to_string(objectId));
        ResolvedSceneryPlacement resolved{object.assetPath, pose,
                                          absoluteHeight || ground.has_value()};
        if (useCache && resolved.worldHeightResolved) {
            resolvedPlacements.emplace(objectId, resolved);
        }
        return resolved;
    }

    const auto attached = attachedObjectsById.find(objectId);
    if (attached != attachedObjectsById.end()) {
        if (!attached->second.transformValid) {
            return std::nullopt;
        }
        const IndexedAttachedObject& object = attached->second;
        const std::optional<ResolvedSceneryPlacement> parent =
            resolveMapSceneryPlacement(object.parentId, useCache);
        if (!parent.has_value()) {
            return std::nullopt;
        }
        ModelAttachmentPoint attachment;
        if (object.attachmentIndex < 0) {
            return std::nullopt;
        }
        const SceneryObjectConfig* parentMetadata =
            loadSceneryObjectMetadata(parent->assetPath);
        if (parentMetadata != nullptr) {
            if (!parentMetadata->attachmentPoints.empty()) {
                const std::optional<std::size_t> attachmentOffset =
                    openbus::map::mapSceneryAttachmentPointOffset(
                        object.attachmentIndex, parentMetadata->attachmentPoints.size());
                if (!attachmentOffset.has_value()) {
                    return std::nullopt;
                }
                attachment = parentMetadata->attachmentPoints[*attachmentOffset];
            } else if (object.attachmentIndex != 0) {
                return std::nullopt;
            }
        } else if (object.attachmentIndex != 0) {
            return std::nullopt;
        }
        ResolvedSceneryPlacement resolved{
            object.assetPath,
            openbus::map::composeMapAttachedObjectPose(
                parent->pose, attachment.translation, attachment.rotationDegrees,
                object.rotationDegrees),
            parent->worldHeightResolved};
        applyTerrainAlignment(resolved.pose, object.splineTerrainAlign,
                              "attached object ID=" + std::to_string(objectId));
        if (useCache && resolved.worldHeightResolved) {
            resolvedPlacements.emplace(objectId, resolved);
        }
        return resolved;
    }

    const auto splineAttachment = splineAttachmentTileById.find(objectId);
    if (splineAttachment == splineAttachmentTileById.end() ||
        splineAttachment->second >= map.tiles.size()) {
        return std::nullopt;
    }
    const auto parentTile = splineAttachmentParentTiles.find(splineAttachment->second);
    if (parentTile == splineAttachmentParentTiles.end()) {
        throw std::logic_error("Cross-tile spline attachment parent was not prepared");
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
            parentTile->second.reference.y, mapSplineResolver, splineCantWidth, map.coordinates);
    if (!parentPose.has_value()) {
        return std::nullopt;
    }
    ResolvedSceneryPlacement resolved{record->assetPath, *parentPose, true};
    applyTerrainAlignment(resolved.pose, record->splineTerrainAlign,
                          "spline attachment parent ID=" + std::to_string(objectId));
    if (useCache) {
        resolvedPlacements.emplace(objectId, resolved);
    }
    return resolved;
}

void MapSceneLoader::buildTileContent(const openbus::map::MapTileReference& tileReference,
                                      const openbus::map::MapTileData& tile,
                                      PreparedRoadTile& preparedRoads) {
    const auto tileStart = std::chrono::steady_clock::now();
    const int tileX = tileReference.x;
    const int tileY = tileReference.y;
    gameLog.Debug("Map content tile " + tileLabel(tileX, tileY) + ": " +
                  openbus::pathToUtf8(tileReference.textPath));
    TraceScope tileTrace("map", "MapRenderer.processTile");
    ++tilesProcessed;
    sceneryPlacementsParsed +=
        tile.sceneryObjects.size() + tile.attachedObjects.size() + tile.splineAttachments.size();
    splinePlacementsParsed += tile.splines.size();
    const std::string tileName = tileLabel(tileX, tileY);
    for (const ConfigurationDiagnostic& diagnostic : tile.diagnostics.entries) {
        recordSkipped(skipped, "unimplemented map tile sections [" + diagnostic.keyword + "]",
                      tileName + " line=" + std::to_string(diagnostic.line), 1,
                      diagnostic.severity == ConfigurationDiagnostic::Severity::Error
                          ? LogLevel::Error
                          : LogLevel::Warning);
    }
    const auto reportVariableParent = [&](int objectId, const std::optional<int>& parentId) {
        if (parentId.has_value()) {
            const bool resolved = sceneryObjectsById.contains(*parentId) ||
                                  attachedObjectsById.contains(*parentId) ||
                                  splineAttachmentTileById.contains(*parentId);
            if (ambiguousMapObjectIds.contains(*parentId)) {
                recordSkipped(skipped, "[varparent] references ambiguous map object IDs",
                              tileName + " id=" + std::to_string(objectId) +
                                  " parentObjectId=" + std::to_string(*parentId));
            } else if (!resolved) {
                recordSkipped(skipped, "[varparent] references to missing map object IDs",
                              tileName + " id=" + std::to_string(objectId) +
                                  " parentObjectId=" + std::to_string(*parentId));
            }
        }
    };
    for (const openbus::map::MapSceneryPlacement& object : tile.sceneryObjects) {
        reportVariableParent(object.id, object.variableParentId);
    }
    for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
        reportVariableParent(object.id, object.variableParentId);
    }
    for (const openbus::map::MapSplineAttachment& attachment : tile.splineAttachments) {
        reportVariableParent(attachment.id, attachment.variableParentId);
    }
    for (const openbus::map::MapSplinePlacement& spline : tile.splines) {
        if (spline.splineTerrainAlign2.has_value()) {
            recordSkipped(skipped,
                          "[spline_terrain_align_2] values parsed but alignment not applied",
                          tileName + " spline=" + std::to_string(spline.splineId));
        }
    }
    const std::optional<openbus::map::WaterData>& tileWater = preparedRoads.waterData;
    if (tile.hasWaterMarker) {
        if (!tileReference.hasWaterFile) {
            recordSkipped(skipped, "[water] markers without a .map.water sidecar",
                          tileName + " " + openbus::pathToUtf8(tileReference.textPath.filename()));
        } else if (!preparedRoads.waterError.empty()) {
            recordSkipped(skipped, "invalid .map.water sidecars",
                          tileName + ": " + preparedRoads.waterError);
        } else if (!preparedRoads.waterVertices.empty()) {
            Impl::WaterBatch batch;
            batch.indexCount = preparedRoads.waterIndices.size();
            batch.tileX = tileX;
            batch.tileY = tileY;
            batch.minZ = preparedRoads.waterMinimumHeight;
            batch.maxZ = preparedRoads.waterMaximumHeight;
            batch.meanZ = preparedRoads.waterMeanHeight;
            pglGenBuffers(1, &batch.buffer);
            pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(preparedRoads.waterVertices.size() *
                                                      sizeof(ModelVertex)),
                          preparedRoads.waterVertices.data(), GL_STATIC_DRAW);
            pglGenBuffers(1, &batch.indexBuffer);
            pglBindBuffer(GL_ARRAY_BUFFER, batch.indexBuffer);
            pglBufferData(GL_ARRAY_BUFFER,
                          static_cast<std::ptrdiff_t>(preparedRoads.waterIndices.size() *
                                                      sizeof(std::uint32_t)),
                          preparedRoads.waterIndices.data(), GL_STATIC_DRAW);
            if (batch.buffer == 0 || batch.indexBuffer == 0 || glGetError() != GL_NO_ERROR) {
                if (batch.buffer != 0) {
                    deleteRendererBuffers(1, &batch.buffer);
                }
                if (batch.indexBuffer != 0) {
                    deleteRendererBuffers(1, &batch.indexBuffer);
                }
                recordSkipped(skipped, "water surfaces rejected by OpenGL upload", tileName);
            } else {
                impl_->waterBatches.push_back(batch);
            }
        }
    }
    if (tile.hasVariableTerrainMarker) {
        recordSkipped(skipped, "[variable_terrain] tiles (dynamic terrain not rendered)",
                      tileName + " " + openbus::pathToUtf8(tileReference.textPath.filename()));
    }
    if (tile.hasVariableTerrainLightmapMarker) {
        recordSkipped(skipped,
                      "[variable_terrainlightmap] tiles (dynamic terrain lightmaps not rendered)",
                      tileName + " " + openbus::pathToUtf8(tileReference.textPath.filename()));
    }
    const auto addSceneryInstance = [&](const std::string& assetPath,
                                        const std::string& description, double worldX,
                                        double worldY, double localZ, double terrainHeightOffset,
                                        const std::array<double, 6>& rotationDegrees,
                                        std::size_t rotationCount,
                                        const std::array<double, 9>* orientationMatrix = nullptr,
                                        std::vector<std::string> placementLabels = {},
                                        const std::vector<openbus::map::MapTileRule>*
                                            placementRules = nullptr,
                                        std::optional<int> mapObjectId = std::nullopt,
                                        std::optional<int> variableParentId = std::nullopt,
                                        std::optional<int> attachedParentId = std::nullopt) {
        const std::filesystem::path configPath = resolveSceneryConfigPath(assetPath);
        if (configPath.empty()) {
            recordSkipped(skipped, "scenery placements whose .sco config was not found",
                          description);
            return;
        }
        const std::vector<std::size_t>& batchIndexes = loadSceneryAsset(configPath);
        const std::string assetKey = openbus::pathToUtf8(configPath.lexically_normal());
        const bool absoluteHeight = absoluteHeightSceneryAssets.contains(assetKey);
        const double worldZ =
            openbus::map::mapSceneryWorldHeight(localZ, terrainHeightOffset, absoluteHeight);
        const auto editorPassenger = editorPassengerAssets.find(assetKey);
        if (editorPassenger != editorPassengerAssets.end()) {
            const EditorPassengerAsset& asset = editorPassenger->second;
            openbus::map::MapSceneryPose pose;
            pose.x = worldX;
            pose.y = worldY;
            pose.z = worldZ;
            pose.rotationDegrees = rotationDegrees;
            pose.rotationCount = rotationCount;
            if (orientationMatrix != nullptr) {
                pose.orientationMatrix = *orientationMatrix;
                pose.hasOrientationMatrix = true;
            }
            const auto worldPoint = [&](const std::array<double, 3>& local) {
                const auto offset = openbus::map::transformMapSceneryVector(pose, local);
                return std::array<double, 3>{worldX + offset[0], worldY + offset[1],
                                             worldZ + offset[2]};
            };
            EditorPassengerDebugObject debug;
            debug.tileX = tileX;
            debug.tileY = tileY;
            debug.objectId = mapObjectId;
            debug.name = placementLabels.empty() ? std::string() : placementLabels.front();
            debug.busStop = asset.busStop;
            debug.passengerCabin = asset.passengerCabin;
            debug.position = {worldX, worldY, worldZ};
            std::array<double, 3> minimum = {-0.5, -0.5, 0.0};
            std::array<double, 3> maximum = {0.5, 0.5, 1.0};
            if (asset.hasBoundingBox && asset.boundingBox[0] > 0.0 && asset.boundingBox[1] > 0.0 &&
                asset.boundingBox[2] > 0.0) {
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    minimum[axis] = asset.boundingBox[axis + 3] - asset.boundingBox[axis] * 0.5;
                    maximum[axis] = asset.boundingBox[axis + 3] + asset.boundingBox[axis] * 0.5;
                }
            } else if (!asset.positions.empty()) {
                minimum = asset.positions.front().position;
                maximum = minimum;
                for (const PassengerCabinPosition& passenger : asset.positions) {
                    for (std::size_t axis = 0; axis < 3; ++axis) {
                        minimum[axis] = std::min(minimum[axis], passenger.position[axis]);
                        maximum[axis] = std::max(maximum[axis], passenger.position[axis]);
                    }
                }
                minimum[0] -= 0.3;
                minimum[1] -= 0.3;
                maximum[0] += 0.3;
                maximum[1] += 0.3;
                maximum[2] += 1.8;
            }
            for (std::size_t corner = 0; corner < 8; ++corner) {
                debug.bounds[corner] = worldPoint({(corner & 1) ? maximum[0] : minimum[0],
                                                   (corner & 2) ? maximum[1] : minimum[1],
                                                   (corner & 4) ? maximum[2] : minimum[2]});
            }
            for (const PassengerCabinPosition& passenger : asset.positions) {
                debug.passengerPositions.push_back(openbus::map::mapTrafficSceneryPassengerPose(
                    pose, passenger.position, passenger.seatHeight, passenger.rotationDegrees));
            }
            impl_->editorPassengerDebugObjects.push_back(std::move(debug));
        }
        const auto trafficPaths = sceneryTrafficPaths.find(assetKey);
        const bool trafficLightModel = sceneryTrafficLightAssets.contains(assetKey);
        std::optional<openbus::map::MapTrafficSignalSchedule> sceneryTrafficSignal;
        std::map<std::size_t, openbus::map::MapTrafficSignalSchedule>
            sceneryTrafficSignalsByLightIndex;
        std::shared_ptr<openbus::map::MapTrafficSignalController> signalController;
        const auto attachController = [&](openbus::map::MapTrafficSignalSchedule schedule) {
            if (schedule.hasConditionalOverrides && schedule.program != nullptr) {
                if (!signalController) {
                    signalController = std::make_shared<openbus::map::MapTrafficSignalController>(
                        schedule.program);
                }
                schedule.controller = signalController;
            }
            return schedule;
        };
        const auto configuredSignals = sceneryTrafficLightSchedules.find(assetKey);
        if (configuredSignals != sceneryTrafficLightSchedules.end()) {
            for (const auto& configuredSignal : configuredSignals->second) {
                if (!configuredSignal.has_value()) {
                    continue;
                }
                const auto schedule = attachController(*configuredSignal);
                sceneryTrafficSignalsByLightIndex.try_emplace(schedule.lightIndex, schedule);
                if (!sceneryTrafficSignal.has_value()) {
                    sceneryTrafficSignal = schedule;
                }
            }
        }
        if (trafficPaths != sceneryTrafficPaths.end() && !trafficPaths->second.empty()) {
            openbus::map::MapSceneryPose pathPose;
            pathPose.x = worldX;
            pathPose.y = worldY;
            pathPose.z = worldZ;
            pathPose.rotationDegrees = rotationDegrees;
            pathPose.rotationCount = rotationCount;
            if (orientationMatrix != nullptr) {
                std::copy(orientationMatrix->begin(), orientationMatrix->end(),
                          pathPose.orientationMatrix.begin());
                pathPose.hasOrientationMatrix = true;
            }
            const auto toWorldPosition = [&](const std::array<double, 3>& localPosition) {
                std::array<double, 3> position =
                    openbus::map::transformMapSceneryVector(pathPose, localPosition);
                position[0] += worldX;
                position[1] += worldY;
                position[2] += worldZ;
                return position;
            };
            for (const SceneryObjectTrafficPath& path : trafficPaths->second) {
                const std::vector<openbus::map::MapSplineSample> samples =
                    openbus::map::tessellateMapSpline(path.startPosition[0], path.startPosition[1],
                                                      path.headingDegrees, path.length,
                                                      path.radius);
                if (samples.size() < 2) {
                    recordSkipped(skipped, "scenery traffic paths with invalid geometry",
                                  description +
                                      " config=" + openbus::pathToUtf8(configPath.filename()));
                    continue;
                }
                TrafficPathDebugSegment segment;
                segment.points.reserve(samples.size());
                for (const openbus::map::MapSplineSample& sample : samples) {
                    const double localZ = openbus::map::mapSplineElevation(
                        path.startPosition[2], path.length, path.gradientStartPercent,
                        path.gradientEndPercent, std::nullopt, sample.distance);
                    segment.points.push_back(toWorldPosition({sample.x, sample.y, localZ}));
                }
                segment.pathTypeCode = path.pathTypeCode;
                segment.widthMeters = path.widthMeters;
                segment.mapObjectId = mapObjectId;
                segment.pathIndex = path.pathIndex;
                segment.tileX = tileX;
                segment.tileY = tileY;
                const std::optional<openbus::map::MapTrafficSignalSchedule> pathSignal =
                    path.trafficSignal.has_value()
                        ? std::optional<openbus::map::MapTrafficSignalSchedule>(
                              attachController(*path.trafficSignal))
                        : std::nullopt;
                segment.trafficSignal = pathSignal;
                if (pathSignal.has_value()) {
                    sceneryTrafficSignalsByLightIndex.try_emplace(pathSignal->lightIndex,
                                                                  *pathSignal);
                }
                if (pathSignal.has_value() && !sceneryTrafficSignal.has_value()) {
                    sceneryTrafficSignal = pathSignal;
                }
                if (path.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_ROAD) {
                    openbus::map::MapTrafficRouteSegment lane;
                    lane.points = segment.points;
                    lane.spline.tileX = tileX;
                    lane.spline.tileY = tileY;
                    lane.spline.coordinates = map.coordinates;
                    if (mapObjectId.has_value()) {
                        lane.spline.placement.splineId = *mapObjectId;
                        lane.sceneryPathIndex = path.pathIndex;
                    }
                    lane.trafficSignal = pathSignal;
                    if (placementRules != nullptr) {
                        lane.pathRules =
                            openbus::map::mapTrafficRulesForPath(*placementRules, path.pathIndex);
                    }
                    segment.pathRules = lane.pathRules;
                    const auto indexedPaths =
                        mapObjectId ? mapWideRoadSceneryPathIndexes.find(*mapObjectId)
                                    : mapWideRoadSceneryPathIndexes.end();
                    const bool indexedMapWide =
                        indexedPaths != mapWideRoadSceneryPathIndexes.end() &&
                        indexedPaths->second.contains(path.pathIndex);
                    if (!indexedMapWide && tileWater &&
                               !openbus::map::mapTrafficRouteSegmentClearsWater(
                                   lane, *tileWater, map.coordinates.tileSize)) {
                        recordSkipped(skipped, "scenery road paths without water clearance",
                                      tileName +
                                          " object ID=" + std::to_string(mapObjectId.value_or(0)) +
                                          " path=" + std::to_string(path.pathIndex));
                    } else if (!indexedMapWide) {
                        roadTrafficLaneNetworkDirty = true;
                        impl_->roadTrafficLaneNetwork.addSegment(std::move(lane));
                    }
                } else if (path.pathTypeCode == openbus::map::MAP_SPLINE_PATH_TYPE_HUMAN) {
                    openbus::map::MapTrafficRouteSegment lane;
                    lane.points = segment.points;
                    lane.pathWidthMeters =
                        std::isfinite(path.widthMeters) ? std::max(1.0, path.widthMeters) : 1.0;
                    lane.trafficSignal = pathSignal;
                    const auto indexedPaths =
                        mapObjectId ? mapWideHumanSceneryPathIndexes.find(*mapObjectId)
                                    : mapWideHumanSceneryPathIndexes.end();
                    const bool indexedMapWide =
                        indexedPaths != mapWideHumanSceneryPathIndexes.end() &&
                        indexedPaths->second.contains(path.pathIndex);
                    if (!indexedMapWide) {
                        humanTrafficLaneNetworkDirty = true;
                        impl_->humanTrafficLaneNetwork.addSegment(std::move(lane));
                    }
                }
                impl_->sceneryTrafficPathSegments.push_back(segment);
            }
        }
        if (signalController) {
            impl_->trafficSignalControllers.push_back(std::move(signalController));
        }
        auto lodSet = sceneryLodThresholdSetByAsset.find(assetKey);
        if (lodSet == sceneryLodThresholdSetByAsset.end()) {
            const std::size_t index = impl_->sceneryLodThresholdSets.size();
            impl_->sceneryLodThresholdSets.emplace_back();
            lodSet = sceneryLodThresholdSetByAsset.emplace(assetKey, index).first;
        }
        const std::size_t lodThresholdSetIndex = lodSet->second;
        std::size_t scriptStateIndex = std::numeric_limits<std::size_t>::max();
        if (sceneryAssetsNeedingVariables.contains(assetKey) || trafficLightModel ||
            sceneryTrafficSignal.has_value()) {
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
                    sceneryRuntimeDefinitions.emplace(assetKey, std::move(sharedDefinition)).first;
            }
            scriptState->definition = definition->second;
            scriptState->trafficSignal = std::move(sceneryTrafficSignal);
            scriptState->trafficSignalsByLightIndex = std::move(sceneryTrafficSignalsByLightIndex);
            scriptState->mapObjectId = mapObjectId;
            scriptState->variableParentId =
                variableParentId.has_value() ? variableParentId : attachedParentId;
            scriptState->resolvedSignalParentObjectId = scriptState->variableParentId;
            scriptState->worldPosition = {worldX, worldY, worldZ};
            scriptState->trafficLightModel = trafficLightModel;
            scriptStateIndex = impl_->sceneryScriptStates.size();
            impl_->sceneryScriptStates.push_back(std::move(scriptState));
            impl_->sceneryScriptStatesByTile[tileKey(tileX, tileY)].push_back(scriptStateIndex);
        }
        // Editor-only junctions still own the schedules referenced by visible heads.
        if (editorOnlySceneryAssets.contains(assetKey)) {
            recordSkipped(skipped, "map scenery placements marked [onlyeditor]",
                          description + " config=" + openbus::pathToUtf8(configPath.filename()), 1,
                          LogLevel::Info);
            return;
        }
        if (batchIndexes.empty()) {
            recordSkipped(skipped, "scenery placements with no renderable asset batches",
                          description + " config=" + openbus::pathToUtf8(configPath.filename()));
        } else {
            ++impl_->sceneryObjectCount;
            ++impl_->sceneryObjectsByTile[tileKey(tileX, tileY)];
        }
        for (const std::size_t batchIndex : batchIndexes) {
            if (batchIndex >= impl_->sceneryBatches.size()) {
                continue;
            }
            const double radius = impl_->sceneryBatches[batchIndex].boundingRadius *
                                  impl_->sceneryProjectionRadiusScale;
            Impl::SceneryInstance instance;
            instance.batchIndex = batchIndex;
            instance.mapObjectId = mapObjectId;
            instance.x = worldX;
            instance.y = worldY;
            instance.z = worldZ;
            instance.boundingRadius = radius;
            instance.poleCrash =
                mapObjectId.has_value() && poleCrashSceneryAssets.contains(assetKey);
            instance.baseTransform = makeSceneryInstanceTransform(
                worldX, worldY, worldZ, rotationDegrees, rotationCount, orientationMatrix);
            instance.transform = instance.baseTransform;
            instance.scriptStateIndex = scriptStateIndex;
            instance.lodThresholdSetIndex = lodThresholdSetIndex;
            instance.sourceTileX = tileX;
            instance.sourceTileY = tileY;
            const int chunkX = static_cast<int>(std::floor(worldX / map.coordinates.tileSize));
            const int chunkY = static_cast<int>(std::floor(worldY / map.coordinates.tileSize));
            const std::uint64_t chunkKey = tileKey(chunkX, chunkY);
            Impl::SceneryChunk& chunk = impl_->sceneryChunks[chunkKey];
            impl_->sceneryInstancesBySourceTile[tileKey(tileX, tileY)].push_back(
                {chunkKey, chunk.instances.size()});
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
            recordSkipped(skipped, "[object] sections with invalid transforms",
                          objectDescription + " rawTransform=(" + object.rawTransformFields[0] +
                              "," + object.rawTransformFields[1] + "," +
                              object.rawTransformFields[2] + "," + object.rawTransformFields[3] +
                              "," + object.rawTransformFields[4] + "," +
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
                           object.labels, &object.rules, object.id, object.variableParentId);
    }
    for (const openbus::map::MapAttachedObject& object : tile.attachedObjects) {
        const std::string description = tileName + " [attachObj] id=" + std::to_string(object.id) +
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
            recordSkipped(skipped,
                          "[attachObj] records with missing parents, attachment points, or cycles",
                          description + " anchor=" + std::to_string(object.attachPointIndexValue));
            continue;
        }
        addSceneryInstance(placement->assetPath, description, placement->pose.x, placement->pose.y,
                           placement->pose.z, 0.0, placement->pose.rotationDegrees,
                           placement->pose.rotationCount, &placement->pose.orientationMatrix,
                           object.labels, &object.rules, object.id, object.variableParentId,
                           object.attachedToObjectId);
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
                                                   mapSplineResolver, splineCantWidth,
                                                   map.coordinates);
        if (poses.empty()) {
            recordSkipped(skipped, "spline attachment records with no local placement",
                          description);
            continue;
        }
        for (const openbus::map::MapSceneryPose& pose : poses) {
            openbus::map::MapSceneryPose renderedPose = pose;
            applyTerrainAlignment(renderedPose, attachment.splineTerrainAlign, description);
            addSceneryInstance(
                attachment.assetPath, description, renderedPose.x, renderedPose.y, renderedPose.z,
                0.0, renderedPose.rotationDegrees, renderedPose.rotationCount,
                renderedPose.hasOrientationMatrix ? &renderedPose.orientationMatrix : nullptr, {},
                &attachment.rules, attachment.id, attachment.variableParentId);
        }
    }
    for (const auto& [reason, example, count] : preparedRoads.skipped) {
        recordSkipped(skipped, reason, example, count);
    }
    for (const std::string& profileKey : preparedRoads.editorOnlyProfileKeys) {
        if (editorOnlySplineProfiles.insert(profileKey).second) {
            recordSkipped(skipped, "editor-only spline profiles",
                          openbus::pathToUtf8(openbus::pathFromUtf8(profileKey).filename()), 1,
                          LogLevel::Info);
        }
    }
    for (const auto& [path, image] : preparedRoads.decodedTextures) {
        rememberPreparedTexture(path, image);
    }
    for (const auto& [path, error] : preparedRoads.textureErrors) {
        preparedTextureErrors.try_emplace(path, error);
    }
    for (auto& [batchKey, geometry] : preparedRoads.geometryByBatch) {
        roadVertices.emplace(batchKey, std::move(geometry));
    }
    gameLog.Debug("Map content tile loaded: " + tileName +
                  "; loaded tiles=" + std::to_string(tilesProcessed) +
                  ", scenery=" + std::to_string(sceneryPlacementsParsed) +
                  ", splines=" + std::to_string(splinePlacementsParsed) +
                  ", cached spline profiles=" + std::to_string(splineProfiles.size()) + "; took " +
                  std::to_string(elapsedMilliseconds(tileStart)) + " ms.");
}

} // namespace openbus::rendering
