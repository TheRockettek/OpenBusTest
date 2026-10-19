// Map drawing: terrain, water, roads and instanced scenery with culling/sorting.

#include "render/map/MapRendererInternal.h"

namespace openbus::rendering {

void MapRenderer::draw(bool renderTypeFourOnly) const {
    TraceScope trace("map", "MapRenderer::draw");
    IndirectCullingBatch indirectBatch;
    IndirectCullingScope indirectScope;
    bool indirect = false;
    // Map and vehicle passes share the clockwise convention; querying the
    // previous state forced a driver sync each call.
    glFrontFace(GL_CW);
    const Matrix4& view = modelViewMatrix();
    const ViewFrustum frustum = buildViewFrustum(projectionMatrix());
    const double cameraX = -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]);
    const double cameraY = -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]);
    const double cameraZ = -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14]);
    const std::vector<Impl::ActiveSceneryChunk>& activeSceneryChunks =
        impl_->activeSceneryChunks[renderTypeFourOnly ? 1 : 0];
    std::vector<CullingItem> cullingItems;
    std::vector<float> lodBoundaries;
    std::vector<std::uint32_t> visibility;
    std::unordered_map<const void*, std::size_t> cullingIndices;
    bool gpuCulling = false;
    if (computeCullingAvailable() || indirectCullingAvailable()) {
        TraceScope phase("map", "MapRenderer::draw.computeCulling");
        std::vector<std::uint32_t> lodOffsets;
        for (const auto& thresholds : impl_->sceneryLodThresholdSets) {
            lodOffsets.push_back(appendCullingLodBoundaries(lodBoundaries, thresholds));
        }
        const auto addSphere = [&](const void* key, const std::array<double, 3>& center,
                                   double radius, double minX, double minY, double maxX,
                                   double maxY, std::size_t count) -> CullingItem& {
            cullingIndices.emplace(key, cullingItems.size());
            CullingItem item = makeCullingItem(view, center, radius);
            item.draw[0] = count <= static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())
                               ? static_cast<std::uint32_t>(count)
                               : std::numeric_limits<std::uint32_t>::max();
            item.metadata[0] |= CullMapDistance;
            item.distanceBounds = {
                static_cast<float>(minX - cameraX), static_cast<float>(minY - cameraY),
                static_cast<float>(maxX - cameraX), static_cast<float>(maxY - cameraY)};
            cullingItems.push_back(item);
            return cullingItems.back();
        };
        const auto addBounds = [&](const void* key, double minX, double minY, double minZ,
                                   double maxX, double maxY, double maxZ, std::size_t count) {
            addSphere(key, {(minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5},
                      std::hypot(maxX - minX, maxY - minY, maxZ - minZ) * 0.5, minX, minY, maxX,
                      maxY, count);
        };
        const auto addTile = [&](const auto& tile) {
            if (!impl_->activeTileKeys.contains(tileKey(tile.tileX, tile.tileY)))
                return;
            const double x = tile.tileX * openbus::map::OMSI_TILE_SIZE_METERS;
            const double y = tile.tileY * openbus::map::OMSI_TILE_SIZE_METERS;
            addBounds(&tile, x, y, tile.minZ, x + openbus::map::OMSI_TILE_SIZE_METERS,
                      y + openbus::map::OMSI_TILE_SIZE_METERS, tile.maxZ, tile.indexCount);
        };
        if (!renderTypeFourOnly) {
            for (const auto& tile : impl_->terrainBuffers)
                addTile(tile);
            for (const auto& water : impl_->waterBatches)
                addTile(water);
            for (const auto& road : impl_->roadBatches) {
                if (road.hasVisibilityBounds &&
                    impl_->activeTileKeys.contains(tileKey(road.tileX, road.tileY))) {
                    addBounds(&road, road.minX, road.minY, road.minZ, road.maxX, road.maxY,
                              road.maxZ, road.indexCount);
                }
            }
        }
        for (const Impl::ActiveSceneryChunk& active : activeSceneryChunks) {
            const Impl::SceneryChunk& chunk = *active.chunk;
            // Broad-phase chunk rejection avoids CPU material/resource work for
            // entire distant groups; individual sphere/LOD tests remain on GPU.
            if (indirectCullingAvailable() &&
                (squaredDistanceToBounds(cameraX, cameraY, chunk.minX, chunk.minY, chunk.maxX,
                                         chunk.maxY) > MAP_VISIBILITY_RADIUS_SQUARED ||
                 outsideMapFrustum(frustum, view,
                                   {(chunk.minX + chunk.maxX) * 0.5,
                                    (chunk.minY + chunk.maxY) * 0.5,
                                    (chunk.minZ + chunk.maxZ) * 0.5},
                                   std::hypot(chunk.maxX - chunk.minX, chunk.maxY - chunk.minY,
                                              chunk.maxZ - chunk.minZ) *
                                       0.5)))
                continue;
            for (const Impl::SceneryInstance* instancePointer : active.instances) {
                const Impl::SceneryInstance& instance = *instancePointer;
                const auto& batch = impl_->sceneryBatches[instance.batchIndex];
                auto& item = addSphere(
                    &instance, {instance.x, instance.y, instance.z}, instance.boundingRadius,
                    instance.x - instance.boundingRadius, instance.y - instance.boundingRadius,
                    instance.x + instance.boundingRadius, instance.y + instance.boundingRadius,
                    batch.vertexCount);
                item.metadata[0] |= CullLod;
                item.metadata[1] = static_cast<std::uint32_t>(batch.sceneryLodIndex);
                if (instance.lodThresholdSetIndex < lodOffsets.size()) {
                    item.metadata[2] = lodOffsets[instance.lodThresholdSetIndex];
                    item.metadata[3] = static_cast<std::uint32_t>(
                        impl_->sceneryLodThresholdSets[instance.lodThresholdSetIndex].size());
                }
                // Map assets without thresholds still select LOD zero.
                if (item.metadata[3] == 0 && batch.sceneryLodIndex != 0) {
                    item.metadata[0] |= CullReject;
                }
            }
        }
        if (indirectCullingAvailable()) {
            indirect = indirectBatch.dispatch(cullingItems, makeCullingPlanes(projectionMatrix()),
                                              lodBoundaries);
            gpuCulling = indirect;
        } else {
            gpuCulling = computeCull(cullingItems, makeCullingPlanes(projectionMatrix()),
                                     lodBoundaries, visibility);
        }
    }
    const auto gpuVisible = [&](const void* key) {
        const auto found = cullingIndices.find(key);
        return found != cullingIndices.end() && (indirect || visibility[found->second] != 0);
    };
    const auto selectIndirect = [&](const void* key) {
        if (!indirect)
            return;
        const auto found = cullingIndices.find(key);
        indirectBatch.select(found != cullingIndices.end()
                                 ? found->second
                                 : std::numeric_limits<std::size_t>::max());
    };
    const auto outsideVisibilityBounds = [&](double minX, double minY, double maxX, double maxY) {
        return squaredDistanceToBounds(cameraX, cameraY, minX, minY, maxX, maxY) >
               MAP_VISIBILITY_RADIUS_SQUARED;
    };
    const auto outsideFrustumBounds = [&](double minX, double minY, double minZ, double maxX,
                                          double maxY, double maxZ) {
        const std::array<double, 3> center = {(minX + maxX) * 0.5, (minY + maxY) * 0.5,
                                              (minZ + maxZ) * 0.5};
        const double halfX = (maxX - minX) * 0.5;
        const double halfY = (maxY - minY) * 0.5;
        const double halfZ = (maxZ - minZ) * 0.5;
        return outsideMapFrustum(frustum, view, center,
                                 std::sqrt(halfX * halfX + halfY * halfY + halfZ * halfZ));
    };
    const auto refreshSceneryTexture = [&](Impl::SceneryScriptState& state, bool textTexture,
                                           int index, const ModelMaterial& material) -> GLuint {
        if (!state.runtime || index < 0) {
            return 0;
        }
        Impl::DynamicSceneryTexture& texture = state.dynamicTextures[{textTexture, index}];
        ScriptRuntime::ScriptTextureSnapshot snapshot;
        const bool copied =
            texture.texture == 0
                ? (textTexture ? state.runtime->copyTextTexture(index, snapshot)
                               : state.runtime->copyScriptTexture(index, snapshot))
                : (textTexture
                       ? state.runtime->copyTextTextureIfChanged(index, texture.revision, snapshot)
                       : state.runtime->copyScriptTextureIfChanged(index, texture.revision,
                                                                   snapshot));
        if (!copied || snapshot.width <= 0 || snapshot.height <= 0) {
            return 0;
        }

        const bool dimensionsChanged =
            texture.width != snapshot.width || texture.height != snapshot.height;
        const bool needsUpload = texture.texture == 0 || dimensionsChanged ||
                                 texture.revision != snapshot.revision ||
                                 texture.filtered != snapshot.filtered;
        const bool wrapChanged =
            texture.wrapS != material.textureWrapS || texture.wrapT != material.textureWrapT;
        if (!needsUpload && !wrapChanged) {
            return texture.texture;
        }
        if (needsUpload) {
            const std::size_t expectedBytes = static_cast<std::size_t>(snapshot.width) *
                                              static_cast<std::size_t>(snapshot.height) * 4U;
            if (snapshot.pixels.size() != expectedBytes) {
                return 0;
            }
        }
        const GLenum textureUnit = GL_TEXTURE0 + (textTexture ? 5 : 4);
        pglActiveTexture(textureUnit);
        if (texture.texture == 0) {
            glGenTextures(1, &texture.texture);
            if (texture.texture == 0) {
                openbus::rendering::invalidateTextureBindings();
                return 0;
            }
            impl_->ownedTextures.push_back(texture.texture);
        }
        glBindTexture(GL_TEXTURE_2D, texture.texture);
        const GLint filter = snapshot.filtered ? GL_LINEAR : GL_NEAREST;
        if (needsUpload) {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        }
        if (wrapChanged) {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, material.textureWrapS);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, material.textureWrapT);
        }
        if (needsUpload) {
            if (dimensionsChanged || texture.width == 0 || texture.height == 0) {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, snapshot.width, snapshot.height, 0, GL_RGBA,
                             GL_UNSIGNED_BYTE, snapshot.pixels.data());
            } else {
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, snapshot.width, snapshot.height, GL_RGBA,
                                GL_UNSIGNED_BYTE, snapshot.pixels.data());
            }
            if (glGetError() != GL_NO_ERROR) {
                openbus::rendering::invalidateTextureBindings();
                return 0;
            }
            texture.width = snapshot.width;
            texture.height = snapshot.height;
            texture.revision = snapshot.revision;
            texture.filtered = snapshot.filtered;
        }
        if (wrapChanged) {
            texture.wrapS = material.textureWrapS;
            texture.wrapT = material.textureWrapT;
        }
        openbus::rendering::invalidateTextureBindings();
        return texture.texture;
    };

    const auto drawTerrain = [&]() {
        TraceScope phase("map", "MapRenderer::draw.terrain");
        ModelMaterial material;
        material.texture = impl_->texture;
        material.textured = true;
        material.detailTexture = impl_->baseDetailTexture;
        material.useDetailTexture = impl_->baseDetailTexture != 0;
        for (const Impl::TileBuffer& tile : impl_->terrainBuffers) {
            if (!impl_->activeTileKeys.contains(tileKey(tile.tileX, tile.tileY))) {
                continue;
            }
            const double minX =
                static_cast<double>(tile.tileX) * openbus::map::OMSI_TILE_SIZE_METERS;
            const double minY =
                static_cast<double>(tile.tileY) * openbus::map::OMSI_TILE_SIZE_METERS;
            if (gpuCulling && !gpuVisible(&tile))
                continue;
            if (!gpuCulling &&
                outsideVisibilityBounds(minX, minY, minX + openbus::map::OMSI_TILE_SIZE_METERS,
                                        minY + openbus::map::OMSI_TILE_SIZE_METERS)) {
                continue;
            }
            if (!gpuCulling &&
                outsideFrustumBounds(minX, minY, tile.minZ,
                                     minX + openbus::map::OMSI_TILE_SIZE_METERS,
                                     minY + openbus::map::OMSI_TILE_SIZE_METERS, tile.maxZ)) {
                continue;
            }
            material.textureScale = tile.baseTextureScale;
            material.detailTextureScale = tile.baseDetailTextureScale;
            selectIndirect(&tile);
            drawIndexedModelBatch(tile.buffer, tile.indexBuffer, tile.indexCount, material,
                                  {1.0, 1.0, 1.0}, 1.0, 0);
            if (!tile.groundTextureLayers.empty()) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDepthFunc(GL_LEQUAL);
                glDepthMask(GL_FALSE);
                for (const ModelMaterial& layer : tile.groundTextureLayers) {
                    drawIndexedModelBatch(tile.buffer, tile.indexBuffer, tile.indexCount, layer,
                                          {1.0, 1.0, 1.0}, 1.0, 2);
                }
                glDepthMask(GL_TRUE);
                glDepthFunc(GL_LESS);
                glDisable(GL_BLEND);
            }
        }
    };
    const auto drawWater = [&]() {
        TraceScope phase("map", "MapRenderer::draw.water");
        std::vector<std::pair<const Impl::WaterBatch*, double>> visibleWater;
        for (const Impl::WaterBatch& water : impl_->waterBatches) {
            if (!impl_->activeTileKeys.contains(tileKey(water.tileX, water.tileY))) {
                continue;
            }
            const double minX =
                static_cast<double>(water.tileX) * openbus::map::OMSI_TILE_SIZE_METERS;
            const double minY =
                static_cast<double>(water.tileY) * openbus::map::OMSI_TILE_SIZE_METERS;
            const double maxX = minX + openbus::map::OMSI_TILE_SIZE_METERS;
            const double maxY = minY + openbus::map::OMSI_TILE_SIZE_METERS;
            if (gpuCulling
                    ? !gpuVisible(&water)
                    : (outsideVisibilityBounds(minX, minY, maxX, maxY) ||
                       outsideFrustumBounds(minX, minY, water.minZ, maxX, maxY, water.maxZ))) {
                continue;
            }
            const double centerX = (minX + maxX) * 0.5;
            const double centerY = (minY + maxY) * 0.5;
            const double viewDepth =
                -(view[2] * centerX + view[6] * centerY + view[10] * water.meanZ + view[14]);
            visibleWater.emplace_back(&water, viewDepth);
        }
        if (visibleWater.empty()) {
            return;
        }
        std::stable_sort(
            visibleWater.begin(), visibleWater.end(),
            [](const auto& first, const auto& second) { return first.second > second.second; });
        ModelMaterial material;
        const std::array<double, 3> waterColor = {0.24, 0.48, 0.60};
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        for (const auto& [water, depth] : visibleWater) {
            static_cast<void>(depth);
            selectIndirect(water);
            drawIndexedModelBatch(water->buffer, water->indexBuffer, water->indexCount, material,
                                  waterColor, 0.62, 0);
        }
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);
    };
    const auto drawSplines = [&]() {
        TraceScope phase("map", "MapRenderer::draw.splines");
        for (const Impl::ModelBatch& road : impl_->roadBatches) {
            if (!impl_->activeTileKeys.contains(tileKey(road.tileX, road.tileY))) {
                continue;
            }
            if (gpuCulling && road.hasVisibilityBounds && !gpuVisible(&road))
                continue;
            if (!gpuCulling && road.hasVisibilityBounds &&
                outsideVisibilityBounds(road.minX, road.minY, road.maxX, road.maxY)) {
                continue;
            }
            if (!gpuCulling && road.hasVisibilityBounds &&
                outsideFrustumBounds(road.minX, road.minY, road.minZ, road.maxX, road.maxY,
                                     road.maxZ)) {
                continue;
            }
            selectIndirect(&road);
            drawIndexedModelBatch(road.buffer, road.indexBuffer, road.indexCount, road.material,
                                  road.color, 1.0, road.alphaMode, road.cullBackFaces);
        }
    };
    std::vector<Impl::SceneryDraw>& sceneryDraws = impl_->sceneryDrawScratch;
    sceneryDraws.clear();
    if (impl_->resolvedFreeTexturesByBatch.size() < impl_->sceneryBatches.size()) {
        impl_->resolvedFreeTexturesByBatch.resize(impl_->sceneryBatches.size());
    }
    const auto drawSceneryInstance = [&](const Impl::SceneryInstance& instance,
                                         const Impl::ModelBatch& batch,
                                         const ModelMaterial& sceneryMaterial, int alphaMode) {
        selectIndirect(&instance);
        pushMatrix();
        multiplyMatrix(instance.transform);
        drawModelBatch(batch.buffer, batch.vertexCount, sceneryMaterial, batch.color, batch.alpha,
                       alphaMode, batch.cullBackFaces);
        popMatrix();
    };
    // Returns the texture to apply, or nullopt when the batch material is left unchanged.
    const auto resolveFreeTexture =
        [&](const Impl::ModelBatch& batch,
            const std::string& selectedTexture) -> std::optional<GLuint> {
        const std::filesystem::path texturePath = findSceneryTexture(
            batch.textureObjectRoot, batch.textureModelRoot, impl_->omsiRoot, selectedTexture);
        const std::string textureKey =
            texturePath.empty()
                ? lowercasePathComponent(
                      normalizedAssetPath(selectedTexture).lexically_normal().generic_string())
                : texturePath.lexically_normal().generic_string();
        if (impl_->unavailableSceneryTextures.contains(textureKey)) {
            return std::nullopt;
        }
        if (texturePath.empty()) {
            impl_->unavailableSceneryTextures.insert(textureKey);
            gameLog.Log("Map scenery free texture not found: " + selectedTexture);
            return std::nullopt;
        }
        auto texture = impl_->dynamicSceneryTextures.find(textureKey);
        if (texture == impl_->dynamicSceneryTextures.end()) {
            try {
                const GLuint loadedTexture = loadTexture(texturePath);
                impl_->ownedTextures.push_back(loadedTexture);
                texture = impl_->dynamicSceneryTextures.emplace(textureKey, loadedTexture).first;
            } catch (const std::exception& error) {
                impl_->unavailableSceneryTextures.insert(textureKey);
                gameLog.Log("Could not load map scenery free texture " +
                            texturePath.generic_string() + ": " + error.what());
                return std::nullopt;
            }
        }
        return texture->second;
    };
    {
        TraceScope prepareTrace("map", "MapRenderer::draw.prepareScenery");
        for (const Impl::ActiveSceneryChunk& active : activeSceneryChunks) {
            const Impl::SceneryChunk& chunk = *active.chunk;
            if (!gpuCulling &&
                (outsideVisibilityBounds(chunk.minX, chunk.minY, chunk.maxX, chunk.maxY) ||
                 outsideFrustumBounds(chunk.minX, chunk.minY, chunk.minZ, chunk.maxX, chunk.maxY,
                                      chunk.maxZ))) {
                continue;
            }
            for (const Impl::SceneryInstance* instancePointer : active.instances) {
                const Impl::SceneryInstance& instance = *instancePointer;
                if (gpuCulling
                        ? !gpuVisible(&instance)
                        : (outsideVisibilityBounds(instance.x - instance.boundingRadius,
                                                   instance.y - instance.boundingRadius,
                                                   instance.x + instance.boundingRadius,
                                                   instance.y + instance.boundingRadius) ||
                           outsideMapFrustum(frustum, view, {instance.x, instance.y, instance.z},
                                             instance.boundingRadius))) {
                    continue;
                }
                const Impl::ModelBatch& batch = impl_->sceneryBatches[instance.batchIndex];
                const std::vector<double>* lodThresholds =
                    instance.lodThresholdSetIndex < impl_->sceneryLodThresholdSets.size()
                        ? &impl_->sceneryLodThresholdSets[instance.lodThresholdSetIndex]
                        : nullptr;
                static const std::vector<double> noLodThresholds;
                const std::size_t selectedLod =
                    gpuCulling ? batch.sceneryLodIndex
                               : openbus::map::selectMapSceneryLod(
                                     lodThresholds != nullptr ? *lodThresholds : noLodThresholds,
                                     std::hypot(instance.x - cameraX, instance.y - cameraY,
                                                instance.z - cameraZ));
                if (!gpuCulling && batch.sceneryLodIndex != selectedLod) {
                    continue;
                }
                if (!batch.visibilityVariable.empty()) {
                    const bool hasScriptState =
                        instance.scriptStateIndex < impl_->sceneryScriptStates.size();
                    const float value =
                        hasScriptState
                            ? impl_->sceneryScriptStates[instance.scriptStateIndex]->variables.get(
                                  batch.visibilityVariable)
                            : 0.0F;
                    if (value != static_cast<float>(batch.visibilityValue)) {
                        continue;
                    }
                }
                ModelMaterial sceneryMaterial = batch.material;
                int alphaMode = batch.alphaMode;
                if (instance.scriptStateIndex < impl_->sceneryScriptStates.size()) {
                    Impl::SceneryScriptState& scriptState =
                        *impl_->sceneryScriptStates[instance.scriptStateIndex];
                    if (batch.textTextureIndex >= 0) {
                        sceneryMaterial.textTexture = refreshSceneryTexture(
                            scriptState, true, batch.textTextureIndex, sceneryMaterial);
                        sceneryMaterial.useTextTexture = sceneryMaterial.textTexture != 0;
                        if (sceneryMaterial.useTextTexture) {
                            // Text glyphs include partial alpha for antialiasing and a
                            // transparent background, so draw them as a blended overlay.
                            alphaMode = 2;
                        }
                    } else if (batch.scriptTextureIndex >= 0) {
                        sceneryMaterial.freeTexture = refreshSceneryTexture(
                            scriptState, false, batch.scriptTextureIndex, sceneryMaterial);
                        sceneryMaterial.useFreeTexture = sceneryMaterial.freeTexture != 0;
                    } else if (!batch.freeTextureVariable.empty()) {
                        const std::string selectedTexture =
                            scriptState.variables.getString(batch.freeTextureVariable);
                        if (!selectedTexture.empty()) {
                            auto& resolvedTextures =
                                impl_->resolvedFreeTexturesByBatch[instance.batchIndex];
                            auto resolved = resolvedTextures.find(selectedTexture);
                            if (resolved == resolvedTextures.end()) {
                                resolved = resolvedTextures
                                               .emplace(selectedTexture,
                                                        resolveFreeTexture(batch, selectedTexture))
                                               .first;
                            }
                            if (resolved->second.has_value()) {
                                sceneryMaterial.freeTexture = *resolved->second;
                                sceneryMaterial.useFreeTexture = *resolved->second != 0;
                            }
                        }
                    }
                }
                const double viewDepth = -(view[2] * instance.x + view[6] * instance.y +
                                           view[10] * instance.z + view[14]);
                sceneryDraws.push_back(
                    {&instance, &batch, std::move(sceneryMaterial), alphaMode, viewDepth});
            }
        }
    }

    const auto sceneryDrawGroup = [](int renderType) -> std::size_t {
        switch (renderType) {
        case -1:
            return 0;
        case -2:
            return 1;
        case -3:
            return 2;
        case 1:
            return 3;
        case 3:
            return 5;
        case 4:
            return 6;
        default:
            return 4;
        }
    };
    for (std::size_t group = 0; group < Impl::SCENERY_DRAW_GROUP_COUNT; ++group) {
        impl_->opaqueSceneryScratch[group].clear();
        impl_->transparentSceneryScratch[group].clear();
    }
    for (const Impl::SceneryDraw& scenery : sceneryDraws) {
        const std::size_t group = sceneryDrawGroup(scenery.batch->renderType);
        if (scenery.alphaMode == 2 || scenery.alphaMode == 3) {
            impl_->transparentSceneryScratch[group].push_back(&scenery);
        } else {
            impl_->opaqueSceneryScratch[group].push_back(&scenery);
        }
    }

    using SceneryDraw = Impl::SceneryDraw;
    const auto drawSceneryTypes = [&](std::size_t group) {
        TraceScope phase("map", "MapRenderer::draw.submitScenery");
        const std::vector<const SceneryDraw*>& opaqueScenery = impl_->opaqueSceneryScratch[group];
        std::vector<const SceneryDraw*>& transparentScenery =
            impl_->transparentSceneryScratch[group];
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        for (const SceneryDraw* scenery : opaqueScenery) {
            glDepthMask(scenery->batch->noZwrite ? GL_FALSE : GL_TRUE);
            // OMSI shadow overlays commonly request both flags: they must not
            // write depth, but still need depth testing so they cannot bleed
            // through foreground scenery or vehicles.
            const bool skipDepthCheck = scenery->batch->noZcheck && !scenery->batch->noZwrite;
            if (skipDepthCheck) {
                glDisable(GL_DEPTH_TEST);
            }
            drawSceneryInstance(*scenery->instance, *scenery->batch, scenery->material,
                                scenery->alphaMode);
            if (skipDepthCheck) {
                glEnable(GL_DEPTH_TEST);
            }
        }
        std::stable_sort(transparentScenery.begin(), transparentScenery.end(),
                         [](const SceneryDraw* first, const SceneryDraw* second) {
                             return first->viewDepth > second->viewDepth;
                         });
        if (transparentScenery.empty()) {
            glDepthMask(GL_TRUE);
            glDepthFunc(GL_LESS);
            glEnable(GL_DEPTH_TEST);
            return;
        }

        const bool hasTransmapDepthPrepass = std::any_of(
            transparentScenery.begin(), transparentScenery.end(), [](const SceneryDraw* scenery) {
                return scenery->alphaMode == 2 && scenery->material.useTransmap &&
                       !scenery->material.useTextTexture && !scenery->batch->noZwrite &&
                       !scenery->batch->noZcheck;
            });
        if (hasTransmapDepthPrepass) {
            GLboolean previousColorMask[4] = {GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE};
            glGetBooleanv(GL_COLOR_WRITEMASK, previousColorMask);
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
            for (const SceneryDraw* scenery : transparentScenery) {
                if (scenery->alphaMode != 2 || !scenery->material.useTransmap ||
                    scenery->material.useTextTexture || scenery->batch->noZwrite ||
                    scenery->batch->noZcheck) {
                    continue;
                }
                // The shader's alpha-mode-3 threshold keeps only fully opaque
                // transmap texels. Record those surfaces in depth before the
                // blended pass, matching the vehicle renderer's treatment.
                drawSceneryInstance(*scenery->instance, *scenery->batch, scenery->material, 3);
            }
            glColorMask(previousColorMask[0], previousColorMask[1], previousColorMask[2],
                        previousColorMask[3]);
        }
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        for (const SceneryDraw* scenery : transparentScenery) {
            // Text textures are transparent overlays: write depth only where the
            // shader retains visible glyph pixels, leaving the cleared background
            // transparent for the rest of the scene.
            const bool hasOpaqueTransmapDepth =
                scenery->alphaMode == 2 && scenery->material.useTransmap &&
                !scenery->material.useTextTexture && !scenery->batch->noZwrite &&
                !scenery->batch->noZcheck;
            glDepthFunc(hasOpaqueTransmapDepth ? GL_LEQUAL : GL_LESS);
            glDepthMask(scenery->material.useTextTexture && !scenery->batch->noZwrite ? GL_TRUE
                                                                                      : GL_FALSE);
            const bool skipDepthCheck = scenery->batch->noZcheck && !scenery->batch->noZwrite;
            if (skipDepthCheck) {
                glDisable(GL_DEPTH_TEST);
            }
            drawSceneryInstance(*scenery->instance, *scenery->batch, scenery->material,
                                scenery->alphaMode);
            if (skipDepthCheck) {
                glEnable(GL_DEPTH_TEST);
            }
        }
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);
    };

    if (renderTypeFourOnly) {
        drawSceneryTypes(6);
    } else {
        drawSceneryTypes(0);
        drawTerrain();
        drawWater();
        drawSceneryTypes(1);
        drawSplines();
        drawSceneryTypes(2);
        drawSceneryTypes(3);
        drawSceneryTypes(4);
        drawSceneryTypes(5);
    }
}

} // namespace openbus::rendering
