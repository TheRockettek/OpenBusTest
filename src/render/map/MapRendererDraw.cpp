// Map drawing: terrain, water, roads and instanced scenery with culling/sorting.

#include "render/map/MapRendererInternal.h"

#include "core/Utf8Path.h"

#include <string_view>

namespace openbus::rendering {
namespace {

constexpr GLenum anySamplesPassed = 0x8C2F;
constexpr GLenum queryWait = 0x8E13;

const std::vector<PrimitiveVertex>& occlusionProxyCube() {
    static const std::vector<PrimitiveVertex> vertices = [] {
        constexpr std::array<std::array<float, 3>, 8> corners = {{
            {-0.5F, -0.5F, -0.5F},
            {0.5F, -0.5F, -0.5F},
            {0.5F, 0.5F, -0.5F},
            {-0.5F, 0.5F, -0.5F},
            {-0.5F, -0.5F, 0.5F},
            {0.5F, -0.5F, 0.5F},
            {0.5F, 0.5F, 0.5F},
            {-0.5F, 0.5F, 0.5F},
        }};
        constexpr std::array<std::array<int, 4>, 6> faces = {{
            {0, 1, 2, 3},
            {4, 7, 6, 5},
            {0, 4, 5, 1},
            {1, 5, 6, 2},
            {2, 6, 7, 3},
            {4, 0, 3, 7},
        }};
        std::vector<PrimitiveVertex> result;
        result.reserve(36);
        for (const auto& face : faces) {
            for (const int index : {face[0], face[1], face[2], face[0], face[2], face[3]}) {
                const auto& point = corners[static_cast<std::size_t>(index)];
                result.push_back({point[0], point[1], point[2], 1.0F, 1.0F, 1.0F});
            }
        }
        return result;
    }();
    return vertices;
}

bool depthOcclusionQueriesRequested() {
    const char* setting = openbus::getEnvironment("OPENBUS_OCCLUSION_CULLING");
    return setting != nullptr && std::string_view(setting) == "queries";
}

bool depthOcclusionQueriesAvailable() {
    return pglGenQueries != nullptr && pglDeleteQueries != nullptr && pglBeginQuery != nullptr &&
           pglEndQuery != nullptr && pglBeginConditionalRender != nullptr &&
           pglEndConditionalRender != nullptr;
}

} // namespace

void MapRenderer::drawHumanTrafficActor(const std::filesystem::path& humanConfigPath,
                                        const openbus::map::MapTrafficPose& pose,
                                        double gaitPhaseRadians, double gaitWeight) const {
    const std::string cacheKey = openbus::pathToUtf8(humanConfigPath.lexically_normal());
    auto model = impl_->humanTrafficModels.find(cacheKey);
    if (model == impl_->humanTrafficModels.end()) {
        if (impl_->failedHumanTrafficModels.contains(cacheKey)) {
            return;
        }
        if (!impl_->pendingHumanTrafficModels.contains(cacheKey)) {
            throw std::logic_error("Human traffic model was not requested before drawing");
        }
        try {
            if (!finishHumanTrafficModelLoad(cacheKey)) {
                return;
            }
        } catch (const std::exception& error) {
            impl_->pendingHumanTrafficModels.erase(cacheKey);
            impl_->failedHumanTrafficModels.insert(cacheKey);
            gameLog.Warning("Could not asynchronously load human traffic model '" +
                            openbus::pathToUtf8(humanConfigPath) + "': " + error.what());
            return;
        }
        model = impl_->humanTrafficModels.find(cacheKey);
        if (model == impl_->humanTrafficModels.end()) {
            return;
        }
    }
    if (model->second.batchIndexes.size() != model->second.texturePaths.size() ||
        model->second.batchIndexes.size() != model->second.textureImages.size()) {
        throw std::logic_error("Human traffic model texture state is inconsistent");
    }
    for (std::size_t index = 0; index < model->second.batchIndexes.size(); ++index) {
        std::shared_future<std::shared_ptr<Image>>& imageRequest =
            model->second.textureImages[index];
        if (!imageRequest.valid() ||
            imageRequest.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            continue;
        }

        const std::filesystem::path& texturePath = model->second.texturePaths[index];
        const std::string textureKey = openbus::pathToUtf8(texturePath.lexically_normal());
        Impl::ModelBatch& batch = impl_->sceneryBatches[model->second.batchIndexes[index]];
        try {
            const auto existingTexture = impl_->dynamicSceneryTextures.find(textureKey);
            if (existingTexture != impl_->dynamicSceneryTextures.end()) {
                batch.material.texture = existingTexture->second;
            } else {
                const std::shared_ptr<Image> image = imageRequest.get();
                if (!image) {
                    throw std::runtime_error("Texture decoder returned no image");
                }
                const GLuint texture = uploadTextureImage(*image);
                const auto [insertedTexture, inserted] =
                    impl_->dynamicSceneryTextures.emplace(textureKey, texture);
                if (!inserted) {
                    glDeleteTextures(1, &texture);
                } else {
                    impl_->ownedTextures.push_back(texture);
                }
                batch.material.texture = insertedTexture->second;
            }
            batch.material.textured = batch.material.texture != 0;
        } catch (const std::exception& error) {
            gameLog.Warning("Could not asynchronously load human model texture " +
                            openbus::pathToUtf8(texturePath) + ": " + error.what());
        }
        imageRequest = {};
        impl_->humanTextureRequests.erase(textureKey);
    }
    const double headingDegrees = pose.headingRadians * 180.0 / std::acos(-1.0);
    const Matrix4 transform =
        makeSceneryInstanceTransform(pose.position[0], pose.position[1], pose.position[2],
                                     {headingDegrees, 0.0, 0.0, 0.0, 0.0, 0.0}, 1, nullptr);
    for (const std::size_t batchIndex : model->second.batchIndexes) {
        if (batchIndex >= impl_->sceneryBatches.size()) {
            throw std::logic_error("Human traffic model references an invalid mesh batch");
        }
    }

    pushMatrix();
    multiplyMatrix(transform);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    const auto drawBatches = [&](bool transparent) {
        for (const std::size_t batchIndex : model->second.batchIndexes) {
            const Impl::ModelBatch& batch = impl_->sceneryBatches[batchIndex];
            const bool isTransparent = batch.alphaMode == 2 || batch.alphaMode == 3;
            if (isTransparent != transparent) {
                continue;
            }
            drawModelBatch(batch.buffer, batch.vertexCount, batch.material, batch.color,
                           batch.alpha, batch.alphaMode, batch.cullBackFaces,
                           HumanGaitParameters{gaitPhaseRadians,
                                               gaitWeight,
                                               model->second.heightMeters,
                                               model->second.upperArmBetaDegrees,
                                               model->second.armSwing,
                                               model->second.hipSway,
                                               model->second.waistAnimation,
                                               model->second.shoulderXHeightFraction,
                                               model->second.shoulderZHeightFraction,
                                               model->second.hipXHeightFraction,
                                               model->second.hipZHeightFraction,
                                               model->second.kneeXHeightFraction,
                                               model->second.kneeYHeightFraction,
                                               model->second.kneeZHeightFraction,
                                               model->second.elbowXHeightFraction,
                                               model->second.elbowZHeightFraction,
                                               model->second.handXHeightFraction,
                                               model->second.handZHeightFraction,
                                               model->second.neckYHeightFraction,
                                               model->second.neckZHeightFraction,
                                               model->second.waistYHeightFraction,
                                               model->second.waistZHeightFraction,
                                               model->second.fingerXHeightFraction,
                                               model->second.fingerZHeightFraction});
        }
    };
    drawBatches(false);

    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    drawBatches(true);

    popMatrix();
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glDisable(GL_BLEND);
}

void MapRenderer::draw(bool renderTypeFourOnly, double minimumObjectViewportFraction) const {
    TraceScope trace("map", "MapRenderer::draw");
    IndirectCullingBatch& indirectBatch = impl_->indirectCullingBatch;
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
    const bool depthOcclusionQueries =
        depthOcclusionQueriesRequested() && depthOcclusionQueriesAvailable();
    const bool transparentOcclusionQueries =
        depthOcclusionQueries &&
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_OCCLUSION_TRANSPARENT"));
    const bool geometryOcclusionQueries =
        depthOcclusionQueries &&
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_OCCLUSION_MAP_GEOMETRY"));
    const bool waterOcclusionQueries =
        depthOcclusionQueries &&
        parseEnabledFlag(openbus::getEnvironment("OPENBUS_OCCLUSION_WATER"));
    impl_->nextOcclusionQuery = 0;
    if (depthOcclusionQueriesRequested() && !depthOcclusionQueriesAvailable() &&
        !impl_->occlusionQueryUnavailableLogged) {
        gameLog.Warning("Depth occlusion queries unavailable; continuing without query culling");
        impl_->occlusionQueryUnavailableLogged = true;
    }
    if (depthOcclusionQueries && !impl_->occlusionQueryEnabledLogged) {
        gameLog.Log("Depth occlusion queries enabled for opaque map scenery");
        impl_->occlusionQueryEnabledLogged = true;
    }
    if (transparentOcclusionQueries && !impl_->transparentOcclusionQueryEnabledLogged) {
        gameLog.Log("Depth occlusion queries enabled for ordinary transparent map scenery");
        impl_->transparentOcclusionQueryEnabledLogged = true;
    }
    if (geometryOcclusionQueries && !impl_->geometryOcclusionQueryEnabledLogged) {
        gameLog.Log("Depth occlusion queries enabled for bounded terrain and opaque roads");
        impl_->geometryOcclusionQueryEnabledLogged = true;
    }
    if (waterOcclusionQueries && !impl_->waterOcclusionQueryEnabledLogged) {
        gameLog.Log("Depth occlusion queries enabled for bounded water");
        impl_->waterOcclusionQueryEnabledLogged = true;
    }
    const std::vector<Impl::ActiveSceneryChunk>& activeSceneryChunks =
        impl_->activeSceneryChunks[renderTypeFourOnly ? 1 : 0];
    bool gpuCulling = false;
    if (indirectCullingAvailable()) {
        TraceScope phase("map", "MapRenderer::draw.computeCulling");
        auto& cullingItems = impl_->cullingItemsScratch;
        auto& lodBoundaries = impl_->cullingLodBoundariesScratch;
        auto& lodOffsets = impl_->cullingLodOffsetsScratch;
        const auto cullingPlanes = makeCullingPlanes(projectionMatrix());
        const auto projectedSize = makeCullingProjectedSize(
            projectionMatrix(), projectedSizeFadeStart(minimumObjectViewportFraction));
        cullingItems.clear();
        lodBoundaries.clear();
        lodOffsets.clear();
        if (++impl_->cullingGeneration == 0) {
            for (auto& tile : impl_->terrainBuffers)
                tile.cullingSlot = {};
            for (auto& water : impl_->waterBatches)
                water.cullingSlot = {};
            for (auto& road : impl_->roadBatches)
                road.cullingSlot = {};
            for (auto& [key, chunk] : impl_->sceneryChunks) {
                static_cast<void>(key);
                for (auto& instance : chunk.instances)
                    instance.cullingSlot = {};
            }
            impl_->cullingGeneration = 1;
        }
        for (const auto& thresholds : impl_->sceneryLodThresholdSets) {
            lodOffsets.push_back(appendCullingLodBoundaries(lodBoundaries, thresholds));
        }
        const auto addSphere = [&](const auto* key, CullingItem item, double minX, double minY,
                                   double maxX, double maxY, std::size_t count) {
            key->cullingSlot = {cullingItems.size(), impl_->cullingGeneration};
            item.draw[0] = count <= static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())
                               ? static_cast<std::uint32_t>(count)
                               : std::numeric_limits<std::uint32_t>::max();
            item.metadata[0] |= CullMapDistance;
            item.distanceBounds = {
                static_cast<float>(minX - cameraX), static_cast<float>(minY - cameraY),
                static_cast<float>(maxX - cameraX), static_cast<float>(maxY - cameraY)};
            cullingItems.push_back(item);
        };
        const auto addBounds = [&](const auto* key, double minX, double minY, double minZ,
                                   double maxX, double maxY, double maxZ, std::size_t count) {
            const auto item = makeCullingItem(
                view, {(minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5},
                std::hypot(maxX - minX, maxY - minY, maxZ - minZ) * 0.5);
            if (!cullingFrustumMayIntersect(item, cullingPlanes))
                return;
            addSphere(key, item, minX, minY, maxX, maxY, count);
        };
        const auto addTile = [&](const auto& tile) {
            if (!impl_->activeTileKeys.contains(tileKey(tile.tileX, tile.tileY)))
                return;
            const double x = tile.tileX * impl_->coordinates.tileSize;
            const double y = tile.tileY * impl_->coordinates.tileSize;
            addBounds(&tile, x, y, tile.minZ, x + impl_->coordinates.tileSize,
                      y + impl_->coordinates.tileSize, tile.maxZ, tile.indexCount);
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
            if (squaredDistanceToBounds(cameraX, cameraY, chunk.minX, chunk.minY, chunk.maxX,
                                        chunk.maxY) > MAP_VISIBILITY_RADIUS_SQUARED ||
                outsideMapFrustum(frustum, view,
                                  {(chunk.minX + chunk.maxX) * 0.5, (chunk.minY + chunk.maxY) * 0.5,
                                   (chunk.minZ + chunk.maxZ) * 0.5},
                                  std::hypot(chunk.maxX - chunk.minX, chunk.maxY - chunk.minY,
                                             chunk.maxZ - chunk.minZ) *
                                      0.5))
                continue;
            for (const std::size_t instanceIndex : active.instanceIndexes) {
                const Impl::SceneryInstance& instance = active.chunk->instances[instanceIndex];
                const auto& batch = impl_->sceneryBatches[instance.batchIndex];
                auto item = makeCullingItem(view, {instance.x, instance.y, instance.z},
                                            instance.boundingRadius);
                if (!cullingFrustumMayIntersect(item, cullingPlanes))
                    continue;
                item.metadata[0] |= CullLod | CullProjectedSize;
                item.metadata[1] = static_cast<std::uint32_t>(batch.sceneryLodIndex);
                if (instance.lodThresholdSetIndex < lodOffsets.size()) {
                    item.metadata[2] = lodOffsets[instance.lodThresholdSetIndex];
                    item.metadata[3] = static_cast<std::uint32_t>(
                        impl_->sceneryLodThresholdSets[instance.lodThresholdSetIndex].size());
                }
                // Map assets without thresholds still select LOD zero.
                if (item.metadata[3] == 0 && batch.sceneryLodIndex != 0) {
                    continue;
                }
                if (!cullingLodMayMatch(item, lodBoundaries) ||
                    !cullingProjectedSizeMayPass(item, projectedSize))
                    continue;
                addSphere(&instance, item, instance.x - instance.boundingRadius,
                          instance.y - instance.boundingRadius,
                          instance.x + instance.boundingRadius,
                          instance.y + instance.boundingRadius, batch.vertexCount);
            }
        }
        indirect = indirectBatch.dispatch(cullingItems, cullingPlanes, lodBoundaries, projectedSize,
                                          depthOcclusionQueries);
        gpuCulling = indirect;
    }
    const auto gpuVisible = [&](const auto* key) {
        return key->cullingSlot.generation == impl_->cullingGeneration;
    };
    const auto selectIndirect = [&](const auto* key) {
        if (!indirect)
            return;
        indirectBatch.select(gpuVisible(key) ? key->cullingSlot.index
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

    struct OcclusionPassState {
        std::array<GLboolean, 4> colorMask = {GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE};
        GLenum depthFunc = GL_LESS;
        GLboolean depthMask = GL_TRUE;
        bool cullFace = false;
        bool captured = false;
    };
    const auto beginOcclusionProxy = [&](const std::array<double, 3>& center, double radius,
                                         OcclusionPassState* state) {
        if (state == nullptr || !std::isfinite(radius) || radius <= 0.0)
            return false;
        const double proxyRadius = radius * 1.001 + 0.001;
        if (!boxFullyBeyondNearPlane(frustum, view, center, proxyRadius))
            return false;
        if (impl_->nextOcclusionQuery == impl_->occlusionQueries.size()) {
            GLuint newQuery = 0;
            pglGenQueries(1, &newQuery);
            if (newQuery != 0) {
                impl_->occlusionQueries.push_back(newQuery);
            } else if (!impl_->occlusionQueryAllocationLogged) {
                gameLog.Warning("Could not allocate a depth occlusion query; "
                                "rendering candidates conservatively");
                impl_->occlusionQueryAllocationLogged = true;
            }
        }
        if (impl_->nextOcclusionQuery >= impl_->occlusionQueries.size())
            return false;
        const GLuint query = impl_->occlusionQueries[impl_->nextOcclusionQuery++];
        if (!state->captured) {
            glGetBooleanv(GL_COLOR_WRITEMASK, state->colorMask.data());
            state->cullFace = glIsEnabled(GL_CULL_FACE) == GL_TRUE;
            state->captured = true;
        }
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        if (state->cullFace)
            glDisable(GL_CULL_FACE);
        pglBeginQuery(anySamplesPassed, query);
        pushMatrix();
        translate(center[0], center[1], center[2]);
        scale(proxyRadius * 2.0, proxyRadius * 2.0, proxyRadius * 2.0);
        const bool proxySubmitted = drawStaticPrimitives(
            impl_->occlusionProxyBuffer, occlusionProxyCube(), GL_TRIANGLES, 1.0F, 0, indirect);
        popMatrix();
        pglEndQuery(anySamplesPassed);
        if (proxySubmitted) {
            pglBeginConditionalRender(query, queryWait);
        } else if (!impl_->occlusionProxyFailureLogged) {
            gameLog.Warning(
                "Could not submit an occlusion proxy; rendering candidate conservatively");
            impl_->occlusionProxyFailureLogged = true;
        }
        glColorMask(state->colorMask[0], state->colorMask[1], state->colorMask[2],
                    state->colorMask[3]);
        glDepthMask(state->depthMask);
        glDepthFunc(state->depthFunc);
        if (state->cullFace)
            glEnable(GL_CULL_FACE);
        return proxySubmitted;
    };
    const auto beginBoundsOcclusion = [&](double minX, double minY, double minZ, double maxX,
                                          double maxY, double maxZ, OcclusionPassState* state) {
        if (state == nullptr)
            return false;
        const std::array<double, 3> minimum = {minX, minY, minZ};
        const std::array<double, 3> maximum = {maxX, maxY, maxZ};
        std::array<double, 3> center = {};
        double radius = 0.0;
        double magnitude = 1.0;
        for (std::size_t axis = 0; axis < center.size(); ++axis) {
            if (!std::isfinite(minimum[axis]) || !std::isfinite(maximum[axis]) ||
                minimum[axis] > maximum[axis])
                return false;
            center[axis] = minimum[axis] + (maximum[axis] - minimum[axis]) * 0.5;
            radius = std::max(radius, (maximum[axis] - minimum[axis]) * 0.5);
            magnitude = std::max({magnitude, std::abs(minimum[axis]), std::abs(maximum[axis])});
        }
        radius += magnitude * std::numeric_limits<float>::epsilon() * 4.0;
        return beginOcclusionProxy(center, radius, state);
    };

    const auto drawTerrain = [&]() {
        TraceScope phase("map", "MapRenderer::draw.terrain");
        OcclusionPassState occlusionState;
        ModelMaterial material;
        material.texture = impl_->texture;
        material.textured = true;
        material.detailTexture = impl_->baseDetailTexture;
        material.useDetailTexture = impl_->baseDetailTexture != 0;
        for (const Impl::TileBuffer& tile : impl_->terrainBuffers) {
            if (!impl_->activeTileKeys.contains(tileKey(tile.tileX, tile.tileY))) {
                continue;
            }
            const double minX = static_cast<double>(tile.tileX) * impl_->coordinates.tileSize;
            const double minY = static_cast<double>(tile.tileY) * impl_->coordinates.tileSize;
            if (gpuCulling && !gpuVisible(&tile))
                continue;
            if (!gpuCulling &&
                outsideVisibilityBounds(minX, minY, minX + impl_->coordinates.tileSize,
                                        minY + impl_->coordinates.tileSize)) {
                continue;
            }
            if (!gpuCulling &&
                outsideFrustumBounds(minX, minY, tile.minZ, minX + impl_->coordinates.tileSize,
                                     minY + impl_->coordinates.tileSize, tile.maxZ)) {
                continue;
            }
            material.textureScale = tile.baseTextureScale;
            material.detailTextureScale = tile.baseDetailTextureScale;
            selectIndirect(&tile);
            const bool conditionalRender =
                beginBoundsOcclusion(minX, minY, tile.minZ, minX + impl_->coordinates.tileSize,
                                     minY + impl_->coordinates.tileSize, tile.maxZ,
                                     geometryOcclusionQueries ? &occlusionState : nullptr);
            pushMatrix();
            multiplyMatrix(translationMatrix({minX, minY, 0.0}));
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
            popMatrix();
            if (conditionalRender)
                pglEndConditionalRender();
        }
    };
    const auto drawWater = [&]() {
        TraceScope phase("map", "MapRenderer::draw.water");
        std::vector<std::pair<const Impl::WaterBatch*, double>> visibleWater;
        for (const Impl::WaterBatch& water : impl_->waterBatches) {
            if (!impl_->activeTileKeys.contains(tileKey(water.tileX, water.tileY))) {
                continue;
            }
            const double minX = static_cast<double>(water.tileX) * impl_->coordinates.tileSize;
            const double minY = static_cast<double>(water.tileY) * impl_->coordinates.tileSize;
            const double maxX = minX + impl_->coordinates.tileSize;
            const double maxY = minY + impl_->coordinates.tileSize;
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
        OcclusionPassState occlusionState;
        occlusionState.depthFunc = GL_LEQUAL;
        occlusionState.depthMask = GL_FALSE;
        for (const auto& [water, depth] : visibleWater) {
            static_cast<void>(depth);
            selectIndirect(water);
            const double minX = static_cast<double>(water->tileX) * impl_->coordinates.tileSize;
            const double minY = static_cast<double>(water->tileY) * impl_->coordinates.tileSize;
            const bool conditionalRender =
                beginBoundsOcclusion(minX, minY, water->minZ, minX + impl_->coordinates.tileSize,
                                     minY + impl_->coordinates.tileSize, water->maxZ,
                                     waterOcclusionQueries ? &occlusionState : nullptr);
            pushMatrix();
            multiplyMatrix(translationMatrix({minX, minY, 0.0}));
            drawIndexedModelBatch(water->buffer, water->indexBuffer, water->indexCount, material,
                                  waterColor, 0.62, 0);
            popMatrix();
            if (conditionalRender)
                pglEndConditionalRender();
        }
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);
    };
    const auto drawSplines = [&]() {
        TraceScope phase("map", "MapRenderer::draw.splines");
        OcclusionPassState occlusionState;
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
            const bool conditionalRender =
                road.hasVisibilityBounds && road.alphaMode == 0 && !road.noZcheck &&
                !road.noZwrite &&
                beginBoundsOcclusion(road.minX, road.minY, road.minZ, road.maxX, road.maxY,
                                     road.maxZ,
                                     geometryOcclusionQueries ? &occlusionState : nullptr);
            pushMatrix();
            multiplyMatrix(translationMatrix(
                {static_cast<double>(road.tileX) * impl_->coordinates.tileSize,
                 static_cast<double>(road.tileY) * impl_->coordinates.tileSize, 0.0}));
            drawIndexedModelBatch(road.buffer, road.indexBuffer, road.indexCount, road.material,
                                  road.color, 1.0, road.alphaMode, road.cullBackFaces);
            popMatrix();
            if (conditionalRender)
                pglEndConditionalRender();
        }
    };
    std::vector<Impl::SceneryDraw>& sceneryDraws = impl_->sceneryDrawScratch;
    sceneryDraws.clear();
    if (impl_->resolvedFreeTexturesByBatch.size() < impl_->sceneryBatches.size()) {
        impl_->resolvedFreeTexturesByBatch.resize(impl_->sceneryBatches.size());
    }
    const auto drawSceneryInstance = [&](const Impl::SceneryInstance& instance,
                                         const Impl::ModelBatch& batch,
                                         const ModelMaterial& sceneryMaterial, int alphaMode,
                                         OcclusionPassState* occlusionState = nullptr,
                                         double visibilityFade = 1.0, double alphaScale = 1.0) {
        selectIndirect(&instance);
        const bool conditionalRender =
            !instance.poleCrash && beginOcclusionProxy({instance.x, instance.y, instance.z},
                                                       instance.boundingRadius, occlusionState);
        pushMatrix();
        multiplyMatrix(instance.transform);
        drawModelBatch(batch.buffer, batch.vertexCount, sceneryMaterial, batch.color,
                       batch.alpha * alphaScale, alphaMode, batch.cullBackFaces, std::nullopt,
                       visibilityFade);
        popMatrix();
        if (conditionalRender) {
            pglEndConditionalRender();
        }
    };
    // Returns the texture to apply, or nullopt when the batch material is left unchanged.
    const auto resolveFreeTexture =
        [&](const Impl::ModelBatch& batch,
            const std::string& selectedTexture) -> std::optional<GLuint> {
        const std::filesystem::path texturePath = findSceneryTexture(
            batch.textureObjectRoot, batch.textureModelRoot, impl_->omsiRoot, selectedTexture);
        const std::string textureKey =
            texturePath.empty() ? lowercasePathComponent(openbus::pathToUtf8(
                                      normalizedAssetPath(selectedTexture).lexically_normal()))
                                : openbus::pathToUtf8(texturePath.lexically_normal());
        if (impl_->unavailableSceneryTextures.contains(textureKey)) {
            return std::nullopt;
        }
        if (texturePath.empty()) {
            impl_->unavailableSceneryTextures.insert(textureKey);
            gameLog.Warning("Map scenery free texture not found: " + selectedTexture);
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
                gameLog.Warning("Could not load map scenery free texture " +
                                openbus::pathToUtf8(texturePath) + ": " + error.what());
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
            for (const std::size_t instanceIndex : active.instanceIndexes) {
                const Impl::SceneryInstance& instance = active.chunk->instances[instanceIndex];
                if (gpuCulling
                        ? !gpuVisible(&instance)
                        : (outsideVisibilityBounds(instance.x - instance.boundingRadius,
                                                   instance.y - instance.boundingRadius,
                                                   instance.x + instance.boundingRadius,
                                                   instance.y + instance.boundingRadius) ||
                           sphereBelowMinimumViewportFraction(
                               view, projectionMatrix(), {instance.x, instance.y, instance.z},
                               instance.boundingRadius,
                               projectedSizeFadeStart(minimumObjectViewportFraction)) ||
                           outsideMapFrustum(frustum, view, {instance.x, instance.y, instance.z},
                                             instance.boundingRadius))) {
                    continue;
                }
                const Impl::ModelBatch& batch = impl_->sceneryBatches[instance.batchIndex];
                const double visibilityFade = projectedSizeFadeAlpha(
                    view, projectionMatrix(), {instance.x, instance.y, instance.z},
                    instance.boundingRadius, minimumObjectViewportFraction);
                if (visibilityFade <= 0.0) {
                    continue;
                }
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
                double alphaScale = batch.alphaScaleVariable.empty() ? 1.0 : 0.0;
                if (instance.scriptStateIndex < impl_->sceneryScriptStates.size()) {
                    Impl::SceneryScriptState& scriptState =
                        *impl_->sceneryScriptStates[instance.scriptStateIndex];
                    bool selectedItem = false;
                    if (batch.nightmapSelection) {
                        const ModelMaterialState& selected = selectModelMaterial(
                            *batch.nightmapSelection, [&](const std::string& name) {
                                return scriptState.variables.get(name);
                            });
                        selectedItem = &selected != &*batch.nightmapSelection;
                        if (selectedItem) {
                            const auto texture =
                                batch.itemNightmaps.find(selected.nightmapTextureName);
                            sceneryMaterial.nightmap =
                                texture == batch.itemNightmaps.end() ? 0 : texture->second;
                            sceneryMaterial.useNightmap = sceneryMaterial.nightmap != 0;
                        }
                    }
                    sceneryMaterial.nightmapStrength =
                        selectedItem || !batch.visibilityVariable.empty()
                            ? 1.0F
                            : std::clamp(scriptState.variables.get("nightlighta"), 0.0F, 1.0F);
                    if (!batch.texcoordTransXVariable.empty()) {
                        sceneryMaterial.texcoordOffsetX = static_cast<float>(
                            scriptState.variables.get(batch.texcoordTransXVariable));
                    }
                    if (!batch.texcoordTransYVariable.empty()) {
                        sceneryMaterial.texcoordOffsetY = static_cast<float>(
                            scriptState.variables.get(batch.texcoordTransYVariable));
                    }
                    if (!batch.lightmapStrengthVariable.empty()) {
                        sceneryMaterial.lightmapStrength = std::clamp(
                            scriptState.variables.get(batch.lightmapStrengthVariable), 0.0F, 1.0F);
                    }
                    if (!batch.alphaScaleVariable.empty()) {
                        alphaScale = std::clamp(scriptState.variables.get(batch.alphaScaleVariable),
                                                0.0F, 1.0F);
                    }
                    if (batch.textTextureIndex >= 0) {
                        sceneryMaterial.textTexture = refreshSceneryTexture(
                            scriptState, true, batch.textTextureIndex, sceneryMaterial);
                        sceneryMaterial.useTextTexture = sceneryMaterial.textTexture != 0;
                        if (sceneryMaterial.useTextTexture) {
                            // Text glyphs include partial alpha for antialiasing and a
                            // transparent background, so draw them as a blended overlay.
                            alphaMode = 2;
                        }
                        if (alphaScale <= 0.0) {
                            continue;
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
                if (visibilityFade < 1.0) {
                    alphaMode = 2;
                }
                const double viewDepth = -(view[2] * instance.x + view[6] * instance.y +
                                           view[10] * instance.z + view[14]);
                sceneryDraws.push_back({&instance, &batch, std::move(sceneryMaterial), alphaMode,
                                        viewDepth, visibilityFade, alphaScale});
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
        std::vector<const SceneryDraw*>& opaqueScenery = impl_->opaqueSceneryScratch[group];
        std::vector<const SceneryDraw*>& transparentScenery =
            impl_->transparentSceneryScratch[group];
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        OcclusionPassState occlusionState;
        if (depthOcclusionQueries) {
            const auto reorderable = [](const SceneryDraw* scenery) {
                return scenery->alphaMode == 0 && !scenery->batch->noZwrite &&
                       !scenery->batch->noZcheck;
            };
            for (auto begin = opaqueScenery.begin(); begin != opaqueScenery.end();) {
                if (!reorderable(*begin)) {
                    ++begin;
                    continue;
                }
                const auto end = std::find_if_not(begin, opaqueScenery.end(), reorderable);
                std::stable_sort(begin, end,
                                 [](const SceneryDraw* first, const SceneryDraw* second) {
                                     return first->viewDepth < second->viewDepth;
                                 });
                begin = end;
            }
        }
        for (const SceneryDraw* scenery : opaqueScenery) {
            glDepthMask(scenery->batch->noZwrite ? GL_FALSE : GL_TRUE);
            // OMSI shadow overlays commonly request both flags: they must not
            // write depth, but still need depth testing so they cannot bleed
            // through foreground scenery or vehicles.
            const bool skipDepthCheck =
                scenery->batch->noZcheck && !scenery->batch->noZwrite && scenery->alphaMode == 0;
            if (skipDepthCheck) {
                glDisable(GL_DEPTH_TEST);
            }
            drawSceneryInstance(*scenery->instance, *scenery->batch, scenery->material,
                                scenery->alphaMode,
                                depthOcclusionQueries && scenery->alphaMode == 0 &&
                                        !scenery->batch->noZwrite && !scenery->batch->noZcheck
                                    ? &occlusionState
                                    : nullptr,
                                scenery->visibilityFade, scenery->alphaScale);
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
                       !scenery->material.useTextTexture && !scenery->batch->noZwrite;
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
                    scenery->material.useTextTexture || scenery->batch->noZwrite) {
                    continue;
                }
                // The shader's alpha-mode-3 threshold keeps only fully opaque
                // transmap texels. Record those surfaces in depth before the
                // blended pass, matching the vehicle renderer's treatment.
                drawSceneryInstance(*scenery->instance, *scenery->batch, scenery->material, 3,
                                    nullptr, scenery->visibilityFade, scenery->alphaScale);
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
                !scenery->material.useTextTexture && !scenery->batch->noZwrite;
            occlusionState.depthFunc = hasOpaqueTransmapDepth ? GL_LEQUAL : GL_LESS;
            occlusionState.depthMask =
                scenery->material.useTextTexture && !scenery->batch->noZwrite ? GL_TRUE : GL_FALSE;
            glDepthFunc(occlusionState.depthFunc);
            glDepthMask(occlusionState.depthMask);
            // Transparent scenery such as windows must remain occluded by solid geometry,
            // even when an asset requests noZcheck. Keep noZwrite semantics for blending.
            drawSceneryInstance(*scenery->instance, *scenery->batch, scenery->material,
                                scenery->alphaMode,
                                transparentOcclusionQueries && !scenery->batch->noZwrite &&
                                        !scenery->batch->noZcheck
                                    ? &occlusionState
                                    : nullptr,
                                scenery->visibilityFade, scenery->alphaScale);
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
