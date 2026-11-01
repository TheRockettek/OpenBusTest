// Opaque/transparent batch submission for vehicle meshes.

#include "app/vehicle/Vehicle.h"
#include "app/vehicle/VehicleDrawSorting.h"

#include <algorithm>
#include <map>
#include <numbers>
#include <tuple>

namespace {

Vehicle::DrawProfileStats vehicleDrawProfileStats;
using TransparentSortProfileKey = std::tuple<std::string, std::string, std::size_t, std::size_t>;
std::map<TransparentSortProfileKey, Vehicle::DrawProfileStats::TransparentSortBatch>
    transparentSortBatchProfileStats;

} // namespace

void Vehicle::setOneSidedFrontFaceConvention() {
    glFrontFace(GL_CW);
    // Face selection is conditional in the model shader so generated and
    // imported geometry can use different sidedness per draw.
    glDisable(GL_CULL_FACE);
}

void Vehicle::drawBatch(Batch& batch, double alpha, bool forceUntextured,
                        const std::array<double, 3>* overrideColor, int alphaModeOverride,
                        bool forceDepthTest, GLuint transparentIndexBuffer, double visibilityFade,
                        bool forceNoDepthWrite) {
    if (alpha <= 0.0) {
        return;
    }
    const int alphaMode = alphaModeOverride >= 0 ? alphaModeOverride : batch.alphaMode;
    const bool noZwrite = forceNoDepthWrite || (alphaModeOverride >= 0 ? false : batch.noZwrite);
    openbus::rendering::configureModelAlphaState(alphaMode, noZwrite);
    // Ground-projected fake shadows must still be occluded by vehicle
    // geometry; keep their noZwrite behavior but honor the depth test.
    const bool noZcheck = batch.noZcheck && !forceDepthTest;
    if (noZcheck) {
        glDisable(GL_DEPTH_TEST);
    } else if (forceDepthTest) {
        glEnable(GL_DEPTH_TEST);
    }
    const bool materialBatch = !batch.materialTextures.empty() && !forceUntextured;
    const std::vector<GLuint>* materialTextureIds = nullptr;
    {
        TraceScope phase("render", "Vehicle::drawBatch.resolveTextures");
        if (materialBatch) {
            batch.materialTextureIds.resize(batch.materialTextures.size());
            batch.materialTextureFlips.resize(batch.materialTextures.size());
            std::fill(batch.materialTextureFlips.begin(), batch.materialTextureFlips.end(), false);
            for (std::size_t sourceIndex = 0; sourceIndex < batch.materialTextures.size();
                 ++sourceIndex) {
                MaterialTextureSource& source = batch.materialTextures[sourceIndex];
                ensureMaterialTexture(source);
                if (!openbus::rendering::reflectionPassActive()) {
                    const int reflectionIndex =
                        openbus::rendering::reflectionTextureIndex(source.textureName);
                    const unsigned int reflectionTexture =
                        openbus::rendering::reflectionTextureForIndex(reflectionIndex);
                    if (reflectionTexture != 0) {
                        source.texture = reflectionTexture;
                        source.textureArray = false;
                        batch.materialTextureFlips[sourceIndex] = true;
                    }
                }
                batch.materialTextureIds[sourceIndex] = source.texture;
            }
            materialTextureIds = &batch.materialTextureIds;
        } else {
            ensureTexture(batch);
        }
        ensureAuxiliaryTexture(batch, batch.lightmap);
        ensureAuxiliaryTexture(batch, batch.nightmap);
        ensureAuxiliaryTexture(batch, batch.transmap);
        ensureAuxiliaryTexture(batch, batch.bumpmap);
        updateFreeTexture(batch);
    }
    const std::array<double, 3>& color = overrideColor == nullptr ? batch.color : *overrideColor;
    TraceScope phase("render", "Vehicle::drawBatch.prepareAndSubmit");
    openbus::rendering::ModelMaterial material;
    for (std::size_t index = 0; index < 3; ++index) {
        material.specularColor[index] = static_cast<float>(batch.specularColor[index]);
        material.emissionColor[index] = static_cast<float>(batch.emissionColor[index]);
    }
    material.specularPower = static_cast<float>(batch.specularPower);
    material.texture = materialBatch ? 0 : batch.texture;
    material.textureArray = materialBatch ? false : batch.textureArray;
    material.textured = materialBatch ? false : batch.textured && !forceUntextured;
    material.textureAlphaOnly = batch.textureAlphaOnly;
    material.textureWrapS = batch.textureWrapS;
    material.textureWrapT = batch.textureWrapT;
    material.lightmap = batch.lightmap.texture;
    material.nightmap = batch.nightmap.texture;
    material.transmap = batch.transmap.texture;
    material.bumpmap = batch.bumpmap.texture;
    material.freeTexture = batch.freeTexture;
    // Text textures use their own shader sampler. They share the dynamic
    // upload storage with script/free textures, but must not be left only
    // in the free-texture sampler or the shader will never display them.
    material.textTexture = batch.textTextureIndex >= 0 ? batch.freeTexture : 0;
    material.useLightmap = !forceUntextured && material.lightmap != 0;
    material.useNightmap = !forceUntextured && material.nightmap != 0;
    material.useTransmap = !forceUntextured && material.transmap != 0;
    material.useBumpmap = !forceUntextured && material.bumpmap != 0;
    material.useFreeTexture =
        !forceUntextured && batch.textTextureIndex < 0 && material.freeTexture != 0;
    material.useTextTexture =
        !forceUntextured && batch.textTextureIndex >= 0 && material.textTexture != 0;
    if (!forceUntextured) {
        applyInteriorLightMaterial(batch, material);
    }
    material.lightmapStrength = static_cast<float>(
        batch.lightmapStrengthVariable.empty()
            ? 1.0F
            : std::clamp(variables.get(batch.lightmapStrengthVariable), 0.0F, 1.0F));
    // Script-selected emissive items and [visible]-switched lamp meshes illuminate
    // even in daytime. Ordinary base nightmaps still follow environmental lighting.
    const float nightlight =
        batch.selectedMaterialItem || batch.visibilitySwitched
            ? 1.0F
            : std::max(variables.get("nightlighta"), 1.0F - variables.get("envir_brightness"));
    material.nightmapStrength = std::clamp(nightlight, 0.0F, 1.0F);
    material.bumpmapStrength = static_cast<float>(std::clamp(batch.bumpmapStrength, 0.0, 1.0));
    material.texcoordOffsetX = static_cast<float>(
        batch.texcoordTransXVariable.empty() ? 0.0 : variables.get(batch.texcoordTransXVariable));
    material.texcoordOffsetY = static_cast<float>(
        batch.texcoordTransYVariable.empty() ? 0.0 : variables.get(batch.texcoordTransYVariable));
    if (!openbus::rendering::reflectionPassActive() && !materialBatch) {
        const int reflectionIndex = openbus::rendering::reflectionTextureIndex(
            batch.textureName.empty() ? batch.texturePath.string() : batch.textureName);
        const unsigned int reflectionTexture =
            openbus::rendering::reflectionTextureForIndex(reflectionIndex);
        if (reflectionTexture != 0) {
            material.texture = reflectionTexture;
            material.textureArray = false;
            material.textured = true;
            material.flipTextureY = true;
        }
    }
    if (materialBatch) {
        drawMaterialBatch(batch.buffer, batch.vertexCount, batch.materialColors,
                          *materialTextureIds, batch.materialTextureFlips, transparentIndexBuffer,
                          visibilityFade);
    } else if (transparentIndexBuffer != 0) {
        openbus::rendering::drawIndexedModelBatch(
            batch.buffer, transparentIndexBuffer, batch.vertexCount, material, color, alpha,
            forceUntextured ? 0 : alphaMode, true, visibilityFade);
    } else {
        drawModelBatch(batch.buffer, batch.vertexCount, material, color, alpha,
                       forceUntextured ? 0 : alphaMode, true, std::nullopt, visibilityFade);
    }
    if (noZcheck) {
        glEnable(GL_DEPTH_TEST);
    }
}

GLuint Vehicle::sortTransparentBatch(Batch& batch, const DisplayPart& part,
                                     std::size_t batchIndex) {
    const auto& modelView = openbus::rendering::modelViewMatrix();
    const std::array<double, 3> depthAxis =
        openbus::app::vehicle::transparentTriangleDepthAxis(modelView);
    const std::vector<Vertex>* const source = batch.vertices.values.get();
    for (const Batch::TransparentSortCacheEntry& entry : batch.transparentSortCache) {
        if (entry.valid && entry.source == source && entry.vertexCount == batch.vertices.size() &&
            entry.depthAxis == depthAxis) {
            if (openbus::rendering::IsRecording()) {
                ++vehicleDrawProfileStats.transparentSortCacheHits;
            }
            TraceScope cacheHit("render", "Vehicle::draw.transparentSortCacheHit");
            return entry.indexBuffer;
        }
    }
    auto profileEntry = transparentSortBatchProfileStats.end();
    if (openbus::rendering::IsRecording()) {
        ++vehicleDrawProfileStats.transparentSortCacheMisses;
        const TransparentSortProfileKey key{part.meshName, part.meshIdentifier, batchIndex,
                                            batch.vertices.size()};
        profileEntry = transparentSortBatchProfileStats.try_emplace(key).first;
        auto& batchStats = profileEntry->second;
        batchStats.meshName = part.meshName;
        batchStats.meshIdentifier = part.meshIdentifier;
        batchStats.batchIndex = batchIndex;
        batchStats.vertexCount = batch.vertices.size();
        ++batchStats.misses;
    }
    TraceScope sortTrace("render", "Vehicle::draw.transparentTriangleSort");
    static thread_local std::vector<openbus::app::vehicle::TransparentTriangleDepth>
        triangleDepthScratch;
    static thread_local std::vector<std::size_t> triangleOrderScratch;
    static thread_local std::vector<GLuint> sortedIndices;
    {
        TraceScope phase("render", "Vehicle::draw.transparentTriangleSort.depthOrder");
        openbus::app::vehicle::transparentTriangleOrder(batch.vertices, modelView,
                                                        triangleDepthScratch, triangleOrderScratch);
    }
    const std::vector<std::size_t>& triangleOrder = triangleOrderScratch;
    if (triangleOrder.empty() ||
        batch.vertices.size() > static_cast<std::size_t>(std::numeric_limits<GLuint>::max())) {
        return 0;
    }
    sortedIndices.resize(batch.vertices.size());
    {
        TraceScope phase("render", "Vehicle::draw.transparentTriangleSort.buildIndices");
        std::size_t sortedIndex = 0;
        for (const std::size_t index : triangleOrder) {
            sortedIndices[sortedIndex++] = static_cast<GLuint>(index);
            sortedIndices[sortedIndex++] = static_cast<GLuint>(index + 1);
            sortedIndices[sortedIndex++] = static_cast<GLuint>(index + 2);
        }
    }
    if (openbus::rendering::IsRecording()) {
        vehicleDrawProfileStats.transparentTrianglesSorted += triangleOrder.size();
        if (profileEntry != transparentSortBatchProfileStats.end()) {
            profileEntry->second.trianglesSorted += triangleOrder.size();
        }
    }
    sortTrace.finish();
    std::size_t cacheSlot = batch.transparentSortCache.size();
    for (std::size_t index = 0; index < batch.transparentSortCache.size(); ++index) {
        if (!batch.transparentSortCache[index].valid) {
            cacheSlot = index;
            break;
        }
    }
    if (cacheSlot == batch.transparentSortCache.size()) {
        cacheSlot = batch.transparentSortCacheReplacementIndex;
        batch.transparentSortCacheReplacementIndex =
            (batch.transparentSortCacheReplacementIndex + 1) % batch.transparentSortCache.size();
    }
    Batch::TransparentSortCacheEntry& entry = batch.transparentSortCache[cacheSlot];
    GLuint indexBuffer = entry.indexBuffer;
    if (indexBuffer == 0) {
        pglGenBuffers(1, &indexBuffer);
        if (indexBuffer == 0) {
            return 0;
        }
        assets->trackBuffer(indexBuffer);
    }
    TraceScope uploadTrace("render", "Vehicle::draw.transparentIndexUpload");
    pglBindBuffer(GL_ARRAY_BUFFER, indexBuffer);
    pglBufferData(GL_ARRAY_BUFFER,
                  static_cast<std::ptrdiff_t>(sortedIndices.size() * sizeof(GLuint)),
                  sortedIndices.data(), GL_STATIC_DRAW);
    entry.depthAxis = depthAxis;
    entry.source = source;
    entry.vertexCount = batch.vertices.size();
    entry.indexBuffer = indexBuffer;
    entry.valid = true;
    return indexBuffer;
}

Vehicle::DrawProfileStats Vehicle::consumeDrawProfileStats() {
    DrawProfileStats result = vehicleDrawProfileStats;
    result.topTransparentSortBatches.reserve(transparentSortBatchProfileStats.size());
    for (const auto& [key, stats] : transparentSortBatchProfileStats) {
        static_cast<void>(key);
        result.topTransparentSortBatches.push_back(stats);
    }
    std::sort(result.topTransparentSortBatches.begin(), result.topTransparentSortBatches.end(),
              [](const DrawProfileStats::TransparentSortBatch& first,
                 const DrawProfileStats::TransparentSortBatch& second) {
                  if (first.trianglesSorted != second.trianglesSorted) {
                      return first.trianglesSorted > second.trianglesSorted;
                  }
                  return first.misses > second.misses;
              });
    constexpr std::size_t maxLoggedBatches = 5;
    if (result.topTransparentSortBatches.size() > maxLoggedBatches) {
        result.topTransparentSortBatches.resize(maxLoggedBatches);
    }
    vehicleDrawProfileStats = {};
    transparentSortBatchProfileStats.clear();
    return result;
}

void Vehicle::draw(RenderViewContext context, VehicleRenderPass renderPass,
                   const Matrix4* groundShadowTransform, double minimumObjectViewportFraction) {
    TraceScope trace("render", "Vehicle::draw");
    openbus::rendering::IndirectCullingScope indirectScope;
    bool indirect = false;
    if (!loaded) {
        return;
    }
    const bool reflectionPass = openbus::rendering::reflectionPassActive();
    const bool renderOpaque = renderPass != VehicleRenderPass::Transparent;
    const bool classifyTransparent =
        !reflectionPass || openbus::rendering::reflectionTransparentEnabled();
    const bool renderTransparent = renderPass != VehicleRenderPass::Opaque && classifyTransparent;
    const double cullingMinimumFraction =
        reflectionPass && !classifyTransparent
            ? minimumObjectViewportFraction
            : openbus::rendering::projectedSizeFadeStart(minimumObjectViewportFraction);
    if (!reflectionPass && renderPass != VehicleRenderPass::Transparent) {
        updateAnimationStates();
    }
    // Texture uploads must happen on the OpenGL thread, so decoding and GL
    // upload are deliberately split between the worker and draw paths.
    textureUploadStart = std::chrono::steady_clock::now();

    const auto& modelView = openbus::rendering::modelViewMatrix();
    const auto& projection = openbus::rendering::projectionMatrix();
    const bool reusePreparedDraw =
        renderPass == VehicleRenderPass::Transparent && preparedDrawListsValid &&
        preparedDrawContext == context && preparedDrawReflection == reflectionPass &&
        preparedDrawTransparent == classifyTransparent &&
        preparedDrawIndirect == openbus::rendering::indirectCullingAvailable() &&
        preparedDrawFrustumCulling == frustumCulling &&
        preparedDrawMinimumObjectViewportFraction == minimumObjectViewportFraction &&
        preparedDrawAnimationGeneration == animationGeneration &&
        preparedDrawMaterialGeneration == materialSelectionGeneration &&
        preparedDrawModelView == modelView && preparedDrawProjection == projection;
    ViewFrustum frustum;

    {
        TraceScope phase("render", "Vehicle::draw.setup");

        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        setOneSidedFrontFaceConvention();
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
    }

    {
        if (!reusePreparedDraw && frustumCulling) {
            TraceScope phase("render", "Vehicle::draw.frustumSetup");
            frustum = buildViewFrustum(openbus::rendering::projectionMatrix());
        }
    }

    if (reusePreparedDraw) {
        indirect = preparedDrawIndirect;
    } else if (openbus::rendering::indirectCullingAvailable() &&
               !indirectCandidatePartIndices.empty()) {
        TraceScope phase("render", "Vehicle::draw.prepareIndirect");
        using namespace openbus::rendering;
        static thread_local std::vector<CullingItem> items;
        static thread_local std::vector<float> boundaries;
        items.clear();
        boundaries.clear();
        appendCullingLodBoundaries(boundaries, lodThresholds);
        const auto cullingPlanes = makeCullingPlanes(projection);
        const auto projectedSize = makeCullingProjectedSize(projection, cullingMinimumFraction);
        const bool profileRecording = openbus::rendering::IsRecording();
        {
            TraceScope itemBuild("render", "Vehicle::draw.prepareIndirect.collectItems");
            for (const std::size_t partIndex : indirectCandidatePartIndices) {
                if (profileRecording) {
                    ++vehicleDrawProfileStats.indirectCandidateParts;
                }
                if (partIndex >= displayLists.size()) {
                    if (profileRecording) {
                        ++vehicleDrawProfileStats.indirectRejectedForStateOrView;
                    }
                    continue;
                }
                DisplayPart& part = displayLists[partIndex];
                part.indirectEligible = false;
                if (partIndex >= variableVisibleParts.size() || !variableVisibleParts[partIndex] ||
                    !openbus::rendering::viewpointMatches(part.viewpoint, context)) {
                    if (profileRecording) {
                        ++vehicleDrawProfileStats.indirectRejectedForStateOrView;
                    }
                    continue;
                }
                auto item = makeCullingItem(modelView, animatedPartCenter(part), part.radius);
                part.visibilityFade =
                    classifyTransparent
                        ? projectedSizeFadeAlpha(modelView, projection, animatedPartCenter(part),
                                                 part.radius, minimumObjectViewportFraction)
                        : 1.0;
                item.metadata[0] = frustumCulling ? CullFrustum : 0U;
                item.metadata[0] |= CullProjectedSize;
                if (part.lodIndex >= 0 && !boundaries.empty()) {
                    item.metadata[0] |= CullLod;
                    item.metadata[1] = static_cast<std::uint32_t>(part.lodIndex);
                    item.metadata[3] = static_cast<std::uint32_t>(boundaries.size());
                }
                if (!cullingFrustumMayIntersect(item, cullingPlanes)) {
                    if (profileRecording) {
                        ++vehicleDrawProfileStats.indirectRejectedByFrustum;
                    }
                    continue;
                }
                if (!cullingLodMayMatch(item, boundaries)) {
                    if (profileRecording) {
                        ++vehicleDrawProfileStats.indirectRejectedByLod;
                    }
                    continue;
                }
                if (!cullingProjectedSizeMayPass(item, projectedSize)) {
                    if (profileRecording) {
                        ++vehicleDrawProfileStats.indirectRejectedByProjectedSize;
                    }
                    continue;
                }
                if (profileRecording) {
                    ++vehicleDrawProfileStats.indirectEligibleParts;
                }
                part.indirectEligible = true;
                for (Batch& batch : part.batches) {
                    item.draw[0] = batch.vertexCount <= static_cast<std::size_t>(
                                                            std::numeric_limits<GLsizei>::max())
                                       ? static_cast<std::uint32_t>(batch.vertexCount)
                                       : std::numeric_limits<std::uint32_t>::max();
                    batch.indirectSlot = items.size();
                    items.push_back(item);
                }
            }
        }
        if (profileRecording) {
            ++vehicleDrawProfileStats.indirectPrepareCalls;
            vehicleDrawProfileStats.indirectItems += items.size();
            vehicleDrawProfileStats.indirectEmptyResultPreparations += items.empty() ? 1U : 0U;
        }
        const bool dispatched =
            preparedIndirectBatch.dispatch(items, cullingPlanes, boundaries, projectedSize);
        if (profileRecording && !items.empty()) {
            ++vehicleDrawProfileStats.indirectDispatches;
            vehicleDrawProfileStats.indirectDispatchFailures += dispatched ? 0U : 1U;
        }
        indirect = dispatched;
    } else if (openbus::rendering::indirectCullingAvailable() &&
               openbus::rendering::IsRecording()) {
        ++vehicleDrawProfileStats.indirectEmptyCandidateSkips;
    }
    const auto selectIndirect = [&](const Batch& batch) {
        if (indirect) {
            if (batch.indirectSlot == std::numeric_limits<std::size_t>::max()) {
                static bool loggedMissingSlot = false;
                if (!loggedMissingSlot) {
                    gameLog.Warning(
                        "Vehicle indirect command slot missing; drawing conservatively");
                    loggedMissingSlot = true;
                }
            }
            preparedIndirectBatch.select(batch.indirectSlot);
        }
    };
    const auto visible = [&](const DisplayPart& part, std::size_t partIndex,
                             bool includeEnhancedLight = false) {
        // TraceScope phase("render", "Vehicle::draw.visible");
        if (partIndex >= variableVisibleParts.size() || !variableVisibleParts[partIndex]) {
            return false;
        }
        if (indirect && part.indirectEligible)
            return part.indirectEligible;
        if (indirect && !includeEnhancedLight) {
            return false;
        }
        const std::array<double, 3> center = animatedPartCenter(part);
        const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
        const std::array<double, 4> eye = transformPoint(modelView, local);
        part.cachedViewDepth = -eye[2];
        part.viewDepthCacheGeneration = viewDepthGeneration;
        part.visibilityFade = classifyTransparent ? openbus::rendering::projectedSizeFadeAlpha(
                                                        modelView, projection, center, part.radius,
                                                        minimumObjectViewportFraction)
                                                  : 1.0;
        if (part.visibilityFade <= 0.0) {
            return false;
        }
        if (openbus::rendering::sphereBelowMinimumViewportFraction(
                modelView, projection, center, part.radius, cullingMinimumFraction)) {
            return false;
        }
        const double distance = std::sqrt(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
        if (!lodThresholds.empty() && part.lodIndex >= 0) {
            std::size_t selectedLod = lodThresholds.size() - 1;
            for (std::size_t index = 0; index < lodThresholds.size(); ++index) {
                const double boundary = 25.0 / std::max(lodThresholds[index], 0.001);
                if (distance <= boundary) {
                    selectedLod = index;
                    break;
                }
            }
            if (static_cast<std::size_t>(part.lodIndex) != selectedLod) {
                return false;
            }
        }
        if (!frustumCulling) {
            return true;
        }
        if (sphereOutsideFrustum(frustum, eye, part.radius)) {
            return false;
        }
        return true;
    };
    const auto viewDepth = [&](const DisplayPart& part) {
        // TraceScope phase("render", "Vehicle::draw.viewDepth");
        if (part.viewDepthCacheGeneration == viewDepthGeneration) {
            return part.cachedViewDepth;
        }
        const std::array<double, 3> center = animatedPartCenter(part);
        const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
        const double depth = -transformPoint(modelView, local)[2];
        part.cachedViewDepth = depth;
        part.viewDepthCacheGeneration = viewDepthGeneration;
        return depth;
    };
    std::vector<DisplayPart*>& opaqueParts = opaquePartsScratch;
    std::vector<TransparentBatch>& noDepthOpaqueBatches = noDepthOpaqueBatchesScratch;
    std::vector<TransparentBatch>& transparentBatches = transparentBatchesScratch;
    if (!reusePreparedDraw) {
        ++viewDepthGeneration;
        opaqueParts.clear();
        noDepthOpaqueBatches.clear();
        transparentBatches.clear();
        opaqueParts.reserve(displayLists.size());
        if (classifyTransparent) {
            transparentBatches.reserve(displayLists.size());
            noDepthOpaqueBatches.reserve(displayLists.size());
        }
        {
            TraceScope phase("render", "Vehicle::draw.classifyParts");
            for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
                DisplayPart& part = displayLists[partIndex];
                const bool viewpointMatches =
                    openbus::rendering::viewpointMatches(part.viewpoint, context);
                if (!viewpointMatches || !visible(part, partIndex)) {
                    continue;
                }
                bool hasOpaqueBatch = false;
                for (Batch& batch : part.batches) {
                    updateMaterialChange(batch);
                    const bool textTexture = batch.textTextureIndex >= 0;
                    hasOpaqueBatch =
                        hasOpaqueBatch || (part.visibilityFade >= 1.0 && batch.alphaMode == 0 &&
                                           !batch.noZwrite && !textTexture);
                    if (classifyTransparent && (batch.alphaMode != 0 || batch.noZwrite ||
                                                textTexture || part.visibilityFade < 1.0)) {
                        transparentBatches.push_back(
                            {&batch, &part, viewDepth(part), part.renderType});
                    }
                }
                if (renderOpaque && hasOpaqueBatch) {
                    opaqueParts.push_back(&part);
                }
            }
        }
        {
            TraceScope phase("render", "Vehicle::draw.sortBatches");
            std::sort(opaqueParts.begin(), opaqueParts.end(),
                      [&](const DisplayPart* first, const DisplayPart* second) {
                          return viewDepth(*first) < viewDepth(*second);
                      });
            openbus::app::vehicle::sortTransparentBatches(transparentBatches);
            std::stable_sort(noDepthOpaqueBatches.begin(), noDepthOpaqueBatches.end(),
                             [](const TransparentBatch& first, const TransparentBatch& second) {
                                 if (first.renderType != second.renderType) {
                                     return first.renderType < second.renderType;
                                 }
                                 return first.depth > second.depth;
                             });
        }
        preparedDrawListsValid = renderPass == VehicleRenderPass::Opaque;
        if (preparedDrawListsValid) {
            preparedDrawContext = context;
            preparedDrawReflection = reflectionPass;
            preparedDrawTransparent = classifyTransparent;
            preparedDrawIndirect = indirect;
            preparedDrawFrustumCulling = frustumCulling;
            preparedDrawMinimumObjectViewportFraction = minimumObjectViewportFraction;
            preparedDrawAnimationGeneration = animationGeneration;
            preparedDrawMaterialGeneration = materialSelectionGeneration;
            preparedDrawModelView = modelView;
            preparedDrawProjection = projection;
        }
    }
    pushMatrix();
    translate(0.0, 0.0, modelOffsetZ);
    interiorLightRootModelView = openbus::rendering::modelViewMatrix();
    if (renderOpaque) {
        TraceScope phase("render", "Vehicle::draw.opaquePass");
        for (DisplayPart* part : opaqueParts) {
            pushMatrix();
            applyDrawTransform(*part, groundShadowTransform);
            for (Batch& batch : part->batches) {
                const double alpha = alphaScale(batch);
                if (alpha <= 0.0) {
                    continue;
                }
                if (batch.alphaMode != 0 || batch.noZwrite || batch.textTextureIndex >= 0) {
                    continue;
                }
                selectIndirect(batch);
                drawBatch(batch, alpha);
                if (!reflectionPass) {
                    drawEnvironmentMap(batch, alpha);
                }
            }
            popMatrix();
        }
    }
    if (renderOpaque) {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
            DisplayPart& part = displayLists[partIndex];
            if (part.enhancedLightBatches.empty() ||
                !openbus::rendering::viewpointMatches(part.viewpoint, context) ||
                !visible(part, partIndex, true)) {
                continue;
            }
            pushMatrix();
            applyDrawTransform(part, nullptr);
            drawEnhancedLights(part);
            popMatrix();
        }
    }
    if (renderTransparent) {
        {
            TraceScope phase("render", "Vehicle::draw.transparentDepthPrepass");
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glDisable(GL_POLYGON_OFFSET_FILL);
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
            for (const TransparentBatch& transparent : transparentBatches) {
                Batch& batch = *transparent.batch;
                if (batch.textTextureIndex >= 0 || batch.alphaMode != 2 ||
                    batch.transmap.name.empty() || batch.noZwrite || batch.noZcheck) {
                    continue;
                }
                pushMatrix();
                applyDrawTransform(*transparent.part, groundShadowTransform);
                const double alpha = alphaScale(batch);
                if (alpha > 0.0) {
                    selectIndirect(batch);
                    drawBatch(batch, alpha, false, nullptr, 3, false, 0,
                              transparent.part->visibilityFade);
                }
                popMatrix();
            }
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        }
        glDepthMask(GL_FALSE);
        glDisable(GL_BLEND);
        {
            TraceScope phase("render", "Vehicle::draw.noDepthPass");
            for (const TransparentBatch& noDepthOpaque : noDepthOpaqueBatches) {
                Batch& batch = *noDepthOpaque.batch;
                pushMatrix();
                applyDrawTransform(*noDepthOpaque.part, groundShadowTransform);
                const double alpha = alphaScale(batch);
                if (alpha <= 0.0) {
                    popMatrix();
                    continue;
                }
                selectIndirect(batch);
                drawBatch(batch, alpha);
                if (!reflectionPass) {
                    drawEnvironmentMap(batch, alpha);
                }
                popMatrix();
            }
        }
        glDepthMask(GL_FALSE);
        {
            TraceScope phase("render", "Vehicle::draw.transparentPass");
            for (const TransparentBatch& transparent : transparentBatches) {
                Batch& batch = *transparent.batch;
                const bool fading = transparent.part->visibilityFade < 1.0;
                const int alphaMode = batch.textTextureIndex >= 0 || fading ? 2 : batch.alphaMode;
                pushMatrix();
                applyDrawTransform(*transparent.part, groundShadowTransform);
                const double alpha = alphaScale(batch);
                if (alpha <= 0.0) {
                    popMatrix();
                    continue;
                }
                if (alphaMode == 1) {
                    glDisable(GL_BLEND);
                    glEnable(GL_POLYGON_OFFSET_FILL);
                    glPolygonOffset(-1.0f, -1.0f);
                } else if (alphaMode == 2) {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDepthFunc(GL_LEQUAL);
                    glEnable(GL_POLYGON_OFFSET_FILL);
                    glPolygonOffset(-1.0f, -1.0f);
                } else if (batch.noZwrite) {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                } else {
                    glDisable(GL_BLEND);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                }
                GLuint transparentIndexBuffer = 0;
                if (alphaMode == 2 && batch.textTextureIndex < 0 && !batch.transmap.name.empty()) {
                    const std::size_t batchIndex =
                        static_cast<std::size_t>(&batch - transparent.part->batches.data());
                    transparentIndexBuffer =
                        sortTransparentBatch(batch, *transparent.part, batchIndex);
                }
                selectIndirect(batch);
                drawBatch(batch, alpha, false, nullptr,
                          batch.textTextureIndex >= 0 || fading ? 2 : -1,
                          transparent.part->isShadow, transparentIndexBuffer,
                          transparent.part->visibilityFade, fading);
                if (!reflectionPass) {
                    drawEnvironmentMap(batch, alpha * transparent.part->visibilityFade);
                }
                popMatrix();
            }
        }
    }
    {
        TraceScope phase("render", "Vehicle::draw.cleanup");
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);
        glDisable(GL_POLYGON_OFFSET_FILL);
        setOneSidedFrontFaceConvention();
        glDepthMask(GL_TRUE);
    }
    popMatrix();
}

void Vehicle::drawEnhancedLights(DisplayPart& part) {
    const Matrix4 partModelView = openbus::rendering::modelViewMatrix();
    using Vector3 = std::array<double, 3>;
    const auto dot = [](const Vector3& first, const Vector3& second) {
        return first[0] * second[0] + first[1] * second[1] + first[2] * second[2];
    };
    const auto cross = [](const Vector3& first, const Vector3& second) {
        return Vector3{first[1] * second[2] - first[2] * second[1],
                       first[2] * second[0] - first[0] * second[2],
                       first[0] * second[1] - first[1] * second[0]};
    };
    const auto normalize = [](const Vector3& value) -> std::optional<Vector3> {
        const double length =
            std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
        if (length <= 1.0e-9) {
            return std::nullopt;
        }
        return Vector3{value[0] / length, value[1] / length, value[2] / length};
    };
    const auto scale = [](const Vector3& value, double amount) {
        return Vector3{value[0] * amount, value[1] * amount, value[2] * amount};
    };
    const auto add = [](const Vector3& first, const Vector3& second) {
        return Vector3{first[0] + second[0], first[1] + second[1], first[2] + second[2]};
    };

    for (DisplayPart::EnhancedLightBatch& lightBatch : part.enhancedLightBatches) {
        if (lightBatch.lightIndex >= enhancedLights.size() ||
            lightBatch.lightIndex >= enhancedLightControllers.size()) {
            continue;
        }
        const ModelEnhancedLight& light = enhancedLights[lightBatch.lightIndex];
        const std::vector<double>& parameters = light.parameters;
        const std::size_t requiredParameters = light.enhanced ? 22 : 11;
        if (parameters.size() < requiredParameters || lightBatch.currentBrightness <= 0.0) {
            continue;
        }
        const std::size_t sizeIndex = light.enhanced ? 14 : 6;
        const std::size_t colorIndex = light.enhanced ? 11 : 3;
        const std::size_t offsetIndex = light.enhanced ? 18 : 8;
        if (parameters[sizeIndex] <= 0.0) {
            continue;
        }
        Batch& batch = lightBatch.batch;
        ensureTexture(batch);
        if (batch.texture == 0) {
            continue;
        }

        const std::array<double, 4> localPosition = {parameters[1], -parameters[0], parameters[2],
                                                     1.0};
        const std::array<double, 4> transformedPosition =
            transformPoint(partModelView, localPosition);
        Vector3 center = {transformedPosition[0], transformedPosition[1], transformedPosition[2]};
        center[2] += parameters[offsetIndex];
        Vector3 planeNormal = {0.0, 0.0, 1.0};
        Vector3 planeUp = {0.0, 1.0, 0.0};
        double coneBrightness = 1.0;
        if (light.enhanced) {
            const auto transformedDirection =
                transformPoint(partModelView, {parameters[4], -parameters[3], parameters[5], 0.0});
            const auto viewDirection = normalize(
                {transformedDirection[0], transformedDirection[1], transformedDirection[2]});
            const auto transformedUp =
                transformPoint(partModelView, {parameters[7], -parameters[6], parameters[8], 0.0});
            auto up = normalize({transformedUp[0], transformedUp[1], transformedUp[2]});
            const auto toCamera = normalize({-center[0], -center[1], -center[2]});
            const int directionMode = static_cast<int>(std::lround(parameters[9]));
            const int rotationMode = static_cast<int>(std::lround(parameters[10]));
            if (!viewDirection || directionMode < 0 || directionMode > 1 || rotationMode < 0 ||
                rotationMode > 2) {
                continue;
            }
            if (directionMode == 0) {
                if (!toCamera) {
                    continue;
                }
                const double angle =
                    std::acos(std::clamp(dot(*viewDirection, *toCamera), -1.0, 1.0)) *
                    (180.0 / std::numbers::pi);
                const double innerAngle = std::clamp(parameters[15], 0.0, 180.0);
                const double outerAngle = std::clamp(parameters[16], 0.0, 180.0);
                if (angle > innerAngle) {
                    coneBrightness = outerAngle <= innerAngle || angle >= outerAngle
                                         ? 0.0
                                         : (outerAngle - angle) / (outerAngle - innerAngle);
                }
                if (coneBrightness <= 0.0) {
                    continue;
                }
            }

            if (!up) {
                up = std::abs((*viewDirection)[1]) < 0.95
                         ? std::optional<Vector3>(Vector3{0.0, 1.0, 0.0})
                         : std::optional<Vector3>(Vector3{1.0, 0.0, 0.0});
            }
            planeNormal = *viewDirection;
            if (rotationMode == 1 && toCamera) {
                const double vertical = dot(planeNormal, *up);
                const Vector3 cameraHorizontal = add(*toCamera, scale(*up, -dot(*toCamera, *up)));
                if (const auto normalizedCameraHorizontal = normalize(cameraHorizontal)) {
                    const double horizontalMagnitude =
                        std::sqrt(std::max(0.0, 1.0 - vertical * vertical));
                    planeNormal = add(scale(*normalizedCameraHorizontal, horizontalMagnitude),
                                      scale(*up, vertical));
                }
            } else if (rotationMode == 2 && toCamera) {
                planeNormal = *toCamera;
            }
            if (rotationMode == 2) {
                planeUp = add(*up, scale(planeNormal, -dot(*up, planeNormal)));
                if (!normalize(planeUp)) {
                    planeUp = std::abs(planeNormal[1]) < 0.95 ? Vector3{0.0, 1.0, 0.0}
                                                              : Vector3{1.0, 0.0, 0.0};
                }
            } else {
                planeUp = *up;
            }
        }
        const auto normalizedNormal = normalize(planeNormal);
        if (!normalizedNormal) {
            continue;
        }
        planeNormal = *normalizedNormal;
        const auto normalizedUp = normalize(planeUp);
        if (!normalizedUp) {
            continue;
        }
        planeUp = *normalizedUp;
        auto planeRight = normalize(cross(planeUp, planeNormal));
        if (!planeRight) {
            const Vector3 fallbackUp =
                std::abs(planeNormal[1]) < 0.95 ? Vector3{0.0, 1.0, 0.0} : Vector3{1.0, 0.0, 0.0};
            planeRight = normalize(cross(fallbackUp, planeNormal));
        }
        if (!planeRight) {
            continue;
        }
        const auto orthogonalUp = normalize(cross(planeNormal, *planeRight));
        if (!orthogonalUp) {
            continue;
        }
        planeUp = *orthogonalUp;

        const double halfSize = parameters[sizeIndex] * 0.5;
        const auto corner = [&](double rightAmount, double upAmount) {
            return add(center, add(scale(*planeRight, rightAmount * halfSize),
                                   scale(planeUp, upAmount * halfSize)));
        };
        const auto vertex = [](const Vector3& position, float u, float v) {
            return Vertex{static_cast<float>(position[0]),
                          static_cast<float>(position[1]),
                          static_cast<float>(position[2]),
                          u,
                          v,
                          0.0F,
                          0.0F,
                          0.0F,
                          1.0F};
        };
        batch.vertices = std::vector<Vertex>{
            vertex(corner(-1.0, 1.0), 0.0F, 0.0F),  vertex(corner(1.0, -1.0), 1.0F, 1.0F),
            vertex(corner(-1.0, -1.0), 0.0F, 1.0F), vertex(corner(-1.0, 1.0), 0.0F, 0.0F),
            vertex(corner(1.0, 1.0), 1.0F, 0.0F),   vertex(corner(1.0, -1.0), 1.0F, 1.0F)};
        batch.vertexCount = batch.vertices.size();
        if (batch.buffer == 0) {
            pglGenBuffers(1, &batch.buffer);
            if (batch.buffer == 0) {
                gameLog.Error("Could not allocate enhanced-light billboard geometry");
                continue;
            }
            assets->trackBuffer(batch.buffer);
            batch.bufferTracked = true;
        }
        pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
        pglBufferData(GL_ARRAY_BUFFER,
                      static_cast<std::ptrdiff_t>(batch.vertices.size() * sizeof(Vertex)),
                      batch.vertices.data(), GL_DYNAMIC_DRAW);

        batch.alphaMode = 2;
        batch.noZwrite = true;
        batch.noZcheck = false;
        batch.textureAlphaOnly = true;
        batch.color = {std::clamp(parameters[colorIndex] / 255.0, 0.0, 1.0),
                       std::clamp(parameters[colorIndex + 1] / 255.0, 0.0, 1.0),
                       std::clamp(parameters[colorIndex + 2] / 255.0, 0.0, 1.0)};
        batch.emissionColor = {1.0, 1.0, 1.0};

        openbus::rendering::setModelViewMatrix(openbus::rendering::identityMatrix());
        drawBatch(batch, std::clamp(lightBatch.currentBrightness * coneBrightness, 0.0, 1.0), false,
                  nullptr, 2, true, 0, part.visibilityFade, true);
        openbus::rendering::setModelViewMatrix(partModelView);
    }
}

void Vehicle::drawEnvironmentMap(Batch& batch, double alpha) {
    TraceScope trace("render", "Vehicle::drawEnvironmentMap");
    ensureEnvironmentTexture(batch);
    if (batch.environmentTexture == 0 || !batch.hasNormals || alpha <= 0.0) {
        return;
    }
    const double reflectionStrength = std::clamp(
        alpha * batch.environmentStrength * openbus::rendering::ENVIRONMENT_MAP_OPACITY, 0.0, 1.0);
    if (reflectionStrength <= 0.0) {
        return;
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    drawEnvironmentBatch(batch.buffer, batch.vertexCount, batch.environmentTexture,
                         reflectionStrength);
    glDisable(GL_BLEND);
}
