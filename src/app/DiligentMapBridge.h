#pragma once

#include "app/DiligentFrameRenderer.h"
#include "map/MapSceneryPlacement.h"
#include "render/CameraMath.h"
#include "render/ComputeCulling.h"
#include "render/MatrixMath.h"
#include "render/diligent/DiligentMapLighting.h"
#include "render/diligent/DiligentSceneryAsset.h"
#include "render/diligent/DiligentTexture2D.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

class Variables;
class ScriptRuntime;

namespace openbus::app {

struct DiligentMapStreamingUpdate {
    // False means map-wide road indexing is still running; no tile scheduling or eviction occurred.
    bool roadIndexReady = false;
    std::size_t scheduledTiles = 0;
    std::size_t publishedTiles = 0;
    std::size_t removedTiles = 0;
};

struct DiligentMapSceneryPlacement {
    std::uint64_t motionObjectId = 0;
    std::filesystem::path configPath;
    openbus::rendering::Matrix4 worldTransform = {};
    std::array<double, 3> worldPosition = {};
    double boundingRadius = 0.0;
    int sourceTileX = 0;
    int sourceTileY = 0;
    const ::Variables* variables = nullptr;
    std::size_t seasonIndex = 0;
    std::filesystem::path assetRoot;
    ::ScriptRuntime* scriptRuntime = nullptr;
};

struct DiligentMapSceneryUpdate {
    std::size_t requestedAssets = 0;
    std::size_t requestedSeasonalTextures = 0;
    std::size_t publishedResources = 0;
    std::size_t removedResources = 0;
    std::size_t removedRuntimeTextureCaches = 0;
    std::vector<DiligentMapSceneryPlacement> placements;
};

struct DiligentMapFrameResourceUpdate {
    DiligentMapStreamingUpdate tiles;
    DiligentMapSceneryUpdate scenery;
};

struct DiligentMapSceneryRuntimeState {
    std::span<Diligent::ITextureView* const> seasonalBatchTextures;
    std::span<Diligent::ITextureView* const> dynamicBatchTextures;
    std::span<Diligent::ITextureView* const> textBatchTextures;
    std::span<Diligent::ITextureView* const> seasonalTreeTextures;
    float visibilityFade = 1.0F;
    std::optional<double> weatherBrightness;
    bool reverseWinding = false;
};

struct DiligentMapSceneryPlacementDrawResult {
    std::uint64_t motionObjectId = 0;
    enum class Status {
        AssetPending,
        RuntimeTexturePending,
        Culled,
        Submitted
    } status = Status::AssetPending;
    std::optional<openbus::rendering::DiligentSceneryDrawResult> drawResult;
};

inline float diligentMapSceneryVisibilityFade(const DiligentMapSceneryPlacement& placement,
                                              const openbus::rendering::Matrix4& view,
                                              const openbus::rendering::Matrix4& projection,
                                              double minimumObjectViewportFraction) {
    if (!std::isfinite(minimumObjectViewportFraction) || minimumObjectViewportFraction < 0.0 ||
        minimumObjectViewportFraction > 1.0 || !std::isfinite(placement.boundingRadius) ||
        placement.boundingRadius < 0.0 ||
        !std::all_of(
            placement.worldPosition.begin(), placement.worldPosition.end(),
            [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Invalid Diligent map scenery visibility-fade inputs");
    }
    return static_cast<float>(openbus::rendering::projectedSizeFadeAlpha(
        view, projection, placement.worldPosition, placement.boundingRadius,
        minimumObjectViewportFraction));
}

enum class DiligentMapTileViewPass : std::uint8_t {
    Terrain,
    OpaqueRoads,
    CutoutRoads,
    Water,
    TransparentRoads,
};

enum class DiligentMapMainViewPass : std::uint8_t {
    All,
    Opaque,
    Transparent,
};

struct DiligentMapMainViewDrawResult {
    std::size_t terrainBatches = 0;
    std::size_t opaqueRoadBatches = 0;
    std::size_t cutoutRoadBatches = 0;
    std::size_t waterBatches = 0;
    std::size_t transparentRoadBatches = 0;
};

// Gather opaque/cutout terrain and road casters for one spotlight depth layer.
// Water and transparent road materials do not cast into the production opaque shadow pass.
[[nodiscard]] std::vector<openbus::rendering::DiligentShadowCaster>
buildDiligentMapTileShadowCasters(const openbus::rendering::DiligentMapTileSet& tiles,
                                  double tileSize, const openbus::rendering::Matrix4& view,
                                  const openbus::rendering::Matrix4& lightClipFromView);

// Submit the main-view tile phases using the frame host's current matrices and motion history.
[[nodiscard]] DiligentMapMainViewDrawResult
drawDiligentMapMainView(openbus::rendering::DiligentFrameRenderer& frameRenderer,
                        Diligent::ITextureView& fallbackTexture,
                        const openbus::rendering::DiligentLightingParameters& lighting = {},
                        DiligentMapMainViewPass pass = DiligentMapMainViewPass::All,
                        const openbus::rendering::DiligentMapWeather* weather = nullptr);

// Submit one map-tile phase with the supplied view. The caller owns pass ordering and
// secondary-view temporal history; these draws intentionally do not update main-view history.
std::size_t drawDiligentMapTilesInView(
    const openbus::rendering::DiligentMapTileSet& tiles,
    openbus::rendering::DiligentTexturedMeshRenderer& meshes, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const openbus::rendering::Matrix4& viewProjection,
    DiligentMapTileViewPass pass,
    const openbus::rendering::DiligentMaterialState* materialOverride = nullptr,
    const openbus::rendering::DiligentLightingParameters& lighting = {},
    const openbus::rendering::DiligentMapWeather* weather = nullptr);

inline std::vector<std::size_t>
orderDiligentMapSceneryPlacements(std::span<const DiligentMapSceneryPlacement> placements,
                                  const openbus::rendering::Matrix4& view,
                                  openbus::rendering::DiligentSceneryPass pass) {
    std::vector<std::size_t> order;
    order.reserve(placements.size());
    for (std::size_t index = 0; index < placements.size(); ++index) {
        order.push_back(index);
    }
    if (pass != openbus::rendering::DiligentSceneryPass::Transparent) {
        return order;
    }

    std::vector<double> viewDepths;
    viewDepths.reserve(placements.size());
    for (const DiligentMapSceneryPlacement& placement : placements) {
        const auto& position = placement.worldPosition;
        const double depth =
            view[2] * position[0] + view[6] * position[1] + view[10] * position[2] + view[14];
        if (!std::isfinite(depth)) {
            throw std::invalid_argument("Diligent map scenery placement has invalid view depth");
        }
        viewDepths.push_back(depth);
    }
    std::stable_sort(order.begin(), order.end(), [&](std::size_t left, std::size_t right) {
        return viewDepths[left] < viewDepths[right];
    });
    return order;
}

// MapSource supplies mapDefinition(), omsiRoot(), baseGroundTextureIndex(), and
// roadConnectionIndex(). Call outside scene submission; preparation failures propagate.
template <typename MapSource>
DiligentMapStreamingUpdate
updateDiligentMapTiles(openbus::rendering::DiligentFrameRenderer& frameRenderer,
                       const MapSource& mapSource, double worldX, double worldY, int radius) {
    DiligentMapStreamingUpdate update;
    const auto roadConnections = mapSource.roadConnectionIndex();
    if (!roadConnections) {
        return update;
    }

    update.roadIndexReady = true;
    update.scheduledTiles = frameRenderer.requestMapTileWindow(
        mapSource.mapDefinition(), worldX, worldY, radius, mapSource.omsiRoot(),
        mapSource.baseGroundTextureIndex(), roadConnections);
    update.publishedTiles = frameRenderer.publishReadyMapTiles();
    update.removedTiles = frameRenderer.removeInactiveMapTiles();
    return update;
}

// MapRenderer exposes its active radius so the Diligent request stays aligned with production
// tile selection instead of duplicating the renderer's streaming policy at each call site.
template <typename MapSource>
DiligentMapStreamingUpdate
updateDiligentMapTiles(openbus::rendering::DiligentFrameRenderer& frameRenderer,
                       const MapSource& mapSource, double worldX, double worldY) {
    return updateDiligentMapTiles(frameRenderer, mapSource, worldX, worldY,
                                  mapSource.activeTileRadius());
}

// Call outside scene submission after MapRenderer has updated its active window.
template <typename MapSource>
DiligentMapSceneryUpdate
updateDiligentMapScenery(openbus::rendering::DiligentFrameRenderer& frameRenderer,
                         const MapSource& mapSource) {
    const auto sources = mapSource.activeSceneryAssetSources();
    DiligentMapSceneryUpdate update;
    std::size_t seasonIndex = 0;
    if constexpr (requires { mapSource.activeTreeSeason(); }) {
        seasonIndex = mapSource.activeTreeSeason();
    }
    if (seasonIndex >= openbus::rendering::map_detail::treeSeasonDirectories.size()) {
        throw std::invalid_argument("Map scenery season index is out of range");
    }
    const auto activePlacements = mapSource.activeSceneryPlacements();
    std::vector<std::uint64_t> activeMotionObjectIds;
    activeMotionObjectIds.reserve(activePlacements.size());
    for (const auto& placement : activePlacements) {
        update.placements.push_back({placement.motionObjectId, placement.configPath,
                                     placement.worldTransform, placement.worldPosition,
                                     placement.boundingRadius, placement.sourceTileX,
                                     placement.sourceTileY, placement.variables, seasonIndex,
                                     mapSource.omsiRoot(), placement.scriptRuntime});
        activeMotionObjectIds.push_back(placement.motionObjectId);
    }
    update.removedRuntimeTextureCaches =
        frameRenderer.retainMapSceneryRuntimeTextures(activeMotionObjectIds);
    update.removedResources =
        frameRenderer.retainSceneryAssets(sources.assets, sources.seasonalTextures);
    for (const auto& [configPath, omsiRoot] : sources.assets) {
        update.requestedAssets += frameRenderer.requestSceneryAsset(configPath, omsiRoot) ? 1U : 0U;
    }
    for (const std::filesystem::path& path : sources.seasonalTextures) {
        update.requestedSeasonalTextures +=
            frameRenderer.requestSeasonalSceneryTexture(path) ? 1U : 0U;
    }
    update.publishedResources = frameRenderer.publishReadySceneryAssets();
    return update;
}

// Update the active tile window and retained scenery resources together before scene submission.
// The MapRenderer must have completed its own streaming update for the current player position.
template <typename MapSource>
DiligentMapFrameResourceUpdate
updateDiligentMapFrameResources(openbus::rendering::DiligentFrameRenderer& frameRenderer,
                                const MapSource& mapSource, double worldX, double worldY) {
    DiligentMapFrameResourceUpdate update;
    update.tiles = updateDiligentMapTiles(frameRenderer, mapSource, worldX, worldY);
    if (!update.tiles.roadIndexReady) {
        return update;
    }
    update.scenery = updateDiligentMapScenery(frameRenderer, mapSource);
    return update;
}

// Advance the production map stream and then synchronize Diligent resources for that same
// position. Call before scene submission so tile selection and placement snapshots agree.
template <typename MapSource>
DiligentMapFrameResourceUpdate
updateDiligentMapFrameResourcesAtPosition(openbus::rendering::DiligentFrameRenderer& frameRenderer,
                                          MapSource& mapSource, double worldX, double worldY) {
    mapSource.updateStreaming(worldX, worldY);
    return updateDiligentMapFrameResources(frameRenderer, mapSource, worldX, worldY);
}

inline openbus::rendering::DiligentSceneryDrawResult drawDiligentMapSceneryPlacement(
    openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const DiligentMapSceneryPlacement& placement,
    const openbus::rendering::DiligentSceneryAsset& asset,
    openbus::rendering::DiligentSceneryPass pass, Diligent::ITextureView& fallbackTexture,
    std::span<const openbus::rendering::DiligentSceneryOpaqueDraw> batchDraws,
    std::span<const openbus::rendering::DiligentSceneryOpaqueDraw> treeDraws,
    std::uint64_t viewId = 0, bool geometryHistoryValid = true,
    const openbus::rendering::DiligentLightingParameters& lighting = {}) {
    return frameRenderer.drawSceneryPlacement(
        asset, placement.worldTransform, placement.boundingRadius, pass, fallbackTexture,
        batchDraws, treeDraws, placement.motionObjectId, viewId, geometryHistoryValid, lighting,
        placement.variables);
}

inline openbus::rendering::DiligentSceneryDrawResult drawDiligentMapSceneryInView(
    const openbus::rendering::DiligentSceneryAsset& asset,
    openbus::rendering::DiligentSceneryPass pass,
    openbus::rendering::DiligentTexturedMeshRenderer& meshes, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const openbus::rendering::Matrix4& modelViewProjection,
    const openbus::rendering::Matrix4& modelView,
    std::span<const openbus::rendering::DiligentSceneryOpaqueDraw> batchDraws,
    std::span<const openbus::rendering::DiligentSceneryOpaqueDraw> treeDraws = {},
    const openbus::rendering::DiligentLightingParameters& lighting = {},
    const ::Variables* variables = nullptr) {
    const auto batches = asset.batches();
    const auto trees = asset.trees();
    const bool transparentTrees =
        pass == openbus::rendering::DiligentSceneryPass::Transparent &&
        (std::any_of(trees.begin(), trees.end(),
                     [](const auto& tree) { return tree.source.noZwrite; }) ||
         (treeDraws.size() == trees.size() &&
          std::any_of(treeDraws.begin(), treeDraws.end(),
                      [](const auto& draw) { return draw.visibilityFade < 1.0F; })));
    if (batchDraws.size() != batches.size() ||
        ((pass == openbus::rendering::DiligentSceneryPass::Opaque || transparentTrees) &&
         treeDraws.size() != trees.size())) {
        throw std::invalid_argument("Diligent secondary-view scenery draw state count mismatch");
    }

    std::vector<std::size_t> order;
    order.reserve(batches.size());
    for (std::size_t index = 0; index < batches.size(); ++index) {
        const auto& batch = batches[index].source.batch;
        const bool usesTextTexture = batch.textTextureIndex >= 0;
        const auto& draw = batchDraws[index];
        if (!std::isfinite(draw.visibilityFade) || draw.visibilityFade < 0.0F ||
            draw.visibilityFade > 1.0F) {
            throw std::invalid_argument(
                "Diligent secondary-view scenery visibility fade must be finite and in [0, 1]");
        }
        const bool fadingOpaqueBatch = !usesTextTexture && !batch.noZwrite &&
                                       draw.visibilityFade < 1.0F &&
                                       (batch.alphaMode == 0 || batch.alphaMode == 1);
        const bool matchesPass =
            (pass == openbus::rendering::DiligentSceneryPass::Opaque && !usesTextTexture &&
             !batch.noZwrite && (batch.alphaMode == 0 || batch.alphaMode == 1) &&
             !fadingOpaqueBatch) ||
            (pass == openbus::rendering::DiligentSceneryPass::Transparent &&
             (usesTextTexture || batch.noZwrite || batch.alphaMode == 2 || batch.alphaMode == 3 ||
              fadingOpaqueBatch)) ||
            (pass == openbus::rendering::DiligentSceneryPass::TransmapDepthPrepass &&
             batch.alphaMode == 2);
        if (!matchesPass) {
            continue;
        }
        const ::Variables* selectedVariables = variables != nullptr ? variables : draw.variables;
        if (draw.visibilityFade == 0.0F ||
            (!batch.visibilityVariable.empty() &&
             (selectedVariables != nullptr ? selectedVariables->get(batch.visibilityVariable)
                                           : 0.0F) != static_cast<float>(batch.visibilityValue))) {
            continue;
        }
        order.push_back(index);
    }
    openbus::rendering::DiligentSceneryDrawResult result;
    std::vector<std::size_t> transparentTreeOrder;
    if (pass == openbus::rendering::DiligentSceneryPass::Transparent) {
        for (std::size_t index = 0; index < trees.size(); ++index) {
            const auto& draw = treeDraws[index];
            if (!std::isfinite(draw.visibilityFade) || draw.visibilityFade < 0.0F ||
                draw.visibilityFade > 1.0F) {
                throw std::invalid_argument(
                    "Diligent secondary-view tree visibility fade must be finite and in [0, 1]");
            }
            const bool transparentTree = trees[index].source.noZwrite || draw.visibilityFade < 1.0F;
            if (!transparentTree) {
                continue;
            }
            if (draw.visibilityFade > 0.0F && trees[index].mesh != nullptr) {
                transparentTreeOrder.push_back(index);
            }
        }
    }
    const auto transparentOrder =
        pass == openbus::rendering::DiligentSceneryPass::Transparent
            ? openbus::rendering::orderDiligentTransparentScenery(batches, order, trees,
                                                                  transparentTreeOrder, modelView)
            : std::vector<openbus::rendering::DiligentScenerySubmission>{};
    const auto drawBatch = [&](std::size_t index) {
        auto draw = batchDraws[index];
        if (variables != nullptr) {
            draw.variables = variables;
        }
        const bool submitted =
            pass == openbus::rendering::DiligentSceneryPass::Opaque
                ? batches[index].drawOpaque(meshes, context, fallbackTexture, modelViewProjection,
                                            draw, lighting)
            : pass == openbus::rendering::DiligentSceneryPass::Transparent
                ? batches[index].drawTransparent(meshes, context, fallbackTexture,
                                                 modelViewProjection, draw, lighting)
                : batches[index].drawTransmapDepthPrepass(meshes, context, fallbackTexture,
                                                          modelViewProjection, draw, lighting);
        result.batchesDrawn += submitted ? 1U : 0U;
    };
    const auto drawTree = [&](std::size_t index) {
        auto draw = treeDraws[index];
        if (variables != nullptr) {
            draw.variables = variables;
        }
        result.treesDrawn +=
            trees[index].draw(meshes, context, fallbackTexture, modelViewProjection, draw, lighting)
                ? 1U
                : 0U;
    };
    if (pass == openbus::rendering::DiligentSceneryPass::Transparent) {
        for (const auto& submission : transparentOrder) {
            if (submission.kind == openbus::rendering::DiligentScenerySubmissionKind::Batch) {
                drawBatch(submission.index);
            } else {
                drawTree(submission.index);
            }
        }
    } else {
        for (const std::size_t index : order) {
            drawBatch(index);
        }
    }
    if (pass == openbus::rendering::DiligentSceneryPass::Opaque) {
        for (std::size_t index = 0; index < trees.size(); ++index) {
            if (trees[index].source.noZwrite || treeDraws[index].visibilityFade < 1.0F) {
                continue;
            }
            const auto& draw = treeDraws[index];
            if (!std::isfinite(draw.visibilityFade) || draw.visibilityFade < 0.0F ||
                draw.visibilityFade > 1.0F) {
                throw std::invalid_argument(
                    "Diligent secondary-view tree visibility fade must be finite and in [0, 1]");
            }
            drawTree(index);
        }
    }
    return result;
}

template <typename TextureLookup>
Diligent::ITextureView* resolveDiligentMapSeasonalTexture(
    std::size_t seasonIndex,
    const openbus::rendering::SharedMapTextureImages::SeasonalPaths& seasonalPaths,
    TextureLookup&& textureLookup) {
    if (seasonIndex >= seasonalPaths.size()) {
        throw std::invalid_argument("Map placement season index is out of range");
    }
    const std::array<std::size_t, 2> candidates =
        seasonIndex == 4 ? std::array<std::size_t, 2>{4, 3}
                         : std::array<std::size_t, 2>{seasonIndex, seasonIndex};
    const std::size_t candidateCount = seasonIndex == 4 ? 2 : 1;
    for (std::size_t candidate = 0; candidate < candidateCount; ++candidate) {
        const std::filesystem::path& path = seasonalPaths[candidates[candidate]];
        if (path.empty()) {
            continue;
        }
        if (Diligent::ITextureView* texture = textureLookup(path)) {
            return texture;
        }
    }
    return nullptr;
}

inline std::vector<openbus::rendering::DiligentShadowCaster> buildDiligentMapSceneryShadowCasters(
    const DiligentMapSceneryPlacement& placement,
    const openbus::rendering::DiligentSceneryAsset& asset, const openbus::rendering::Matrix4& view,
    const openbus::rendering::Matrix4& lightClipFromView,
    std::span<Diligent::ITextureView* const> seasonalBatchTextures = {},
    std::span<Diligent::ITextureView* const> seasonalTreeTextures = {},
    float visibilityFade = 1.0F) {
    using namespace openbus::rendering;
    const auto validMatrix = [](const Matrix4& matrix) {
        return std::all_of(matrix.begin(), matrix.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (placement.motionObjectId == 0 || !validMatrix(placement.worldTransform) ||
        !validMatrix(view) || !validMatrix(lightClipFromView) || !std::isfinite(visibilityFade) ||
        visibilityFade < 0.0F || visibilityFade > 1.0F) {
        throw std::invalid_argument("Invalid Diligent map scenery shadow placement or view");
    }
    if (visibilityFade == 0.0F) {
        return {};
    }

    const auto batches = asset.batches();
    const auto trees = asset.trees();
    const auto validateTextureCount = [](std::size_t supplied, std::size_t expected) {
        if (supplied != 0 && supplied != expected) {
            throw std::invalid_argument(
                "Diligent map scenery shadow texture count does not match the asset");
        }
    };
    validateTextureCount(seasonalBatchTextures.size(), batches.size());
    validateTextureCount(seasonalTreeTextures.size(), trees.size());

    const Matrix4 modelView = multiplyMatrix4(view, placement.worldTransform);
    const auto viewPosition = transformPoint(modelView, {0.0, 0.0, 0.0, 1.0});
    if (!std::all_of(viewPosition.begin(), viewPosition.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Invalid Diligent map scenery shadow placement position");
    }
    const std::size_t selectedLod =
        asset.selectLod(std::hypot(viewPosition[0], viewPosition[1], viewPosition[2]));
    const Matrix4 lightClipFromWorld = multiplyMatrix4(lightClipFromView, view);
    const Matrix4 lightClipFromPlacement =
        multiplyMatrix4(lightClipFromWorld, placement.worldTransform);
    std::vector<DiligentShadowCaster> casters;
    const auto visible = [&](const openbus::map::MapSceneryBatch& batch) {
        return batch.visibilityVariable.empty() ||
               (placement.variables != nullptr ? placement.variables->get(batch.visibilityVariable)
                                               : 0.0F) == static_cast<float>(batch.visibilityValue);
    };
    const auto appendCaster = [&](const DiligentTexturedMesh* mesh, int alphaMode,
                                  Diligent::ITextureView* texture,
                                  const openbus::map::MapSceneryBatch& batch,
                                  const std::array<float, 4>& textureTransform, double baseAlpha) {
        if (mesh == nullptr || (alphaMode != 0 && alphaMode != 1 && alphaMode != 3) ||
            batch.noZwrite || batch.noZcheck || !visible(batch)) {
            return;
        }
        const double clampedAlpha = std::clamp(baseAlpha * visibilityFade, 0.0, 1.0);
        const double alphaCutoff = alphaMode == 3 ? 0.99 : 0.5;
        if (alphaMode != 0 && clampedAlpha < alphaCutoff) {
            return;
        }
        DiligentShadowCaster caster;
        caster.mesh = mesh;
        caster.modelViewProjection = lightClipFromPlacement;
        caster.textureTransform = textureTransform;
        if (alphaMode != 0 && texture != nullptr) {
            caster.alphaTexture = texture;
            caster.alphaCutoff = static_cast<float>(alphaCutoff / clampedAlpha);
        }
        casters.push_back(caster);
    };

    for (std::size_t index = 0; index < batches.size(); ++index) {
        const DiligentSceneryBatch& batch = batches[index];
        const auto& source = batch.source.batch;
        if (source.textTextureIndex >= 0 || source.sceneryLodIndex != selectedLod ||
            source.noZwrite || source.noZcheck) {
            continue;
        }
        const bool fadingOpaque =
            visibilityFade < 1.0F && (source.alphaMode == 0 || source.alphaMode == 1);
        const bool transmapCutout =
            source.alphaMode == 2 ||
            (source.alphaMode == 3 && batch.textures.transmap != nullptr) ||
            ((source.alphaMode == 0 || source.alphaMode == 1) && fadingOpaque);
        if ((source.alphaMode == 2 && batch.textures.transmap == nullptr) ||
            (fadingOpaque && batch.textures.transmap == nullptr) ||
            (source.alphaMode != 0 && source.alphaMode != 1 && source.alphaMode != 2 &&
             source.alphaMode != 3)) {
            continue;
        }
        Diligent::ITextureView* texture = nullptr;
        int shadowAlphaMode = source.alphaMode;
        if (transmapCutout) {
            texture = batch.textures.transmap != nullptr
                          ? &batch.textures.transmap->shaderResourceView()
                          : nullptr;
            shadowAlphaMode = 3;
        } else {
            texture = seasonalBatchTextures.empty() ? nullptr : seasonalBatchTextures[index];
            if (texture == nullptr && batch.textures.base != nullptr) {
                texture = &batch.textures.base->shaderResourceView();
            }
        }
        const auto runtime = batch.resolveRuntime(placement.variables, std::nullopt);
        const bool flipY = batch.source.batch.material.flipTextureY;
        const float scaleY = flipY ? -1.0F : 1.0F;
        const float offsetX = runtime.texcoordOffsetX.value_or(0.0F);
        const float offsetY = (flipY ? 1.0F : 0.0F) + runtime.texcoordOffsetY.value_or(0.0F);
        const std::array<float, 4> textureTransform = {1.0F, scaleY, offsetX, offsetY};
        appendCaster(batch.mesh.get(), shadowAlphaMode, texture, source, textureTransform,
                     source.alpha * runtime.alphaScale);
    }
    for (std::size_t index = 0; index < trees.size(); ++index) {
        const DiligentSceneryTreeBatch& tree = trees[index];
        if (tree.source.sceneryLodIndex != selectedLod || tree.source.noZwrite ||
            tree.source.noZcheck ||
            (visibilityFade < 1.0F && (tree.source.alphaMode == 0 || tree.source.alphaMode == 1))) {
            continue;
        }
        Diligent::ITextureView* texture =
            seasonalTreeTextures.empty() ? nullptr : seasonalTreeTextures[index];
        if (texture == nullptr && tree.texture != nullptr) {
            texture = &tree.texture->shaderResourceView();
        }
        const bool flipY = tree.source.material.flipTextureY;
        const std::array<float, 4> textureTransform = {1.0F, flipY ? -1.0F : 1.0F, 0.0F,
                                                       flipY ? 1.0F : 0.0F};
        appendCaster(tree.mesh.get(), tree.source.alphaMode, texture, tree.source, textureTransform,
                     tree.source.alpha);
    }
    return casters;
}

struct DiligentMapSceneryDrawStates {
    std::vector<openbus::rendering::DiligentSceneryOpaqueDraw> batches;
    std::vector<openbus::rendering::DiligentSceneryOpaqueDraw> trees;
};

inline std::size_t
selectDiligentMapSceneryLod(const openbus::rendering::DiligentSceneryAsset& asset,
                            const openbus::rendering::Matrix4& modelView) {
    const auto center = openbus::rendering::transformPoint(modelView, {0.0, 0.0, 0.0, 1.0});
    if (!std::all_of(center.begin(), center.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent map scenery view position is invalid");
    }
    return asset.selectLod(std::hypot(center[0], center[1], center[2]));
}

inline DiligentMapSceneryDrawStates makeDiligentMapSceneryDrawStates(
    openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const DiligentMapSceneryPlacement& placement,
    const openbus::rendering::DiligentSceneryAsset& asset,
    std::span<Diligent::ITextureView* const> seasonalBatchTextures = {},
    std::span<Diligent::ITextureView* const> dynamicBatchTextures = {},
    std::span<Diligent::ITextureView* const> textBatchTextures = {},
    std::span<Diligent::ITextureView* const> seasonalTreeTextures = {}, float visibilityFade = 1.0F,
    std::optional<double> weatherBrightness = std::nullopt, bool reverseWinding = false,
    std::optional<std::size_t> sceneryLodIndex = std::nullopt) {
    const auto validateCount = [](std::size_t supplied, std::size_t expected,
                                  const char* resourceName) {
        if (supplied != 0 && supplied != expected) {
            throw std::invalid_argument(std::string("Diligent map scenery ") + resourceName +
                                        " count does not match the prepared asset");
        }
    };
    const auto batches = asset.batches();
    const auto trees = asset.trees();
    validateCount(seasonalBatchTextures.size(), batches.size(), "seasonal texture");
    validateCount(dynamicBatchTextures.size(), batches.size(), "dynamic texture");
    validateCount(textBatchTextures.size(), batches.size(), "text texture");
    validateCount(seasonalTreeTextures.size(), trees.size(), "seasonal tree texture");
    for (std::size_t index = 0; index < batches.size(); ++index) {
        const auto& source = batches[index].source.batch;
        if ((source.scriptTextureIndex >= 0 || !source.freeTextureVariable.empty()) &&
            (dynamicBatchTextures.empty() || dynamicBatchTextures[index] == nullptr)) {
            throw std::invalid_argument(
                "Diligent map scenery dynamic texture is required for this batch");
        }
        if (source.textTextureIndex >= 0 &&
            (textBatchTextures.empty() || textBatchTextures[index] == nullptr)) {
            throw std::invalid_argument(
                "Diligent map scenery text texture is required for this batch");
        }
    }

    const auto resolveSeasonalTexture =
        [&frameRenderer, seasonIndex = placement.seasonIndex](
            const std::shared_ptr<const openbus::rendering::SharedMapTextureImages::SeasonalPaths>&
                seasonalPaths) -> Diligent::ITextureView* {
        if (!seasonalPaths) {
            return nullptr;
        }
        return resolveDiligentMapSeasonalTexture(
            seasonIndex, *seasonalPaths, [&frameRenderer](const std::filesystem::path& path) {
                const auto* texture = frameRenderer.seasonalSceneryTexture(path);
                return texture ? &texture->shaderResourceView() : nullptr;
            });
    };
    const auto makeDraw =
        [&](std::size_t index,
            const std::shared_ptr<const openbus::rendering::SharedMapTextureImages::SeasonalPaths>&
                seasonalPaths,
            std::span<Diligent::ITextureView* const> seasonalTextures,
            std::span<Diligent::ITextureView* const> dynamicTextures,
            std::span<Diligent::ITextureView* const> textTextures) {
            openbus::rendering::DiligentSceneryOpaqueDraw draw;
            draw.variables = placement.variables;
            draw.weatherBrightness = weatherBrightness;
            draw.visibilityFade = visibilityFade;
            draw.reverseWinding = reverseWinding;
            draw.sceneryLodIndex = sceneryLodIndex;
            draw.seasonalTexture = seasonalTextures.empty() ? resolveSeasonalTexture(seasonalPaths)
                                                            : seasonalTextures[index];
            if (!dynamicTextures.empty()) {
                draw.dynamicTexture = dynamicTextures[index];
            }
            if (!textTextures.empty()) {
                draw.textTexture = textTextures[index];
            }
            return draw;
        };

    DiligentMapSceneryDrawStates states;
    states.batches.reserve(batches.size());
    for (std::size_t index = 0; index < batches.size(); ++index) {
        states.batches.push_back(makeDraw(index, batches[index].source.seasonTexturePaths,
                                          seasonalBatchTextures, dynamicBatchTextures,
                                          textBatchTextures));
    }
    states.trees.reserve(trees.size());
    for (std::size_t index = 0; index < trees.size(); ++index) {
        states.trees.push_back(
            makeDraw(index, trees[index].seasonTexturePaths, seasonalTreeTextures, {}, {}));
    }
    return states;
}

inline openbus::rendering::DiligentSceneryDrawResult drawDiligentMapSceneryPlacementWithResources(
    openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const DiligentMapSceneryPlacement& placement,
    const openbus::rendering::DiligentSceneryAsset& asset,
    openbus::rendering::DiligentSceneryPass pass, Diligent::ITextureView& fallbackTexture,
    std::span<Diligent::ITextureView* const> seasonalBatchTextures = {},
    std::span<Diligent::ITextureView* const> dynamicBatchTextures = {},
    std::span<Diligent::ITextureView* const> textBatchTextures = {},
    std::span<Diligent::ITextureView* const> seasonalTreeTextures = {}, std::uint64_t viewId = 0,
    float visibilityFade = 1.0F, std::optional<double> weatherBrightness = std::nullopt,
    bool reverseWinding = false, bool geometryHistoryValid = true,
    const openbus::rendering::DiligentLightingParameters& lighting = {}) {
    DiligentMapSceneryDrawStates states = makeDiligentMapSceneryDrawStates(
        frameRenderer, placement, asset, seasonalBatchTextures, dynamicBatchTextures,
        textBatchTextures, seasonalTreeTextures, visibilityFade, weatherBrightness, reverseWinding,
        selectDiligentMapSceneryLod(
            asset, openbus::rendering::multiplyMatrix4(frameRenderer.frameData().currentView,
                                                       placement.worldTransform)));
    return frameRenderer.drawSceneryPlacement(
        asset, placement.worldTransform, placement.boundingRadius, pass, fallbackTexture,
        states.batches, states.trees, placement.motionObjectId, viewId, geometryHistoryValid,
        lighting, placement.variables);
}

inline openbus::rendering::DiligentSceneryDrawResult
drawDiligentMapSceneryPlacementInViewWithResources(
    openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const DiligentMapSceneryPlacement& placement,
    const openbus::rendering::DiligentSceneryAsset& asset,
    openbus::rendering::DiligentSceneryPass pass,
    openbus::rendering::DiligentTexturedMeshRenderer& meshes, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const openbus::rendering::Matrix4& modelViewProjection,
    const openbus::rendering::Matrix4& modelView,
    std::span<Diligent::ITextureView* const> seasonalBatchTextures = {},
    std::span<Diligent::ITextureView* const> dynamicBatchTextures = {},
    std::span<Diligent::ITextureView* const> textBatchTextures = {},
    std::span<Diligent::ITextureView* const> seasonalTreeTextures = {}, float visibilityFade = 1.0F,
    std::optional<double> weatherBrightness = std::nullopt, bool reverseWinding = false,
    const openbus::rendering::DiligentLightingParameters& lighting = {}) {
    const DiligentMapSceneryDrawStates states = makeDiligentMapSceneryDrawStates(
        frameRenderer, placement, asset, seasonalBatchTextures, dynamicBatchTextures,
        textBatchTextures, seasonalTreeTextures, visibilityFade, weatherBrightness, reverseWinding,
        selectDiligentMapSceneryLod(asset, modelView));
    return drawDiligentMapSceneryInView(asset, pass, meshes, context, fallbackTexture,
                                        modelViewProjection, modelView, states.batches,
                                        states.trees, lighting, placement.variables);
}

inline std::optional<openbus::rendering::DiligentSceneryDrawResult>
drawCachedDiligentMapSceneryPlacement(
    openbus::rendering::DiligentFrameRenderer& frameRenderer,
    const DiligentMapSceneryPlacement& placement, openbus::rendering::DiligentSceneryPass pass,
    Diligent::ITextureView& fallbackTexture,
    std::span<Diligent::ITextureView* const> seasonalBatchTextures = {},
    std::span<Diligent::ITextureView* const> dynamicBatchTextures = {},
    std::span<Diligent::ITextureView* const> textBatchTextures = {},
    std::span<Diligent::ITextureView* const> seasonalTreeTextures = {}, std::uint64_t viewId = 0,
    float visibilityFade = 1.0F, std::optional<double> weatherBrightness = std::nullopt,
    bool reverseWinding = false, bool geometryHistoryValid = true,
    const openbus::rendering::DiligentLightingParameters& lighting = {}) {
    const auto* asset = frameRenderer.sceneryAsset(placement.configPath, placement.assetRoot);
    if (asset == nullptr) {
        return std::nullopt;
    }
    return drawDiligentMapSceneryPlacementWithResources(
        frameRenderer, placement, *asset, pass, fallbackTexture, seasonalBatchTextures,
        dynamicBatchTextures, textBatchTextures, seasonalTreeTextures, viewId, visibilityFade,
        weatherBrightness, reverseWinding, geometryHistoryValid, lighting);
}

inline std::vector<DiligentMapSceneryPlacementDrawResult> drawDiligentMapSceneryPlacements(
    openbus::rendering::DiligentFrameRenderer& frameRenderer,
    std::span<const DiligentMapSceneryPlacement> placements,
    openbus::rendering::DiligentSceneryPass pass, Diligent::ITextureView& fallbackTexture,
    const openbus::rendering::Matrix4& view,
    std::span<const DiligentMapSceneryRuntimeState> runtimeStates = {}, std::uint64_t viewId = 0,
    bool geometryHistoryValid = true,
    const openbus::rendering::DiligentLightingParameters& lighting = {},
    std::optional<double> weatherBrightness = std::nullopt,
    double minimumObjectViewportFraction = 0.0) {
    if (!runtimeStates.empty() && runtimeStates.size() != placements.size()) {
        throw std::invalid_argument(
            "Diligent map scenery runtime states do not match the placement count");
    }

    const std::vector<std::size_t> order =
        orderDiligentMapSceneryPlacements(placements, view, pass);
    std::vector<DiligentMapSceneryPlacementDrawResult> results;
    results.reserve(order.size());
    for (const std::size_t index : order) {
        const DiligentMapSceneryPlacement& placement = placements[index];
        if (placement.motionObjectId == 0) {
            throw std::invalid_argument(
                "Diligent map scenery placement requires a stable motion object identifier");
        }
        DiligentMapSceneryRuntimeState runtime =
            runtimeStates.empty() ? DiligentMapSceneryRuntimeState{} : runtimeStates[index];
        if (!runtime.weatherBrightness.has_value()) {
            runtime.weatherBrightness = weatherBrightness;
        }
        if (!std::isfinite(minimumObjectViewportFraction) || minimumObjectViewportFraction < 0.0 ||
            minimumObjectViewportFraction > 1.0) {
            throw std::invalid_argument(
                "Diligent map scenery minimum viewport fraction is invalid");
        }
        if (minimumObjectViewportFraction > 0.0) {
            runtime.visibilityFade *= diligentMapSceneryVisibilityFade(
                placement, view, frameRenderer.frameData().currentProjection,
                minimumObjectViewportFraction);
        }
        if (runtime.visibilityFade == 0.0F) {
            openbus::rendering::DiligentSceneryDrawResult drawResult;
            drawResult.culled = true;
            results.push_back({placement.motionObjectId,
                               DiligentMapSceneryPlacementDrawResult::Status::Culled,
                               std::move(drawResult)});
            continue;
        }
        const auto* asset = frameRenderer.sceneryAsset(placement.configPath, placement.assetRoot);
        if (asset == nullptr) {
            results.push_back({placement.motionObjectId,
                               DiligentMapSceneryPlacementDrawResult::Status::AssetPending,
                               std::nullopt});
            continue;
        }
        if (!frameRenderer.isSceneryPlacementVisible(placement.worldTransform,
                                                     placement.boundingRadius)) {
            openbus::rendering::DiligentSceneryDrawResult drawResult;
            drawResult.culled = true;
            results.push_back({placement.motionObjectId,
                               DiligentMapSceneryPlacementDrawResult::Status::Culled,
                               std::move(drawResult)});
            continue;
        }

        std::vector<Diligent::ITextureView*> dynamicBatchTextures;
        std::vector<Diligent::ITextureView*> textBatchTextures;
        if (runtimeStates.empty() && !frameRenderer.synchronizeMapSceneryRuntimeTextures(
                                         placement.motionObjectId, *asset, placement.scriptRuntime,
                                         dynamicBatchTextures, textBatchTextures)) {
            results.push_back({placement.motionObjectId,
                               DiligentMapSceneryPlacementDrawResult::Status::RuntimeTexturePending,
                               std::nullopt});
            continue;
        }

        const auto drawResult =
            runtimeStates.empty() ? drawDiligentMapSceneryPlacementWithResources(
                                        frameRenderer, placement, *asset, pass, fallbackTexture,
                                        runtime.seasonalBatchTextures, dynamicBatchTextures,
                                        textBatchTextures, runtime.seasonalTreeTextures, viewId,
                                        runtime.visibilityFade, runtime.weatherBrightness,
                                        runtime.reverseWinding, geometryHistoryValid, lighting)
                                  : drawDiligentMapSceneryPlacementWithResources(
                                        frameRenderer, placement, *asset, pass, fallbackTexture,
                                        runtime.seasonalBatchTextures, runtime.dynamicBatchTextures,
                                        runtime.textBatchTextures, runtime.seasonalTreeTextures,
                                        viewId, runtime.visibilityFade, runtime.weatherBrightness,
                                        runtime.reverseWinding, geometryHistoryValid, lighting);
        results.push_back({placement.motionObjectId,
                           DiligentMapSceneryPlacementDrawResult::Status::Submitted, drawResult});
    }
    return results;
}

inline bool
isDiligentMapSceneryPlacementVisibleInView(const DiligentMapSceneryPlacement& placement,
                                           const openbus::rendering::Matrix4& view,
                                           const openbus::rendering::Matrix4& projection) {
    const auto isFiniteMatrix = [](const openbus::rendering::Matrix4& matrix) {
        return std::all_of(matrix.begin(), matrix.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (!isFiniteMatrix(placement.worldTransform) || !isFiniteMatrix(view) ||
        !isFiniteMatrix(projection) || !std::isfinite(placement.boundingRadius) ||
        placement.boundingRadius < 0.0) {
        throw std::invalid_argument(
            "Diligent secondary-view scenery bounds or matrices are invalid");
    }

    const openbus::rendering::Matrix4 modelView =
        openbus::rendering::multiplyMatrix4(view, placement.worldTransform);
    const std::array<double, 4> center =
        openbus::rendering::transformPoint(modelView, {0.0, 0.0, 0.0, 1.0});
    const double transformScale =
        (std::max)({std::hypot(placement.worldTransform[0], placement.worldTransform[1],
                               placement.worldTransform[2]),
                    std::hypot(placement.worldTransform[4], placement.worldTransform[5],
                               placement.worldTransform[6]),
                    std::hypot(placement.worldTransform[8], placement.worldTransform[9],
                               placement.worldTransform[10])});
    const double viewRadius = placement.boundingRadius * transformScale;
    if (!std::isfinite(transformScale) || !std::isfinite(viewRadius) ||
        !std::all_of(center.begin(), center.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent secondary-view scenery bounds exceed numeric range");
    }
    return !openbus::rendering::sphereOutsideFrustum(
        openbus::rendering::buildViewFrustum(projection), center, viewRadius);
}

inline std::vector<DiligentMapSceneryPlacementDrawResult> drawDiligentMapSceneryPlacementsInView(
    openbus::rendering::DiligentFrameRenderer& frameRenderer,
    std::span<const DiligentMapSceneryPlacement> placements,
    openbus::rendering::DiligentSceneryPass pass,
    openbus::rendering::DiligentTexturedMeshRenderer& meshes, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const openbus::rendering::Matrix4& view,
    const openbus::rendering::Matrix4& projection,
    std::span<const DiligentMapSceneryRuntimeState> runtimeStates = {},
    const openbus::rendering::DiligentLightingParameters& lighting = {},
    double minimumObjectViewportFraction = 0.0) {
    if (!runtimeStates.empty() && runtimeStates.size() != placements.size()) {
        throw std::invalid_argument(
            "Diligent secondary-view scenery runtime states do not match the placement count");
    }
    const std::vector<std::size_t> order =
        orderDiligentMapSceneryPlacements(placements, view, pass);
    std::vector<DiligentMapSceneryPlacementDrawResult> results;
    results.reserve(order.size());
    for (const std::size_t index : order) {
        const DiligentMapSceneryPlacement& placement = placements[index];
        if (placement.motionObjectId == 0) {
            throw std::invalid_argument(
                "Diligent map scenery placement requires a stable object identifier");
        }
        const DiligentMapSceneryRuntimeState runtime =
            runtimeStates.empty() ? DiligentMapSceneryRuntimeState{} : runtimeStates[index];
        if (!isDiligentMapSceneryPlacementVisibleInView(placement, view, projection)) {
            openbus::rendering::DiligentSceneryDrawResult drawResult;
            drawResult.culled = true;
            results.push_back({placement.motionObjectId,
                               DiligentMapSceneryPlacementDrawResult::Status::Culled,
                               std::move(drawResult)});
            continue;
        }
        const float visibilityFade = runtime.visibilityFade * diligentMapSceneryVisibilityFade(
                                                                  placement, view, projection,
                                                                  minimumObjectViewportFraction);
        if (visibilityFade == 0.0F) {
            openbus::rendering::DiligentSceneryDrawResult drawResult;
            drawResult.culled = true;
            results.push_back({placement.motionObjectId,
                               DiligentMapSceneryPlacementDrawResult::Status::Culled,
                               std::move(drawResult)});
            continue;
        }

        const auto* asset = frameRenderer.sceneryAsset(placement.configPath, placement.assetRoot);
        if (asset == nullptr) {
            results.push_back({placement.motionObjectId,
                               DiligentMapSceneryPlacementDrawResult::Status::AssetPending,
                               std::nullopt});
            continue;
        }

        std::vector<Diligent::ITextureView*> dynamicBatchTextures;
        std::vector<Diligent::ITextureView*> textBatchTextures;
        if (runtimeStates.empty() && !frameRenderer.synchronizeMapSceneryRuntimeTextures(
                                         placement.motionObjectId, *asset, placement.scriptRuntime,
                                         dynamicBatchTextures, textBatchTextures)) {
            results.push_back({placement.motionObjectId,
                               DiligentMapSceneryPlacementDrawResult::Status::RuntimeTexturePending,
                               std::nullopt});
            continue;
        }

        const openbus::rendering::Matrix4 modelView =
            openbus::rendering::multiplyMatrix4(view, placement.worldTransform);
        const openbus::rendering::Matrix4 modelViewProjection =
            openbus::rendering::multiplyMatrix4(projection, modelView);
        const auto drawResult =
            runtimeStates.empty()
                ? drawDiligentMapSceneryPlacementInViewWithResources(
                      frameRenderer, placement, *asset, pass, meshes, context, fallbackTexture,
                      modelViewProjection, modelView, runtime.seasonalBatchTextures,
                      dynamicBatchTextures, textBatchTextures, runtime.seasonalTreeTextures,
                      visibilityFade, runtime.weatherBrightness, runtime.reverseWinding, lighting)
                : drawDiligentMapSceneryPlacementInViewWithResources(
                      frameRenderer, placement, *asset, pass, meshes, context, fallbackTexture,
                      modelViewProjection, modelView, runtime.seasonalBatchTextures,
                      runtime.dynamicBatchTextures, runtime.textBatchTextures,
                      runtime.seasonalTreeTextures, visibilityFade, runtime.weatherBrightness,
                      runtime.reverseWinding, lighting);
        results.push_back({placement.motionObjectId,
                           DiligentMapSceneryPlacementDrawResult::Status::Submitted, drawResult});
    }
    return results;
}

// Call during scene submission with the view and camera position for that view.
template <typename MapSource>
void uploadDiligentMapLights(openbus::rendering::DiligentFrameRenderer& frameRenderer,
                             const MapSource& mapSource, const openbus::rendering::Matrix4& view,
                             const std::array<double, 3>& cameraWorldPosition, double strength,
                             std::size_t maximumLights) {
    const auto activeLights = mapSource.activeSceneryLights();
    frameRenderer.setMapLights(activeLights, view, cameraWorldPosition, strength, maximumLights);
}

// Mirrors the production map-light policy: light output fades with environment brightness and
// clears stale lights when the weather system has no brightness value.
template <typename MapSource>
void uploadDiligentMapLightsForEnvironment(openbus::rendering::DiligentFrameRenderer& frameRenderer,
                                           const MapSource& mapSource,
                                           const openbus::rendering::Matrix4& view,
                                           const std::array<double, 3>& cameraWorldPosition,
                                           std::optional<double> environmentBrightness,
                                           std::size_t maximumLights) {
    uploadDiligentMapLights(
        frameRenderer, mapSource, view, cameraWorldPosition,
        openbus::rendering::mapLightStrengthForEnvironmentBrightness(environmentBrightness),
        maximumLights);
}

// Configure the caller-owned renderer for an auxiliary view without changing main-view lights.
template <typename MapSource>
std::size_t uploadDiligentMapLightsInView(openbus::rendering::DiligentTexturedMeshRenderer& meshes,
                                          Diligent::IDeviceContext& context,
                                          const MapSource& mapSource,
                                          const openbus::rendering::Matrix4& view,
                                          const std::array<double, 3>& cameraWorldPosition,
                                          double strength, std::size_t maximumLights) {
    const auto activeLights = mapSource.activeSceneryLights();
    const auto viewLights = openbus::rendering::buildDiligentMapPointLights(
        activeLights, view, cameraWorldPosition, strength, maximumLights);
    meshes.setPointLights(context, viewLights);
    return viewLights.size();
}

template <typename MapSource>
std::size_t uploadDiligentMapLightsInViewForEnvironment(
    openbus::rendering::DiligentTexturedMeshRenderer& meshes, Diligent::IDeviceContext& context,
    const MapSource& mapSource, const openbus::rendering::Matrix4& view,
    const std::array<double, 3>& cameraWorldPosition, std::optional<double> environmentBrightness,
    std::size_t maximumLights) {
    return uploadDiligentMapLightsInView(
        meshes, context, mapSource, view, cameraWorldPosition,
        openbus::rendering::mapLightStrengthForEnvironmentBrightness(environmentBrightness),
        maximumLights);
}

} // namespace openbus::app
