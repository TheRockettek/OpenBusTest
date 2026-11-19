#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "app/DiligentMapBridge.h"

#include "render/diligent/DiligentMapTileAsset.h"
#include "render/diligent/DiligentMapTileSet.h"
#include "render/diligent/DiligentTexture2D.h"

#include <cmath>
#include <stdexcept>

namespace openbus::app {

std::vector<openbus::rendering::DiligentShadowCaster>
buildDiligentMapTileShadowCasters(const openbus::rendering::DiligentMapTileSet& tiles,
                                  double tileSize, const openbus::rendering::Matrix4& view,
                                  const openbus::rendering::Matrix4& lightClipFromView) {
    using namespace openbus::rendering;
    const auto validMatrix = [](const Matrix4& matrix) {
        return std::all_of(matrix.begin(), matrix.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (!std::isfinite(tileSize) || tileSize <= 0.0 || !validMatrix(view) ||
        !validMatrix(lightClipFromView)) {
        throw std::invalid_argument("Invalid Diligent map shadow view or tile size");
    }

    std::vector<DiligentShadowCaster> casters;
    const auto appendBatch = [&](const DiligentMapMeshBatch& batch,
                                 const Matrix4& lightClipFromTile) {
        if (!batch.mesh) {
            return;
        }
        float alphaCutoff = 0.0F;
        switch (batch.material.alphaMode) {
        case DiligentAlphaMode::Opaque:
            break;
        case DiligentAlphaMode::Cutout:
            alphaCutoff = 0.5F;
            break;
        case DiligentAlphaMode::NearOpaque:
            alphaCutoff = 0.99F;
            break;
        case DiligentAlphaMode::Blend:
        case DiligentAlphaMode::NearOpaqueBlend:
            return;
        }
        DiligentShadowCaster caster;
        caster.mesh = batch.mesh.get();
        caster.modelViewProjection = lightClipFromTile;
        caster.alphaTexture =
            alphaCutoff > 0.0F && batch.texture ? &batch.texture->shaderResourceView() : nullptr;
        caster.alphaCutoff = caster.alphaTexture != nullptr ? alphaCutoff : 0.0F;
        caster.textureTransform = {batch.material.textureScale[0], batch.material.textureScale[1],
                                   batch.material.textureOffset[0],
                                   batch.material.textureOffset[1]};
        casters.push_back(caster);
    };

    for (const DiligentMapTileAsset* tile : tiles.activeTiles()) {
        if (tile == nullptr) {
            throw std::logic_error("Diligent map shadow tile set contains a null active tile");
        }
        const Matrix4 modelView = multiplyMatrix4(view, tile->worldTransform(tileSize));
        const Matrix4 lightClipFromTile = multiplyMatrix4(lightClipFromView, modelView);
        for (const DiligentMapMeshBatch& terrain : tile->terrainLayers()) {
            appendBatch(terrain, lightClipFromTile);
        }
        for (const DiligentMapMeshBatch& road : tile->roads()) {
            appendBatch(road, lightClipFromTile);
        }
    }
    return casters;
}

DiligentMapMainViewDrawResult
drawDiligentMapMainView(openbus::rendering::DiligentFrameRenderer& frameRenderer,
                        Diligent::ITextureView& fallbackTexture,
                        const openbus::rendering::DiligentLightingParameters& lighting,
                        DiligentMapMainViewPass pass,
                        const openbus::rendering::DiligentMapWeather* weather) {
    if (pass != DiligentMapMainViewPass::All && pass != DiligentMapMainViewPass::Opaque &&
        pass != DiligentMapMainViewPass::Transparent) {
        throw std::invalid_argument("Invalid Diligent main-view map pass");
    }

    using openbus::rendering::DiligentMapRoadPass;

    const auto& frame = frameRenderer.frameData();
    const auto viewProjection =
        openbus::rendering::multiplyMatrix4(frame.currentProjection, frame.currentView);
    DiligentMapMainViewDrawResult result;
    if (pass == DiligentMapMainViewPass::All || pass == DiligentMapMainViewPass::Opaque) {
        result.terrainBatches = frameRenderer.drawMapTerrain(fallbackTexture, viewProjection,
                                                             nullptr, lighting, nullptr, weather);
        result.opaqueRoadBatches =
            frameRenderer.drawMapRoads(DiligentMapRoadPass::Opaque, fallbackTexture, viewProjection,
                                       nullptr, lighting, nullptr, weather);
        result.cutoutRoadBatches =
            frameRenderer.drawMapRoads(DiligentMapRoadPass::Cutout, fallbackTexture, viewProjection,
                                       nullptr, lighting, nullptr, weather);
    }
    if (pass == DiligentMapMainViewPass::All || pass == DiligentMapMainViewPass::Transparent) {
        result.waterBatches =
            frameRenderer.drawMapWater(fallbackTexture, viewProjection, nullptr, lighting);
        result.transparentRoadBatches =
            frameRenderer.drawMapRoads(DiligentMapRoadPass::Transparent, fallbackTexture,
                                       viewProjection, nullptr, lighting, nullptr, weather);
    }
    return result;
}

std::size_t drawDiligentMapTilesInView(
    const openbus::rendering::DiligentMapTileSet& tiles,
    openbus::rendering::DiligentTexturedMeshRenderer& meshes, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const openbus::rendering::Matrix4& viewProjection,
    DiligentMapTileViewPass pass, const openbus::rendering::DiligentMaterialState* materialOverride,
    const openbus::rendering::DiligentLightingParameters& lighting,
    const openbus::rendering::DiligentMapWeather* weather) {
    using openbus::rendering::DiligentMapRoadPass;
    switch (pass) {
    case DiligentMapTileViewPass::Terrain:
        return tiles.drawTerrain(meshes, context, fallbackTexture, viewProjection, materialOverride,
                                 lighting, nullptr, nullptr, nullptr, weather);
    case DiligentMapTileViewPass::OpaqueRoads:
        return tiles.drawRoads(DiligentMapRoadPass::Opaque, meshes, context, fallbackTexture,
                               viewProjection, materialOverride, lighting, nullptr, nullptr,
                               nullptr, weather);
    case DiligentMapTileViewPass::CutoutRoads:
        return tiles.drawRoads(DiligentMapRoadPass::Cutout, meshes, context, fallbackTexture,
                               viewProjection, materialOverride, lighting, nullptr, nullptr,
                               nullptr, weather);
    case DiligentMapTileViewPass::Water:
        return tiles.drawWater(meshes, context, fallbackTexture, viewProjection, materialOverride,
                               lighting);
    case DiligentMapTileViewPass::TransparentRoads:
        return tiles.drawRoads(DiligentMapRoadPass::Transparent, meshes, context, fallbackTexture,
                               viewProjection, materialOverride, lighting, nullptr, nullptr,
                               nullptr, weather);
    }
    throw std::invalid_argument("Invalid Diligent map-tile view pass");
}

} // namespace openbus::app
