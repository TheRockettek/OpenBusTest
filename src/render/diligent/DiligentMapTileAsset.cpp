#include "render/diligent/DiligentMapTileAsset.h"

#include "render/diligent/DiligentTexture2D.h"
#include "render/MatrixMath.h"

#include <Graphics/GraphicsEngine/interface/RenderDevice.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace openbus::rendering {
namespace {

using TextureCache = std::unordered_map<const Image*, std::shared_ptr<DiligentTexture2D>>;
using TexturePathCache =
    std::unordered_map<std::filesystem::path, std::shared_ptr<DiligentTexture2D>>;

std::shared_ptr<DiligentTexture2D> uploadTexture(Diligent::IRenderDevice& device,
                                                 const std::shared_ptr<const Image>& image,
                                                 TextureCache& textureCache, bool generateMipmaps) {
    if (!image) {
        return {};
    }
    const auto found = textureCache.find(image.get());
    if (found != textureCache.end()) {
        return found->second;
    }
    auto texture = std::make_shared<DiligentTexture2D>(device, *image, generateMipmaps);
    textureCache.emplace(image.get(), texture);
    return texture;
}

std::shared_ptr<DiligentTexture2D> uploadTexture(Diligent::IRenderDevice& device,
                                                 const std::filesystem::path& path,
                                                 TexturePathCache& textureCache,
                                                 bool generateMipmaps) {
    if (path.empty()) {
        return {};
    }
    const std::filesystem::path normalizedPath = path.lexically_normal();
    const auto found = textureCache.find(normalizedPath);
    if (found != textureCache.end()) {
        return found->second;
    }
    auto texture = DiligentTexture2D::load(device, normalizedPath, generateMipmaps);
    std::shared_ptr<DiligentTexture2D> sharedTexture(std::move(texture));
    textureCache.emplace(normalizedPath, sharedTexture);
    return sharedTexture;
}

std::shared_ptr<DiligentTexture2D>
uploadTexture(Diligent::IRenderDevice& device, const std::shared_ptr<const Image>& image,
              const std::filesystem::path& path, TextureCache& imageCache,
              TexturePathCache& pathCache, bool generateMipmaps) {
    if (image && !path.empty()) {
        throw std::invalid_argument(
            "Diligent map texture source cannot be both decoded and file-backed");
    }
    return image ? uploadTexture(device, image, imageCache, generateMipmaps)
                 : uploadTexture(device, path, pathCache, generateMipmaps);
}

std::shared_ptr<DiligentTexturedMesh> uploadMesh(Diligent::IRenderDevice& device,
                                                 std::span<const ModelVertex> vertices,
                                                 std::span<const std::uint32_t> indices) {
    if (vertices.empty()) {
        if (!indices.empty()) {
            throw std::invalid_argument("Diligent map geometry has indices but no vertices");
        }
        return {};
    }
    if (indices.empty()) {
        return std::make_shared<DiligentTexturedMesh>(device, vertices);
    }
    return std::make_shared<DiligentTexturedMesh>(device, vertices, indices);
}

DiligentAlphaMode roadAlphaMode(int alphaMode) {
    switch (alphaMode) {
    case 0:
        return DiligentAlphaMode::Opaque;
    case 1:
        return DiligentAlphaMode::Cutout;
    case 2:
        return DiligentAlphaMode::Blend;
    default:
        throw std::invalid_argument("Diligent map road has an unsupported alpha mode");
    }
}

void uploadWaterAndRoads(Diligent::IRenderDevice& device, int tileX, int tileY,
                         const openbus::map::MapWaterMesh* water,
                         const DiligentMapWaterSource& waterSource,
                         std::span<const DiligentMapRoadSource> roads, TextureCache& textureCache,
                         TexturePathCache& pathTextureCache,
                         std::unique_ptr<DiligentMapMeshBatch>& waterBatch,
                         std::vector<DiligentMapMeshBatch>& roadBatches, bool generateMipmaps) {
    if (water != nullptr) {
        waterBatch = std::make_unique<DiligentMapMeshBatch>();
        waterBatch->tileX = tileX;
        waterBatch->tileY = tileY;
        waterBatch->minimumHeight = water->minimumHeight;
        waterBatch->maximumHeight = water->maximumHeight;
        waterBatch->meanHeight = water->meanHeight;
        waterBatch->mesh = uploadMesh(device, water->vertices, water->indices);
        waterBatch->texture = uploadTexture(device, waterSource.texture, waterSource.texturePath,
                                            textureCache, pathTextureCache, generateMipmaps);
        waterBatch->normalMapTexture =
            uploadTexture(device, waterSource.normalMap, waterSource.normalMapPath, textureCache,
                          pathTextureCache, generateMipmaps);
        waterBatch->environmentMapTexture =
            uploadTexture(device, waterSource.environmentMap, waterSource.environmentMapPath,
                          textureCache, pathTextureCache, generateMipmaps);
        waterBatch->material = waterSource.material;
        if (waterBatch->environmentMapTexture && waterBatch->material.environmentMap == nullptr) {
            waterBatch->material.environmentMap =
                &waterBatch->environmentMapTexture->shaderResourceView();
        }
        waterBatch->textured = static_cast<bool>(waterBatch->texture);
    } else if (waterSource.texture || waterSource.normalMap || waterSource.environmentMap ||
               !waterSource.texturePath.empty() || !waterSource.normalMapPath.empty() ||
               !waterSource.environmentMapPath.empty()) {
        throw std::invalid_argument("Diligent map water texture has no water geometry");
    }

    roadBatches.reserve(roads.size());
    for (const DiligentMapRoadSource& source : roads) {
        if (source.geometry == nullptr) {
            throw std::invalid_argument("Diligent map road source has no geometry");
        }
        const openbus::map::RoadMeshGeometry& geometry = *source.geometry;
        if (geometry.tileX != tileX || geometry.tileY != tileY) {
            throw std::invalid_argument("Diligent map road geometry belongs to another tile");
        }
        DiligentMapMeshBatch batch;
        batch.tileX = geometry.tileX;
        batch.tileY = geometry.tileY;
        batch.texturePath = geometry.texturePath;
        batch.minimumHeight = geometry.minZ;
        batch.maximumHeight = geometry.maxZ;
        if (std::isfinite(geometry.minX) && std::isfinite(geometry.maxX) &&
            std::isfinite(geometry.minY) && std::isfinite(geometry.maxY)) {
            batch.sortCenter = {(geometry.minX + geometry.maxX) * 0.5,
                                (geometry.minY + geometry.maxY) * 0.5,
                                (geometry.minZ + geometry.maxZ) * 0.5};
            batch.hasSortCenter =
                std::all_of(batch.sortCenter.begin(), batch.sortCenter.end(),
                            [](double coordinate) { return std::isfinite(coordinate); });
        }
        batch.textured = geometry.textured;
        batch.supportsMoisture = geometry.supportsMoisture;
        batch.weatherClassified = geometry.weatherClassified;
        batch.alphaMode = geometry.alphaMode;
        batch.mesh = uploadMesh(device, geometry.vertices, geometry.indices);
        if (!batch.mesh) {
            throw std::invalid_argument("Diligent map road geometry contains no vertices");
        }
        std::filesystem::path texturePath = source.texturePath;
        if (!source.texture && texturePath.empty() && geometry.textured) {
            texturePath = geometry.texturePath;
        }
        batch.texture = uploadTexture(device, source.texture, texturePath, textureCache,
                                      pathTextureCache, generateMipmaps);
        if (geometry.textured && !batch.texture) {
            throw std::invalid_argument("Textured Diligent map road has no usable texture source");
        }
        batch.material = source.material;
        batch.material.alphaMode = roadAlphaMode(geometry.alphaMode);
        roadBatches.push_back(std::move(batch));
    }
}

} // namespace

DiligentMaterialState applyDiligentRoadWeather(DiligentMaterialState material,
                                               bool weatherClassified, bool supportsMoisture,
                                               const DiligentMapWeather& weather) {
    const auto validFraction = [](float value) {
        return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
    };
    if (!validFraction(weather.groundSnow) || !validFraction(weather.roadSnow) ||
        !validFraction(weather.roadWater)) {
        throw std::invalid_argument("Diligent map weather values must be finite fractions");
    }

    const float snow = weatherClassified ? weather.roadSnow : 0.0F;
    material.snowCoverage = 0.85F * snow;
    const float wetness = weather.roadWater * (1.0F - 0.85F * snow);
    if (supportsMoisture && wetness > 0.0F) {
        const float albedo = 1.0F - 0.25F * wetness;
        for (std::size_t channel = 0; channel < 3; ++channel) {
            material.baseColor[channel] *= albedo;
            material.specularColor[channel] = 0.18F * wetness;
        }
        material.specularPower = 48.0F;
    }
    return material;
}

bool DiligentMapMeshBatch::draw(
    DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
    const DiligentMaterialState* materialOverride, Diligent::ITextureView* textureOverride,
    const DiligentLightingParameters& lighting, const DiligentMotionParameters* motion,
    Diligent::ITextureView* normalMap) const {
    if (!mesh) {
        return false;
    }
    Diligent::ITextureView& selectedTexture = textureOverride != nullptr ? *textureOverride
                                              : texture ? texture->shaderResourceView()
                                                        : fallbackTexture;
    DiligentMaterialState selectedMaterial =
        materialOverride != nullptr ? *materialOverride : material;
    if (selectedMaterial.transMap == nullptr && transMapTexture) {
        selectedMaterial.transMap = &transMapTexture->shaderResourceView();
    }
    if (selectedMaterial.detailMap == nullptr && detailMapTexture) {
        selectedMaterial.detailMap = &detailMapTexture->shaderResourceView();
    }
    if (selectedMaterial.environmentMap == nullptr && environmentMapTexture) {
        selectedMaterial.environmentMap = &environmentMapTexture->shaderResourceView();
    }
    Diligent::ITextureView* selectedNormalMap = normalMap != nullptr ? normalMap
                                                : normalMapTexture
                                                    ? &normalMapTexture->shaderResourceView()
                                                    : nullptr;
    if (motion != nullptr) {
        renderer.drawMotion(context, *mesh, selectedTexture, modelViewProjection, *motion,
                            selectedMaterial, lighting, selectedNormalMap);
    } else {
        renderer.draw(context, *mesh, selectedTexture, modelViewProjection, selectedMaterial,
                      lighting, selectedNormalMap);
    }
    return true;
}

DiligentMapTileAsset::DiligentMapTileAsset(Diligent::IRenderDevice& device, int tileX, int tileY,
                                           const openbus::map::MapTerrainMesh* terrain,
                                           std::shared_ptr<const Image> terrainTexture,
                                           const openbus::map::MapWaterMesh* water,
                                           DiligentMapWaterSource waterSource,
                                           std::span<const DiligentMapRoadSource> roads,
                                           bool generateMipmaps)
    : tileX_(tileX), tileY_(tileY) {
    TextureCache textureCache;
    TexturePathCache pathTextureCache;
    if (terrain != nullptr) {
        DiligentMapMeshBatch batch;
        batch.tileX = tileX_;
        batch.tileY = tileY_;
        batch.minimumHeight = terrain->minimumHeight;
        batch.maximumHeight = terrain->maximumHeight;
        batch.mesh = uploadMesh(device, terrain->vertices, terrain->indices);
        batch.texture = uploadTexture(device, terrainTexture, textureCache, generateMipmaps);
        batch.textured = static_cast<bool>(batch.texture);
        terrainLayers_.push_back(std::move(batch));
    } else if (terrainTexture) {
        throw std::invalid_argument("Diligent map terrain texture has no terrain geometry");
    }
    uploadWaterAndRoads(device, tileX_, tileY_, water, waterSource, roads, textureCache,
                        pathTextureCache, water_, roads_, generateMipmaps);
}

DiligentMapTileAsset::DiligentMapTileAsset(
    Diligent::IRenderDevice& device, int tileX, int tileY,
    const openbus::map::MapTerrainMesh* terrain,
    std::span<const DiligentMapTerrainLayerSource> terrainLayers,
    const openbus::map::MapWaterMesh* water, DiligentMapWaterSource waterSource,
    std::span<const DiligentMapRoadSource> roads, bool generateMipmaps)
    : tileX_(tileX), tileY_(tileY) {
    TextureCache textureCache;
    TexturePathCache pathTextureCache;
    if (terrain == nullptr && !terrainLayers.empty()) {
        throw std::invalid_argument("Diligent map terrain layers have no terrain geometry");
    }
    if (terrain != nullptr) {
        const std::shared_ptr<DiligentTexturedMesh> mesh =
            uploadMesh(device, terrain->vertices, terrain->indices);
        const std::size_t layerCount = std::max<std::size_t>(terrainLayers.size(), 1);
        terrainLayers_.reserve(layerCount);
        for (std::size_t index = 0; index < layerCount; ++index) {
            const DiligentMapTerrainLayerSource* source =
                terrainLayers.empty() ? nullptr : &terrainLayers[index];
            DiligentMapMeshBatch batch;
            batch.tileX = tileX_;
            batch.tileY = tileY_;
            batch.minimumHeight = terrain->minimumHeight;
            batch.maximumHeight = terrain->maximumHeight;
            batch.mesh = mesh;
            if (source != nullptr) {
                batch.texture = uploadTexture(device, source->texture, source->texturePath,
                                              textureCache, pathTextureCache, generateMipmaps);
                batch.transMapTexture =
                    uploadTexture(device, source->transMap, source->transMapPath, textureCache,
                                  pathTextureCache, generateMipmaps);
                batch.detailMapTexture =
                    uploadTexture(device, source->detailMap, source->detailMapPath, textureCache,
                                  pathTextureCache, generateMipmaps);
                batch.material = source->material;
                if (batch.transMapTexture) {
                    batch.material.transMap = &batch.transMapTexture->shaderResourceView();
                }
                if (batch.detailMapTexture) {
                    batch.material.detailMap = &batch.detailMapTexture->shaderResourceView();
                }
            }
            batch.textured = static_cast<bool>(batch.texture);
            terrainLayers_.push_back(std::move(batch));
        }
    }
    uploadWaterAndRoads(device, tileX_, tileY_, water, waterSource, roads, textureCache,
                        pathTextureCache, water_, roads_, generateMipmaps);
}

DiligentMapTileAsset::~DiligentMapTileAsset() = default;

int DiligentMapTileAsset::tileX() const noexcept {
    return tileX_;
}

int DiligentMapTileAsset::tileY() const noexcept {
    return tileY_;
}

Matrix4 DiligentMapTileAsset::worldTransform(double tileSize) const {
    if (!std::isfinite(tileSize) || tileSize <= 0.0) {
        throw std::invalid_argument("Diligent map tile size must be finite and positive");
    }
    const double originX = static_cast<double>(tileX_) * tileSize;
    const double originY = static_cast<double>(tileY_) * tileSize;
    if (!std::isfinite(originX) || !std::isfinite(originY)) {
        throw std::invalid_argument("Diligent map tile origin must be finite");
    }
    Matrix4 transform = {1.0, 0.0, 0.0, 0.0, 0.0,     1.0,     0.0, 0.0,
                         0.0, 0.0, 1.0, 0.0, originX, originY, 0.0, 1.0};
    return transform;
}

Matrix4 DiligentMapTileAsset::modelViewProjection(const Matrix4& viewProjection,
                                                  double tileSize) const {
    return multiplyMatrix4(viewProjection, worldTransform(tileSize));
}

DiligentMotionParameters DiligentMapTileAsset::motionParameters(const MotionDataFrame& frame,
                                                                MotionHistory& history,
                                                                double tileSize) const {
    const Matrix4 currentModelView = multiplyMatrix4(frame.currentView, worldTransform(tileSize));
    return DiligentMotionParameters::fromHistory(frame,
                                                 history.previousModelView(currentModelView));
}

const DiligentMapMeshBatch* DiligentMapTileAsset::terrain() const noexcept {
    return terrainLayers_.empty() ? nullptr : &terrainLayers_.front();
}

std::span<const DiligentMapMeshBatch> DiligentMapTileAsset::terrainLayers() const noexcept {
    return terrainLayers_;
}

const DiligentMapMeshBatch* DiligentMapTileAsset::water() const noexcept {
    return water_.get();
}

std::span<const DiligentMapMeshBatch> DiligentMapTileAsset::roads() const noexcept {
    return roads_;
}

} // namespace openbus::rendering
