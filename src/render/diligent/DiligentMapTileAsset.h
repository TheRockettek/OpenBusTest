#pragma once

#include "assets/TextureLoader.h"
#include "map/MapRoadRenderGeometry.h"
#include "map/MapSurfaceGeometry.h"
#include "render/diligent/DiligentTexturedMesh.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace Diligent {
struct IRenderDevice;
}

namespace openbus::rendering {

class DiligentTexture2D;

enum class DiligentMapRoadPass : std::uint8_t {
    Opaque,
    Cutout,
    Transparent,
};

struct DiligentMapWeather {
    float groundSnow = 0.0F;
    float roadSnow = 0.0F;
    float roadWater = 0.0F;
};

struct DiligentMapRoadSource {
    const openbus::map::RoadMeshGeometry* geometry = nullptr;
    std::shared_ptr<const Image> texture;
    DiligentMaterialState material;
    std::filesystem::path texturePath;
};

[[nodiscard]] DiligentMaterialState applyDiligentRoadWeather(DiligentMaterialState material,
                                                             bool weatherClassified,
                                                             bool supportsMoisture,
                                                             const DiligentMapWeather& weather);

struct DiligentMapWaterSource {
    std::shared_ptr<const Image> texture;
    std::shared_ptr<const Image> normalMap;
    std::shared_ptr<const Image> environmentMap;
    std::filesystem::path texturePath;
    std::filesystem::path normalMapPath;
    std::filesystem::path environmentMapPath;
    DiligentMaterialState material;

    DiligentMapWaterSource() = default;
    DiligentMapWaterSource(std::shared_ptr<const Image> sourceTexture)
        : texture(std::move(sourceTexture)) {}
    DiligentMapWaterSource(std::shared_ptr<Image> sourceTexture)
        : texture(std::move(sourceTexture)) {}
};

struct DiligentMapTerrainLayerSource {
    std::shared_ptr<const Image> texture;
    std::shared_ptr<const Image> transMap;
    std::shared_ptr<const Image> detailMap;
    DiligentMaterialState material;
    std::filesystem::path texturePath;
    std::filesystem::path transMapPath;
    std::filesystem::path detailMapPath;
};

struct DiligentMapMeshBatch {
    int tileX = 0;
    int tileY = 0;
    std::filesystem::path texturePath;
    double minimumHeight = 0.0;
    double maximumHeight = 0.0;
    double meanHeight = 0.0;
    std::array<double, 3> sortCenter = {};
    bool hasSortCenter = false;
    bool textured = false;
    bool supportsMoisture = false;
    bool weatherClassified = false;
    int alphaMode = 0;
    std::shared_ptr<DiligentTexturedMesh> mesh;
    std::shared_ptr<DiligentTexture2D> texture;
    std::shared_ptr<DiligentTexture2D> normalMapTexture;
    std::shared_ptr<DiligentTexture2D> environmentMapTexture;
    std::shared_ptr<DiligentTexture2D> transMapTexture;
    std::shared_ptr<DiligentTexture2D> detailMapTexture;
    DiligentMaterialState material;

    // The caller supplies tile/world transforms, pass ordering, dynamic material state,
    // and temporal history. The fallback is used if neither an uploaded nor override texture
    // exists.
    bool draw(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
              Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
              const DiligentMaterialState* materialOverride = nullptr,
              Diligent::ITextureView* textureOverride = nullptr,
              const DiligentLightingParameters& lighting = {},
              const DiligentMotionParameters* motion = nullptr,
              Diligent::ITextureView* normalMap = nullptr) const;
};

// Uploads the exact CPU-prepared terrain, water, and road geometry used by MapRenderer.
// It preserves source road order and shares texture uploads for identical decoded images.
class DiligentMapTileAsset final {
  public:
    DiligentMapTileAsset(Diligent::IRenderDevice& device, int tileX, int tileY,
                         const openbus::map::MapTerrainMesh* terrain,
                         std::shared_ptr<const Image> terrainTexture,
                         const openbus::map::MapWaterMesh* water,
                         DiligentMapWaterSource waterSource,
                         std::span<const DiligentMapRoadSource> roads,
                         bool generateMipmaps = false);
    DiligentMapTileAsset(Diligent::IRenderDevice& device, int tileX, int tileY,
                         const openbus::map::MapTerrainMesh* terrain,
                         std::span<const DiligentMapTerrainLayerSource> terrainLayers,
                         const openbus::map::MapWaterMesh* water,
                         DiligentMapWaterSource waterSource,
                         std::span<const DiligentMapRoadSource> roads,
                         bool generateMipmaps = false);
    ~DiligentMapTileAsset();
    DiligentMapTileAsset(const DiligentMapTileAsset&) = delete;
    DiligentMapTileAsset& operator=(const DiligentMapTileAsset&) = delete;

    [[nodiscard]] int tileX() const noexcept;
    [[nodiscard]] int tileY() const noexcept;
    // Terrain, water, and roads are tile-local; translate them to the production map grid.
    [[nodiscard]] Matrix4 worldTransform(double tileSize) const;
    [[nodiscard]] Matrix4 modelViewProjection(const Matrix4& viewProjection, double tileSize) const;
    [[nodiscard]] DiligentMotionParameters
    motionParameters(const MotionDataFrame& frame, MotionHistory& history, double tileSize) const;
    [[nodiscard]] const DiligentMapMeshBatch* terrain() const noexcept;
    [[nodiscard]] std::span<const DiligentMapMeshBatch> terrainLayers() const noexcept;
    [[nodiscard]] const DiligentMapMeshBatch* water() const noexcept;
    [[nodiscard]] std::span<const DiligentMapMeshBatch> roads() const noexcept;

  private:
    int tileX_ = 0;
    int tileY_ = 0;
    std::vector<DiligentMapMeshBatch> terrainLayers_;
    std::unique_ptr<DiligentMapMeshBatch> water_;
    std::vector<DiligentMapMeshBatch> roads_;
};

} // namespace openbus::rendering
