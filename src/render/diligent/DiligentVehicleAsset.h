#pragma once

#include "assets/TextureLoader.h"
#include "config/ModelConfigTypes.h"
#include "render/diligent/DiligentTexturedMesh.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <variant>
#include <vector>

namespace Diligent {
struct IRenderDevice;
}

namespace gli {
class texture;
}

namespace openbus::rendering {

class DiligentTexture2D;
class DiligentTexture2DArray;
class DiligentRuntimeTexture2D;

using DiligentVehicleTextureSource =
    std::variant<std::monostate, std::shared_ptr<const Image>, std::shared_ptr<const CompressedDds>,
                 std::shared_ptr<const gli::texture>>;

struct DiligentVehicleAuxiliaryTextureSources {
    DiligentVehicleTextureSource lightMap;
    DiligentVehicleTextureSource nightMap;
    DiligentVehicleTextureSource transMap;
    DiligentVehicleTextureSource detailMap;
    DiligentVehicleTextureSource freeMap;
    DiligentVehicleTextureSource textMap;
    DiligentVehicleTextureSource environmentMap;
    DiligentVehicleTextureSource normalMap;
};

struct DiligentVehicleAuxiliaryTextures {
    std::shared_ptr<DiligentTexture2D> lightMap;
    std::shared_ptr<DiligentTexture2D> nightMap;
    std::shared_ptr<DiligentTexture2D> transMap;
    std::shared_ptr<DiligentTexture2D> detailMap;
    std::shared_ptr<DiligentTexture2D> freeMap;
    std::shared_ptr<DiligentTexture2D> textMap;
    std::shared_ptr<DiligentTexture2D> environmentMap;
    std::shared_ptr<DiligentTexture2D> normalMap;
};

struct DiligentVehicleBatchSource {
    int materialIndex = 0;
    std::span<const ModelVertex> vertices;
    DiligentVehicleTextureSource texture;
    DiligentMaterialState material;
    std::span<const std::shared_ptr<const Image>> textureArrayLayers;
    DiligentVehicleAuxiliaryTextureSources auxiliaryTextures;
    std::shared_ptr<const gli::texture> compressedTextureArray;
    std::size_t animationGroupIndex = 0;
    std::size_t sourceBatchIndex = 0;
};

struct DiligentVehicleSubmesh {
    int materialIndex = 0;
    std::size_t sourceBatchIndex = 0;
    std::array<double, 3> localCenter = {};
    std::unique_ptr<DiligentTexturedMesh> mesh;
    std::shared_ptr<DiligentTexture2D> texture;
    std::shared_ptr<DiligentTexture2DArray> textureArray;
    DiligentVehicleAuxiliaryTextures auxiliaryTextures;
    DiligentMaterialState material;
    std::size_t animationGroupIndex = 0;

    // Runtime script textures can be supplied through the optional material override.
    void draw(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
              Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
              const DiligentMaterialState* materialOverride = nullptr,
              Diligent::ITextureView* baseTextureOverride = nullptr,
              const DiligentLightingParameters& lighting = {},
              const DiligentMotionParameters* motion = nullptr,
              Diligent::ITextureView* normalMap = nullptr) const;
};

struct DiligentVehicleMaterialOverride {
    int materialIndex = 0;
    const DiligentMaterialState* material = nullptr;
    Diligent::ITextureView* baseTexture = nullptr;
    const DiligentRuntimeTexture2D* runtimeTexture = nullptr;
    const DiligentRuntimeTexture2D* freeMapTexture = nullptr;
    const DiligentRuntimeTexture2D* textMapTexture = nullptr;
    std::optional<std::size_t> animationGroupIndex;
    std::optional<std::size_t> sourceBatchIndex;
};

struct DiligentVehicleGroupTransform {
    std::size_t animationGroupIndex = 0;
    Matrix4 modelViewProjection = {};
    Matrix4 modelView = {};
    bool visible = true;
    // Omitted values inherit the draw call's lighting and motion state.
    std::optional<DiligentLightingParameters> lighting;
    std::optional<DiligentMotionParameters> motion;

    [[nodiscard]] static DiligentVehicleGroupTransform fromMotionHistory(
        std::size_t animationGroupIndex, const MotionDataFrame& frame, MotionHistory& history,
        MotionObjectKey objectKey, const Matrix4& currentModelView,
        bool geometryHistoryValid = true,
        double maximumTranslationDeltaMeters = std::numeric_limits<double>::infinity());
    [[nodiscard]] static DiligentVehicleGroupTransform fromAnimationsAndHistory(
        std::size_t animationGroupIndex, const MotionDataFrame& frame, MotionHistory& history,
        MotionObjectKey objectKey, const Matrix4& parentModelView,
        std::span<const ModelAnimation> animations, std::span<const double> animationAmounts,
        bool geometryHistoryValid = true,
        double maximumTranslationDeltaMeters = std::numeric_limits<double>::infinity());
};

enum class DiligentVehicleRenderPass : std::uint8_t {
    All,
    Opaque,
    Transparent,
};

struct DiligentVehicleAnimationGroupInput {
    std::size_t animationGroupIndex = 0;
    std::optional<std::size_t> parentAnimationGroupIndex;
    std::span<const ModelAnimation> animations;
    std::span<const double> animationAmounts;
    bool geometryHistoryValid = true;
};

// Composes a vehicle's animation-group hierarchy and records stable per-group motion history.
// Parent groups must be included in the input, even when they have no animated submeshes.
std::vector<DiligentVehicleGroupTransform> buildDiligentVehicleGroupTransforms(
    std::span<const DiligentVehicleAnimationGroupInput> groups, const MotionDataFrame& frame,
    MotionHistory& history, std::uint64_t vehicleObjectId, const Matrix4& parentModelView,
    double maximumTranslationDeltaMeters = std::numeric_limits<double>::infinity());

// Uploads production-prepared vehicle triangle batches while preserving their source order.
// The caller retains animation state and supplies transforms/history per animation group.
class DiligentVehicleAsset final {
  public:
    DiligentVehicleAsset(Diligent::IRenderDevice& device,
                         std::span<const DiligentVehicleBatchSource> batches,
                         bool generateMipmaps = false);

    [[nodiscard]] std::span<const DiligentVehicleSubmesh> submeshes() const noexcept;

    [[nodiscard]] std::vector<DiligentShadowCaster>
    shadowCasters(const Matrix4& lightClipFromView, const Matrix4& modelView,
                  std::span<const DiligentVehicleGroupTransform> groupTransforms = {},
                  std::span<const DiligentVehicleMaterialOverride> materialOverrides = {}) const;

    // Opaque batches retain source order; blended batches are submitted back-to-front.
    // modelView must include the same object transform used by modelViewProjection.
    void draw(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
              Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
              const Matrix4& modelView, const DiligentLightingParameters& lighting = {},
              const DiligentMotionParameters* motion = nullptr,
              std::span<const DiligentVehicleMaterialOverride> materialOverrides = {},
              Diligent::ITextureView* normalMap = nullptr,
              std::span<const DiligentVehicleGroupTransform> groupTransforms = {},
              DiligentVehicleRenderPass pass = DiligentVehicleRenderPass::All) const;
    [[nodiscard]] std::size_t
    drawPass(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
             Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
             const Matrix4& modelView, DiligentVehicleRenderPass pass,
             const DiligentLightingParameters& lighting = {},
             const DiligentMotionParameters* motion = nullptr,
             std::span<const DiligentVehicleMaterialOverride> materialOverrides = {},
             Diligent::ITextureView* normalMap = nullptr,
             std::span<const DiligentVehicleGroupTransform> groupTransforms = {}) const;

  private:
    std::vector<DiligentVehicleSubmesh> submeshes_;
};

} // namespace openbus::rendering
