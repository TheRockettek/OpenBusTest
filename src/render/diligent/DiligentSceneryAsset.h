#pragma once

#include "map/SceneryMaterialRuntime.h"
#include "render/diligent/DiligentTexturedMesh.h"
#include "render/map/PreparedMapScenery.h"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace Diligent {
struct IRenderDevice;
}

namespace openbus::rendering {

class DiligentTexture2D;

struct DiligentSceneryTextures {
    std::shared_ptr<DiligentTexture2D> base;
    std::shared_ptr<DiligentTexture2D> transmap;
    std::shared_ptr<DiligentTexture2D> lightmap;
    std::shared_ptr<DiligentTexture2D> nightmap;
    std::shared_ptr<DiligentTexture2D> bumpmap;
    std::shared_ptr<DiligentTexture2D> environment;
    std::shared_ptr<DiligentTexture2D> freeTexture;
    // Entries correspond by index to the prepared item-lightmap and item-nightmap lists.
    std::vector<std::shared_ptr<DiligentTexture2D>> itemLightmaps;
    std::vector<std::shared_ptr<DiligentTexture2D>> itemNightmaps;
};

struct DiligentSceneryOpaqueDraw {
    const Variables* variables = nullptr;
    std::optional<double> weatherBrightness;
    // Override the uploaded base texture with a streamed seasonal texture.
    Diligent::ITextureView* seasonalTexture = nullptr;
    // Runtime views for script/free and text-atlas materials; their updates and
    // lifetime remain owned by the caller through the draw.
    Diligent::ITextureView* dynamicTexture = nullptr;
    Diligent::ITextureView* textTexture = nullptr;
    float visibilityFade = 1.0F;
    std::optional<std::size_t> sceneryLodIndex;
    // The caller must determine this from the current model-view transform.
    bool reverseWinding = false;
};

enum class DiligentSceneryPass : std::uint8_t {
    Opaque,
    Transparent,
    TransmapDepthPrepass,
};

struct DiligentSceneryDrawResult {
    std::size_t batchesDrawn = 0;
    std::size_t treesDrawn = 0;
    bool motionHistoryValid = false;
    bool culled = false;
};

struct DiligentSceneryBatch {
    PreparedSceneryBatch source;
    std::shared_ptr<DiligentTexturedMesh> mesh;
    DiligentSceneryTextures textures;
    std::array<double, 3> localCenter = {};

    // The returned selectedItem, if any, is valid while this batch's selection remains alive.
    [[nodiscard]] openbus::map::SceneryMaterialRuntimeState
    resolveRuntime(const Variables* variables, std::optional<double> weatherBrightness) const;

    // The caller owns pass ordering (including back-to-front sorting of transparent
    // draws), visibility/LOD culling, fallback texture, and temporal history.
    // Runtime script/free and text-atlas views are supplied through the draw arguments;
    // false means hidden or zero visibility fade.
    bool drawOpaque(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
                    Diligent::ITextureView& fallbackWhite, const Matrix4& modelViewProjection,
                    const DiligentSceneryOpaqueDraw& draw,
                    const DiligentLightingParameters& lighting = {},
                    const DiligentMotionParameters* motion = nullptr) const;
    bool drawTransparent(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
                         Diligent::ITextureView& fallbackWhite, const Matrix4& modelViewProjection,
                         const DiligentSceneryOpaqueDraw& draw,
                         const DiligentLightingParameters& lighting = {},
                         const DiligentMotionParameters* motion = nullptr) const;
    bool drawTransmapDepthPrepass(DiligentTexturedMeshRenderer& renderer,
                                  Diligent::IDeviceContext& context,
                                  Diligent::ITextureView& fallbackWhite,
                                  const Matrix4& modelViewProjection,
                                  const DiligentSceneryOpaqueDraw& draw,
                                  const DiligentLightingParameters& lighting = {}) const;

  private:
    bool drawPass(DiligentSceneryPass pass, DiligentTexturedMeshRenderer& renderer,
                  Diligent::IDeviceContext& context, Diligent::ITextureView& fallbackWhite,
                  const Matrix4& modelViewProjection, const DiligentSceneryOpaqueDraw& draw,
                  const DiligentLightingParameters& lighting,
                  const DiligentMotionParameters* motion) const;
};

struct DiligentSceneryTreeBatch {
    openbus::map::MapSceneryBatch source;
    std::array<double, 3> localCenter = {};
    std::string textureName;
    std::filesystem::path texturePath;
    std::shared_ptr<const SharedMapTextureImages::SeasonalPaths> seasonTexturePaths;
    std::shared_ptr<DiligentTexturedMesh> mesh;
    std::shared_ptr<DiligentTexture2D> texture;

    // The caller supplies the tree's world/view transform and season-streamed texture.
    // Returns false for a missing asset or fully faded tree.
    bool draw(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
              Diligent::ITextureView& fallbackWhite, const Matrix4& modelViewProjection,
              const DiligentSceneryOpaqueDraw& draw,
              const DiligentLightingParameters& lighting = {},
              const DiligentMotionParameters* motion = nullptr) const;
};

enum class DiligentScenerySubmissionKind : std::uint8_t {
    Batch,
    Tree,
};

struct DiligentScenerySubmission {
    DiligentScenerySubmissionKind kind = DiligentScenerySubmissionKind::Batch;
    std::size_t index = 0;
};

[[nodiscard]] std::vector<DiligentScenerySubmission>
orderDiligentTransparentScenery(std::span<const DiligentSceneryBatch> batches,
                                std::span<const std::size_t> batchIndices,
                                std::span<const DiligentSceneryTreeBatch> trees,
                                std::span<const std::size_t> treeIndices, const Matrix4& modelView);

// GPU resources made from CPU-prepared scenery. This is not a scene renderer:
// callers interpret the retained metadata, manage dynamic textures and seasonal
// streaming, and submit draws in the correct passes and order.
class DiligentSceneryAsset final {
  public:
    DiligentSceneryAsset(Diligent::IRenderDevice& device, const PreparedSceneryAsset& prepared,
                         bool generateMipmaps = false);

    [[nodiscard]] std::span<const DiligentSceneryBatch> batches() const noexcept;
    [[nodiscard]] std::span<const DiligentSceneryTreeBatch> trees() const noexcept;
    [[nodiscard]] std::size_t selectLod(double distance) const noexcept;
    [[nodiscard]] bool mayRenderPass(DiligentSceneryPass pass, float visibilityFade) const noexcept;
    // Missing optional or authored textures remain visible to callers rather than failing silently.
    [[nodiscard]] std::span<const std::string> diagnostics() const noexcept;

  private:
    std::vector<double> lodThresholds_;
    std::vector<DiligentSceneryBatch> batches_;
    std::vector<DiligentSceneryTreeBatch> trees_;
    std::vector<std::string> diagnostics_;
    bool hasOpaqueBatches_ = false;
    bool hasTransparentBatches_ = false;
    bool hasTransmapDepthBatches_ = false;
    bool hasOpaqueTrees_ = false;
    bool hasTransparentTrees_ = false;
};

} // namespace openbus::rendering
