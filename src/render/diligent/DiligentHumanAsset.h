#pragma once

#include "assets/TextureLoader.h"
#include "render/HumanMeshGeometry.h"
#include "render/diligent/DiligentTexturedMesh.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <unordered_map>
#include <vector>

namespace Diligent {
struct IRenderDevice;
}

namespace openbus::rendering {

class DiligentTexture2D;

struct DiligentHumanPoseKey {
    std::uint64_t objectId = 0;
    std::uint64_t viewId = 0;
    std::uint64_t subObjectId = 0;

    bool operator==(const DiligentHumanPoseKey&) const = default;
};

struct DiligentHumanPoseKeyHash {
    std::size_t operator()(const DiligentHumanPoseKey& key) const noexcept;
};

struct DiligentHumanPoseHistoryResult {
    // Borrowed from committed history until this frame completes or history is invalidated.
    const HumanBonePalette* previousPose = nullptr;
    bool historyValid = false;
};

struct DiligentHumanDrawResult {
    bool drawn = false;
    bool culled = false;
    bool poseHistoryValid = false;
    bool motionHistoryValid = false;
};

class DiligentHumanPoseHistory final {
  public:
    void beginFrame(const MotionDataFrame& frame);
    [[nodiscard]] DiligentHumanPoseHistoryResult
    recordRenderedPose(DiligentHumanPoseKey key, const HumanBonePalette& currentPose,
                       bool geometryHistoryValid = true, bool seated = false,
                       std::uint64_t assetGeneration = 0);
    void completeFrame();
    void invalidate() noexcept;

  private:
    struct PoseSnapshot {
        HumanBonePalette pose = {};
        bool seated = false;
        std::uint64_t assetGeneration = 0;
    };

    std::unordered_map<DiligentHumanPoseKey, PoseSnapshot, DiligentHumanPoseKeyHash> previousPoses_;
    std::unordered_map<DiligentHumanPoseKey, PoseSnapshot, DiligentHumanPoseKeyHash> currentPoses_;
    std::uint64_t activeFrameId_ = 0;
    std::uint64_t previousFrameId_ = 0;
    bool hasPreviousFrame_ = false;
    bool frameHistoryValid_ = false;
    bool frameActive_ = false;
};

struct DiligentHumanSubmesh {
    int materialIndex = 0;
    std::array<double, 3> localCenter = {};
    std::unique_ptr<DiligentTexturedMesh> mesh;
    std::unique_ptr<DiligentTexturedMesh> seatedMesh;
    std::shared_ptr<DiligentTexture2D> texture;
    DiligentMaterialState material;

    // The caller supplies rendered-frame pose history and controls submesh ordering.
    void draw(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
              Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
              const HumanBonePalette& currentPose, const HumanBonePalette* previousPose,
              bool seated, const DiligentLightingParameters& lighting = {},
              const DiligentMotionParameters* motion = nullptr) const;
};

enum class DiligentHumanRenderPass : std::uint8_t {
    All,
    Opaque,
    Transparent,
};

// Uploads the engine's CPU-prepared, material-grouped human geometry directly to Diligent.
// Textures and material state are keyed by the same source material indexes.
class DiligentHumanAsset final {
  public:
    DiligentHumanAsset(Diligent::IRenderDevice& device, const HumanMeshGeometry& geometry,
                       const std::unordered_map<int, std::shared_ptr<const Image>>& textures,
                       const std::unordered_map<int, DiligentMaterialState>& materials = {},
                       std::uint64_t sourceGeneration = 0);

    [[nodiscard]] std::span<const DiligentHumanSubmesh> submeshes() const noexcept;
    [[nodiscard]] std::uint64_t sourceGeneration() const noexcept;
    [[nodiscard]] bool hasSubmeshesForPass(DiligentHumanRenderPass pass) const;
    void draw(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
              Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
              const Matrix4& modelView, const HumanBonePalette& currentPose,
              const HumanBonePalette* previousPose, bool seated,
              const DiligentLightingParameters& lighting = {},
              const DiligentMotionParameters* motion = nullptr) const;
    [[nodiscard]] std::size_t
    drawPass(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
             Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
             const Matrix4& modelView, const HumanBonePalette& currentPose,
             const HumanBonePalette* previousPose, bool seated, DiligentHumanRenderPass pass,
             const DiligentLightingParameters& lighting = {},
             const DiligentMotionParameters* motion = nullptr) const;
    // Returned casters borrow this asset's meshes/textures and the caller's current pose.
    [[nodiscard]] std::vector<DiligentShadowCaster>
    shadowCasters(const Matrix4& modelViewProjection, const HumanBonePalette& currentPose,
                  bool seated) const;

  private:
    std::vector<DiligentHumanSubmesh> submeshes_;
    std::uint64_t sourceGeneration_ = 0;
};

} // namespace openbus::rendering
