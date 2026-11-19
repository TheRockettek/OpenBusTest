#pragma once

#include "render/diligent/DiligentHumanAsset.h"
#include "render/MotionData.h"
#include "render/map/MapRenderer.h"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace openbus::rendering {

class DiligentFrameRenderer;
class MapRenderer;

struct DiligentHumanSceneDrawState {
    std::uint64_t objectId = 0;
    const DiligentHumanAsset* asset = nullptr;
    Matrix4 modelViewProjection = {};
    Matrix4 modelView = {};
    const HumanBonePalette* currentPose = nullptr;
    const HumanBonePalette* previousPose = nullptr;
    bool seated = false;
    DiligentLightingParameters lighting{};
    const DiligentMotionParameters* motion = nullptr;
};

struct DiligentHumanScenePassResult {
    std::size_t actorsDrawn = 0;
    std::size_t submeshesDrawn = 0;
    std::vector<std::uint64_t> submissionOrder;
};

struct DiligentHumanPlacementSceneDrawState {
    DiligentHumanPoseKey poseKey{};
    MotionObjectKey motionKey{};
    const DiligentHumanAsset* asset = nullptr;
    Matrix4 worldTransform = {};
    double boundingRadius = 0.0;
    HumanBonePalette currentPose = {};
    bool seated = false;
    DiligentLightingParameters lighting{};
    bool geometryHistoryValid = true;
    bool visible = true;
};

struct DiligentHumanPlacementActorResult {
    DiligentHumanPoseKey poseKey{};
    bool drawn = false;
    bool culled = false;
    bool poseHistoryValid = false;
    bool motionHistoryValid = false;
};

struct DiligentHumanPlacementScenePassResult {
    std::size_t actorsDrawn = 0;
    std::size_t actorsCulled = 0;
    std::vector<std::uint64_t> submissionOrder;
    std::vector<DiligentHumanPoseKey> poseSubmissionOrder;
    std::vector<DiligentHumanPlacementActorResult> actorResults;
};

[[nodiscard]] inline DiligentHumanPoseKey makeDiligentHumanPoseKey(MotionObjectKey motionKey,
                                                                   std::uint64_t viewId) noexcept {
    return {motionKey.object, viewId, motionKey.subObject};
}

struct DiligentHumanShadowDrawState {
    const DiligentHumanAsset* asset = nullptr;
    Matrix4 modelViewProjection = {};
    const HumanBonePalette* currentPose = nullptr;
    bool seated = false;
};

struct DiligentHumanShadowPassResult {
    std::size_t actorsWithCasters = 0;
    std::size_t castersDrawn = 0;
};

// Starts or completes CPU-only human model preparation, then publishes the GPU asset
// outside scene submission. Returns false while model textures are still decoding.
[[nodiscard]] bool prepareDiligentHumanAsset(MapRenderer& mapRenderer,
                                             DiligentFrameRenderer& frameRenderer,
                                             const std::filesystem::path& humanConfigPath);
[[nodiscard]] bool prepareDiligentHumanAsset(DiligentFrameRenderer& frameRenderer,
                                             std::string_view modelKey,
                                             const HumanTrafficRenderSource& source);

// Opaque actors retain input order; transparent actors are sorted back-to-front by view-space
// origin, with each asset sorting its blended material groups.
[[nodiscard]] DiligentHumanScenePassResult drawDiligentHumanScenePass(
    DiligentFrameRenderer& frameRenderer, Diligent::ITextureView& fallbackTexture,
    std::span<const DiligentHumanSceneDrawState> actors, DiligentHumanRenderPass pass);

// Route managed placements through frame-host frustum/history handling for each pass.
// Transparent actors are submitted back-to-front in the active view.
[[nodiscard]] DiligentHumanPlacementScenePassResult drawDiligentHumanPlacementScenePass(
    DiligentFrameRenderer& frameRenderer, Diligent::ITextureView& fallbackTexture,
    std::span<const DiligentHumanPlacementSceneDrawState> actors, DiligentHumanRenderPass pass);

// Converts a production traffic pose and gait/seat state into an owned, pass-ready placement.
// Returns no state while the corresponding prepared GPU asset is unavailable.
[[nodiscard]] std::optional<DiligentHumanPlacementSceneDrawState>
prepareDiligentHumanTrafficPlacement(DiligentFrameRenderer& frameRenderer,
                                     std::string_view modelKey,
                                     const HumanTrafficRenderSource& source,
                                     const openbus::map::MapTrafficPose& trafficPose,
                                     double gaitPhaseRadians, double gaitWeight,
                                     std::optional<HumanSeatPose> seat, double boundingRadius,
                                     DiligentHumanPoseKey poseKey, MotionObjectKey motionKey,
                                     const DiligentLightingParameters& lighting = {},
                                     bool geometryHistoryValid = true, bool visible = true);

[[nodiscard]] std::size_t
drawDiligentHumanShadowLayer(DiligentFrameRenderer& frameRenderer, const DiligentHumanAsset& asset,
                             Diligent::Uint32 lightIndex, const Matrix4& modelViewProjection,
                             const HumanBonePalette& currentPose, bool seated);

// Gather all pedestrian casters before submitting the selected shadow layer so it is cleared
// once per light rather than once per actor.
[[nodiscard]] DiligentHumanShadowPassResult
drawDiligentHumanShadowSceneLayer(DiligentFrameRenderer& frameRenderer, Diligent::Uint32 lightIndex,
                                  std::span<const DiligentHumanShadowDrawState> actors);

[[nodiscard]] DiligentHumanDrawResult drawDiligentHumanTrafficActor(
    DiligentFrameRenderer& frameRenderer, std::string_view modelKey,
    const HumanTrafficRenderSource& source, const openbus::map::MapTrafficPose& trafficPose,
    double gaitPhaseRadians, double gaitWeight, std::optional<HumanSeatPose> seat,
    double boundingRadius, Diligent::ITextureView& fallbackTexture, DiligentHumanPoseKey poseKey,
    MotionObjectKey motionKey, const DiligentLightingParameters& lighting = {},
    bool geometryHistoryValid = true, bool visible = true);

// Submit a prepared pedestrian in an auxiliary view using its current pose only. Frustum
// visibility is evaluated from the supplied view/projection, and no main-view history is changed.
[[nodiscard]] DiligentHumanDrawResult
drawDiligentHumanAssetInView(const DiligentHumanAsset& asset, DiligentTexturedMeshRenderer& meshes,
                             Diligent::IDeviceContext& context,
                             Diligent::ITextureView& fallbackTexture, const Matrix4& worldTransform,
                             double boundingRadius, const Matrix4& view, const Matrix4& projection,
                             const HumanBonePalette& currentPose, bool seated,
                             const DiligentLightingParameters& lighting = {}, bool visible = true);

[[nodiscard]] DiligentHumanDrawResult drawDiligentHumanTrafficActorInView(
    DiligentFrameRenderer& frameRenderer, std::string_view modelKey,
    const HumanTrafficRenderSource& source, const openbus::map::MapTrafficPose& trafficPose,
    double gaitPhaseRadians, double gaitWeight, std::optional<HumanSeatPose> seat,
    double boundingRadius, DiligentTexturedMeshRenderer& meshes, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const Matrix4& view, const Matrix4& projection,
    const DiligentLightingParameters& lighting = {}, bool visible = true);

} // namespace openbus::rendering
