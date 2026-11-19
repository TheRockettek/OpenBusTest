#include "app/DiligentHumanBridge.h"

#include "app/DiligentFrameRenderer.h"
#include "render/MatrixMath.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <unordered_set>

namespace openbus::rendering {
namespace {

bool matrixIsFinite(const Matrix4& matrix) {
    return std::all_of(matrix.begin(), matrix.end(),
                       [](double value) { return std::isfinite(value); });
}

struct HumanTrafficActorState {
    Matrix4 worldTransform = identityMatrix();
    HumanBonePalette currentPose = {};
    bool seated = false;
};

HumanTrafficActorState makeHumanTrafficActorState(const HumanTrafficRenderSource& source,
                                                  const openbus::map::MapTrafficPose& trafficPose,
                                                  double gaitPhaseRadians, double gaitWeight,
                                                  std::optional<HumanSeatPose> seat) {
    if (!std::isfinite(gaitPhaseRadians) || !std::isfinite(gaitWeight) ||
        !std::isfinite(trafficPose.headingRadians) ||
        !std::all_of(trafficPose.position.begin(), trafficPose.position.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Human traffic pose must contain finite values");
    }
    if (seat && (!std::isfinite(seat->seatHeight) || !std::isfinite(seat->fraction))) {
        throw std::invalid_argument("Human seated pose requires finite height and fraction");
    }

    const bool canSit = seat && seat->seatHeight > 0.0 && seat->fraction > 0.0;
    const double seatedFraction = canSit ? std::clamp(seat->fraction, 0.0, 1.0) : 0.0;
    const double heightOffset =
        canSit ? humanSeatAnchorOffset(seat->seatHeight, source.joints, seatedFraction) +
                     (source.seatHeight ? humanConfiguredSeatHeightOffset(
                                              *source.seatHeight, source.joints, seatedFraction)
                                        : 0.0)
               : 0.0;
    HumanGaitParameters gait = source.gait;
    gait.phaseRadians = gaitPhaseRadians;
    gait.weight = gaitWeight;

    HumanTrafficActorState state;
    state.currentPose = makeHumanBonePalette(source.joints, gait, seatedFraction);
    state.seated = seatedFraction > 0.0;
    const double headingDegrees = trafficPose.headingRadians * 180.0 / std::acos(-1.0);
    state.worldTransform =
        multiplyMatrix4(translationMatrix({trafficPose.position[0], trafficPose.position[1],
                                           trafficPose.position[2] + heightOffset}),
                        rotationMatrix(-headingDegrees, 0.0, 0.0, 1.0));
    return state;
}

} // namespace

DiligentHumanScenePassResult drawDiligentHumanScenePass(
    DiligentFrameRenderer& frameRenderer, Diligent::ITextureView& fallbackTexture,
    std::span<const DiligentHumanSceneDrawState> actors, DiligentHumanRenderPass pass) {
    if (pass != DiligentHumanRenderPass::All && pass != DiligentHumanRenderPass::Opaque &&
        pass != DiligentHumanRenderPass::Transparent) {
        throw std::invalid_argument("Diligent human scene pass is invalid");
    }

    DiligentHumanScenePassResult result;
    std::vector<std::size_t> order(actors.size());
    std::unordered_set<std::uint64_t> uniqueIds;
    uniqueIds.reserve(actors.size());
    for (std::size_t index = 0; index < actors.size(); ++index) {
        const DiligentHumanSceneDrawState& actor = actors[index];
        if (actor.objectId == 0 || !uniqueIds.insert(actor.objectId).second) {
            throw std::invalid_argument(
                "Diligent human scene pass requires unique stable object IDs");
        }
        if (actor.asset == nullptr || actor.currentPose == nullptr) {
            throw std::invalid_argument(
                "Diligent human scene pass requires an asset and current pose");
        }
        if (!matrixIsFinite(actor.modelViewProjection) || !matrixIsFinite(actor.modelView)) {
            throw std::invalid_argument("Diligent human scene draw has invalid matrices");
        }
        order[index] = index;
    }

    if (pass == DiligentHumanRenderPass::Transparent) {
        std::stable_sort(order.begin(), order.end(), [&](std::size_t left, std::size_t right) {
            return actors[left].modelView[14] < actors[right].modelView[14];
        });
    }

    result.submissionOrder.reserve(order.size());
    for (const std::size_t index : order) {
        const DiligentHumanSceneDrawState& actor = actors[index];
        const std::size_t submeshes = frameRenderer.drawHumanAssetPass(
            *actor.asset, fallbackTexture, actor.modelViewProjection, actor.modelView,
            *actor.currentPose, actor.previousPose, actor.seated, pass, actor.lighting,
            actor.motion);
        result.submissionOrder.push_back(actor.objectId);
        result.submeshesDrawn += submeshes;
        if (submeshes != 0) {
            ++result.actorsDrawn;
        }
    }
    return result;
}

DiligentHumanPlacementScenePassResult drawDiligentHumanPlacementScenePass(
    DiligentFrameRenderer& frameRenderer, Diligent::ITextureView& fallbackTexture,
    std::span<const DiligentHumanPlacementSceneDrawState> actors, DiligentHumanRenderPass pass) {
    if (pass != DiligentHumanRenderPass::All && pass != DiligentHumanRenderPass::Opaque &&
        pass != DiligentHumanRenderPass::Transparent) {
        throw std::invalid_argument("Diligent human placement scene pass is invalid");
    }

    const Matrix4& view = frameRenderer.frameData().currentView;
    std::vector<std::size_t> order(actors.size());
    std::vector<double> viewDepths(actors.size());
    std::unordered_set<DiligentHumanPoseKey, DiligentHumanPoseKeyHash> uniqueIds;
    uniqueIds.reserve(actors.size());
    for (std::size_t index = 0; index < actors.size(); ++index) {
        const DiligentHumanPlacementSceneDrawState& actor = actors[index];
        if (actor.poseKey.objectId == 0 || !uniqueIds.insert(actor.poseKey).second) {
            throw std::invalid_argument(
                "Diligent human placement pass requires unique stable pose keys");
        }
        if (actor.asset == nullptr || actor.motionKey.isStatic()) {
            throw std::invalid_argument(
                "Diligent human placement pass requires an asset, pose, and motion key");
        }
        if (actor.poseKey.objectId != actor.motionKey.object ||
            actor.poseKey.subObjectId != actor.motionKey.subObject) {
            throw std::invalid_argument(
                "Diligent human pose and motion keys must identify the same object");
        }
        if (!matrixIsFinite(actor.worldTransform) || !std::isfinite(actor.boundingRadius) ||
            actor.boundingRadius < 0.0) {
            throw std::invalid_argument("Diligent human placement state is invalid");
        }

        const Matrix4 modelView = multiplyMatrix4(view, actor.worldTransform);
        const std::array<double, 4> center = transformPoint(modelView, {0.0, 0.0, 0.0, 1.0});
        if (!std::all_of(center.begin(), center.end(),
                         [](double value) { return std::isfinite(value); })) {
            throw std::invalid_argument("Diligent human placement view depth is invalid");
        }
        viewDepths[index] = center[2];
        order[index] = index;
    }

    if (pass == DiligentHumanRenderPass::Transparent) {
        std::stable_sort(order.begin(), order.end(), [&](std::size_t left, std::size_t right) {
            return viewDepths[left] < viewDepths[right];
        });
    }

    DiligentHumanPlacementScenePassResult result;
    result.submissionOrder.reserve(order.size());
    result.poseSubmissionOrder.reserve(order.size());
    result.actorResults.reserve(order.size());
    for (const std::size_t index : order) {
        const DiligentHumanPlacementSceneDrawState& actor = actors[index];
        const DiligentHumanDrawResult draw = frameRenderer.drawHumanPlacement(
            *actor.asset, actor.worldTransform, actor.boundingRadius, fallbackTexture,
            actor.poseKey, actor.motionKey, actor.currentPose, actor.seated, actor.lighting,
            actor.geometryHistoryValid, actor.visible, pass);
        result.submissionOrder.push_back(actor.poseKey.objectId);
        result.poseSubmissionOrder.push_back(actor.poseKey);
        result.actorsDrawn += draw.drawn ? 1U : 0U;
        result.actorsCulled += draw.culled ? 1U : 0U;
        result.actorResults.push_back({actor.poseKey, draw.drawn, draw.culled,
                                       draw.poseHistoryValid, draw.motionHistoryValid});
    }
    return result;
}

std::optional<DiligentHumanPlacementSceneDrawState> prepareDiligentHumanTrafficPlacement(
    DiligentFrameRenderer& frameRenderer, std::string_view modelKey,
    const HumanTrafficRenderSource& source, const openbus::map::MapTrafficPose& trafficPose,
    double gaitPhaseRadians, double gaitWeight, std::optional<HumanSeatPose> seat,
    double boundingRadius, DiligentHumanPoseKey poseKey, MotionObjectKey motionKey,
    const DiligentLightingParameters& lighting, bool geometryHistoryValid, bool visible) {
    if (poseKey.objectId == 0 || motionKey.isStatic()) {
        throw std::invalid_argument(
            "Diligent human traffic placement requires stable pose and motion keys");
    }
    if (poseKey.objectId != motionKey.object || poseKey.subObjectId != motionKey.subObject) {
        throw std::invalid_argument(
            "Diligent human pose and motion keys must identify the same object");
    }
    if (!std::isfinite(boundingRadius) || boundingRadius < 0.0) {
        throw std::invalid_argument(
            "Diligent human traffic placement radius must be finite and non-negative");
    }
    if (!frameRenderer.humanAssetCurrent(modelKey, source.sourceGeneration)) {
        return std::nullopt;
    }
    const DiligentHumanAsset* asset = frameRenderer.humanAsset(modelKey);
    if (asset == nullptr) {
        throw std::logic_error("Diligent human asset disappeared after generation validation");
    }

    const HumanTrafficActorState trafficState =
        makeHumanTrafficActorState(source, trafficPose, gaitPhaseRadians, gaitWeight, seat);
    DiligentHumanPlacementSceneDrawState drawState;
    drawState.poseKey = poseKey;
    drawState.motionKey = motionKey;
    drawState.asset = asset;
    drawState.worldTransform = trafficState.worldTransform;
    drawState.boundingRadius = boundingRadius;
    drawState.currentPose = trafficState.currentPose;
    drawState.seated = trafficState.seated;
    drawState.lighting = lighting;
    drawState.geometryHistoryValid = geometryHistoryValid;
    drawState.visible = visible;
    return drawState;
}

std::size_t drawDiligentHumanShadowLayer(DiligentFrameRenderer& frameRenderer,
                                         const DiligentHumanAsset& asset,
                                         Diligent::Uint32 lightIndex,
                                         const Matrix4& modelViewProjection,
                                         const HumanBonePalette& currentPose, bool seated) {
    const std::vector<DiligentShadowCaster> casters =
        asset.shadowCasters(modelViewProjection, currentPose, seated);
    return frameRenderer.drawOpaqueSpotlightShadowLayer(lightIndex, casters);
}

DiligentHumanShadowPassResult
drawDiligentHumanShadowSceneLayer(DiligentFrameRenderer& frameRenderer, Diligent::Uint32 lightIndex,
                                  std::span<const DiligentHumanShadowDrawState> actors) {
    DiligentHumanShadowPassResult result;
    std::vector<DiligentShadowCaster> casters;
    for (const DiligentHumanShadowDrawState& actor : actors) {
        if (actor.asset == nullptr || actor.currentPose == nullptr) {
            throw std::invalid_argument(
                "Diligent human shadow scene requires an asset and current pose per actor");
        }
        if (!std::all_of(actor.modelViewProjection.begin(), actor.modelViewProjection.end(),
                         [](double value) { return std::isfinite(value); })) {
            throw std::invalid_argument("Diligent human shadow matrix must be finite");
        }
        auto actorCasters =
            actor.asset->shadowCasters(actor.modelViewProjection, *actor.currentPose, actor.seated);
        if (!actorCasters.empty()) {
            ++result.actorsWithCasters;
            casters.insert(casters.end(), actorCasters.begin(), actorCasters.end());
        }
    }

    result.castersDrawn = frameRenderer.drawOpaqueSpotlightShadowLayer(lightIndex, casters);
    return result;
}

DiligentHumanDrawResult drawDiligentHumanTrafficActor(
    DiligentFrameRenderer& frameRenderer, std::string_view modelKey,
    const HumanTrafficRenderSource& source, const openbus::map::MapTrafficPose& trafficPose,
    double gaitPhaseRadians, double gaitWeight, std::optional<HumanSeatPose> seat,
    double boundingRadius, Diligent::ITextureView& fallbackTexture, DiligentHumanPoseKey poseKey,
    MotionObjectKey motionKey, const DiligentLightingParameters& lighting,
    bool geometryHistoryValid, bool visible) {
    const auto drawState = prepareDiligentHumanTrafficPlacement(
        frameRenderer, modelKey, source, trafficPose, gaitPhaseRadians, gaitWeight, seat,
        boundingRadius, poseKey, motionKey, lighting, geometryHistoryValid, visible);
    if (!drawState) {
        throw std::logic_error("Diligent human asset is not prepared for this source generation");
    }
    return frameRenderer.drawHumanPlacement(
        *drawState->asset, drawState->worldTransform, drawState->boundingRadius, fallbackTexture,
        drawState->poseKey, drawState->motionKey, drawState->currentPose, drawState->seated,
        drawState->lighting, drawState->geometryHistoryValid, drawState->visible);
}

DiligentHumanDrawResult
drawDiligentHumanAssetInView(const DiligentHumanAsset& asset, DiligentTexturedMeshRenderer& meshes,
                             Diligent::IDeviceContext& context,
                             Diligent::ITextureView& fallbackTexture, const Matrix4& worldTransform,
                             double boundingRadius, const Matrix4& view, const Matrix4& projection,
                             const HumanBonePalette& currentPose, bool seated,
                             const DiligentLightingParameters& lighting, bool visible) {
    if (!std::isfinite(boundingRadius) || boundingRadius < 0.0) {
        throw std::invalid_argument(
            "Diligent secondary-view human placement radius must be finite and non-negative");
    }
    const auto isFiniteMatrix = [](const Matrix4& matrix) {
        return std::all_of(matrix.begin(), matrix.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (!isFiniteMatrix(worldTransform) || !isFiniteMatrix(view) || !isFiniteMatrix(projection)) {
        throw std::invalid_argument(
            "Diligent secondary-view human transforms must contain finite values");
    }
    if (!visible || asset.submeshes().empty()) {
        return {};
    }

    const Matrix4 modelView = multiplyMatrix4(view, worldTransform);
    const std::array<double, 4> center = transformPoint(modelView, {0.0, 0.0, 0.0, 1.0});
    const double transformScale =
        (std::max)({std::hypot(worldTransform[0], worldTransform[1], worldTransform[2]),
                    std::hypot(worldTransform[4], worldTransform[5], worldTransform[6]),
                    std::hypot(worldTransform[8], worldTransform[9], worldTransform[10])});
    const double viewRadius = boundingRadius * transformScale;
    if (!std::isfinite(transformScale) || !std::isfinite(viewRadius) ||
        !std::all_of(center.begin(), center.end(),
                     [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("Diligent secondary-view human bounds exceed numeric range");
    }
    if (sphereOutsideFrustum(buildViewFrustum(projection), center, viewRadius)) {
        DiligentHumanDrawResult result;
        result.culled = true;
        return result;
    }

    const Matrix4 modelViewProjection = multiplyMatrix4(projection, modelView);
    asset.draw(meshes, context, fallbackTexture, modelViewProjection, modelView, currentPose,
               nullptr, seated, lighting, nullptr);
    DiligentHumanDrawResult result;
    result.drawn = true;
    return result;
}

DiligentHumanDrawResult drawDiligentHumanTrafficActorInView(
    DiligentFrameRenderer& frameRenderer, std::string_view modelKey,
    const HumanTrafficRenderSource& source, const openbus::map::MapTrafficPose& trafficPose,
    double gaitPhaseRadians, double gaitWeight, std::optional<HumanSeatPose> seat,
    double boundingRadius, DiligentTexturedMeshRenderer& meshes, Diligent::IDeviceContext& context,
    Diligent::ITextureView& fallbackTexture, const Matrix4& view, const Matrix4& projection,
    const DiligentLightingParameters& lighting, bool visible) {
    const DiligentHumanAsset* asset = frameRenderer.humanAsset(modelKey);
    if (asset == nullptr || !frameRenderer.humanAssetCurrent(modelKey, source.sourceGeneration)) {
        throw std::logic_error("Diligent human asset is not prepared for this source generation");
    }
    const HumanTrafficActorState state =
        makeHumanTrafficActorState(source, trafficPose, gaitPhaseRadians, gaitWeight, seat);
    return drawDiligentHumanAssetInView(*asset, meshes, context, fallbackTexture,
                                        state.worldTransform, boundingRadius, view, projection,
                                        state.currentPose, state.seated, lighting, visible);
}

} // namespace openbus::rendering
