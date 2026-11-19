#include "render/diligent/DiligentSceneFrame.h"

#include <Graphics/GraphicsEngine/interface/DeviceContext.h>

#include <cmath>
#include <stdexcept>

namespace openbus::rendering {

DiligentSceneFrame::DiligentSceneFrame(Diligent::IRenderDevice& device, Diligent::Uint32 width,
                                       Diligent::Uint32 height,
                                       Diligent::TEXTURE_FORMAT outputFormat,
                                       bool includeLinearDepth, DiligentMultisampling multisampling)
    : targets_(device, width, height, includeLinearDepth, multisampling),
      colorPresent_(device, outputFormat) {
    using namespace Diligent;
    if (includeLinearDepth) {
        meshes_ = std::make_unique<DiligentTexturedMeshRenderer>(
            device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT, TEX_FORMAT_RG32_FLOAT,
            TEX_FORMAT_R8_UNORM, TEX_FORMAT_R32_FLOAT, multisampling);
    } else {
        meshes_ = std::make_unique<DiligentTexturedMeshRenderer>(
            device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT, TEX_FORMAT_RG32_FLOAT,
            TEX_FORMAT_R8_UNORM, multisampling);
    }
}

DiligentSceneFrame::~DiligentSceneFrame() = default;

void DiligentSceneFrame::resize(Diligent::IDeviceContext& immediateContext, Diligent::Uint32 width,
                                Diligent::Uint32 height) {
    if (state_ != State::Ready) {
        throw std::logic_error("Cannot resize Diligent scene targets before completing the frame");
    }
    if (width == targets_.width() && height == targets_.height()) {
        return;
    }
    targets_.resize(immediateContext, width, height);
    invalidateNextHistory_ = true;
    state_ = State::Ready;
}

void DiligentSceneFrame::invalidateHistory() {
    if (state_ != State::Ready) {
        throw std::logic_error("Cannot invalidate Diligent frame history during scene drawing");
    }
    invalidateNextHistory_ = true;
}

void DiligentSceneFrame::beginScene(Diligent::IDeviceContext& context, const MotionDataFrame& frame,
                                    const std::array<float, 4>& clearColor) {
    if (state_ != State::Ready) {
        throw std::logic_error("Complete the previous Diligent frame before beginning another");
    }
    if (frame.width != static_cast<int>(targets_.width()) ||
        frame.height != static_cast<int>(targets_.height()) || frame.width <= 0 ||
        frame.height <= 0 || !std::isfinite(frame.timestampSeconds) ||
        !std::isfinite(frame.deltaSeconds) || frame.deltaSeconds < 0.0 ||
        (hasCompletedFrame_ && (frame.frameId <= lastCompletedFrameId_ ||
                                frame.timestampSeconds < lastCompletedTimestamp_))) {
        throw std::invalid_argument("Diligent scene frame metadata is invalid or out of order");
    }
    MotionDataFrame nextFrame = frame;
    if (invalidateNextHistory_ || !hasCompletedFrame_) {
        nextFrame.historyValid = false;
    }

    targets_.clear(context, clearColor);
    targets_.bind(context);
    const Diligent::Viewport viewport{
        0.0F, 0.0F, static_cast<float>(targets_.width()), static_cast<float>(targets_.height()),
        0.0F, 1.0F};
    context.SetViewports(1, &viewport, targets_.width(), targets_.height());
    frame_ = nextFrame;
    invalidateNextHistory_ = false;
    state_ = State::Scene;
}

void DiligentSceneFrame::finishScene(Diligent::IDeviceContext& context,
                                     Diligent::ITextureView& destination,
                                     DiligentColorFilter filter, DiligentDebugView debugView) {
    if (state_ != State::Scene) {
        throw std::logic_error("Diligent scene must be active before composition");
    }
    targets_.resolve(context);
    if (debugView == DiligentDebugView::None) {
        colorPresent_.draw(context, targets_.colorTexture(), destination, filter);
    } else {
        Diligent::ITextureView* linearDepth =
            debugView == DiligentDebugView::LinearDepth ? targets_.linearDepthTexture() : nullptr;
        if (debugView == DiligentDebugView::LinearDepth && linearDepth == nullptr) {
            throw std::logic_error("Diligent linear-depth debug is unavailable for this frame");
        }
        colorPresent_.drawDebug(context, targets_.motionVectorTexture(),
                                targets_.motionValidityTexture(), destination, debugView,
                                linearDepth);
    }
    state_ = State::Composited;
}

void DiligentSceneFrame::completeFrame() {
    if (state_ != State::Composited) {
        throw std::logic_error("Diligent scene must be composited before completing the frame");
    }
    lastCompletedFrameId_ = frame_.frameId;
    lastCompletedTimestamp_ = frame_.timestampSeconds;
    hasCompletedFrame_ = true;
    state_ = State::Ready;
}

void DiligentSceneFrame::abortFrame() noexcept {
    state_ = State::Ready;
    invalidateNextHistory_ = true;
}

const MotionDataFrame& DiligentSceneFrame::frameData() const {
    if (state_ == State::Ready) {
        throw std::logic_error("Diligent frame metadata is unavailable outside an active frame");
    }
    return frame_;
}

DiligentMotionRenderTargets& DiligentSceneFrame::targets() noexcept {
    return targets_;
}

Diligent::Uint32 DiligentSceneFrame::maximumAnisotropy() const noexcept {
    return meshes_->maximumAnisotropy();
}

DiligentTexturedMeshRenderer& DiligentSceneFrame::meshes() {
    if (state_ != State::Scene) {
        throw std::logic_error("Diligent mesh renderer is available only during scene submission");
    }
    return *meshes_;
}

} // namespace openbus::rendering
