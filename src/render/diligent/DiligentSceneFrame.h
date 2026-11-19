#pragma once

#include "render/MotionData.h"
#include "render/diligent/DiligentColorPresent.h"
#include "render/diligent/DiligentMotionRenderTargets.h"
#include "render/diligent/DiligentMultisampling.h"
#include "render/diligent/DiligentTexturedMesh.h"

#include <array>
#include <cstdint>
#include <memory>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// Coordinates one Diligent scene pass from metadata target binding through color composition.
// The caller owns window presentation, UI composition, scene submission, and MotionHistory.
class DiligentSceneFrame final {
  public:
    DiligentSceneFrame(Diligent::IRenderDevice& device, Diligent::Uint32 width,
                       Diligent::Uint32 height, Diligent::TEXTURE_FORMAT outputFormat,
                       bool includeLinearDepth = true, DiligentMultisampling multisampling = {});
    ~DiligentSceneFrame();
    DiligentSceneFrame(const DiligentSceneFrame&) = delete;
    DiligentSceneFrame& operator=(const DiligentSceneFrame&) = delete;

    void resize(Diligent::IDeviceContext& immediateContext, Diligent::Uint32 width,
                Diligent::Uint32 height);
    void invalidateHistory();
    void beginScene(Diligent::IDeviceContext& context, const MotionDataFrame& frame,
                    const std::array<float, 4>& clearColor = {0, 0, 0, 1});
    // A selected debug view replaces normal color filtering during composition.
    void finishScene(Diligent::IDeviceContext& context, Diligent::ITextureView& destination,
                     DiligentColorFilter filter = DiligentColorFilter::None,
                     DiligentDebugView debugView = DiligentDebugView::None);
    // Call after composition and any UI/presentation work for this frame has completed.
    void completeFrame();
    // Discard a failed submission and make the next frame's temporal history invalid.
    void abortFrame() noexcept;

    [[nodiscard]] const MotionDataFrame& frameData() const;
    [[nodiscard]] DiligentMotionRenderTargets& targets() noexcept;
    [[nodiscard]] Diligent::Uint32 maximumAnisotropy() const noexcept;
    [[nodiscard]] DiligentTexturedMeshRenderer& meshes();

  private:
    enum class State : std::uint8_t { Ready, Scene, Composited };
    DiligentMotionRenderTargets targets_;
    DiligentColorPresent colorPresent_;
    std::unique_ptr<DiligentTexturedMeshRenderer> meshes_;
    MotionDataFrame frame_;
    std::uint64_t lastCompletedFrameId_ = 0;
    double lastCompletedTimestamp_ = 0.0;
    bool hasCompletedFrame_ = false;
    bool invalidateNextHistory_ = true;
    State state_ = State::Ready;
};

} // namespace openbus::rendering
