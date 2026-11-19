#pragma once

#include <Graphics/GraphicsEngine/interface/GraphicsTypes.h>

#include <memory>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

enum class DiligentColorFilter {
    None,
    Fxaa,
    Smaa,
};

enum class DiligentDebugView {
    None,
    MotionVectors,
    MotionMagnitude,
    MotionValidity,
    LinearDepth,
};

// GPU-only, single-sample color composition or temporal debug visualization into a render target.
// Resampling uses linear filtering and clamp-to-edge; view formats control color-space conversion.
// The caller owns frame ordering, presentation and synchronization.
class DiligentColorPresent final {
  public:
    DiligentColorPresent(Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT targetFormat);
    ~DiligentColorPresent();
    DiligentColorPresent(const DiligentColorPresent&) = delete;
    DiligentColorPresent& operator=(const DiligentColorPresent&) = delete;

    // Both views must be single-sample 2D views on the renderer's device, and may not refer
    // to the same texture. This binds the target, viewport, pipeline and shader resources.
    void draw(Diligent::IDeviceContext& context, Diligent::ITextureView& source,
              Diligent::ITextureView& destination,
              DiligentColorFilter filter = DiligentColorFilter::None);
    // MotionVectors encodes signed previous-minus-current pixel displacement in R/G, centered at
    // 0.5 with a +/-32-pixel range; B shows magnitude. MotionMagnitude uses a 0-32 pixel grayscale
    // range, and MotionValidity displays valid pixels white and invalid pixels black. LinearDepth
    // logarithmically maps 0-500 view-space units to black-to-white and requires a linear-depth
    // texture. Invalid vectors are highlighted magenta in the vector and magnitude views.
    void drawDebug(Diligent::IDeviceContext& context, Diligent::ITextureView& motionVectors,
                   Diligent::ITextureView& motionValidity, Diligent::ITextureView& destination,
                   DiligentDebugView view, Diligent::ITextureView* linearDepth = nullptr);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
