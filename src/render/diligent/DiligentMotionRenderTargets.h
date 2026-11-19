#pragma once

#include <Graphics/GraphicsEngine/interface/GraphicsTypes.h>
#include "render/diligent/DiligentMultisampling.h"

#include <array>
#include <memory>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// Color, motion, validity, optional linear-depth and depth-stencil attachments.
// The target set owns all textures and views; the caller owns frame ordering and history policy.
class DiligentMotionRenderTargets final {
  public:
    DiligentMotionRenderTargets(Diligent::IRenderDevice& device, Diligent::Uint32 width,
                                Diligent::Uint32 height, bool includeLinearDepth = true,
                                DiligentMultisampling multisampling = {});
    ~DiligentMotionRenderTargets();
    DiligentMotionRenderTargets(const DiligentMotionRenderTargets&) = delete;
    DiligentMotionRenderTargets& operator=(const DiligentMotionRenderTargets&) = delete;

    // Waits for immediate-context work before replacing attachments. Zero-sized resizes are
    // rejected.
    void resize(Diligent::IDeviceContext& immediateContext, Diligent::Uint32 width,
                Diligent::Uint32 height);
    void bind(Diligent::IDeviceContext& context) const;
    // Clears motion, validity, and optional linear depth to zero; depth-stencil to clearDepth.
    void clear(Diligent::IDeviceContext& context,
               const std::array<float, 4>& clearColor = {0.0F, 0.0F, 0.0F, 1.0F},
               float clearDepth = 1.0F) const;
    // For MSAA, average color and select all metadata and depth from the nearest
    // depth sample. Single-sample targets need no resolve. Call after scene draws.
    void resolve(Diligent::IDeviceContext& context) const;

    [[nodiscard]] Diligent::Uint32 width() const noexcept;
    [[nodiscard]] Diligent::Uint32 height() const noexcept;
    [[nodiscard]] bool hasLinearDepth() const noexcept;
    [[nodiscard]] Diligent::Uint32 samples() const noexcept;

    [[nodiscard]] Diligent::ITextureView& colorRenderTarget() const;
    // Texture views refer to the resolved attachments when multisampling is enabled.
    // Reacquire borrowed views after resize.
    [[nodiscard]] Diligent::ITextureView& colorTexture() const;
    [[nodiscard]] Diligent::ITextureView& motionVectorRenderTarget() const;
    [[nodiscard]] Diligent::ITextureView& motionVectorTexture() const;
    [[nodiscard]] Diligent::ITextureView& motionValidityRenderTarget() const;
    [[nodiscard]] Diligent::ITextureView& motionValidityTexture() const;
    [[nodiscard]] Diligent::ITextureView* linearDepthRenderTarget() const noexcept;
    [[nodiscard]] Diligent::ITextureView* linearDepthTexture() const noexcept;
    [[nodiscard]] Diligent::ITextureView& depthStencilTarget() const;
    [[nodiscard]] Diligent::ITextureView& depthTexture() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
