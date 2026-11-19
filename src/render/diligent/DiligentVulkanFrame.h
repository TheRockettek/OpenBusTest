#pragma once

#include "render/RendererFramePresenter.h"

#include <Graphics/GraphicsEngine/interface/SwapChain.h>
#include <Graphics/GraphicsEngineVulkan/interface/EngineFactoryVk.h>

#include <memory>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// Owns the Vulkan device, immediate context and swap chain for one native window.
// The caller remains responsible for creating the platform window and binding
// render targets before drawing.
class DiligentVulkanFrame final : public RendererFramePresenter {
  public:
    DiligentVulkanFrame(const Diligent::NativeWindow& nativeWindow,
                        const Diligent::SwapChainDesc& swapChainDesc, Diligent::Uint32 queueId = 0);
    ~DiligentVulkanFrame();
    DiligentVulkanFrame(const DiligentVulkanFrame&) = delete;
    DiligentVulkanFrame& operator=(const DiligentVulkanFrame&) = delete;

    [[nodiscard]] RendererBackend backend() const noexcept override {
        return RendererBackend::Diligent;
    }
    [[nodiscard]] Diligent::IRenderDevice& device() const noexcept;
    [[nodiscard]] Diligent::IDeviceContext& immediateContext() const noexcept;
    [[nodiscard]] Diligent::ISwapChain& swapChain() const noexcept;
    [[nodiscard]] Diligent::ITextureView& backBufferRenderTarget() const;
    [[nodiscard]] bool isSuspended() const noexcept override;

    // A zero dimension suspends presentation until a nonzero size is supplied.
    void resize(Diligent::Uint32 width, Diligent::Uint32 height) override;
    void setVerticalSync(bool enabled) noexcept override;
    void present() override;
    void waitForIdle() override;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
