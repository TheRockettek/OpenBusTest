#include "render/diligent/DiligentVulkanFrame.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>
#include <Graphics/GraphicsEngineVulkan/interface/EngineFactoryVk.h>

#include <stdexcept>

namespace openbus::rendering {
namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

struct DiligentVulkanFrame::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Diligent::RefCntAutoPtr<Diligent::IDeviceContext> immediateContext;
    Diligent::RefCntAutoPtr<Diligent::ISwapChain> swapChain;
    Diligent::Uint32 syncInterval = 1;
    bool suspended = false;

    Impl(const Diligent::NativeWindow& nativeWindow,
         const Diligent::SwapChainDesc& requestedSwapChainDesc, Diligent::Uint32 queueId) {
        using namespace Diligent;
        if (requestedSwapChainDesc.Width == 0 || requestedSwapChainDesc.Height == 0 ||
            requestedSwapChainDesc.ColorBufferFormat == TEX_FORMAT_UNKNOWN) {
            throw std::invalid_argument(
                "Diligent Vulkan frame requires a valid swap-chain size and format");
        }

        IEngineFactoryVk* factory = GetEngineFactoryVk();
        require(factory != nullptr, "Diligent Vulkan engine factory is unavailable");

        ImmediateContextCreateInfo contextInfo;
        contextInfo.QueueId = queueId;
        EngineVkCreateInfo engineInfo;
        engineInfo.pImmediateContextInfo = &contextInfo;
        engineInfo.NumImmediateContexts = 1;

        IRenderDevice* rawDevice = nullptr;
        IDeviceContext* rawContexts[] = {nullptr};
        factory->CreateDeviceAndContextsVk(engineInfo, &rawDevice, rawContexts);
        device.Attach(rawDevice);
        immediateContext.Attach(rawContexts[0]);
        require(device && immediateContext,
                "Diligent failed to create a Vulkan device and immediate context");

        ISwapChain* rawSwapChain = nullptr;
        factory->CreateSwapChainVk(device, immediateContext, requestedSwapChainDesc, nativeWindow,
                                   &rawSwapChain);
        swapChain.Attach(rawSwapChain);
        require(static_cast<bool>(swapChain), "Diligent failed to create a Vulkan swap chain");
    }
};

DiligentVulkanFrame::DiligentVulkanFrame(const Diligent::NativeWindow& nativeWindow,
                                         const Diligent::SwapChainDesc& swapChainDesc,
                                         Diligent::Uint32 queueId)
    : impl_(std::make_unique<Impl>(nativeWindow, swapChainDesc, queueId)) {}

DiligentVulkanFrame::~DiligentVulkanFrame() = default;

Diligent::IRenderDevice& DiligentVulkanFrame::device() const noexcept {
    return *impl_->device;
}

Diligent::IDeviceContext& DiligentVulkanFrame::immediateContext() const noexcept {
    return *impl_->immediateContext;
}

Diligent::ISwapChain& DiligentVulkanFrame::swapChain() const noexcept {
    return *impl_->swapChain;
}

Diligent::ITextureView& DiligentVulkanFrame::backBufferRenderTarget() const {
    if (impl_->suspended) {
        throw std::logic_error("Diligent Vulkan frame is suspended");
    }
    auto* renderTarget = impl_->swapChain->GetCurrentBackBufferRTV();
    require(renderTarget != nullptr, "Diligent Vulkan swap chain has no current back buffer");
    return *renderTarget;
}

bool DiligentVulkanFrame::isSuspended() const noexcept {
    return impl_->suspended;
}

void DiligentVulkanFrame::resize(Diligent::Uint32 width, Diligent::Uint32 height) {
    if (width == 0 || height == 0) {
        impl_->suspended = true;
        return;
    }
    const auto& currentDesc = impl_->swapChain->GetDesc();
    if (currentDesc.Width == width && currentDesc.Height == height) {
        impl_->suspended = false;
        return;
    }
    impl_->immediateContext->WaitForIdle();
    impl_->swapChain->Resize(width, height);
    impl_->suspended = false;
}

void DiligentVulkanFrame::setVerticalSync(bool enabled) noexcept {
    impl_->syncInterval = enabled ? 1 : 0;
}

void DiligentVulkanFrame::present() {
    if (impl_->suspended) {
        return;
    }
    impl_->immediateContext->Flush();
    impl_->immediateContext->FinishFrame();
    impl_->swapChain->Present(impl_->syncInterval);
}

void DiligentVulkanFrame::waitForIdle() {
    impl_->immediateContext->WaitForIdle();
}

} // namespace openbus::rendering
