#include "render/diligent/DiligentReflectionTarget.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace openbus::rendering {
namespace {

struct ColorAttachment {
    Diligent::RefCntAutoPtr<Diligent::ITexture> texture;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> renderTarget;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> shaderResource;
};

struct DepthAttachment {
    Diligent::RefCntAutoPtr<Diligent::ITexture> texture;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> depthStencil;
    Diligent::RefCntAutoPtr<Diligent::ITextureView> shaderResource;
};

struct Attachments {
    ColorAttachment color;
    DepthAttachment depth;
};

void validateFormat(Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT format,
                    Diligent::BIND_FLAGS requiredFlags, const char* name) {
    if (format == Diligent::TEX_FORMAT_UNKNOWN ||
        (device.GetTextureFormatInfoExt(format).BindFlags & requiredFlags) != requiredFlags) {
        throw std::invalid_argument(std::string("Diligent reflection ") + name +
                                    " format does not support required bindings");
    }
}

Attachments createAttachments(Diligent::IRenderDevice& device, Diligent::Uint32 size,
                              Diligent::TEXTURE_FORMAT colorFormat,
                              Diligent::TEXTURE_FORMAT depthFormat) {
    using namespace Diligent;
    if (size == 0) {
        throw std::invalid_argument("Diligent reflection target size must be positive");
    }
    validateFormat(device, colorFormat, BIND_RENDER_TARGET | BIND_SHADER_RESOURCE, "color");
    validateFormat(device, depthFormat, BIND_DEPTH_STENCIL | BIND_SHADER_RESOURCE, "depth");

    TextureDesc colorDesc;
    colorDesc.Name = "OpenBus Diligent reflection color";
    colorDesc.Type = RESOURCE_DIM_TEX_2D;
    colorDesc.Width = size;
    colorDesc.Height = size;
    colorDesc.MipLevels = 1;
    colorDesc.SampleCount = 1;
    colorDesc.Format = colorFormat;
    colorDesc.Usage = USAGE_DEFAULT;
    colorDesc.BindFlags = BIND_RENDER_TARGET | BIND_SHADER_RESOURCE;

    Attachments attachments;
    device.CreateTexture(colorDesc, nullptr, &attachments.color.texture);
    if (!attachments.color.texture) {
        throw std::runtime_error("Failed to create Diligent reflection color texture");
    }
    attachments.color.renderTarget =
        attachments.color.texture->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    attachments.color.shaderResource =
        attachments.color.texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    if (!attachments.color.renderTarget || !attachments.color.shaderResource) {
        throw std::runtime_error("Failed to create Diligent reflection color views");
    }

    TextureDesc depthDesc;
    depthDesc.Name = "OpenBus Diligent reflection depth";
    depthDesc.Type = RESOURCE_DIM_TEX_2D;
    depthDesc.Width = size;
    depthDesc.Height = size;
    depthDesc.MipLevels = 1;
    depthDesc.SampleCount = 1;
    depthDesc.Format = depthFormat;
    depthDesc.Usage = USAGE_DEFAULT;
    depthDesc.BindFlags = BIND_DEPTH_STENCIL | BIND_SHADER_RESOURCE;

    device.CreateTexture(depthDesc, nullptr, &attachments.depth.texture);
    if (!attachments.depth.texture) {
        throw std::runtime_error("Failed to create Diligent reflection depth texture");
    }
    attachments.depth.depthStencil =
        attachments.depth.texture->GetDefaultView(TEXTURE_VIEW_DEPTH_STENCIL);
    attachments.depth.shaderResource =
        attachments.depth.texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    if (!attachments.depth.depthStencil || !attachments.depth.shaderResource) {
        throw std::runtime_error("Failed to create Diligent reflection depth views");
    }
    return attachments;
}

} // namespace

struct DiligentReflectionTarget::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Diligent::Uint32 size = 0;
    Diligent::TEXTURE_FORMAT colorFormat;
    Diligent::TEXTURE_FORMAT depthFormat;
    Attachments attachments;

    Impl(Diligent::IRenderDevice& renderDevice, Diligent::Uint32 targetSize,
         Diligent::TEXTURE_FORMAT targetColorFormat, Diligent::TEXTURE_FORMAT targetDepthFormat)
        : device(&renderDevice), size(targetSize), colorFormat(targetColorFormat),
          depthFormat(targetDepthFormat),
          attachments(
              createAttachments(renderDevice, targetSize, targetColorFormat, targetDepthFormat)) {}
};

DiligentReflectionTarget::DiligentReflectionTarget(Diligent::IRenderDevice& device,
                                                   Diligent::Uint32 size,
                                                   Diligent::TEXTURE_FORMAT colorFormat,
                                                   Diligent::TEXTURE_FORMAT depthFormat)
    : impl_(std::make_unique<Impl>(device, size, colorFormat, depthFormat)) {}

DiligentReflectionTarget::~DiligentReflectionTarget() = default;

void DiligentReflectionTarget::resize(Diligent::Uint32 size) {
    if (size == 0) {
        throw std::invalid_argument("Diligent reflection target size must be positive");
    }
    if (size == impl_->size) {
        return;
    }
    Attachments replacement =
        createAttachments(*impl_->device, size, impl_->colorFormat, impl_->depthFormat);
    impl_->attachments = std::move(replacement);
    impl_->size = size;
}

void DiligentReflectionTarget::bind(Diligent::IDeviceContext& context) const {
    Diligent::ITextureView* color = impl_->attachments.color.renderTarget;
    context.SetRenderTargets(1, &color, impl_->attachments.depth.depthStencil,
                             Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    const Diligent::Viewport viewport{
        0.0F, 0.0F, static_cast<float>(impl_->size), static_cast<float>(impl_->size), 0.0F, 1.0F};
    context.SetViewports(1, &viewport, impl_->size, impl_->size);
}

void DiligentReflectionTarget::clear(Diligent::IDeviceContext& context,
                                     const std::array<float, 4>& color, float depth) const {
    if (!std::all_of(color.begin(), color.end(),
                     [](float value) { return std::isfinite(value); }) ||
        !std::isfinite(depth) || depth < 0.0F || depth > 1.0F) {
        throw std::invalid_argument("Diligent reflection clear values are invalid");
    }
    bind(context);
    context.ClearRenderTarget(impl_->attachments.color.renderTarget, color.data(),
                              Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(impl_->attachments.depth.depthStencil, Diligent::CLEAR_DEPTH_FLAG,
                              depth, 0, Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
}

Diligent::Uint32 DiligentReflectionTarget::size() const noexcept {
    return impl_->size;
}

Diligent::TEXTURE_FORMAT DiligentReflectionTarget::colorFormat() const noexcept {
    return impl_->colorFormat;
}

Diligent::TEXTURE_FORMAT DiligentReflectionTarget::depthFormat() const noexcept {
    return impl_->depthFormat;
}

Diligent::ITextureView& DiligentReflectionTarget::colorRenderTarget() const {
    return *impl_->attachments.color.renderTarget;
}

Diligent::ITextureView& DiligentReflectionTarget::colorTexture() const {
    return *impl_->attachments.color.shaderResource;
}

Diligent::ITextureView& DiligentReflectionTarget::depthStencilTarget() const {
    return *impl_->attachments.depth.depthStencil;
}

Diligent::ITextureView& DiligentReflectionTarget::depthTexture() const {
    return *impl_->attachments.depth.shaderResource;
}

} // namespace openbus::rendering
