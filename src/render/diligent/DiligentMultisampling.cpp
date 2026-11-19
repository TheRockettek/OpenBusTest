#include "render/diligent/DiligentMultisampling.h"
#include "render/diligent/DiligentRenderUtils.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace openbus::rendering {
namespace {

struct Attachments {
    Diligent::RefCntAutoPtr<Diligent::ITexture> color;
    Diligent::RefCntAutoPtr<Diligent::ITexture> depth;
    Diligent::RefCntAutoPtr<Diligent::ITexture> resolved;
};

Attachments createAttachments(Diligent::IRenderDevice& device, Diligent::Uint32 width,
                              Diligent::Uint32 height, Diligent::Uint32 samples) {
    using namespace Diligent;
    if (width == 0 || height == 0 || samples < 2) {
        throw std::invalid_argument("MSAA target dimensions and sample count are invalid");
    }
    diligent_detail::validateSamples(device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT, samples);
    TextureDesc desc;
    desc.Name = "OpenBus MSAA scene color";
    desc.Type = RESOURCE_DIM_TEX_2D;
    desc.Width = width;
    desc.Height = height;
    desc.MipLevels = 1;
    desc.SampleCount = samples;
    desc.Format = TEX_FORMAT_RGBA8_UNORM;
    desc.BindFlags = BIND_RENDER_TARGET;
    Attachments result;
    device.CreateTexture(desc, nullptr, &result.color);
    desc.Name = "OpenBus MSAA scene depth";
    desc.Format = TEX_FORMAT_D32_FLOAT;
    desc.BindFlags = BIND_DEPTH_STENCIL;
    device.CreateTexture(desc, nullptr, &result.depth);
    desc.Name = "OpenBus resolved scene color";
    desc.Format = TEX_FORMAT_RGBA8_UNORM;
    desc.SampleCount = 1;
    desc.BindFlags = BIND_SHADER_RESOURCE;
    device.CreateTexture(desc, nullptr, &result.resolved);
    if (!result.color || !result.depth || !result.resolved ||
        !result.color->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET) ||
        !result.depth->GetDefaultView(TEXTURE_VIEW_DEPTH_STENCIL) ||
        !result.resolved->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE)) {
        throw std::runtime_error("Failed to create Diligent MSAA attachments");
    }
    return result;
}

} // namespace

DiligentMultisamplingSelection selectDiligentMultisampling(Diligent::IRenderDevice& device,
                                                           bool includeLinearDepth,
                                                           DiligentMultisampling requested) {
    using namespace Diligent;
    if (!std::has_single_bit(requested.samples) || requested.samples > SAMPLE_COUNT_MAX) {
        throw std::invalid_argument("Diligent MSAA sample count must be a supported power of two");
    }
    if (requested.samples == 1) {
        return {requested, false};
    }

    Uint32 supportedSamples = std::numeric_limits<Uint32>::max();
    const auto includeFormat = [&](TEXTURE_FORMAT format, BIND_FLAGS bindFlag) {
        const auto& info = device.GetTextureFormatInfoExt(format);
        if (!(info.BindFlags & bindFlag)) {
            supportedSamples = 0;
            return;
        }
        supportedSamples &= info.SampleCounts;
    };
    includeFormat(TEX_FORMAT_RGBA8_UNORM, BIND_RENDER_TARGET);
    includeFormat(TEX_FORMAT_RG32_FLOAT, BIND_RENDER_TARGET);
    includeFormat(TEX_FORMAT_R8_UNORM, BIND_RENDER_TARGET);
    includeFormat(TEX_FORMAT_D32_FLOAT, BIND_DEPTH_STENCIL);
    if (includeLinearDepth) {
        includeFormat(TEX_FORMAT_R32_FLOAT, BIND_RENDER_TARGET);
    }
    return selectDiligentMultisamplingFromSupport(supportedSamples, requested);
}

DiligentMultisamplingSelection
selectDiligentMultisamplingFromSupport(Diligent::Uint32 supportedSampleCounts,
                                       DiligentMultisampling requested) {
    using namespace Diligent;
    if (!std::has_single_bit(requested.samples) || requested.samples > SAMPLE_COUNT_MAX) {
        throw std::invalid_argument("Diligent MSAA sample count must be a supported power of two");
    }
    if (requested.samples == 1) {
        return {requested, false};
    }

    for (Uint32 samples = requested.samples; samples >= 2; samples /= 2) {
        if (supportedSampleCounts & samples) {
            return {{samples}, false};
        }
        if (samples == 2) {
            break;
        }
    }
    return {{1}, true};
}

struct DiligentMultisampleTargets::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Attachments attachments;
    Diligent::Uint32 width;
    Diligent::Uint32 height;
    Diligent::Uint32 samples;
    Impl(Diligent::IRenderDevice& device, Diligent::Uint32 width, Diligent::Uint32 height,
         Diligent::Uint32 samples)
        : device(&device), attachments(createAttachments(device, width, height, samples)),
          width(width), height(height), samples(samples) {}
};

DiligentMultisampleTargets::DiligentMultisampleTargets(Diligent::IRenderDevice& device,
                                                       Diligent::Uint32 width,
                                                       Diligent::Uint32 height,
                                                       Diligent::Uint32 samples)
    : impl_(std::make_unique<Impl>(device, width, height, samples)) {}
DiligentMultisampleTargets::~DiligentMultisampleTargets() = default;

void DiligentMultisampleTargets::resize(Diligent::IDeviceContext& context, Diligent::Uint32 width,
                                        Diligent::Uint32 height) {
    if (width == 0 || height == 0) {
        throw std::invalid_argument("MSAA target dimensions must be positive");
    }
    if (width == impl_->width && height == impl_->height) {
        return;
    }
    context.WaitForIdle();
    auto replacement = createAttachments(*impl_->device, width, height, impl_->samples);
    impl_->attachments = std::move(replacement);
    impl_->width = width;
    impl_->height = height;
}

void DiligentMultisampleTargets::bind(Diligent::IDeviceContext& context) const {
    using namespace Diligent;
    ITextureView* color = impl_->attachments.color->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    context.SetRenderTargets(1, &color,
                             impl_->attachments.depth->GetDefaultView(TEXTURE_VIEW_DEPTH_STENCIL),
                             RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
}

void DiligentMultisampleTargets::clear(Diligent::IDeviceContext& context,
                                       const std::array<float, 4>& color, float depth) const {
    using namespace Diligent;
    if (!std::all_of(color.begin(), color.end(),
                     [](float value) { return std::isfinite(value); }) ||
        !std::isfinite(depth) || depth < 0.0F || depth > 1.0F) {
        throw std::invalid_argument("MSAA clear values are invalid");
    }
    bind(context);
    context.ClearRenderTarget(impl_->attachments.color->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET),
                              color.data(), RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearDepthStencil(impl_->attachments.depth->GetDefaultView(TEXTURE_VIEW_DEPTH_STENCIL),
                              CLEAR_DEPTH_FLAG, depth, 0,
                              RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
}

void DiligentMultisampleTargets::resolve(Diligent::IDeviceContext& context) const {
    using namespace Diligent;
    context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
    ResolveTextureSubresourceAttribs attributes;
    attributes.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    attributes.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
    context.ResolveTextureSubresource(impl_->attachments.color, impl_->attachments.resolved,
                                      attributes);
}

Diligent::ITextureView& DiligentMultisampleTargets::resolvedColor() const {
    return *impl_->attachments.resolved->GetDefaultView(Diligent::TEXTURE_VIEW_SHADER_RESOURCE);
}
Diligent::Uint32 DiligentMultisampleTargets::width() const noexcept {
    return impl_->width;
}
Diligent::Uint32 DiligentMultisampleTargets::height() const noexcept {
    return impl_->height;
}
Diligent::Uint32 DiligentMultisampleTargets::samples() const noexcept {
    return impl_->samples;
}

} // namespace openbus::rendering
