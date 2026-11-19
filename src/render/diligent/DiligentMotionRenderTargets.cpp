#include "render/diligent/DiligentMotionRenderTargets.h"
#include "render/diligent/DiligentRenderUtils.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/PipelineState.h>
#include <Graphics/GraphicsEngine/interface/ShaderResourceBinding.h>
#include <Graphics/GraphicsEngine/interface/ShaderResourceVariable.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace openbus::rendering {
namespace {

constexpr char resolveVertexSource[] = R"(
struct Output { float4 Position : SV_POSITION; };
Output main(uint vertexId : SV_VertexID)
{
    Output result;
    float2 uv = float2((vertexId << 1) & 2, vertexId & 2);
    result.Position = float4(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 0.0, 1.0);
    return result;
}
)";

constexpr char resolvePixelSource[] = R"(
Texture2DMS<float4> ColorSamples;
Texture2DMS<float2> MotionSamples;
Texture2DMS<float> ValiditySamples;
Texture2DMS<float> DepthSamples;
Texture2DMS<float> LinearDepthSamples;
struct Output {
    float4 color : SV_TARGET0;
    float2 motion : SV_TARGET1;
    float validity : SV_TARGET2;
    float depth : SV_DEPTH;
};
struct OutputLinear {
    float4 color : SV_TARGET0;
    float2 motion : SV_TARGET1;
    float validity : SV_TARGET2;
    float linearDepth : SV_TARGET3;
    float depth : SV_DEPTH;
};
OutputLinear resolveSamples(float4 position, out uint nearest) {
    int2 pixel = int2(position.xy);
    uint width, height, count;
    DepthSamples.GetDimensions(width, height, count);
    float4 color = 0.0;
    float depth = 1.0;
    nearest = 0;
    for (uint i = 0; i < count; ++i) {
        color += ColorSamples.Load(pixel, i);
        float sampleDepth = DepthSamples.Load(pixel, i);
        if (sampleDepth < depth) {
            depth = sampleDepth;
            nearest = i;
        }
    }
    OutputLinear result;
    result.color = color / count;
    result.motion = MotionSamples.Load(pixel, nearest);
    result.validity = ValiditySamples.Load(pixel, nearest);
    result.linearDepth = 0;
    result.depth = depth;
    return result;
}
Output main(float4 position : SV_POSITION) {
    uint nearest;
    OutputLinear source = resolveSamples(position, nearest);
    Output result;
    result.color = source.color;
    result.motion = source.motion;
    result.validity = source.validity;
    result.depth = source.depth;
    return result;
}
OutputLinear mainLinear(float4 position : SV_POSITION) {
    uint nearest;
    OutputLinear result = resolveSamples(position, nearest);
    result.linearDepth = LinearDepthSamples.Load(int2(position.xy), nearest);
    return result;
}
)";

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
    ColorAttachment motion;
    ColorAttachment validity;
    ColorAttachment linearDepth;
    DepthAttachment depth;
};

ColorAttachment createColorAttachment(Diligent::IRenderDevice& device, const char* name,
                                      Diligent::Uint32 width, Diligent::Uint32 height,
                                      Diligent::TEXTURE_FORMAT format, Diligent::Uint32 samples) {
    using namespace Diligent;
    TextureDesc desc;
    desc.Name = name;
    desc.Type = RESOURCE_DIM_TEX_2D;
    desc.Width = width;
    desc.Height = height;
    desc.MipLevels = 1;
    desc.SampleCount = samples;
    desc.Format = format;
    desc.Usage = USAGE_DEFAULT;
    desc.BindFlags = BIND_RENDER_TARGET | BIND_SHADER_RESOURCE;

    ColorAttachment attachment;
    device.CreateTexture(desc, nullptr, &attachment.texture);
    if (!attachment.texture) {
        throw std::runtime_error(std::string("Failed to create Diligent render target: ") + name);
    }
    attachment.renderTarget = attachment.texture->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    attachment.shaderResource = attachment.texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    if (!attachment.renderTarget || !attachment.shaderResource) {
        throw std::runtime_error(std::string("Failed to create Diligent render-target views: ") +
                                 name);
    }
    return attachment;
}

DepthAttachment createDepthAttachment(Diligent::IRenderDevice& device, Diligent::Uint32 width,
                                      Diligent::Uint32 height, Diligent::Uint32 samples) {
    using namespace Diligent;
    TextureDesc desc;
    desc.Name = "OpenBus Diligent depth-stencil target";
    desc.Type = RESOURCE_DIM_TEX_2D;
    desc.Width = width;
    desc.Height = height;
    desc.MipLevels = 1;
    desc.SampleCount = samples;
    desc.Format = TEX_FORMAT_D32_FLOAT;
    desc.Usage = USAGE_DEFAULT;
    desc.BindFlags = BIND_DEPTH_STENCIL | BIND_SHADER_RESOURCE;

    DepthAttachment attachment;
    device.CreateTexture(desc, nullptr, &attachment.texture);
    if (!attachment.texture) {
        throw std::runtime_error("Failed to create Diligent depth-stencil target");
    }
    attachment.depthStencil = attachment.texture->GetDefaultView(TEXTURE_VIEW_DEPTH_STENCIL);
    attachment.shaderResource = attachment.texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
    if (!attachment.depthStencil || !attachment.shaderResource) {
        throw std::runtime_error("Failed to create Diligent depth-stencil views");
    }
    return attachment;
}

Attachments createAttachments(Diligent::IRenderDevice& device, Diligent::Uint32 width,
                              Diligent::Uint32 height, bool includeLinearDepth,
                              Diligent::Uint32 samples) {
    using namespace Diligent;
    if (width == 0 || height == 0) {
        throw std::invalid_argument("Diligent render-target dimensions must be positive");
    }
    diligent_detail::validateSamples(device, TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_D32_FLOAT, samples);
    diligent_detail::validateSamples(device, TEX_FORMAT_RG32_FLOAT, TEX_FORMAT_UNKNOWN, samples);
    diligent_detail::validateSamples(device, TEX_FORMAT_R8_UNORM, TEX_FORMAT_UNKNOWN, samples);
    if (includeLinearDepth) {
        diligent_detail::validateSamples(device, TEX_FORMAT_R32_FLOAT, TEX_FORMAT_UNKNOWN, samples);
    }
    for (const auto format : {TEX_FORMAT_RGBA8_UNORM, TEX_FORMAT_RG32_FLOAT, TEX_FORMAT_R8_UNORM,
                              TEX_FORMAT_D32_FLOAT}) {
        if (!(device.GetTextureFormatInfoExt(format).BindFlags & BIND_SHADER_RESOURCE)) {
            throw std::invalid_argument("Diligent motion target format cannot be sampled");
        }
    }
    if (includeLinearDepth &&
        !(device.GetTextureFormatInfoExt(TEX_FORMAT_R32_FLOAT).BindFlags & BIND_SHADER_RESOURCE)) {
        throw std::invalid_argument("Diligent linear depth format cannot be sampled");
    }
    Attachments attachments;
    attachments.color = createColorAttachment(device, "OpenBus Diligent scene color", width, height,
                                              TEX_FORMAT_RGBA8_UNORM, samples);
    attachments.motion = createColorAttachment(device, "OpenBus Diligent motion vectors", width,
                                               height, TEX_FORMAT_RG32_FLOAT, samples);
    attachments.validity = createColorAttachment(device, "OpenBus Diligent motion validity", width,
                                                 height, TEX_FORMAT_R8_UNORM, samples);
    if (includeLinearDepth) {
        attachments.linearDepth = createColorAttachment(
            device, "OpenBus Diligent linear depth", width, height, TEX_FORMAT_R32_FLOAT, samples);
    }
    attachments.depth = createDepthAttachment(device, width, height, samples);
    return attachments;
}

} // namespace

struct DiligentMotionRenderTargets::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Attachments attachments;
    Attachments resolved;
    Diligent::RefCntAutoPtr<Diligent::IPipelineState> resolvePipeline;
    Diligent::RefCntAutoPtr<Diligent::IShaderResourceBinding> resolveBinding;
    Diligent::Uint32 width = 0;
    Diligent::Uint32 height = 0;
    bool includeLinearDepth = false;
    Diligent::Uint32 samples = 1;

    Impl(Diligent::IRenderDevice& renderDevice, Diligent::Uint32 targetWidth,
         Diligent::Uint32 targetHeight, bool withLinearDepth, Diligent::Uint32 sampleCount)
        : device(&renderDevice), width(targetWidth), height(targetHeight),
          includeLinearDepth(withLinearDepth), samples(sampleCount) {
        using namespace Diligent;
        attachments = createAttachments(*device, width, height, includeLinearDepth, samples);
        if (samples == 1) {
            return;
        }
        resolved = createAttachments(*device, width, height, includeLinearDepth, 1);
        auto vertex = diligent_detail::createShader(renderDevice, SHADER_TYPE_VERTEX,
                                                    "OpenBus MSAA resolve VS", resolveVertexSource);
        auto pixel = diligent_detail::createShader(
            renderDevice, SHADER_TYPE_PIXEL, "OpenBus MSAA motion resolve PS", resolvePixelSource,
            false, includeLinearDepth ? "mainLinear" : "main");
        const ShaderResourceVariableDesc variables[] = {
            {SHADER_TYPE_PIXEL, "ColorSamples", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "MotionSamples", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "ValiditySamples", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "DepthSamples", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "LinearDepthSamples", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
        };
        GraphicsPipelineStateCreateInfo info;
        info.PSODesc.Name = "OpenBus motion MSAA resolve";
        info.PSODesc.PipelineType = PIPELINE_TYPE_GRAPHICS;
        info.PSODesc.ResourceLayout.Variables = variables;
        info.PSODesc.ResourceLayout.NumVariables = includeLinearDepth ? 5 : 4;
        info.GraphicsPipeline.NumRenderTargets = includeLinearDepth ? 4 : 3;
        info.GraphicsPipeline.RTVFormats[0] = TEX_FORMAT_RGBA8_UNORM;
        info.GraphicsPipeline.RTVFormats[1] = TEX_FORMAT_RG32_FLOAT;
        info.GraphicsPipeline.RTVFormats[2] = TEX_FORMAT_R8_UNORM;
        if (includeLinearDepth) {
            info.GraphicsPipeline.RTVFormats[3] = TEX_FORMAT_R32_FLOAT;
        }
        info.GraphicsPipeline.DSVFormat = TEX_FORMAT_D32_FLOAT;
        info.GraphicsPipeline.PrimitiveTopology = PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        info.GraphicsPipeline.RasterizerDesc.CullMode = CULL_MODE_NONE;
        info.GraphicsPipeline.DepthStencilDesc.DepthEnable = True;
        info.GraphicsPipeline.DepthStencilDesc.DepthWriteEnable = True;
        info.GraphicsPipeline.DepthStencilDesc.DepthFunc = COMPARISON_FUNC_ALWAYS;
        info.pVS = vertex;
        info.pPS = pixel;
        renderDevice.CreateGraphicsPipelineState(info, &resolvePipeline);
        if (!resolvePipeline) {
            throw std::runtime_error("Failed to create Diligent motion resolve pipeline");
        }
        resolvePipeline->CreateShaderResourceBinding(&resolveBinding, true);
        if (!resolveBinding) {
            throw std::runtime_error("Failed to create Diligent motion resolve bindings");
        }
    }
};

DiligentMotionRenderTargets::DiligentMotionRenderTargets(Diligent::IRenderDevice& device,
                                                         Diligent::Uint32 width,
                                                         Diligent::Uint32 height,
                                                         bool includeLinearDepth,
                                                         DiligentMultisampling multisampling)
    : impl_(std::make_unique<Impl>(device, width, height, includeLinearDepth,
                                   multisampling.samples)) {}

DiligentMotionRenderTargets::~DiligentMotionRenderTargets() = default;

void DiligentMotionRenderTargets::resize(Diligent::IDeviceContext& immediateContext,
                                         Diligent::Uint32 width, Diligent::Uint32 height) {
    if (width == 0 || height == 0) {
        throw std::invalid_argument("Diligent render-target dimensions must be positive");
    }
    if (width == impl_->width && height == impl_->height) {
        return;
    }
    immediateContext.WaitForIdle();
    Attachments newAttachments =
        createAttachments(*impl_->device, width, height, impl_->includeLinearDepth, impl_->samples);
    Attachments newResolved;
    if (impl_->samples > 1) {
        newResolved =
            createAttachments(*impl_->device, width, height, impl_->includeLinearDepth, 1);
    }
    impl_->attachments = std::move(newAttachments);
    impl_->resolved = std::move(newResolved);
    impl_->width = width;
    impl_->height = height;
}

void DiligentMotionRenderTargets::resolve(Diligent::IDeviceContext& context) const {
    using namespace Diligent;
    if (impl_->samples == 1) {
        return;
    }
    auto& source = impl_->attachments;
    const std::array<std::pair<const char*, ITextureView*>, 5> resources = {{
        {"ColorSamples", source.color.shaderResource},
        {"MotionSamples", source.motion.shaderResource},
        {"ValiditySamples", source.validity.shaderResource},
        {"DepthSamples", source.depth.shaderResource},
        {"LinearDepthSamples", source.linearDepth.shaderResource},
    }};
    const std::size_t count = impl_->includeLinearDepth ? 5 : 4;
    for (std::size_t i = 0; i < count; ++i) {
        auto* variable =
            impl_->resolveBinding->GetVariableByName(SHADER_TYPE_PIXEL, resources[i].first);
        if (!variable) {
            throw std::runtime_error("Diligent motion resolve resource binding is missing");
        }
        variable->Set(resources[i].second);
    }
    auto& destination = impl_->resolved;
    std::array<ITextureView*, 4> targets = {
        destination.color.renderTarget, destination.motion.renderTarget,
        destination.validity.renderTarget, destination.linearDepth.renderTarget};
    context.SetRenderTargets(impl_->includeLinearDepth ? 4 : 3, targets.data(),
                             destination.depth.depthStencil,
                             RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    const Viewport viewport{
        0.0F, 0.0F, static_cast<float>(impl_->width), static_cast<float>(impl_->height),
        0.0F, 1.0F};
    context.SetViewports(1, &viewport, impl_->width, impl_->height);
    context.SetPipelineState(impl_->resolvePipeline);
    context.CommitShaderResources(impl_->resolveBinding, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    DrawAttribs draw;
    draw.NumVertices = 3;
    context.Draw(draw);
}

void DiligentMotionRenderTargets::bind(Diligent::IDeviceContext& context) const {
    using namespace Diligent;
    std::array<ITextureView*, 4> renderTargets = {
        impl_->attachments.color.renderTarget,
        impl_->attachments.motion.renderTarget,
        impl_->attachments.validity.renderTarget,
        impl_->attachments.linearDepth.renderTarget,
    };
    const Uint32 renderTargetCount = impl_->includeLinearDepth ? 4 : 3;
    context.SetRenderTargets(renderTargetCount, renderTargets.data(),
                             impl_->attachments.depth.depthStencil,
                             RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
}

void DiligentMotionRenderTargets::clear(Diligent::IDeviceContext& context,
                                        const std::array<float, 4>& clearColor,
                                        float clearDepth) const {
    if (!std::all_of(clearColor.begin(), clearColor.end(),
                     [](float value) { return std::isfinite(value); }) ||
        !std::isfinite(clearDepth) || clearDepth < 0.0F || clearDepth > 1.0F) {
        throw std::invalid_argument("Diligent render-target clear values are invalid");
    }
    bind(context);
    constexpr float clearMetadata[] = {0.0F, 0.0F, 0.0F, 0.0F};
    context.ClearRenderTarget(impl_->attachments.color.renderTarget, clearColor.data(),
                              Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(impl_->attachments.motion.renderTarget, clearMetadata,
                              Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    context.ClearRenderTarget(impl_->attachments.validity.renderTarget, clearMetadata,
                              Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    if (impl_->includeLinearDepth) {
        context.ClearRenderTarget(impl_->attachments.linearDepth.renderTarget, clearMetadata,
                                  Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    }
    context.ClearDepthStencil(impl_->attachments.depth.depthStencil, Diligent::CLEAR_DEPTH_FLAG,
                              clearDepth, 0, Diligent::RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
}

Diligent::Uint32 DiligentMotionRenderTargets::width() const noexcept {
    return impl_->width;
}

Diligent::Uint32 DiligentMotionRenderTargets::height() const noexcept {
    return impl_->height;
}

bool DiligentMotionRenderTargets::hasLinearDepth() const noexcept {
    return impl_->includeLinearDepth;
}
Diligent::Uint32 DiligentMotionRenderTargets::samples() const noexcept {
    return impl_->samples;
}

Diligent::ITextureView& DiligentMotionRenderTargets::colorRenderTarget() const {
    return *impl_->attachments.color.renderTarget;
}

Diligent::ITextureView& DiligentMotionRenderTargets::colorTexture() const {
    return *(impl_->samples == 1 ? impl_->attachments : impl_->resolved).color.shaderResource;
}

Diligent::ITextureView& DiligentMotionRenderTargets::motionVectorRenderTarget() const {
    return *impl_->attachments.motion.renderTarget;
}

Diligent::ITextureView& DiligentMotionRenderTargets::motionVectorTexture() const {
    return *(impl_->samples == 1 ? impl_->attachments : impl_->resolved).motion.shaderResource;
}

Diligent::ITextureView& DiligentMotionRenderTargets::motionValidityRenderTarget() const {
    return *impl_->attachments.validity.renderTarget;
}

Diligent::ITextureView& DiligentMotionRenderTargets::motionValidityTexture() const {
    return *(impl_->samples == 1 ? impl_->attachments : impl_->resolved).validity.shaderResource;
}

Diligent::ITextureView* DiligentMotionRenderTargets::linearDepthRenderTarget() const noexcept {
    return impl_->attachments.linearDepth.renderTarget;
}

Diligent::ITextureView* DiligentMotionRenderTargets::linearDepthTexture() const noexcept {
    return (impl_->samples == 1 ? impl_->attachments : impl_->resolved).linearDepth.shaderResource;
}

Diligent::ITextureView& DiligentMotionRenderTargets::depthStencilTarget() const {
    return *impl_->attachments.depth.depthStencil;
}

Diligent::ITextureView& DiligentMotionRenderTargets::depthTexture() const {
    return *(impl_->samples == 1 ? impl_->attachments : impl_->resolved).depth.shaderResource;
}

} // namespace openbus::rendering
