#include "render/diligent/DiligentColorPresent.h"
#include "render/diligent/DiligentRenderUtils.h"
#include "render/diligent/DiligentSmaa.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/Buffer.h>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/PipelineState.h>
#include <Graphics/GraphicsEngine/interface/ShaderResourceBinding.h>
#include <Graphics/GraphicsEngine/interface/ShaderResourceVariable.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <algorithm>
#include <array>
#include <stdexcept>

namespace openbus::rendering {
namespace {

constexpr char vertexSource[] = R"(
struct Output
{
    float4 Position : SV_POSITION;
    float2 UV : TEXCOORD0;
};
Output main(uint vertexId : SV_VertexID)
{
    Output output;
    output.UV = float2((vertexId << 1) & 2, vertexId & 2);
    output.Position = float4(output.UV.x * 2.0 - 1.0, 1.0 - output.UV.y * 2.0, 0.0, 1.0);
    return output;
}
)";

constexpr char pixelSource[] = R"(
Texture2D SceneColor;
SamplerState SceneColor_sampler;
float4 main(float2 uv : TEXCOORD0) : SV_TARGET
{
    return SceneColor.Sample(SceneColor_sampler, uv);
}
float luma(float3 color)
{
    return dot(color, float3(0.299, 0.587, 0.114));
}
float4 fxaa(float2 uv : TEXCOORD0) : SV_TARGET
{
    uint width, height;
    SceneColor.GetDimensions(width, height);
    float2 inverseResolution = 1.0 / float2(width, height);
    float4 center = SceneColor.Sample(SceneColor_sampler, uv);
    float nw = luma(SceneColor.Sample(SceneColor_sampler, uv + float2(-1, -1) * inverseResolution).rgb);
    float ne = luma(SceneColor.Sample(SceneColor_sampler, uv + float2(1, -1) * inverseResolution).rgb);
    float sw = luma(SceneColor.Sample(SceneColor_sampler, uv + float2(-1, 1) * inverseResolution).rgb);
    float se = luma(SceneColor.Sample(SceneColor_sampler, uv + float2(1, 1) * inverseResolution).rgb);
    float middle = luma(center.rgb);
    float minimum = min(middle, min(min(nw, ne), min(sw, se)));
    float maximum = max(middle, max(max(nw, ne), max(sw, se)));
    if (maximum - minimum < max(0.0312, maximum * 0.125))
        return center;
    float2 direction = float2(-((nw + ne) - (sw + se)), (nw + sw) - (ne + se));
    float reduce = max((nw + ne + sw + se) * 0.03125, 1.0 / 128.0);
    direction = clamp(direction / (min(abs(direction.x), abs(direction.y)) + reduce),
                      -8.0, 8.0) * inverseResolution;
    float3 colorA = 0.5 * (SceneColor.Sample(SceneColor_sampler, uv + direction * (1.0 / 3.0 - 0.5)).rgb +
                           SceneColor.Sample(SceneColor_sampler, uv + direction * (2.0 / 3.0 - 0.5)).rgb);
    float3 colorB = colorA * 0.5 +
                    0.25 * (SceneColor.Sample(SceneColor_sampler, uv + direction * -0.5).rgb +
                            SceneColor.Sample(SceneColor_sampler, uv + direction * 0.5).rgb);
    float lumaB = luma(colorB);
    return float4(lumaB < minimum || lumaB > maximum ? colorA : colorB, center.a);
}
)";

constexpr char debugPixelSource[] = R"(
Texture2D<float2> MotionVectors;
Texture2D<float> MotionValidity;
Texture2D<float> LinearDepth;
SamplerState DebugSampler;
cbuffer DebugParameters
{
    uint4 DebugOptions;
};
float4 main(float2 uv : TEXCOORD0) : SV_TARGET
{
    if (DebugOptions.x == 4)
    {
        float depth = LinearDepth.Sample(DebugSampler, uv);
        float normalizedDepth = saturate(log2(1.0 + depth) / log2(501.0));
        return float4(normalizedDepth, normalizedDepth, normalizedDepth, 1.0);
    }
    float validity = MotionValidity.Sample(DebugSampler, uv);
    if (DebugOptions.x == 3)
        return float4(validity, validity, validity, 1.0);
    if (validity < 0.5)
        return float4(1.0, 0.0, 1.0, 1.0);
    float2 motion = MotionVectors.Sample(DebugSampler, uv);
    if (DebugOptions.x == 1)
        return float4(saturate(0.5 + motion / 64.0),
                      saturate(length(motion) / 32.0), 1.0);
    float magnitude = saturate(length(motion) / 32.0);
    return float4(magnitude, magnitude, magnitude, 1.0);
}
)";

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

struct DiligentColorPresent::Impl {
    Diligent::TEXTURE_FORMAT targetFormat;
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    std::unique_ptr<DiligentSmaa> smaa;
    struct Pipeline {
        Diligent::RefCntAutoPtr<Diligent::IPipelineState> state;
        Diligent::RefCntAutoPtr<Diligent::IShaderResourceBinding> binding;
    };
    std::array<Pipeline, 2> pipelines;
    Pipeline debugPipeline;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> debugParameters;

    Impl(Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT format)
        : targetFormat(format), device(&device) {
        using namespace Diligent;
        if (format == TEX_FORMAT_UNKNOWN) {
            throw std::invalid_argument("Diligent color composition requires a target format");
        }
        auto vertex = diligent_detail::createShader(device, SHADER_TYPE_VERTEX,
                                                    "OpenBus color present VS", vertexSource);
        const ShaderResourceVariableDesc variable{SHADER_TYPE_PIXEL, "SceneColor",
                                                  SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC};
        SamplerDesc sampler;
        sampler.AddressU = TEXTURE_ADDRESS_CLAMP;
        sampler.AddressV = TEXTURE_ADDRESS_CLAMP;
        sampler.MinFilter = FILTER_TYPE_LINEAR;
        sampler.MagFilter = FILTER_TYPE_LINEAR;
        sampler.MipFilter = FILTER_TYPE_POINT;
        const ImmutableSamplerDesc immutableSampler{SHADER_TYPE_PIXEL, "SceneColor", sampler};
        GraphicsPipelineStateCreateInfo info;
        info.PSODesc.Name = "OpenBus color present";
        info.PSODesc.PipelineType = PIPELINE_TYPE_GRAPHICS;
        info.PSODesc.ResourceLayout.Variables = &variable;
        info.PSODesc.ResourceLayout.NumVariables = 1;
        info.PSODesc.ResourceLayout.ImmutableSamplers = &immutableSampler;
        info.PSODesc.ResourceLayout.NumImmutableSamplers = 1;
        info.GraphicsPipeline.NumRenderTargets = 1;
        info.GraphicsPipeline.RTVFormats[0] = format;
        info.GraphicsPipeline.PrimitiveTopology = PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        info.GraphicsPipeline.RasterizerDesc.CullMode = CULL_MODE_NONE;
        info.GraphicsPipeline.DepthStencilDesc.DepthEnable = False;
        info.pVS = vertex;
        for (std::size_t index = 0; index < pipelines.size(); ++index) {
            const bool fxaa = index == 1;
            auto pixel = diligent_detail::createShader(device, SHADER_TYPE_PIXEL,
                                                       fxaa ? "OpenBus FXAA present PS"
                                                            : "OpenBus color present PS",
                                                       pixelSource, true, fxaa ? "fxaa" : "main");
            info.pPS = pixel;
            auto& pipeline = pipelines[index];
            device.CreateGraphicsPipelineState(info, &pipeline.state);
            require(static_cast<bool>(pipeline.state),
                    "Failed to create Diligent color present pipeline");
            pipeline.state->CreateShaderResourceBinding(&pipeline.binding, true);
            require(static_cast<bool>(pipeline.binding),
                    "Failed to create Diligent color present bindings");
        }
    }

    void ensureDebugPipeline() {
        using namespace Diligent;
        if (debugPipeline.state && debugPipeline.binding && debugParameters) {
            return;
        }
        const ShaderResourceVariableDesc debugVariables[] = {
            {SHADER_TYPE_PIXEL, "MotionVectors", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "MotionValidity", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "LinearDepth", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "DebugParameters", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
        };
        SamplerDesc sampler;
        sampler.AddressU = TEXTURE_ADDRESS_CLAMP;
        sampler.AddressV = TEXTURE_ADDRESS_CLAMP;
        sampler.MinFilter = FILTER_TYPE_LINEAR;
        sampler.MagFilter = FILTER_TYPE_LINEAR;
        sampler.MipFilter = FILTER_TYPE_POINT;
        const ImmutableSamplerDesc debugSampler{SHADER_TYPE_PIXEL, "DebugSampler", sampler};
        auto vertex = diligent_detail::createShader(*device, SHADER_TYPE_VERTEX,
                                                    "OpenBus motion debug VS", vertexSource);
        auto pixel = diligent_detail::createShader(*device, SHADER_TYPE_PIXEL,
                                                   "OpenBus motion debug PS", debugPixelSource);
        GraphicsPipelineStateCreateInfo info;
        info.PSODesc.Name = "OpenBus motion debug";
        info.PSODesc.PipelineType = PIPELINE_TYPE_GRAPHICS;
        info.PSODesc.ResourceLayout.Variables = debugVariables;
        info.PSODesc.ResourceLayout.NumVariables = static_cast<Uint32>(std::size(debugVariables));
        info.PSODesc.ResourceLayout.ImmutableSamplers = &debugSampler;
        info.PSODesc.ResourceLayout.NumImmutableSamplers = 1;
        info.GraphicsPipeline.NumRenderTargets = 1;
        info.GraphicsPipeline.RTVFormats[0] = targetFormat;
        info.GraphicsPipeline.PrimitiveTopology = PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        info.GraphicsPipeline.RasterizerDesc.CullMode = CULL_MODE_NONE;
        info.GraphicsPipeline.DepthStencilDesc.DepthEnable = False;
        info.pVS = vertex;
        info.pPS = pixel;
        device->CreateGraphicsPipelineState(info, &debugPipeline.state);
        require(static_cast<bool>(debugPipeline.state),
                "Failed to create Diligent motion debug pipeline");
        debugPipeline.state->CreateShaderResourceBinding(&debugPipeline.binding, true);
        require(static_cast<bool>(debugPipeline.binding),
                "Failed to create Diligent motion debug bindings");

        BufferDesc bufferDesc;
        bufferDesc.Name = "OpenBus motion debug parameters";
        bufferDesc.Size = sizeof(Uint32) * 4;
        bufferDesc.Usage = USAGE_DYNAMIC;
        bufferDesc.BindFlags = BIND_UNIFORM_BUFFER;
        bufferDesc.CPUAccessFlags = CPU_ACCESS_WRITE;
        device->CreateBuffer(bufferDesc, nullptr, &debugParameters);
        require(static_cast<bool>(debugParameters),
                "Failed to create Diligent motion debug parameters");
    }
};

DiligentColorPresent::DiligentColorPresent(Diligent::IRenderDevice& device,
                                           Diligent::TEXTURE_FORMAT targetFormat)
    : impl_(std::make_unique<Impl>(device, targetFormat)) {}

DiligentColorPresent::~DiligentColorPresent() = default;

void DiligentColorPresent::draw(Diligent::IDeviceContext& context, Diligent::ITextureView& source,
                                Diligent::ITextureView& destination, DiligentColorFilter filter) {
    using namespace Diligent;
    std::size_t pipelineIndex;
    switch (filter) {
    case DiligentColorFilter::None:
        pipelineIndex = 0;
        break;
    case DiligentColorFilter::Fxaa:
        pipelineIndex = 1;
        break;
    case DiligentColorFilter::Smaa:
        if (!impl_->smaa) {
            impl_->smaa = std::make_unique<DiligentSmaa>(*impl_->device, impl_->targetFormat);
        }
        impl_->smaa->draw(context, source, destination);
        return;
    default:
        throw std::invalid_argument("Invalid Diligent color present filter");
    }
    const auto& sourceView = source.GetDesc();
    const auto& targetView = destination.GetDesc();
    const auto& sourceTexture = source.GetTexture()->GetDesc();
    const auto& targetTexture = destination.GetTexture()->GetDesc();
    if (sourceView.ViewType != TEXTURE_VIEW_SHADER_RESOURCE ||
        targetView.ViewType != TEXTURE_VIEW_RENDER_TARGET ||
        sourceView.TextureDim != RESOURCE_DIM_TEX_2D ||
        targetView.TextureDim != RESOURCE_DIM_TEX_2D || sourceTexture.SampleCount != 1 ||
        targetTexture.SampleCount != 1 || targetView.Format != impl_->targetFormat ||
        source.GetTexture() == destination.GetTexture()) {
        throw std::invalid_argument("Diligent color composition requires distinct single-sample "
                                    "2D source and compatible target views");
    }
    const Uint32 mip = std::min(targetView.MostDetailedMip, Uint32{31});
    const Uint32 width = std::max(Uint32{1}, targetTexture.Width >> mip);
    const Uint32 height = std::max(Uint32{1}, targetTexture.Height >> mip);
    auto& pipeline = impl_->pipelines[pipelineIndex];
    auto* texture = pipeline.binding->GetVariableByName(SHADER_TYPE_PIXEL, "SceneColor");
    require(texture != nullptr, "Diligent color present texture binding is missing");
    texture->Set(&source);
    ITextureView* renderTarget = &destination;
    context.SetRenderTargets(1, &renderTarget, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    const Viewport viewport{0.0F, 0.0F, static_cast<float>(width), static_cast<float>(height),
                            0.0F, 1.0F};
    context.SetViewports(1, &viewport, width, height);
    context.SetPipelineState(pipeline.state);
    context.CommitShaderResources(pipeline.binding, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    DrawAttribs draw;
    draw.NumVertices = 3;
    context.Draw(draw);
}

void DiligentColorPresent::drawDebug(Diligent::IDeviceContext& context,
                                     Diligent::ITextureView& motionVectors,
                                     Diligent::ITextureView& motionValidity,
                                     Diligent::ITextureView& destination, DiligentDebugView view,
                                     Diligent::ITextureView* linearDepth) {
    using namespace Diligent;
    if (view == DiligentDebugView::None) {
        throw std::invalid_argument("Diligent motion debug view must be selected");
    }
    Uint32 mode = 0;
    switch (view) {
    case DiligentDebugView::MotionVectors:
        mode = 1;
        break;
    case DiligentDebugView::MotionMagnitude:
        mode = 2;
        break;
    case DiligentDebugView::MotionValidity:
        mode = 3;
        break;
    case DiligentDebugView::LinearDepth:
        mode = 4;
        if (linearDepth == nullptr) {
            throw std::invalid_argument("Diligent linear-depth debug requires a depth texture");
        }
        break;
    case DiligentDebugView::None:
        throw std::invalid_argument("Diligent motion debug view must be selected");
    default:
        throw std::invalid_argument("Invalid Diligent motion debug view");
    }
    impl_->ensureDebugPipeline();
    const auto& motionView = motionVectors.GetDesc();
    const auto& validityView = motionValidity.GetDesc();
    const auto* linearDepthView = linearDepth != nullptr ? &linearDepth->GetDesc() : nullptr;
    const auto& destinationView = destination.GetDesc();
    ITexture* motionTexture = motionVectors.GetTexture();
    ITexture* validityTexture = motionValidity.GetTexture();
    ITexture* linearDepthTexture = linearDepth != nullptr ? linearDepth->GetTexture() : nullptr;
    ITexture* destinationTexture = destination.GetTexture();
    if (motionTexture == nullptr || validityTexture == nullptr || destinationTexture == nullptr ||
        motionView.ViewType != TEXTURE_VIEW_SHADER_RESOURCE ||
        validityView.ViewType != TEXTURE_VIEW_SHADER_RESOURCE ||
        destinationView.ViewType != TEXTURE_VIEW_RENDER_TARGET ||
        motionView.TextureDim != RESOURCE_DIM_TEX_2D ||
        validityView.TextureDim != RESOURCE_DIM_TEX_2D ||
        destinationView.TextureDim != RESOURCE_DIM_TEX_2D ||
        motionTexture->GetDesc().SampleCount != 1 || validityTexture->GetDesc().SampleCount != 1 ||
        destinationTexture->GetDesc().SampleCount != 1 ||
        motionView.Format != TEX_FORMAT_RG32_FLOAT || validityView.Format != TEX_FORMAT_R8_UNORM ||
        destinationView.Format != impl_->targetFormat ||
        motionTexture->GetDesc().Width != validityTexture->GetDesc().Width ||
        motionTexture->GetDesc().Height != validityTexture->GetDesc().Height ||
        destinationTexture == motionTexture || destinationTexture == validityTexture) {
        throw std::invalid_argument(
            "Diligent motion debug requires compatible distinct single-sample 2D views");
    }
    if (mode == 4 && (linearDepthTexture == nullptr || linearDepthView == nullptr ||
                      linearDepthView->ViewType != TEXTURE_VIEW_SHADER_RESOURCE ||
                      linearDepthView->TextureDim != RESOURCE_DIM_TEX_2D ||
                      linearDepthView->Format != TEX_FORMAT_R32_FLOAT ||
                      linearDepthTexture->GetDesc().SampleCount != 1 ||
                      linearDepthTexture->GetDesc().Width != motionTexture->GetDesc().Width ||
                      linearDepthTexture->GetDesc().Height != motionTexture->GetDesc().Height ||
                      linearDepthTexture == destinationTexture)) {
        throw std::invalid_argument(
            "Diligent linear-depth debug requires a compatible distinct single-sample 2D view");
    }

    auto* motionVariable =
        impl_->debugPipeline.binding->GetVariableByName(SHADER_TYPE_PIXEL, "MotionVectors");
    auto* validityVariable =
        impl_->debugPipeline.binding->GetVariableByName(SHADER_TYPE_PIXEL, "MotionValidity");
    auto* linearDepthVariable =
        impl_->debugPipeline.binding->GetVariableByName(SHADER_TYPE_PIXEL, "LinearDepth");
    auto* parametersVariable =
        impl_->debugPipeline.binding->GetVariableByName(SHADER_TYPE_PIXEL, "DebugParameters");
    require(motionVariable != nullptr && validityVariable != nullptr &&
                linearDepthVariable != nullptr && parametersVariable != nullptr,
            "Diligent motion debug bindings are missing");
    motionVariable->Set(&motionVectors);
    validityVariable->Set(&motionValidity);
    linearDepthVariable->Set(mode == 4 ? linearDepth : &motionValidity);

    const std::array<Uint32, 4> parameters{mode, 0, 0, 0};
    void* mapped = nullptr;
    context.MapBuffer(impl_->debugParameters, MAP_WRITE, MAP_FLAG_DISCARD, mapped);
    require(mapped != nullptr, "Failed to map Diligent motion debug parameters");
    std::memcpy(mapped, parameters.data(), sizeof(parameters));
    context.UnmapBuffer(impl_->debugParameters, MAP_WRITE);
    parametersVariable->Set(impl_->debugParameters);

    const Uint32 mip = std::min(destinationView.MostDetailedMip, Uint32{31});
    const Uint32 width = std::max(Uint32{1}, destinationTexture->GetDesc().Width >> mip);
    const Uint32 height = std::max(Uint32{1}, destinationTexture->GetDesc().Height >> mip);
    ITextureView* renderTarget = &destination;
    context.SetRenderTargets(1, &renderTarget, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    const Viewport viewport{0.0F, 0.0F, static_cast<float>(width), static_cast<float>(height),
                            0.0F, 1.0F};
    context.SetViewports(1, &viewport, width, height);
    context.SetPipelineState(impl_->debugPipeline.state);
    context.CommitShaderResources(impl_->debugPipeline.binding,
                                  RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    DrawAttribs draw;
    draw.NumVertices = 3;
    context.Draw(draw);
}

} // namespace openbus::rendering
