#include "render/diligent/DiligentSmaa.h"
#include "render/diligent/DiligentRenderUtils.h"

#include "SmaaShader.h"
#include "AreaTex.h"
#include "SearchTex.h"

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
#include <cstring>
#include <stdexcept>
#include <string>

namespace openbus::rendering {
namespace {

constexpr char vertexSource[] = R"(
struct Output { float4 position : SV_POSITION; float2 uv : TEXCOORD0; };
Output main(uint vertexId : SV_VertexID) {
    Output result;
    result.uv = float2((vertexId << 1) & 2, vertexId & 2);
    result.position = float4(result.uv.x * 2.0 - 1.0, 1.0 - result.uv.y * 2.0, 0.0, 1.0);
    return result;
}
)";

constexpr char pixelWrappers[] = R"(
Texture2D ColorTexture;
Texture2D EdgesTexture;
Texture2D WeightsTexture;
Texture2D AreaTexture;
Texture2D SearchTexture;
float4 edge(float2 uv : TEXCOORD0) : SV_TARGET {
    float4 offsets[3];
    SMAAEdgeDetectionVS(uv, offsets);
    return float4(SMAAColorEdgeDetectionPS(uv, offsets, ColorTexture), 0.0, 1.0);
}
float4 weights(float2 uv : TEXCOORD0) : SV_TARGET {
    float2 pixel;
    float4 offsets[3];
    SMAABlendingWeightCalculationVS(uv, pixel, offsets);
    return SMAABlendingWeightCalculationPS(uv, pixel, offsets, EdgesTexture, AreaTexture,
                                           SearchTexture, float4(0, 0, 0, 0));
}
float4 neighborhood(float2 uv : TEXCOORD0) : SV_TARGET {
    float4 offsets;
    SMAANeighborhoodBlendingVS(uv, offsets);
    return SMAANeighborhoodBlendingPS(uv, offsets, ColorTexture, WeightsTexture);
}
)";

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

Diligent::RefCntAutoPtr<Diligent::ITexture>
createTexture(Diligent::IRenderDevice& device, const char* name, Diligent::Uint32 width,
              Diligent::Uint32 height, Diligent::TEXTURE_FORMAT format, const void* data = nullptr,
              Diligent::Uint32 stride = 0) {
    using namespace Diligent;
    TextureDesc desc;
    desc.Name = name;
    desc.Type = RESOURCE_DIM_TEX_2D;
    desc.Width = width;
    desc.Height = height;
    desc.MipLevels = 1;
    desc.Format = format;
    desc.BindFlags = BIND_SHADER_RESOURCE;
    desc.Usage = data ? USAGE_IMMUTABLE : USAGE_DEFAULT;
    if (!data) {
        desc.BindFlags |= BIND_RENDER_TARGET;
    }
    TextureSubResData subresource;
    subresource.pData = data;
    subresource.Stride = stride;
    TextureData initial;
    initial.pSubResources = &subresource;
    initial.NumSubresources = 1;
    RefCntAutoPtr<ITexture> texture;
    device.CreateTexture(desc, data ? &initial : nullptr, &texture);
    require(static_cast<bool>(texture), "Failed to create Diligent SMAA texture");
    require(texture->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE) != nullptr &&
                (data || texture->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET) != nullptr),
            "Diligent SMAA texture views are missing");
    return texture;
}

} // namespace

struct DiligentSmaa::Impl {
    struct Pass {
        Diligent::RefCntAutoPtr<Diligent::IPipelineState> pipeline;
        Diligent::RefCntAutoPtr<Diligent::IShaderResourceBinding> binding;
    };
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Diligent::TEXTURE_FORMAT format;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> metrics;
    Diligent::RefCntAutoPtr<Diligent::ITexture> area;
    Diligent::RefCntAutoPtr<Diligent::ITexture> search;
    Diligent::RefCntAutoPtr<Diligent::ITexture> edges;
    Diligent::RefCntAutoPtr<Diligent::ITexture> weights;
    std::array<Pass, 3> passes;
    Diligent::Uint32 width = 0;
    Diligent::Uint32 height = 0;

    Impl(Diligent::IRenderDevice& renderDevice, Diligent::TEXTURE_FORMAT targetFormat)
        : device(&renderDevice), format(targetFormat) {
        using namespace Diligent;
        diligent_detail::validateSamples(renderDevice, targetFormat, TEX_FORMAT_UNKNOWN, 1);
        area = createTexture(renderDevice, "SMAA area", AREATEX_WIDTH, AREATEX_HEIGHT,
                             TEX_FORMAT_RG8_UNORM, areaTexBytes, AREATEX_PITCH);
        search = createTexture(renderDevice, "SMAA search", SEARCHTEX_WIDTH, SEARCHTEX_HEIGHT,
                               TEX_FORMAT_R8_UNORM, searchTexBytes, SEARCHTEX_PITCH);
        BufferDesc buffer;
        buffer.Name = "SMAA metrics";
        buffer.Size = sizeof(float) * 4;
        buffer.Usage = USAGE_DYNAMIC;
        buffer.BindFlags = BIND_UNIFORM_BUFFER;
        buffer.CPUAccessFlags = CPU_ACCESS_WRITE;
        renderDevice.CreateBuffer(buffer, nullptr, &metrics);
        require(static_cast<bool>(metrics), "Failed to create Diligent SMAA metrics");

        auto vertex = diligent_detail::createShader(renderDevice, SHADER_TYPE_VERTEX,
                                                    "SMAA fullscreen VS", vertexSource);
        std::string source = "#define SMAA_HLSL_4\n#define SMAA_PRESET_HIGH\n"
                             "#define SMAA_RT_METRICS Metrics\n"
                             "cbuffer SMAAMetrics { float4 Metrics; };\n";
        source += openbusSmaaShader;
        source += pixelWrappers;
        const char* entries[] = {"edge", "weights", "neighborhood"};
        const TEXTURE_FORMAT formats[] = {TEX_FORMAT_RG8_UNORM, TEX_FORMAT_RGBA8_UNORM, format};
        const ShaderResourceVariableDesc variables[] = {
            {SHADER_TYPE_PIXEL, "ColorTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "EdgesTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "WeightsTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "AreaTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
            {SHADER_TYPE_PIXEL, "SearchTexture", SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC},
        };
        SamplerDesc linear;
        linear.AddressU = TEXTURE_ADDRESS_CLAMP;
        linear.AddressV = TEXTURE_ADDRESS_CLAMP;
        linear.MinFilter = FILTER_TYPE_LINEAR;
        linear.MagFilter = FILTER_TYPE_LINEAR;
        linear.MipFilter = FILTER_TYPE_POINT;
        SamplerDesc point = linear;
        point.MinFilter = FILTER_TYPE_POINT;
        point.MagFilter = FILTER_TYPE_POINT;
        const ImmutableSamplerDesc samplers[] = {
            {SHADER_TYPE_PIXEL, "LinearSampler", linear},
            {SHADER_TYPE_PIXEL, "PointSampler", point},
        };
        for (std::size_t i = 0; i < passes.size(); ++i) {
            auto pixel =
                diligent_detail::createShader(renderDevice, SHADER_TYPE_PIXEL, "SMAA pixel stage",
                                              source.c_str(), false, entries[i]);
            GraphicsPipelineStateCreateInfo info;
            info.PSODesc.Name = entries[i];
            info.PSODesc.PipelineType = PIPELINE_TYPE_GRAPHICS;
            info.PSODesc.ResourceLayout.Variables = variables;
            info.PSODesc.ResourceLayout.NumVariables = static_cast<Uint32>(std::size(variables));
            info.PSODesc.ResourceLayout.ImmutableSamplers = samplers;
            info.PSODesc.ResourceLayout.NumImmutableSamplers =
                static_cast<Uint32>(std::size(samplers));
            info.GraphicsPipeline.NumRenderTargets = 1;
            info.GraphicsPipeline.RTVFormats[0] = formats[i];
            info.GraphicsPipeline.PrimitiveTopology = PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
            info.GraphicsPipeline.RasterizerDesc.CullMode = CULL_MODE_NONE;
            info.GraphicsPipeline.DepthStencilDesc.DepthEnable = False;
            info.pVS = vertex;
            info.pPS = pixel;
            auto& pass = passes[i];
            renderDevice.CreateGraphicsPipelineState(info, &pass.pipeline);
            require(static_cast<bool>(pass.pipeline), "Failed to create Diligent SMAA pipeline");
            auto* uniform =
                pass.pipeline->GetStaticVariableByName(SHADER_TYPE_PIXEL, "SMAAMetrics");
            require(uniform != nullptr, "Diligent SMAA metrics binding is missing");
            uniform->Set(metrics);
            pass.pipeline->CreateShaderResourceBinding(&pass.binding, true);
            require(static_cast<bool>(pass.binding), "Failed to create Diligent SMAA bindings");
        }
    }

    void resize(Diligent::Uint32 newWidth, Diligent::Uint32 newHeight) {
        if (width == newWidth && height == newHeight) {
            return;
        }
        auto newEdges = createTexture(*device, "SMAA edges", newWidth, newHeight,
                                      Diligent::TEX_FORMAT_RG8_UNORM);
        auto newWeights = createTexture(*device, "SMAA weights", newWidth, newHeight,
                                        Diligent::TEX_FORMAT_RGBA8_UNORM);
        edges = std::move(newEdges);
        weights = std::move(newWeights);
        width = newWidth;
        height = newHeight;
    }
};

DiligentSmaa::DiligentSmaa(Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT targetFormat)
    : impl_(std::make_unique<Impl>(device, targetFormat)) {}
DiligentSmaa::~DiligentSmaa() = default;

void DiligentSmaa::draw(Diligent::IDeviceContext& context, Diligent::ITextureView& source,
                        Diligent::ITextureView& destination) {
    using namespace Diligent;
    const auto& sourceView = source.GetDesc();
    const auto& targetView = destination.GetDesc();
    const auto& sourceDesc = source.GetTexture()->GetDesc();
    const auto& targetDesc = destination.GetTexture()->GetDesc();
    if (sourceView.ViewType != TEXTURE_VIEW_SHADER_RESOURCE ||
        targetView.ViewType != TEXTURE_VIEW_RENDER_TARGET ||
        sourceView.TextureDim != RESOURCE_DIM_TEX_2D ||
        targetView.TextureDim != RESOURCE_DIM_TEX_2D || sourceDesc.SampleCount != 1 ||
        targetDesc.SampleCount != 1 || targetView.Format != impl_->format ||
        source.GetTexture() == destination.GetTexture()) {
        throw std::invalid_argument("SMAA requires distinct single-sample 2D compatible views");
    }
    const Uint32 srcMip = std::min(sourceView.MostDetailedMip, Uint32{31});
    const Uint32 dstMip = std::min(targetView.MostDetailedMip, Uint32{31});
    const Uint32 width = std::max(Uint32{1}, sourceDesc.Width >> srcMip);
    const Uint32 height = std::max(Uint32{1}, sourceDesc.Height >> srcMip);
    const Uint32 targetWidth = std::max(Uint32{1}, targetDesc.Width >> dstMip);
    const Uint32 targetHeight = std::max(Uint32{1}, targetDesc.Height >> dstMip);
    impl_->resize(width, height);
    const std::array<float, 4> metrics = {1.0F / width, 1.0F / height, static_cast<float>(width),
                                          static_cast<float>(height)};
    void* mapped = nullptr;
    context.MapBuffer(impl_->metrics, MAP_WRITE, MAP_FLAG_DISCARD, mapped);
    require(mapped != nullptr, "Failed to map Diligent SMAA metrics");
    std::memcpy(mapped, metrics.data(), sizeof(metrics));
    context.UnmapBuffer(impl_->metrics, MAP_WRITE);

    const auto bindResource = [](Impl::Pass& pass, const char* name, ITextureView* view) {
        auto* variable = pass.binding->GetVariableByName(SHADER_TYPE_PIXEL, name);
        require(variable != nullptr, "Diligent SMAA texture binding is missing");
        variable->Set(view);
    };
    bindResource(impl_->passes[0], "ColorTexture", &source);
    bindResource(impl_->passes[1], "EdgesTexture",
                 impl_->edges->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE));
    bindResource(impl_->passes[1], "AreaTexture",
                 impl_->area->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE));
    bindResource(impl_->passes[1], "SearchTexture",
                 impl_->search->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE));
    bindResource(impl_->passes[2], "ColorTexture", &source);
    bindResource(impl_->passes[2], "WeightsTexture",
                 impl_->weights->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE));
    ITextureView* targets[] = {impl_->edges->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET),
                               impl_->weights->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET),
                               &destination};
    constexpr float clear[] = {0, 0, 0, 0};
    for (std::size_t i = 0; i < impl_->passes.size(); ++i) {
        context.SetRenderTargets(1, &targets[i], nullptr,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        if (i < 2) {
            context.ClearRenderTarget(targets[i], clear, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        }
        const Uint32 passWidth = i == 2 ? targetWidth : width;
        const Uint32 passHeight = i == 2 ? targetHeight : height;
        const Viewport viewport{0, 0, static_cast<float>(passWidth), static_cast<float>(passHeight),
                                0, 1};
        context.SetViewports(1, &viewport, passWidth, passHeight);
        context.SetPipelineState(impl_->passes[i].pipeline);
        context.CommitShaderResources(impl_->passes[i].binding,
                                      RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        DrawAttribs draw;
        draw.NumVertices = 3;
        context.Draw(draw);
    }
}

} // namespace openbus::rendering
