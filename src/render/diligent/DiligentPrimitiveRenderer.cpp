#include "render/diligent/DiligentPrimitiveRenderer.h"
#include "render/diligent/DiligentRenderUtils.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/Buffer.h>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/PipelineState.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Shader.h>
#include <Graphics/GraphicsEngine/interface/ShaderResourceVariable.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace openbus::rendering {
namespace {

static_assert(std::is_standard_layout_v<PrimitiveVertex>);
static_assert(offsetof(PrimitiveVertex, r) == 3 * sizeof(float));
static_assert(sizeof(PrimitiveVertex) == 6 * sizeof(float));

constexpr char vertexShaderSource[] = R"(
cbuffer PrimitiveTransform
{
    float4 Row0;
    float4 Row1;
    float4 Row2;
    float4 Row3;
    float4 ViewRow0;
    float4 ViewRow1;
    float4 ViewRow2;
    float4 ViewRow3;
};

struct VertexInput
{
    float3 Position : ATTRIB0;
    float3 Color : ATTRIB1;
};

struct VertexOutput
{
    float4 Position : SV_POSITION;
    float3 Color : COLOR0;
    float ViewDepth : TEXCOORD0;
};

VertexOutput main(VertexInput input)
{
    float4 position = float4(input.Position, 1.0);
    VertexOutput output;
    output.Position = float4(dot(Row0, position), dot(Row1, position),
                             dot(Row2, position), dot(Row3, position));
    output.Color = input.Color;
    output.ViewDepth = -dot(ViewRow2, position);
    return output;
}
)";

constexpr char pixelShaderSource[] = R"(
struct PixelOutput
{
    float4 Color : SV_TARGET0;
    float2 Motion : SV_TARGET1;
    float Validity : SV_TARGET2;
    float LinearDepth : SV_TARGET3;
};
PixelOutput main(float3 color : COLOR0, float viewDepth : TEXCOORD0)
{
    PixelOutput output;
    output.Color = float4(color, 1.0);
    output.Motion = float2(0.0, 0.0);
    output.Validity = 0.0;
    output.LinearDepth = max(viewDepth, 0.0);
    return output;
}
)";

constexpr char pixelShaderSourceWithoutLinearDepth[] = R"(
struct PixelOutput
{
    float4 Color : SV_TARGET0;
    float2 Motion : SV_TARGET1;
    float Validity : SV_TARGET2;
};
PixelOutput main(float3 color : COLOR0, float viewDepth : TEXCOORD0)
{
    PixelOutput output;
    output.Color = float4(color, 1.0);
    output.Motion = float2(0.0, 0.0);
    output.Validity = 0.0;
    return output;
}
)";

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

struct DiligentPrimitiveRenderer::Impl {
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    Diligent::RefCntAutoPtr<Diligent::IPipelineState> pipeline;
    Diligent::RefCntAutoPtr<Diligent::IShaderResourceBinding> bindings;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> transformBuffer;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> vertexBuffer;
    std::size_t vertexCapacity = 0;
    float minimumNdcZ = 0.0f;
    bool includeLinearDepth = true;

    Impl(Diligent::IRenderDevice& renderDevice, Diligent::TEXTURE_FORMAT colorFormat,
         Diligent::TEXTURE_FORMAT depthFormat, DiligentMultisampling multisampling,
         bool withLinearDepth)
        : device(&renderDevice), minimumNdcZ(renderDevice.GetDeviceInfo().NDC.MinZ),
          includeLinearDepth(withLinearDepth) {
        using namespace Diligent;
        diligent_detail::validateSamples(renderDevice, colorFormat, depthFormat,
                                         multisampling.samples);
        if (colorFormat == TEX_FORMAT_UNKNOWN) {
            throw std::invalid_argument("Diligent primitive color format must be specified");
        }
        require(std::isfinite(minimumNdcZ), "Diligent device reported invalid NDC depth");

        auto vertexShader = diligent_detail::createShader(
            renderDevice, SHADER_TYPE_VERTEX, "OpenBus primitive VS", vertexShaderSource);
        auto pixelShader = diligent_detail::createShader(
            renderDevice, SHADER_TYPE_PIXEL, "OpenBus primitive PS",
            includeLinearDepth ? pixelShaderSource : pixelShaderSourceWithoutLinearDepth);

        const LayoutElement elements[] = {
            LayoutElement{0, 0, 3, VT_FLOAT32, False},
            LayoutElement{1, 0, 3, VT_FLOAT32, False},
        };
        GraphicsPipelineStateCreateInfo info;
        info.PSODesc.Name = "OpenBus opaque primitive triangles";
        info.PSODesc.PipelineType = PIPELINE_TYPE_GRAPHICS;
        info.GraphicsPipeline.NumRenderTargets = includeLinearDepth ? 4 : 3;
        info.GraphicsPipeline.RTVFormats[0] = colorFormat;
        info.GraphicsPipeline.RTVFormats[1] = TEX_FORMAT_RG32_FLOAT;
        info.GraphicsPipeline.RTVFormats[2] = TEX_FORMAT_R8_UNORM;
        if (includeLinearDepth) {
            info.GraphicsPipeline.RTVFormats[3] = TEX_FORMAT_R32_FLOAT;
        }
        info.GraphicsPipeline.DSVFormat = depthFormat;
        info.GraphicsPipeline.SmplDesc.Count = multisampling.samples;
        info.GraphicsPipeline.PrimitiveTopology = PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        info.GraphicsPipeline.RasterizerDesc.CullMode = CULL_MODE_NONE;
        info.GraphicsPipeline.DepthStencilDesc.DepthEnable = depthFormat != TEX_FORMAT_UNKNOWN;
        info.GraphicsPipeline.InputLayout.LayoutElements = elements;
        info.GraphicsPipeline.InputLayout.NumElements = 2;
        info.pVS = vertexShader;
        info.pPS = pixelShader;
        renderDevice.CreateGraphicsPipelineState(info, &pipeline);
        require(static_cast<bool>(pipeline), "Failed to create Diligent primitive pipeline");

        BufferDesc bufferInfo;
        bufferInfo.Name = "OpenBus primitive transform";
        bufferInfo.Size = sizeof(float) * 32;
        bufferInfo.Usage = USAGE_DYNAMIC;
        bufferInfo.BindFlags = BIND_UNIFORM_BUFFER;
        bufferInfo.CPUAccessFlags = CPU_ACCESS_WRITE;
        renderDevice.CreateBuffer(bufferInfo, nullptr, &transformBuffer);
        require(static_cast<bool>(transformBuffer), "Failed to create primitive transform buffer");
        auto* transform =
            pipeline->GetStaticVariableByName(SHADER_TYPE_VERTEX, "PrimitiveTransform");
        require(transform != nullptr, "Diligent primitive transform binding is missing");
        transform->Set(transformBuffer);
        pipeline->CreateShaderResourceBinding(&bindings, true);
        require(static_cast<bool>(bindings), "Failed to create primitive shader bindings");
    }

    void ensureVertexCapacity(std::size_t required) {
        using namespace Diligent;
        if (required <= vertexCapacity) {
            return;
        }
        const std::size_t maximum = std::numeric_limits<Uint32>::max() / sizeof(PrimitiveVertex);
        if (required > maximum) {
            throw std::length_error("Diligent primitive vertex buffer exceeds supported size");
        }
        const std::size_t doubled = vertexCapacity > maximum / 2 ? maximum : vertexCapacity * 2;
        const std::size_t capacity = std::max(required, doubled);
        BufferDesc info;
        info.Name = "OpenBus primitive vertices";
        info.Size = static_cast<Uint64>(capacity * sizeof(PrimitiveVertex));
        info.Usage = USAGE_DYNAMIC;
        info.BindFlags = BIND_VERTEX_BUFFER;
        info.CPUAccessFlags = CPU_ACCESS_WRITE;
        RefCntAutoPtr<IBuffer> replacement;
        device->CreateBuffer(info, nullptr, &replacement);
        require(static_cast<bool>(replacement), "Failed to create primitive vertex buffer");
        vertexBuffer = std::move(replacement);
        vertexCapacity = capacity;
    }
};

DiligentPrimitiveRenderer::DiligentPrimitiveRenderer(Diligent::IRenderDevice& device,
                                                     Diligent::TEXTURE_FORMAT colorFormat,
                                                     Diligent::TEXTURE_FORMAT depthFormat,
                                                     DiligentMultisampling multisampling,
                                                     bool includeLinearDepth)
    : impl_(std::make_unique<Impl>(device, colorFormat, depthFormat, multisampling,
                                   includeLinearDepth)) {}

DiligentPrimitiveRenderer::~DiligentPrimitiveRenderer() = default;

void DiligentPrimitiveRenderer::draw(Diligent::IDeviceContext& context,
                                     std::span<const PrimitiveVertex> vertices,
                                     const Matrix4& modelViewProjection, const Matrix4& modelView) {
    using namespace Diligent;
    if (vertices.empty()) {
        return;
    }
    if (vertices.size() % 3 != 0) {
        throw std::invalid_argument("Diligent primitive triangle list must be divisible by three");
    }
    const auto clipRows =
        diligent_detail::clipTransformRows(modelViewProjection, impl_->minimumNdcZ);
    const auto viewRows = diligent_detail::matrixRows(modelView);

    impl_->ensureVertexCapacity(vertices.size());
    void* mapped = nullptr;
    context.MapBuffer(impl_->vertexBuffer, MAP_WRITE, MAP_FLAG_DISCARD, mapped);
    require(mapped != nullptr, "Failed to map Diligent primitive vertex buffer");
    std::memcpy(mapped, vertices.data(), vertices.size_bytes());
    context.UnmapBuffer(impl_->vertexBuffer, MAP_WRITE);
    mapped = nullptr;
    context.MapBuffer(impl_->transformBuffer, MAP_WRITE, MAP_FLAG_DISCARD, mapped);
    require(mapped != nullptr, "Failed to map Diligent primitive transform buffer");
    std::memcpy(mapped, clipRows.data(), sizeof(clipRows));
    std::memcpy(static_cast<std::byte*>(mapped) + sizeof(clipRows), viewRows.data(),
                sizeof(viewRows));
    context.UnmapBuffer(impl_->transformBuffer, MAP_WRITE);

    IBuffer* buffer = impl_->vertexBuffer;
    context.SetVertexBuffers(0, 1, &buffer, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION,
                             SET_VERTEX_BUFFERS_FLAG_RESET);
    context.SetPipelineState(impl_->pipeline);
    context.CommitShaderResources(impl_->bindings, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    DrawAttribs attributes;
    attributes.NumVertices = static_cast<Uint32>(vertices.size());
    context.Draw(attributes);
}

} // namespace openbus::rendering
