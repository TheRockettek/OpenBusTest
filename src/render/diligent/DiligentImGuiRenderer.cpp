#include "render/diligent/DiligentImGuiRenderer.h"

#include "render/diligent/DiligentRenderUtils.h"
#include "render/diligent/DiligentTexture2D.h"

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
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace openbus::rendering {
namespace {

constexpr char vertexSource[] = R"(
cbuffer GuiTransform { float4 ScaleOffset; };
struct Input { float2 Position : ATTRIB0; float2 UV : ATTRIB1; float4 Color : ATTRIB2; };
struct Output { float4 Position : SV_POSITION; float2 UV : TEXCOORD0; float4 Color : COLOR0; };
Output main(Input input)
{
    Output output;
    output.Position = float4(input.Position * ScaleOffset.xy + ScaleOffset.zw, 0.0, 1.0);
    output.UV = input.UV;
    output.Color = input.Color;
    return output;
}
)";

constexpr char pixelSource[] = R"(
Texture2D GuiTexture;
SamplerState GuiTexture_sampler;
float4 main(float2 uv : TEXCOORD0, float4 color : COLOR0) : SV_TARGET
{
    return color * GuiTexture.Sample(GuiTexture_sampler, uv);
}
)";

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class ScopedImGuiContext {
  public:
    explicit ScopedImGuiContext(ImGuiContext& context) : previous_(ImGui::GetCurrentContext()) {
        ImGui::SetCurrentContext(&context);
    }
    ~ScopedImGuiContext() {
        ImGui::SetCurrentContext(previous_);
    }

  private:
    ImGuiContext* previous_;
};

} // namespace

struct DiligentImGuiRenderer::Impl {
    static constexpr ImTextureID fontId = 1;
    Diligent::RefCntAutoPtr<Diligent::IRenderDevice> device;
    ImGuiContext& gui;
    Diligent::TEXTURE_FORMAT targetFormat;
    Diligent::RefCntAutoPtr<Diligent::IPipelineState> pipeline;
    Diligent::RefCntAutoPtr<Diligent::IShaderResourceBinding> binding;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> transform;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> vertices;
    Diligent::RefCntAutoPtr<Diligent::IBuffer> indices;
    std::size_t vertexCapacity = 0;
    std::size_t indexCapacity = 0;
    std::unique_ptr<DiligentTexture2D> fontTexture;
    std::unordered_map<ImTextureID, Diligent::RefCntAutoPtr<Diligent::ITextureView>> textures;
    ImTextureID nextId = fontId + 1;

    Impl(Diligent::IRenderDevice& renderDevice, Diligent::TEXTURE_FORMAT format,
         ImGuiContext& guiContext)
        : device(&renderDevice), gui(guiContext), targetFormat(format) {
        using namespace Diligent;
        if (format == TEX_FORMAT_UNKNOWN) {
            throw std::invalid_argument("Diligent ImGui requires a target format");
        }
        ScopedImGuiContext scoped(gui);
        ImGuiIO& io = ImGui::GetIO();
        if (io.BackendRendererUserData != nullptr || io.Fonts->TexID != 0) {
            throw std::logic_error("ImGui context or font atlas already has a renderer");
        }
        auto vs = diligent_detail::createShader(renderDevice, SHADER_TYPE_VERTEX,
                                                "OpenBus ImGui VS", vertexSource);
        auto ps = diligent_detail::createShader(renderDevice, SHADER_TYPE_PIXEL, "OpenBus ImGui PS",
                                                pixelSource, true);
        const LayoutElement elements[] = {
            {0, 0, 2, VT_FLOAT32, False, offsetof(ImDrawVert, pos), sizeof(ImDrawVert)},
            {1, 0, 2, VT_FLOAT32, False, offsetof(ImDrawVert, uv), sizeof(ImDrawVert)},
            {2, 0, 4, VT_UINT8, True, offsetof(ImDrawVert, col), sizeof(ImDrawVert)},
        };
        const ShaderResourceVariableDesc variable{SHADER_TYPE_PIXEL, "GuiTexture",
                                                  SHADER_RESOURCE_VARIABLE_TYPE_DYNAMIC};
        SamplerDesc sampler;
        sampler.AddressU = TEXTURE_ADDRESS_CLAMP;
        sampler.AddressV = TEXTURE_ADDRESS_CLAMP;
        const ImmutableSamplerDesc immutableSampler{SHADER_TYPE_PIXEL, "GuiTexture", sampler};
        GraphicsPipelineStateCreateInfo info;
        info.PSODesc.Name = "OpenBus Diligent ImGui";
        info.PSODesc.PipelineType = PIPELINE_TYPE_GRAPHICS;
        info.PSODesc.ResourceLayout.Variables = &variable;
        info.PSODesc.ResourceLayout.NumVariables = 1;
        info.PSODesc.ResourceLayout.ImmutableSamplers = &immutableSampler;
        info.PSODesc.ResourceLayout.NumImmutableSamplers = 1;
        info.GraphicsPipeline.NumRenderTargets = 1;
        info.GraphicsPipeline.RTVFormats[0] = format;
        info.GraphicsPipeline.PrimitiveTopology = PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        info.GraphicsPipeline.RasterizerDesc.CullMode = CULL_MODE_NONE;
        info.GraphicsPipeline.RasterizerDesc.ScissorEnable = True;
        info.GraphicsPipeline.DepthStencilDesc.DepthEnable = False;
        info.GraphicsPipeline.InputLayout.LayoutElements = elements;
        info.GraphicsPipeline.InputLayout.NumElements = 3;
        auto& blend = info.GraphicsPipeline.BlendDesc.RenderTargets[0];
        blend.BlendEnable = True;
        blend.SrcBlend = BLEND_FACTOR_SRC_ALPHA;
        blend.DestBlend = BLEND_FACTOR_INV_SRC_ALPHA;
        blend.SrcBlendAlpha = BLEND_FACTOR_ONE;
        blend.DestBlendAlpha = BLEND_FACTOR_INV_SRC_ALPHA;
        info.pVS = vs;
        info.pPS = ps;
        device->CreateGraphicsPipelineState(info, &pipeline);
        require(static_cast<bool>(pipeline), "Failed to create Diligent ImGui pipeline");
        BufferDesc buffer;
        buffer.Name = "OpenBus ImGui transform";
        buffer.Size = sizeof(float) * 4;
        buffer.Usage = USAGE_DYNAMIC;
        buffer.BindFlags = BIND_UNIFORM_BUFFER;
        buffer.CPUAccessFlags = CPU_ACCESS_WRITE;
        device->CreateBuffer(buffer, nullptr, &transform);
        require(static_cast<bool>(transform), "Failed to create Diligent ImGui transform");
        auto* transformVariable =
            pipeline->GetStaticVariableByName(SHADER_TYPE_VERTEX, "GuiTransform");
        require(transformVariable != nullptr, "Diligent ImGui transform binding is missing");
        transformVariable->Set(transform);
        pipeline->CreateShaderResourceBinding(&binding, true);
        require(static_cast<bool>(binding), "Failed to create Diligent ImGui bindings");
        rebuildFont();
        io.BackendRendererName = "openbus_diligent";
        io.BackendRendererUserData = this;
        io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
    }

    ~Impl() {
        ScopedImGuiContext scoped(gui);
        ImGuiIO& io = ImGui::GetIO();
        io.Fonts->SetTexID(0);
        io.BackendRendererName = nullptr;
        io.BackendRendererUserData = nullptr;
        io.BackendFlags &= ~ImGuiBackendFlags_RendererHasVtxOffset;
    }

    void rebuildFont() {
        ScopedImGuiContext scoped(gui);
        unsigned char* pixels = nullptr;
        int width = 0, height = 0;
        ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
        std::size_t bytes = 0;
        if (pixels == nullptr || width <= 0 || height <= 0 ||
            !checkedTextureBufferSize(static_cast<std::size_t>(width),
                                      static_cast<std::size_t>(height), 4, bytes)) {
            throw std::runtime_error("ImGui font atlas has invalid RGBA data");
        }
        Image image{width, height, std::vector<std::uint8_t>(pixels, pixels + bytes)};
        auto replacement = std::make_unique<DiligentTexture2D>(*device, image);
        textures.insert_or_assign(fontId, &replacement->shaderResourceView());
        fontTexture = std::move(replacement);
        ImGui::GetIO().Fonts->SetTexID(fontId);
    }

    void ensureBuffer(Diligent::RefCntAutoPtr<Diligent::IBuffer>& buffer, std::size_t& capacity,
                      std::size_t required, std::size_t elementSize, Diligent::BIND_FLAGS flags,
                      const char* name) {
        using namespace Diligent;
        required = std::max(required, std::size_t{1});
        if (required <= capacity) {
            return;
        }
        const std::size_t maximum = std::numeric_limits<Uint32>::max() / elementSize;
        if (required > maximum) {
            throw std::length_error("Diligent ImGui buffer exceeds supported size");
        }
        const std::size_t doubled = capacity > maximum / 2 ? maximum : capacity * 2;
        const std::size_t newCapacity = std::max(required, doubled);
        BufferDesc desc;
        desc.Name = name;
        desc.Size = static_cast<Uint64>(newCapacity * elementSize);
        desc.Usage = USAGE_DYNAMIC;
        desc.BindFlags = flags;
        desc.CPUAccessFlags = CPU_ACCESS_WRITE;
        RefCntAutoPtr<IBuffer> replacement;
        device->CreateBuffer(desc, nullptr, &replacement);
        require(static_cast<bool>(replacement), "Failed to create Diligent ImGui buffer");
        buffer = std::move(replacement);
        capacity = newCapacity;
    }
};

DiligentImGuiRenderer::DiligentImGuiRenderer(Diligent::IRenderDevice& device,
                                             Diligent::TEXTURE_FORMAT targetFormat,
                                             ImGuiContext& guiContext)
    : impl_(std::make_unique<Impl>(device, targetFormat, guiContext)) {}

DiligentImGuiRenderer::~DiligentImGuiRenderer() = default;

void DiligentImGuiRenderer::rebuildFontTexture() {
    impl_->rebuildFont();
}

ImTextureID DiligentImGuiRenderer::registerTexture(Diligent::ITextureView& view) {
    using namespace Diligent;
    if (view.GetDesc().ViewType != TEXTURE_VIEW_SHADER_RESOURCE ||
        view.GetDesc().TextureDim != RESOURCE_DIM_TEX_2D ||
        view.GetTexture()->GetDesc().SampleCount != 1) {
        throw std::invalid_argument("ImGui texture must be a single-sample 2D shader view");
    }
    if (impl_->nextId == 0) {
        throw std::overflow_error("Diligent ImGui texture IDs exhausted");
    }
    const ImTextureID id = impl_->nextId++;
    impl_->textures.emplace(id, &view);
    return id;
}

void DiligentImGuiRenderer::unregisterTexture(ImTextureID id) {
    if (id == Impl::fontId || impl_->textures.erase(id) == 0) {
        throw std::invalid_argument("Diligent ImGui texture ID is not registered");
    }
}

void DiligentImGuiRenderer::render(Diligent::IDeviceContext& context, const ImDrawData& drawData,
                                   Diligent::ITextureView& destination) {
    using namespace Diligent;
    ScopedImGuiContext scoped(impl_->gui);
    const double width = static_cast<double>(drawData.DisplaySize.x) * drawData.FramebufferScale.x;
    const double height = static_cast<double>(drawData.DisplaySize.y) * drawData.FramebufferScale.y;
    if (!drawData.Valid || !std::isfinite(width) || !std::isfinite(height) ||
        !std::isfinite(drawData.DisplayPos.x) || !std::isfinite(drawData.DisplayPos.y) ||
        drawData.DisplaySize.x < 0 || drawData.DisplaySize.y < 0 ||
        drawData.FramebufferScale.x <= 0 || drawData.FramebufferScale.y <= 0 ||
        width > std::numeric_limits<Int32>::max() || height > std::numeric_limits<Int32>::max() ||
        drawData.CmdListsCount != drawData.CmdLists.Size || drawData.TotalVtxCount < 0 ||
        drawData.TotalIdxCount < 0) {
        throw std::invalid_argument("Invalid ImGui draw data");
    }
    if (width <= 0.0 || height <= 0.0) {
        return;
    }
    const Uint32 fbWidth = static_cast<Uint32>(std::llround(width));
    const Uint32 fbHeight = static_cast<Uint32>(std::llround(height));
    if (fbWidth == 0 || fbHeight == 0) {
        return;
    }
    const auto& view = destination.GetDesc();
    const auto& target = destination.GetTexture()->GetDesc();
    const Uint32 mip = std::min(view.MostDetailedMip, Uint32{31});
    if (view.ViewType != TEXTURE_VIEW_RENDER_TARGET || view.TextureDim != RESOURCE_DIM_TEX_2D ||
        target.SampleCount != 1 || view.Format != impl_->targetFormat ||
        std::max(Uint32{1}, target.Width >> mip) != fbWidth ||
        std::max(Uint32{1}, target.Height >> mip) != fbHeight) {
        throw std::invalid_argument("ImGui target format or framebuffer dimensions do not match");
    }
    std::size_t vertexCount = 0, indexCount = 0;
    for (const ImDrawList* list : drawData.CmdLists) {
        if (list == nullptr || list->VtxBuffer.Size < 0 || list->IdxBuffer.Size < 0) {
            throw std::invalid_argument("Invalid ImGui draw list");
        }
        vertexCount += static_cast<std::size_t>(list->VtxBuffer.Size);
        indexCount += static_cast<std::size_t>(list->IdxBuffer.Size);
    }
    if (vertexCount != static_cast<std::size_t>(drawData.TotalVtxCount) ||
        indexCount != static_cast<std::size_t>(drawData.TotalIdxCount)) {
        throw std::invalid_argument("ImGui draw list counts do not match draw data");
    }
    impl_->ensureBuffer(impl_->vertices, impl_->vertexCapacity, vertexCount, sizeof(ImDrawVert),
                        BIND_VERTEX_BUFFER, "OpenBus ImGui vertices");
    impl_->ensureBuffer(impl_->indices, impl_->indexCapacity, indexCount, sizeof(ImDrawIdx),
                        BIND_INDEX_BUFFER, "OpenBus ImGui indices");
    const auto upload = [&](IBuffer& buffer, bool vertexData) {
        void* mapped = nullptr;
        context.MapBuffer(&buffer, MAP_WRITE, MAP_FLAG_DISCARD, mapped);
        require(mapped != nullptr, "Failed to map Diligent ImGui buffer");
        auto* output = static_cast<std::byte*>(mapped);
        for (const ImDrawList* list : drawData.CmdLists) {
            const auto bytes =
                vertexData ? static_cast<std::size_t>(list->VtxBuffer.Size) * sizeof(ImDrawVert)
                           : static_cast<std::size_t>(list->IdxBuffer.Size) * sizeof(ImDrawIdx);
            if (bytes != 0) {
                std::memcpy(output,
                            vertexData ? static_cast<const void*>(list->VtxBuffer.Data)
                                       : static_cast<const void*>(list->IdxBuffer.Data),
                            bytes);
                output += bytes;
            }
        }
        context.UnmapBuffer(&buffer, MAP_WRITE);
    };
    upload(*impl_->vertices, true);
    upload(*impl_->indices, false);
    const float sx = 2.0F / drawData.DisplaySize.x;
    const float sy = -2.0F / drawData.DisplaySize.y;
    const std::array<float, 4> matrix{sx, sy, -1.0F - sx * drawData.DisplayPos.x,
                                      1.0F - sy * drawData.DisplayPos.y};
    if (!std::all_of(matrix.begin(), matrix.end(),
                     [](float value) { return std::isfinite(value); })) {
        throw std::invalid_argument("ImGui projection is not finite");
    }
    void* mapped = nullptr;
    context.MapBuffer(impl_->transform, MAP_WRITE, MAP_FLAG_DISCARD, mapped);
    require(mapped != nullptr, "Failed to map Diligent ImGui transform");
    std::memcpy(mapped, matrix.data(), sizeof(matrix));
    context.UnmapBuffer(impl_->transform, MAP_WRITE);
    const auto setupState = [&]() {
        ITextureView* targetView = &destination;
        context.SetRenderTargets(1, &targetView, nullptr,
                                 RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        const Viewport viewport{
            0.0F, 0.0F, static_cast<float>(fbWidth), static_cast<float>(fbHeight), 0.0F, 1.0F};
        context.SetViewports(1, &viewport, fbWidth, fbHeight);
        IBuffer* buffer = impl_->vertices;
        context.SetVertexBuffers(0, 1, &buffer, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION,
                                 SET_VERTEX_BUFFERS_FLAG_RESET);
        context.SetIndexBuffer(impl_->indices, 0, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.SetPipelineState(impl_->pipeline);
    };
    setupState();
    auto* texture = impl_->binding->GetVariableByName(SHADER_TYPE_PIXEL, "GuiTexture");
    require(texture != nullptr, "Diligent ImGui texture binding is missing");
    Uint32 vertexOffset = 0, indexOffset = 0;
    static_assert(sizeof(ImDrawIdx) == 2 || sizeof(ImDrawIdx) == 4);
    for (const ImDrawList* list : drawData.CmdLists) {
        for (const ImDrawCmd& cmd : list->CmdBuffer) {
            if (cmd.UserCallback != nullptr) {
                if (cmd.UserCallback == ImDrawCallback_ResetRenderState) {
                    setupState();
                } else {
                    cmd.UserCallback(list, &cmd);
                }
                continue;
            }
            if (cmd.IdxOffset > static_cast<unsigned int>(list->IdxBuffer.Size) ||
                cmd.ElemCount > static_cast<unsigned int>(list->IdxBuffer.Size) - cmd.IdxOffset ||
                cmd.VtxOffset > static_cast<unsigned int>(list->VtxBuffer.Size)) {
                throw std::invalid_argument("ImGui draw command exceeds its buffers");
            }
            const float coordinates[] = {
                (cmd.ClipRect.x - drawData.DisplayPos.x) * drawData.FramebufferScale.x,
                (cmd.ClipRect.y - drawData.DisplayPos.y) * drawData.FramebufferScale.y,
                (cmd.ClipRect.z - drawData.DisplayPos.x) * drawData.FramebufferScale.x,
                (cmd.ClipRect.w - drawData.DisplayPos.y) * drawData.FramebufferScale.y};
            if (!std::all_of(std::begin(coordinates), std::end(coordinates),
                             [](float value) { return std::isfinite(value); })) {
                throw std::invalid_argument("ImGui clip rectangle is not finite");
            }
            const auto clipCoordinate = [](float value, Uint32 limit) {
                return static_cast<Int32>(
                    std::clamp(static_cast<double>(value), 0.0, static_cast<double>(limit)));
            };
            const Rect clip{
                clipCoordinate(coordinates[0], fbWidth), clipCoordinate(coordinates[1], fbHeight),
                clipCoordinate(coordinates[2], fbWidth), clipCoordinate(coordinates[3], fbHeight)};
            if (!clip.IsValid() || cmd.ElemCount == 0) {
                continue;
            }
            const auto found = impl_->textures.find(cmd.GetTexID());
            if (found == impl_->textures.end()) {
                throw std::invalid_argument(
                    "ImGui draw command references an unregistered texture");
            }
            if (found->second->GetTexture() == destination.GetTexture()) {
                throw std::invalid_argument("ImGui cannot sample its render target");
            }
            context.SetScissorRects(1, &clip, fbWidth, fbHeight);
            texture->Set(found->second);
            context.CommitShaderResources(impl_->binding,
                                          RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
            DrawIndexedAttribs draw;
            draw.NumIndices = cmd.ElemCount;
            draw.IndexType = sizeof(ImDrawIdx) == 2 ? VT_UINT16 : VT_UINT32;
            draw.FirstIndexLocation = indexOffset + cmd.IdxOffset;
            draw.BaseVertex = vertexOffset + cmd.VtxOffset;
            context.DrawIndexed(draw);
        }
        vertexOffset += static_cast<Uint32>(list->VtxBuffer.Size);
        indexOffset += static_cast<Uint32>(list->IdxBuffer.Size);
    }
}

} // namespace openbus::rendering
