#include "DiligentImGuiRendering.h"

#include "assets/TextureLoader.h"
#include "render/diligent/DiligentImGuiRenderer.h"
#include "render/diligent/DiligentTexture2D.h"

#include <Common/interface/RefCntAutoPtr.hpp>
#include <Graphics/GraphicsEngine/interface/DeviceContext.h>
#include <Graphics/GraphicsEngine/interface/RenderDevice.h>
#include <Graphics/GraphicsEngine/interface/Texture.h>
#include <Graphics/GraphicsEngine/interface/TextureView.h>

#include <cstdint>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

struct CallbackState {
    Diligent::IDeviceContext* context;
    int calls = 0;
};

void resetTestCallback(const ImDrawList*, const ImDrawCmd* cmd) {
    auto& state = *static_cast<CallbackState*>(cmd->UserCallbackData);
    ++state.calls;
    state.context->SetRenderTargets(0, nullptr, nullptr,
                                    Diligent::RESOURCE_STATE_TRANSITION_MODE_NONE);
}

} // namespace

void verifyDiligentImGuiRendering(Diligent::IRenderDevice& device,
                                  Diligent::IDeviceContext& context) {
    using namespace Diligent;
    using namespace openbus::rendering;
    ImGuiContext* previous = ImGui::GetCurrentContext();
    ImGuiContext* gui = ImGui::CreateContext();
    require(gui != nullptr, "Failed to create ImGui probe context");
    struct ContextCleanup {
        ImGuiContext* current;
        ImGuiContext* previous;
        ~ContextCleanup() {
            ImGui::DestroyContext(current);
            ImGui::SetCurrentContext(previous);
        }
    } cleanup{gui, previous};
    ImGui::SetCurrentContext(gui);
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = {32.0F, 32.0F};
    io.DisplayFramebufferScale = {2.0F, 2.0F};
    io.DeltaTime = 1.0F / 60.0F;

    TextureDesc desc;
    desc.Name = "Diligent ImGui test target";
    desc.Type = RESOURCE_DIM_TEX_2D;
    desc.Width = 64;
    desc.Height = 64;
    desc.MipLevels = 1;
    desc.Format = TEX_FORMAT_RGBA8_UNORM;
    desc.BindFlags = BIND_RENDER_TARGET | BIND_SHADER_RESOURCE;
    RefCntAutoPtr<ITexture> target;
    device.CreateTexture(desc, nullptr, &target);
    require(static_cast<bool>(target), "Failed to create ImGui test target");
    auto* rtv = target->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    require(rtv != nullptr, "Failed to create ImGui target view");
    desc.Name = "Diligent ImGui test readback";
    desc.BindFlags = BIND_NONE;
    desc.Usage = USAGE_STAGING;
    desc.CPUAccessFlags = CPU_ACCESS_READ;
    RefCntAutoPtr<ITexture> staging;
    device.CreateTexture(desc, nullptr, &staging);
    require(static_cast<bool>(staging), "Failed to create ImGui staging texture");

    const auto clear = [&]() {
        constexpr float black[] = {0.0F, 0.0F, 0.0F, 1.0F};
        context.SetRenderTargets(1, &rtv, nullptr, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
        context.ClearRenderTarget(rtv, black, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    };
    const auto readback = [&]() {
        context.SetRenderTargets(0, nullptr, nullptr, RESOURCE_STATE_TRANSITION_MODE_NONE);
        CopyTextureAttribs copy;
        copy.pSrcTexture = target;
        copy.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        copy.pDstTexture = staging;
        copy.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        context.CopyTexture(copy);
        context.WaitForIdle();
        MappedTextureSubresource mapped;
        context.MapTextureSubresource(staging, 0, 0, MAP_READ, MAP_FLAG_DO_NOT_WAIT, nullptr,
                                      mapped);
        require(mapped.pData != nullptr, "Failed to map ImGui readback");
        return mapped;
    };
    const auto channel = [](const MappedTextureSubresource& mapped, Uint32 x, Uint32 y,
                            std::size_t component) {
        return *(static_cast<const std::uint8_t*>(mapped.pData) + y * mapped.Stride + x * 4 +
                 component);
    };

    {
        DiligentImGuiRenderer renderer(device, TEX_FORMAT_RGBA8_UNORM, *gui);
        require((io.BackendFlags & ImGuiBackendFlags_RendererHasVtxOffset) != 0 &&
                    io.Fonts->TexID != 0,
                "ImGui renderer did not initialize its font atlas and vertex offsets");
        DiligentTexture2D green(device, Image{1, 1, {0, 255, 0, 128}});
        const ImTextureID greenId = renderer.registerTexture(green.shaderResourceView());
        CallbackState callback{&context};
        ImGui::NewFrame();
        ImGui::GetBackgroundDrawList()->AddRectFilled({22.0F, 12.0F}, {28.0F, 14.0F},
                                                      IM_COL32(255, 255, 0, 255));
        ImDrawList* list = ImGui::GetForegroundDrawList();
        list->PushClipRect({4.0F, 4.0F}, {10.0F, 10.0F}, true);
        list->AddRectFilled({2.0F, 2.0F}, {14.0F, 14.0F}, IM_COL32(255, 0, 0, 255));
        list->PopClipRect();
        list->AddImage(greenId, {14.0F, 2.0F}, {20.0F, 8.0F});
        list->AddCallback(resetTestCallback, &callback);
        list->AddCallback(ImDrawCallback_ResetRenderState, nullptr);
        list->AddRectFilled({22.0F, 2.0F}, {28.0F, 8.0F}, IM_COL32(0, 0, 255, 255));
        list->AddText({1.0F, 16.0F}, IM_COL32(255, 255, 255, 255), "A");
        ImGui::Render();
        ImDrawData& drawData = *ImGui::GetDrawData();
        require(drawData.CmdListsCount >= 2, "ImGui probe did not generate multiple draw lists");
        drawData.DisplayPos = {10.0F, 20.0F};
        for (ImDrawList* drawList : drawData.CmdLists) {
            for (ImDrawVert& vertex : drawList->VtxBuffer) {
                vertex.pos.x += 10.0F;
                vertex.pos.y += 20.0F;
            }
            for (ImDrawCmd& cmd : drawList->CmdBuffer) {
                cmd.ClipRect.x += 10.0F;
                cmd.ClipRect.z += 10.0F;
                cmd.ClipRect.y += 20.0F;
                cmd.ClipRect.w += 20.0F;
            }
        }
        clear();
        renderer.render(context, drawData, *rtv);
        auto mapped = readback();
        const bool clipping = channel(mapped, 9, 9, 0) == 255 && channel(mapped, 5, 5, 0) == 0 &&
                              channel(mapped, 21, 21, 0) == 0;
        const bool blending = channel(mapped, 32, 8, 1) >= 127 &&
                              channel(mapped, 32, 8, 1) <= 129 && channel(mapped, 32, 8, 0) == 0 &&
                              channel(mapped, 32, 8, 3) == 255;
        const bool reset = channel(mapped, 50, 8, 2) == 255 && callback.calls == 1;
        const bool multipleLists =
            channel(mapped, 50, 26, 0) == 255 && channel(mapped, 50, 26, 1) == 255;
        bool textVisible = false;
        for (Uint32 y = 32; y < 62; ++y) {
            for (Uint32 x = 2; x < 24; ++x) {
                textVisible = textVisible || channel(mapped, x, y, 0) > 40;
            }
        }
        context.UnmapTextureSubresource(staging, 0, 0);
        require(clipping && blending && reset && multipleLists && textVisible,
                "ImGui clipping, blending, draw lists, callbacks, or font rendering failed");
        renderer.unregisterTexture(greenId);
        bool unregisteredRejected = false;
        try {
            renderer.render(context, drawData, *rtv);
        } catch (const std::invalid_argument&) {
            unregisteredRejected = true;
        }
        require(unregisteredRejected, "ImGui accepted an unregistered texture ID");
        renderer.rebuildFontTexture();
        require(io.Fonts->TexID != 0, "ImGui font atlas rebuild lost its texture ID");

        io.DisplaySize = {25.0F, 25.0F};
        io.DisplayFramebufferScale = {64.0F / 25.0F, 64.0F / 25.0F};
        ImGui::NewFrame();
        list = ImGui::GetForegroundDrawList();
        for (int i = 0; i < 17000; ++i) {
            list->AddRectFilled({0.0F, 0.0F}, {0.1F, 0.1F}, IM_COL32_WHITE);
        }
        list->AddRectFilled({18.0F, 18.0F}, {24.0F, 24.0F}, IM_COL32(0, 0, 255, 255));
        ImGui::Render();
        ImDrawData& large = *ImGui::GetDrawData();
        bool hasVertexOffset = false;
        for (const ImDrawList* drawList : large.CmdLists) {
            for (const ImDrawCmd& cmd : drawList->CmdBuffer) {
                hasVertexOffset = hasVertexOffset || cmd.VtxOffset > 0;
            }
        }
        require(hasVertexOffset, "ImGui probe did not generate a large-mesh vertex offset");
        clear();
        renderer.render(context, large, *rtv);
        mapped = readback();
        const bool largeMesh = channel(mapped, 54, 54, 2) == 255;
        context.UnmapTextureSubresource(staging, 0, 0);
        require(largeMesh, "ImGui large-mesh vertex offset or fractional DPI scaling failed");

        auto* targetSrv = target->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);
        require(targetSrv != nullptr, "ImGui alias test view is missing");
        const ImTextureID aliasId = renderer.registerTexture(*targetSrv);
        ImGui::NewFrame();
        ImGui::GetForegroundDrawList()->AddImage(aliasId, {1.0F, 1.0F}, {5.0F, 5.0F});
        ImGui::Render();
        bool aliasRejected = false;
        try {
            renderer.render(context, *ImGui::GetDrawData(), *rtv);
        } catch (const std::invalid_argument&) {
            aliasRejected = true;
        }
        renderer.unregisterTexture(aliasId);
        require(aliasRejected, "ImGui accepted its render target as a sampled texture");
    }
    require(io.BackendRendererUserData == nullptr && io.Fonts->TexID == 0 &&
                (io.BackendFlags & ImGuiBackendFlags_RendererHasVtxOffset) == 0,
            "Diligent ImGui renderer did not clean up context state");
}
