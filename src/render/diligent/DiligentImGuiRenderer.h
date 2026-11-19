#pragma once

#include <Graphics/GraphicsEngine/interface/GraphicsTypes.h>
#include <imgui.h>

#include <memory>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

namespace openbus::rendering {

// Renderer for one ImGui context. Destroy this object before destroying its context.
// The caller initializes the platform backend, owns frame ordering and synchronization,
// and must use views belonging to the renderer's device.
class DiligentImGuiRenderer final {
  public:
    DiligentImGuiRenderer(Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT targetFormat,
                          ImGuiContext& guiContext);
    ~DiligentImGuiRenderer();
    DiligentImGuiRenderer(const DiligentImGuiRenderer&) = delete;
    DiligentImGuiRenderer& operator=(const DiligentImGuiRenderer&) = delete;

    // Call after changing the atlas, outside an active ImGui frame.
    void rebuildFontTexture();
    // IDs retain their shader view until unregistered or renderer destruction. Do not unregister
    // an ID until all draw data that references it has been submitted.
    [[nodiscard]] ImTextureID registerTexture(Diligent::ITextureView& view);
    void unregisterTexture(ImTextureID id);

    // Draw data must be from this context after ImGui::Render(). Binds destination and changes
    // context state; callers must restore their own state before subsequent drawing.
    void render(Diligent::IDeviceContext& context, const ImDrawData& drawData,
                Diligent::ITextureView& destination);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
