#pragma once

#include <Graphics/GraphicsEngine/interface/GraphicsTypes.h>
#include <imgui.h>

#include <memory>

struct GLFWwindow;
namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
struct ITextureView;
} // namespace Diligent

// GLFW input and Diligent GPU rendering for one ImGui context. The layer must be
// destroyed before its window and Diligent device.
class DiligentImGuiLayer final {
  public:
    DiligentImGuiLayer(GLFWwindow& window, Diligent::IRenderDevice& device,
                       Diligent::TEXTURE_FORMAT targetFormat);
    ~DiligentImGuiLayer();
    DiligentImGuiLayer(const DiligentImGuiLayer&) = delete;
    DiligentImGuiLayer& operator=(const DiligentImGuiLayer&) = delete;

    void beginFrame(bool manageMouseCursor = true);
    void render(Diligent::IDeviceContext& context, Diligent::ITextureView& destination);
    void finishFrameWithoutRendering() noexcept;
    // Registered texture IDs remain valid until explicitly unregistered or layer destruction.
    [[nodiscard]] ImTextureID registerTexture(Diligent::ITextureView& view);
    void unregisterTexture(ImTextureID id);
    void rebuildFontTexture();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
