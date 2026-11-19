#include "app/DiligentImGuiLayer.h"

#include "render/diligent/DiligentImGuiRenderer.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>

#include <stdexcept>

namespace {

class ContextGuard {
  public:
    explicit ContextGuard(ImGuiContext& context) : previous_(ImGui::GetCurrentContext()) {
        ImGui::SetCurrentContext(&context);
    }
    ~ContextGuard() {
        ImGui::SetCurrentContext(previous_);
    }

  private:
    ImGuiContext* previous_;
};

struct ContextDeleter {
    void operator()(ImGuiContext* context) const {
        ImGui::DestroyContext(context);
    }
};

class GlfwPlatform {
  public:
    GlfwPlatform(GLFWwindow& window, ImGuiContext& gui) : gui_(gui) {
        ContextGuard guard(gui_);
        const int clientApi = glfwGetWindowAttrib(&window, GLFW_CLIENT_API);
        if (clientApi != GLFW_NO_API) {
            throw std::invalid_argument("Diligent Vulkan ImGui requires a GLFW no-API window");
        }
        if (!ImGui_ImplGlfw_InitForVulkan(&window, true)) {
            throw std::runtime_error("Failed to initialize the ImGui GLFW platform backend");
        }
    }
    ~GlfwPlatform() {
        ContextGuard guard(gui_);
        ImGui_ImplGlfw_Shutdown();
    }
    GlfwPlatform(const GlfwPlatform&) = delete;
    GlfwPlatform& operator=(const GlfwPlatform&) = delete;

  private:
    ImGuiContext& gui_;
};

std::unique_ptr<ImGuiContext, ContextDeleter> createContext() {
    IMGUI_CHECKVERSION();
    ImGuiContext* gui = ImGui::CreateContext();
    if (gui == nullptr) {
        throw std::runtime_error("Failed to create ImGui context");
    }
    {
        ContextGuard guard(*gui);
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::StyleColorsDark();
    }
    return std::unique_ptr<ImGuiContext, ContextDeleter>(gui);
}

} // namespace

struct DiligentImGuiLayer::Impl {
    std::unique_ptr<ImGuiContext, ContextDeleter> gui;
    GlfwPlatform platform;
    openbus::rendering::DiligentImGuiRenderer renderer;
    bool frameActive = false;

    Impl(GLFWwindow& window, Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT format)
        : gui(createContext()), platform(window, *gui), renderer(device, format, *gui) {}
};

DiligentImGuiLayer::DiligentImGuiLayer(GLFWwindow& window, Diligent::IRenderDevice& device,
                                       Diligent::TEXTURE_FORMAT targetFormat)
    : impl_(std::make_unique<Impl>(window, device, targetFormat)) {}

DiligentImGuiLayer::~DiligentImGuiLayer() {
    finishFrameWithoutRendering();
}

void DiligentImGuiLayer::beginFrame(bool manageMouseCursor) {
    ImGui::SetCurrentContext(impl_->gui.get());
    if (impl_->frameActive) {
        throw std::logic_error("Diligent ImGui frame has already started");
    }
    ImGuiIO& io = ImGui::GetIO();
    if (manageMouseCursor) {
        io.ConfigFlags &= ~ImGuiConfigFlags_NoMouseCursorChange;
    } else {
        io.ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
    }
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    impl_->frameActive = true;
}

void DiligentImGuiLayer::render(Diligent::IDeviceContext& context,
                                Diligent::ITextureView& destination) {
    ImGui::SetCurrentContext(impl_->gui.get());
    if (!impl_->frameActive) {
        throw std::logic_error("Diligent ImGui frame has not started");
    }
    ImGui::Render();
    impl_->frameActive = false;
    impl_->renderer.render(context, *ImGui::GetDrawData(), destination);
}

void DiligentImGuiLayer::finishFrameWithoutRendering() noexcept {
    if (!impl_ || !impl_->frameActive) {
        return;
    }
    ImGui::SetCurrentContext(impl_->gui.get());
    ImGui::EndFrame();
    impl_->frameActive = false;
}

ImTextureID DiligentImGuiLayer::registerTexture(Diligent::ITextureView& view) {
    return impl_->renderer.registerTexture(view);
}

void DiligentImGuiLayer::unregisterTexture(ImTextureID id) {
    impl_->renderer.unregisterTexture(id);
}

void DiligentImGuiLayer::rebuildFontTexture() {
    impl_->renderer.rebuildFontTexture();
}
