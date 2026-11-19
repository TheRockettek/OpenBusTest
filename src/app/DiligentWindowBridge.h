#pragma once

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#ifndef GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#endif

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#include <Graphics/GraphicsEngine/interface/SwapChain.h>
#include <Graphics/GraphicsEngineVulkan/interface/EngineFactoryVk.h>

#include <memory>
#include <stdexcept>

namespace openbus::core {
struct GraphicsPreferences;
}

namespace openbus::rendering {
class DiligentFrameRenderer;
}

namespace openbus::app {

struct DiligentWindowConfiguration {
    Diligent::NativeWindow nativeWindow;
    Diligent::SwapChainDesc swapChainDesc;
};

inline DiligentWindowConfiguration
makeDiligentWindowConfiguration(GLFWwindow& window, Diligent::Uint32 drawableWidth,
                                Diligent::Uint32 drawableHeight) {
    if (glfwGetWindowAttrib(&window, GLFW_CLIENT_API) != GLFW_NO_API) {
        throw std::invalid_argument("Diligent Vulkan windows must not have a client API");
    }
    if (drawableWidth == 0 || drawableHeight == 0) {
        throw std::invalid_argument(
            "Diligent swap chains require nonzero drawable dimensions at creation");
    }

    DiligentWindowConfiguration configuration;
    configuration.nativeWindow.hWnd = glfwGetWin32Window(&window);
    if (configuration.nativeWindow.hWnd == nullptr) {
        throw std::runtime_error("GLFW did not provide a Win32 handle for the Diligent window");
    }
    configuration.swapChainDesc.Width = drawableWidth;
    configuration.swapChainDesc.Height = drawableHeight;
    configuration.swapChainDesc.ColorBufferFormat = Diligent::TEX_FORMAT_RGBA8_UNORM;
    return configuration;
}

std::unique_ptr<openbus::rendering::DiligentFrameRenderer>
createDiligentFrameRenderer(GLFWwindow& window,
                            const openbus::core::GraphicsPreferences& graphicsPreferences,
                            bool includeLinearDepth = true);

} // namespace openbus::app
