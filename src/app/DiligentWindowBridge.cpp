#include "app/DiligentWindowBridge.h"

#include "app/DiligentFrameRenderer.h"
#include "app/DiligentGraphicsPreferences.h"
#include "core/Logger.h"

namespace openbus::app {

std::unique_ptr<openbus::rendering::DiligentFrameRenderer>
createDiligentFrameRenderer(GLFWwindow& window,
                            const openbus::core::GraphicsPreferences& graphicsPreferences,
                            bool includeLinearDepth) {
    int drawableWidth = 0;
    int drawableHeight = 0;
    glfwGetFramebufferSize(&window, &drawableWidth, &drawableHeight);
    if (drawableWidth <= 0 || drawableHeight <= 0) {
        throw std::invalid_argument(
            "Diligent frame renderer requires a nonzero drawable size at initialization");
    }

    const auto windowConfiguration =
        makeDiligentWindowConfiguration(window, static_cast<Diligent::Uint32>(drawableWidth),
                                        static_cast<Diligent::Uint32>(drawableHeight));
    const auto graphicsOptions = makeDiligentGraphicsOptions(graphicsPreferences);
    auto renderer = std::make_unique<openbus::rendering::DiligentFrameRenderer>(
        window, windowConfiguration.nativeWindow, windowConfiguration.swapChainDesc,
        graphicsOptions, includeLinearDepth);
    if (graphicsPreferences.antiAliasingMode == "msaa" &&
        renderer->targets().samples() < graphicsOptions.multisampling.samples) {
        gameLog.Warning(renderer->targets().samples() == 1
                            ? "Diligent MSAA is unavailable; using FXAA instead."
                            : "Diligent MSAA 4x is unavailable; using 2x MSAA.");
    }
    return renderer;
}

} // namespace openbus::app
