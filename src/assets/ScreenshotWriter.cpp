#include "assets/ScreenshotWriter.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <cstdint>
#include <limits>
#include <vector>

namespace openbus::rendering {

bool saveFramebufferPng(const std::filesystem::path& path, int width, int height) {
    if (width <= 0 || height <= 0 ||
        static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / 4 ||
        static_cast<std::size_t>(height) >
            std::numeric_limits<std::size_t>::max() / (static_cast<std::size_t>(width) * 4)) {
        return false;
    }
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height * 4);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    const std::size_t rowBytes = static_cast<std::size_t>(width) * 4;
    return saveRgbaPng(path, width, height, pixels, rowBytes, true);
}

} // namespace openbus::rendering
