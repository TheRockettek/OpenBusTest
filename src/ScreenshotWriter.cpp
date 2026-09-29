#include "ScreenshotWriter.h"

#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cstdint>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace openbus::rendering {

bool saveFramebufferPng(const std::filesystem::path& path, int width, int height) {
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height * 4);
    std::vector<std::uint8_t> topDown(pixels.size());
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    const std::size_t rowBytes = static_cast<std::size_t>(width) * 4;
    for (int y = 0; y < height; ++y) {
        const std::size_t source = static_cast<std::size_t>(height - 1 - y) * rowBytes;
        const std::size_t target = static_cast<std::size_t>(y) * rowBytes;
        std::copy_n(pixels.data() + source, rowBytes, topDown.data() + target);
    }
    return stbi_write_png(path.string().c_str(), width, height, 4, topDown.data(),
                          static_cast<int>(rowBytes)) != 0;
}

} // namespace openbus::rendering
