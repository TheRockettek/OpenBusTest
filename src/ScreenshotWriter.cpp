#include "ScreenshotWriter.h"

#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <array>
#include <cstdint>
#include <fstream>
#include <vector>

namespace openbus::rendering {

bool saveFramebufferBmp(const std::filesystem::path& path, int width, int height) {
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height * 4);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

    std::array<std::uint8_t, 54> header = {};
    // TODO: Account for four-byte BMP row padding when writing screenshots.
    const std::uint32_t imageSize = static_cast<std::uint32_t>(width * height * 3);
    const std::uint32_t fileSize = 54 + imageSize;
    header[0] = 'B';
    header[1] = 'M';
    header[2] = static_cast<std::uint8_t>(fileSize);
    header[3] = static_cast<std::uint8_t>(fileSize >> 8);
    header[4] = static_cast<std::uint8_t>(fileSize >> 16);
    header[5] = static_cast<std::uint8_t>(fileSize >> 24);
    header[10] = 54;
    header[14] = 40;
    header[18] = static_cast<std::uint8_t>(width);
    header[19] = static_cast<std::uint8_t>(width >> 8);
    header[20] = static_cast<std::uint8_t>(width >> 16);
    header[21] = static_cast<std::uint8_t>(width >> 24);
    header[22] = static_cast<std::uint8_t>(height);
    header[23] = static_cast<std::uint8_t>(height >> 8);
    header[24] = static_cast<std::uint8_t>(height >> 16);
    header[25] = static_cast<std::uint8_t>(height >> 24);
    header[26] = 1;
    header[28] = 24;
    header[34] = static_cast<std::uint8_t>(imageSize);
    header[35] = static_cast<std::uint8_t>(imageSize >> 8);
    header[36] = static_cast<std::uint8_t>(imageSize >> 16);
    header[37] = static_cast<std::uint8_t>(imageSize >> 24);

    std::ofstream output(path, std::ios::binary);
    if (!output) {
        return false;
    }
    output.write(reinterpret_cast<const char*>(header.data()), header.size());
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t index = (static_cast<std::size_t>(y) * width + x) * 4;
            const std::array<std::uint8_t, 3> pixel = {pixels[index + 2], pixels[index + 1],
                                                       pixels[index]};
            output.write(reinterpret_cast<const char*>(pixel.data()), pixel.size());
        }
    }
    return output.good();
}

} // namespace openbus::rendering
