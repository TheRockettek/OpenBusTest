#include "assets/ScreenshotWriter.h"

#include <algorithm>
#include <limits>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace openbus::rendering {

bool saveRgbaPng(const std::filesystem::path& path, int width, int height,
                 std::span<const std::uint8_t> pixels, std::size_t rowStride, bool flipVertically) {
    if (width <= 0 || height <= 0 ||
        static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / 4) {
        return false;
    }
    const std::size_t rowBytes = static_cast<std::size_t>(width) * 4;
    if (rowStride < rowBytes ||
        static_cast<std::size_t>(height) > std::numeric_limits<std::size_t>::max() / rowStride ||
        static_cast<std::size_t>(height) > std::numeric_limits<std::size_t>::max() / rowBytes ||
        pixels.size() < rowStride * static_cast<std::size_t>(height) ||
        rowBytes > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return false;
    }

    std::vector<std::uint8_t> topDown(rowBytes * static_cast<std::size_t>(height));
    for (int y = 0; y < height; ++y) {
        const std::size_t sourceY = static_cast<std::size_t>(flipVertically ? height - 1 - y : y);
        std::copy_n(pixels.data() + sourceY * rowStride, rowBytes,
                    topDown.data() + static_cast<std::size_t>(y) * rowBytes);
    }
    return stbi_write_png(path.string().c_str(), width, height, 4, topDown.data(),
                          static_cast<int>(rowBytes)) != 0;
}

} // namespace openbus::rendering
