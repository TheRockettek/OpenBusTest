#include "render/diligent/DiligentTextureCompression.h"

#include "assets/TextureLoader.h"

#include <gli/gli.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace openbus::rendering::diligent_detail {

gli::texture2d decodeDxt1AlphaMipChain(const gli::texture& image, bool srgb) {
    const auto baseExtent = image.extent(0);
    std::size_t totalDecodedBytes = 0;
    for (std::size_t mip = 0; mip < image.levels(); ++mip) {
        const auto extent = image.extent(mip);
        const std::size_t width = static_cast<std::size_t>(extent.x);
        const std::size_t height = static_cast<std::size_t>(extent.y);
        std::size_t decodedBytes = 0;
        if (extent.x <= 0 || extent.y <= 0 ||
            !checkedTextureBufferSize(width, height, 4, decodedBytes) ||
            totalDecodedBytes > MAX_TEXTURE_BUFFER_BYTES - decodedBytes) {
            throw std::length_error("Diligent decoded BC1 texture exceeds the upload limit");
        }
        totalDecodedBytes += decodedBytes;

        const std::size_t blocksX = (width + 3U) / 4U;
        const std::size_t blocksY = (height + 3U) / 4U;
        if (blocksX > std::numeric_limits<std::size_t>::max() / blocksY ||
            blocksX * blocksY > std::numeric_limits<std::size_t>::max() / 8U ||
            image.size(mip) != blocksX * blocksY * 8U || image.data(0, 0, mip) == nullptr) {
            throw std::invalid_argument("Diligent BC1 mip data does not match its dimensions");
        }
    }

    gli::texture2d decoded(srgb ? gli::FORMAT_RGBA8_SRGB_PACK8 : gli::FORMAT_RGBA8_UNORM_PACK8,
                           gli::extent2d(baseExtent.x, baseExtent.y), image.levels());
    for (std::size_t mip = 0; mip < image.levels(); ++mip) {
        const auto extent = image.extent(mip);
        const std::size_t width = static_cast<std::size_t>(extent.x);
        const std::size_t height = static_cast<std::size_t>(extent.y);
        const std::size_t blocksX = (width + 3U) / 4U;
        const std::size_t blocksY = (height + 3U) / 4U;
        const auto* source = static_cast<const std::uint8_t*>(image.data(0, 0, mip));
        auto* destination = static_cast<std::uint8_t*>(decoded.data(0, 0, mip));
        for (std::size_t blockY = 0; blockY < blocksY; ++blockY) {
            for (std::size_t blockX = 0; blockX < blocksX; ++blockX) {
                const std::size_t blockOffset = (blockY * blocksX + blockX) * 8U;
                const std::uint16_t color0 =
                    static_cast<std::uint16_t>(source[blockOffset]) |
                    (static_cast<std::uint16_t>(source[blockOffset + 1]) << 8U);
                const std::uint16_t color1 =
                    static_cast<std::uint16_t>(source[blockOffset + 2]) |
                    (static_cast<std::uint16_t>(source[blockOffset + 3]) << 8U);
                const std::uint32_t selectors =
                    static_cast<std::uint32_t>(source[blockOffset + 4]) |
                    (static_cast<std::uint32_t>(source[blockOffset + 5]) << 8U) |
                    (static_cast<std::uint32_t>(source[blockOffset + 6]) << 16U) |
                    (static_cast<std::uint32_t>(source[blockOffset + 7]) << 24U);
                std::array<std::array<std::uint8_t, 4>, 4> colors = {};
                const auto decode565 = [](std::uint16_t value, std::uint8_t* color) {
                    color[0] = static_cast<std::uint8_t>(((value >> 11U) & 0x1fU) * 255U / 31U);
                    color[1] = static_cast<std::uint8_t>(((value >> 5U) & 0x3fU) * 255U / 63U);
                    color[2] = static_cast<std::uint8_t>((value & 0x1fU) * 255U / 31U);
                    color[3] = 255;
                };
                decode565(color0, colors[0].data());
                decode565(color1, colors[1].data());
                if (color0 > color1) {
                    for (std::size_t channel = 0; channel < 3; ++channel) {
                        colors[2][channel] = static_cast<std::uint8_t>(
                            (2U * colors[0][channel] + colors[1][channel]) / 3U);
                        colors[3][channel] = static_cast<std::uint8_t>(
                            (colors[0][channel] + 2U * colors[1][channel]) / 3U);
                    }
                    colors[2][3] = colors[3][3] = 255;
                } else {
                    for (std::size_t channel = 0; channel < 3; ++channel) {
                        colors[2][channel] = static_cast<std::uint8_t>(
                            (colors[0][channel] + colors[1][channel]) / 2U);
                    }
                    colors[2][3] = 255;
                    colors[3] = {0, 0, 0, 0};
                }

                for (std::size_t row = 0; row < 4; ++row) {
                    for (std::size_t column = 0; column < 4; ++column) {
                        const std::size_t x = blockX * 4U + column;
                        const std::size_t y = blockY * 4U + row;
                        if (x >= width || y >= height) {
                            continue;
                        }
                        const std::size_t selectorIndex = row * 4U + column;
                        const std::size_t colorIndex = (selectors >> (2U * selectorIndex)) & 0x3U;
                        const std::size_t pixelOffset = (y * width + x) * 4U;
                        std::copy(colors[colorIndex].begin(), colors[colorIndex].end(),
                                  destination + pixelOffset);
                    }
                }
            }
        }
    }
    return decoded;
}

} // namespace openbus::rendering::diligent_detail
