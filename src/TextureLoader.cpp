#include "TextureLoader.h"

#include "Logger.h"
#include "PerfTrace.h"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <string>

extern Logger gameLog;

namespace openbus::rendering {
namespace {

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::uint16_t readU16(const std::vector<std::uint8_t>& data, std::size_t offset) {
    return static_cast<std::uint16_t>(data[offset]) |
           (static_cast<std::uint16_t>(data[offset + 1]) << 8);
}

std::uint32_t readU32(const std::vector<std::uint8_t>& data, std::size_t offset) {
    return static_cast<std::uint32_t>(data[offset]) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(data[offset + 3]) << 24);
}

bool readStbImage(const std::filesystem::path& path, Image& image) {
    int width = 0;
    int height = 0;
    stbi_uc* pixels = stbi_load(path.string().c_str(), &width, &height, nullptr, 4);
    if (pixels == nullptr || width <= 0 || height <= 0) {
        if (pixels != nullptr) {
            stbi_image_free(pixels);
        }
        return false;
    }
    image.width = width;
    image.height = height;
    image.rgba.assign(pixels, pixels + static_cast<std::size_t>(width) * height * 4);
    stbi_image_free(pixels);
    return true;
}

bool readTgaImage(const std::filesystem::path& path, Image& image) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return false;
    }
    std::array<std::uint8_t, 18> header = {};
    input.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
    const bool colorMapped = header[2] == 1 || header[2] == 9;
    const bool trueColor = header[2] == 2 || header[2] == 10;
    if (!input || (!colorMapped && !trueColor)) {
        return false;
    }
    input.seekg(header[0], std::ios::cur);
    image.width = header[12] | (header[13] << 8);
    image.height = header[14] | (header[15] << 8);
    if (image.width <= 0 || image.height <= 0) {
        return false;
    }
    const int pixelBytes = (header[16] + 7) / 8;
    if ((trueColor && (header[16] != 24 && header[16] != 32)) ||
        (colorMapped && (header[16] != 8 && header[16] != 16))) {
        return false;
    }
    const int colorMapEntryBytes = (header[7] + 7) / 8;
    const std::size_t colorMapLength = header[5] | (header[6] << 8);
    const std::size_t colorMapFirst = header[3] | (header[4] << 8);
    std::vector<std::uint8_t> colorMap;
    if (colorMapped) {
        if ((header[7] != 24 && header[7] != 32) || colorMapLength == 0) {
            return false;
        }
        colorMap.resize(colorMapLength * colorMapEntryBytes);
        input.read(reinterpret_cast<char*>(colorMap.data()),
                   static_cast<std::streamsize>(colorMap.size()));
        if (!input) {
            return false;
        }
    }
    const std::size_t pixelCount = static_cast<std::size_t>(image.width) * image.height;
    std::vector<std::uint8_t> source(pixelCount * pixelBytes);
    if (header[2] == 1 || header[2] == 2) {
        input.read(reinterpret_cast<char*>(source.data()),
                   static_cast<std::streamsize>(source.size()));
        if (!input) {
            return false;
        }
    } else {
        std::size_t pixelIndex = 0;
        while (pixelIndex < pixelCount && input) {
            std::uint8_t packetHeader = 0;
            input.read(reinterpret_cast<char*>(&packetHeader), 1);
            const std::size_t packetCount = static_cast<std::size_t>(packetHeader & 0x7F) + 1;
            if (pixelIndex + packetCount > pixelCount) {
                return false;
            }
            if ((packetHeader & 0x80) != 0) {
                std::array<std::uint8_t, 4> pixel = {};
                input.read(reinterpret_cast<char*>(pixel.data()), pixelBytes);
                if (!input) {
                    return false;
                }
                for (std::size_t count = 0; count < packetCount; ++count) {
                    std::copy_n(pixel.data(), pixelBytes,
                                source.data() + (pixelIndex + count) * pixelBytes);
                }
            } else {
                input.read(reinterpret_cast<char*>(source.data() + pixelIndex * pixelBytes),
                           static_cast<std::streamsize>(packetCount * pixelBytes));
                if (!input) {
                    return false;
                }
            }
            pixelIndex += packetCount;
        }
        if (pixelIndex != pixelCount) {
            return false;
        }
    }
    image.rgba.resize(pixelCount * 4);
    const bool topOrigin = (header[17] & 0x20) != 0;
    for (int y = 0; y < image.height; ++y) {
        const int sourceY = topOrigin ? y : image.height - y - 1;
        for (int x = 0; x < image.width; ++x) {
            const int sourceX = (header[17] & 0x10) != 0 ? image.width - x - 1 : x;
            const std::size_t sourceIndex =
                (static_cast<std::size_t>(sourceY) * image.width + sourceX) * pixelBytes;
            const std::size_t targetIndex = (static_cast<std::size_t>(y) * image.width + x) * 4;
            if (colorMapped) {
                const std::size_t paletteIndex =
                    (static_cast<std::size_t>(source[sourceIndex]) |
                     (pixelBytes == 2 ? static_cast<std::size_t>(source[sourceIndex + 1]) << 8
                                      : 0)) -
                    colorMapFirst;
                if (paletteIndex >= colorMapLength) {
                    return false;
                }
                const std::size_t paletteOffset = paletteIndex * colorMapEntryBytes;
                image.rgba[targetIndex + 0] = colorMap[paletteOffset + 2];
                image.rgba[targetIndex + 1] = colorMap[paletteOffset + 1];
                image.rgba[targetIndex + 2] = colorMap[paletteOffset + 0];
                image.rgba[targetIndex + 3] =
                    colorMapEntryBytes == 4 ? colorMap[paletteOffset + 3] : 255;
            } else {
                image.rgba[targetIndex + 0] = source[sourceIndex + 2];
                image.rgba[targetIndex + 1] = source[sourceIndex + 1];
                image.rgba[targetIndex + 2] = source[sourceIndex + 0];
                image.rgba[targetIndex + 3] = pixelBytes == 4 ? source[sourceIndex + 3] : 255;
            }
        }
    }
    return true;
}

bool readBmpImage(const std::filesystem::path& path, Image& image) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return false;
    }
    std::array<std::uint8_t, 54> header = {};
    input.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
    if (!input || header[0] != 'B' || header[1] != 'M') {
        return false;
    }
    const std::uint32_t pixelOffset =
        header[10] | (header[11] << 8) | (header[12] << 16) | (header[13] << 24);
    const std::int32_t width = static_cast<std::int32_t>(header[18] | (header[19] << 8) |
                                                         (header[20] << 16) | (header[21] << 24));
    const std::int32_t height = static_cast<std::int32_t>(header[22] | (header[23] << 8) |
                                                          (header[24] << 16) | (header[25] << 24));
    const std::uint16_t bitsPerPixel = static_cast<std::uint16_t>(header[28] | (header[29] << 8));
    if (width <= 0 || height == 0 || (bitsPerPixel != 24 && bitsPerPixel != 32)) {
        return false;
    }
    if (height == std::numeric_limits<std::int32_t>::min()) {
        return false;
    }
    const int absoluteHeight = std::abs(height);
    const int channels = bitsPerPixel / 8;
    if (static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / channels) {
        return false;
    }
    const std::size_t rowStride = ((static_cast<std::size_t>(width) * channels + 3) / 4) * 4;
    if (rowStride > std::numeric_limits<std::size_t>::max() /
                        static_cast<std::size_t>(absoluteHeight)) {
        return false;
    }
    std::vector<std::uint8_t> source(rowStride * absoluteHeight);
    input.seekg(pixelOffset);
    input.read(reinterpret_cast<char*>(source.data()), static_cast<std::streamsize>(source.size()));
    if (!input) {
        return false;
    }
    image.width = width;
    image.height = absoluteHeight;
    image.rgba.resize(static_cast<std::size_t>(width) * absoluteHeight * 4);
    for (int y = 0; y < absoluteHeight; ++y) {
        const int sourceY = height > 0 ? absoluteHeight - y - 1 : y;
        for (int x = 0; x < width; ++x) {
            const std::size_t sourceIndex = static_cast<std::size_t>(sourceY) * rowStride +
                                            static_cast<std::size_t>(x) * channels;
            const std::size_t targetIndex = (static_cast<std::size_t>(y) * width + x) * 4;
            image.rgba[targetIndex + 0] = source[sourceIndex + 2];
            image.rgba[targetIndex + 1] = source[sourceIndex + 1];
            image.rgba[targetIndex + 2] = source[sourceIndex + 0];
            image.rgba[targetIndex + 3] = channels == 4 ? source[sourceIndex + 3] : 255;
        }
    }
    gameLog.Log("Successfully read image with width: " + std::to_string(image.width) +
                " and height: " + std::to_string(image.height));
    return true;
}

bool readDdsImage(const std::filesystem::path& path, Image& image) {
    TraceScope trace("texture", "readDdsImage");
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return false;
    }
    input.seekg(0, std::ios::end);
    const std::streamoff length = input.tellg();
    input.seekg(0, std::ios::beg);
    if (length < 128) {
        return false;
    }
    std::vector<std::uint8_t> data(static_cast<std::size_t>(length));
    input.read(reinterpret_cast<char*>(data.data()), length);
    if (!input || std::string(data.begin(), data.begin() + 4) != "DDS ") {
        return false;
    }
    image.width = static_cast<int>(readU32(data, 16));
    image.height = static_cast<int>(readU32(data, 12));
    const std::uint32_t pixelFormatFlags = readU32(data, 80);
    const std::string fourCC(data.begin() + 84, data.begin() + 88);
    const bool dxt1 = fourCC == "DXT1";
    const bool dxt2 = fourCC == "DXT2";
    const bool dxt3 = fourCC == "DXT3";
    const bool dxt5 = fourCC == "DXT5";
    const std::uint32_t rgbBitCount = readU32(data, 88);
    const std::uint32_t redMask = readU32(data, 92);
    const std::uint32_t greenMask = readU32(data, 96);
    const std::uint32_t blueMask = readU32(data, 100);
    const std::uint32_t alphaMask = readU32(data, 104);
    const bool dxt1HasAlpha = dxt1 && (pixelFormatFlags & 0x1U) != 0;
    if (image.width <= 0 || image.height <= 0) {
        return false;
    }
    if (!dxt1 && !dxt2 && !dxt3 && !dxt5) {
        if ((pixelFormatFlags & 0x40U) == 0 || (rgbBitCount != 24 && rgbBitCount != 32)) {
            return false;
        }
        const std::size_t bytesPerPixel = rgbBitCount / 8;
        const std::size_t minimumRowPitch = static_cast<std::size_t>(image.width) * bytesPerPixel;
        const std::size_t rowPitch = std::max<std::size_t>(readU32(data, 20), minimumRowPitch);
        if (rowPitch > (std::numeric_limits<std::size_t>::max() - 128) /
                           static_cast<std::size_t>(image.height)) {
            return false;
        }
        const std::size_t requiredSize = 128 + rowPitch * static_cast<std::size_t>(image.height);
        if (data.size() < requiredSize) {
            return false;
        }
        const auto decodeChannel = [](std::uint32_t pixel, std::uint32_t mask,
                                      std::uint8_t fallback) {
            if (mask == 0) {
                return fallback;
            }
            unsigned shift = 0;
            while ((mask & 1U) == 0) {
                mask >>= 1;
                ++shift;
            }
            const std::uint32_t component = (pixel >> shift) & mask;
            return static_cast<std::uint8_t>(
                (static_cast<std::uint64_t>(component) * 255 + mask / 2) / mask);
        };
        image.rgba.resize(static_cast<std::size_t>(image.width) * image.height * 4);
        for (int y = 0; y < image.height; ++y) {
            const std::size_t rowOffset = 128 + static_cast<std::size_t>(y) * rowPitch;
            for (int x = 0; x < image.width; ++x) {
                const std::size_t sourceOffset =
                    rowOffset + static_cast<std::size_t>(x) * bytesPerPixel;
                std::uint32_t pixel = data[sourceOffset] |
                                      (static_cast<std::uint32_t>(data[sourceOffset + 1]) << 8) |
                                      (static_cast<std::uint32_t>(data[sourceOffset + 2]) << 16);
                if (bytesPerPixel == 4) {
                    pixel |= static_cast<std::uint32_t>(data[sourceOffset + 3]) << 24;
                }
                const std::size_t targetOffset =
                    (static_cast<std::size_t>(y) * image.width + x) * 4;
                image.rgba[targetOffset + 0] = decodeChannel(pixel, redMask, 0);
                image.rgba[targetOffset + 1] = decodeChannel(pixel, greenMask, 0);
                image.rgba[targetOffset + 2] = decodeChannel(pixel, blueMask, 0);
                image.rgba[targetOffset + 3] = decodeChannel(pixel, alphaMask, 255);
            }
        }
        return true;
    }
    const std::size_t blocksX = (static_cast<std::size_t>(image.width) + 3) / 4;
    const std::size_t blocksY = (static_cast<std::size_t>(image.height) + 3) / 4;
    const bool explicitAlpha = dxt2 || dxt3;
    const std::size_t blockSize = dxt5 || explicitAlpha ? 16 : 8;
    if (blocksX > std::numeric_limits<std::size_t>::max() / blocksY ||
        blocksX * blocksY > (std::numeric_limits<std::size_t>::max() - 128) / blockSize) {
        return false;
    }
    const std::size_t requiredSize = 128 + blocksX * blocksY * blockSize;
    if (data.size() < requiredSize) {
        return false;
    }
    image.rgba.resize(static_cast<std::size_t>(image.width) * image.height * 4);
    std::size_t sourceOffset = 128;
    for (int blockY = 0; blockY < blocksY; ++blockY) {
        for (int blockX = 0; blockX < blocksX; ++blockX) {
            std::array<std::uint8_t, 8> alphaValues = {};
            std::uint64_t alphaIndices = 0;
            if (explicitAlpha) {
                for (int byte = 0; byte < 8; ++byte) {
                    alphaIndices |= static_cast<std::uint64_t>(data[sourceOffset + byte])
                                    << (8 * byte);
                }
            } else if (dxt5) {
                const std::uint8_t alpha0 = data[sourceOffset];
                const std::uint8_t alpha1 = data[sourceOffset + 1];
                alphaValues[0] = alpha0;
                alphaValues[1] = alpha1;
                if (alpha0 > alpha1) {
                    for (int index = 1; index <= 6; ++index) {
                        alphaValues[index + 1] =
                            static_cast<std::uint8_t>(((7 - index) * alpha0 + index * alpha1) / 7);
                    }
                } else {
                    for (int index = 1; index <= 4; ++index) {
                        alphaValues[index + 1] =
                            static_cast<std::uint8_t>(((5 - index) * alpha0 + index * alpha1) / 5);
                    }
                    alphaValues[6] = 0;
                    alphaValues[7] = 255;
                }
                for (int byte = 0; byte < 6; ++byte) {
                    alphaIndices |= static_cast<std::uint64_t>(data[sourceOffset + 2 + byte])
                                    << (8 * byte);
                }
            }
            const std::size_t colorOffset = sourceOffset + (dxt5 || explicitAlpha ? 8 : 0);
            const std::uint16_t color0 = readU16(data, colorOffset);
            const std::uint16_t color1 = readU16(data, colorOffset + 2);
            const std::uint32_t indices = readU32(data, colorOffset + 4);
            sourceOffset += blockSize;
            std::array<std::array<std::uint8_t, 4>, 4> colors = {};
            auto decode565 = [](std::uint16_t value, std::uint8_t* color) {
                color[0] = static_cast<std::uint8_t>(((value >> 11) & 0x1F) * 255 / 31);
                color[1] = static_cast<std::uint8_t>(((value >> 5) & 0x3F) * 255 / 63);
                color[2] = static_cast<std::uint8_t>((value & 0x1F) * 255 / 31);
                color[3] = 255;
            };
            decode565(color0, colors[0].data());
            decode565(color1, colors[1].data());
            if (color0 > color1 || dxt5 || explicitAlpha) {
                for (int channel = 0; channel < 3; ++channel) {
                    colors[2][channel] = static_cast<std::uint8_t>(
                        (2 * colors[0][channel] + colors[1][channel]) / 3);
                    colors[3][channel] = static_cast<std::uint8_t>(
                        (colors[0][channel] + 2 * colors[1][channel]) / 3);
                }
                colors[2][3] = colors[3][3] = 255;
            } else {
                for (int channel = 0; channel < 3; ++channel) {
                    colors[2][channel] =
                        static_cast<std::uint8_t>((colors[0][channel] + colors[1][channel]) / 2);
                }
                colors[2][3] = 255;
                colors[3] = {0, 0, 0, static_cast<std::uint8_t>(dxt1HasAlpha ? 0 : 255)};
            }
            for (int row = 0; row < 4; ++row) {
                for (int column = 0; column < 4; ++column) {
                    const int x = blockX * 4 + column;
                    const int y = blockY * 4 + row;
                    if (x >= image.width || y >= image.height) {
                        continue;
                    }
                    const std::size_t colorIndex = (indices >> (2 * (row * 4 + column))) & 0x3;
                    const std::size_t pixelIndex = static_cast<std::size_t>(row * 4 + column);
                    const std::size_t target = (static_cast<std::size_t>(y) * image.width + x) * 4;
                    std::copy(colors[colorIndex].begin(), colors[colorIndex].end(),
                              image.rgba.begin() + target);
                    if (explicitAlpha) {
                        image.rgba[target + 3] = static_cast<std::uint8_t>(
                            ((alphaIndices >> (4 * pixelIndex)) & 0xF) * 17);
                    } else if (dxt5) {
                        image.rgba[target + 3] =
                            alphaValues[(alphaIndices >> (3 * pixelIndex)) & 0x7];
                    }
                }
            }
        }
    }
    return true;
}

} // namespace

bool TextureLoader::readImage(const std::filesystem::path& path, Image& image) {
    TraceScope trace("texture", "readImage");
    static const bool verboseTextureReadLogs =
        parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_TEXTURE_READ"));
    if (verboseTextureReadLogs) {
        gameLog.Log("Reading image from path: " + path.string());
    }
    const std::string extension = lower(path.extension().string());
    if (extension == ".png" || extension == ".jpg" || extension == ".jpeg" || extension == ".bmp") {
        if (readStbImage(path, image)) {
            return true;
        }
    }
    if (extension == ".dds") {
        return readDdsImage(path, image);
    }
    if (extension == ".tga") {
        return readTgaImage(path, image);
    }
    if (extension != ".bmp") {
        return false;
    }
    return readBmpImage(path, image);
}

bool TextureLoader::isSafeCompressedDds(const std::filesystem::path& path) {
    TraceScope trace("texture", "isSafeCompressedDds");
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return false;
    }
    std::array<std::uint8_t, 128> header = {};
    input.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
    if (!input || std::string(header.begin(), header.begin() + 4) != "DDS ") {
        return false;
    }
    const auto readHeaderU32 = [&header](std::size_t offset) {
        return static_cast<std::uint32_t>(header[offset]) |
               (static_cast<std::uint32_t>(header[offset + 1]) << 8) |
               (static_cast<std::uint32_t>(header[offset + 2]) << 16) |
               (static_cast<std::uint32_t>(header[offset + 3]) << 24);
    };
    const std::uint32_t width = readHeaderU32(16);
    const std::uint32_t height = readHeaderU32(12);
    const std::string fourCC(header.begin() + 84, header.begin() + 88);
    if (fourCC == "DX10") {
        input.seekg(0, std::ios::end);
        const std::streamoff fileSize = input.tellg();
        return width > 0 && height > 0 && fileSize >= 148;
    }
    const std::size_t blockSize =
        fourCC == "DXT1" ? 8 : (fourCC == "DXT3" || fourCC == "DXT5" ? 16 : 0);
    if (width == 0 || height == 0 || blockSize == 0) {
        return false;
    }
    const std::uint32_t flags = readHeaderU32(8);
    const std::uint32_t declaredLevels = readHeaderU32(28);
    const std::size_t levelCount = (flags & 0x20000U) != 0 ? declaredLevels : 1;
    if (levelCount == 0 || levelCount > 32) {
        return false;
    }
    std::size_t requiredSize = 128;
    std::uint32_t levelWidth = width;
    std::uint32_t levelHeight = height;
    for (std::size_t level = 0; level < levelCount; ++level) {
        const std::size_t blocksX = (static_cast<std::size_t>(levelWidth) + 3) / 4;
        const std::size_t blocksY = (static_cast<std::size_t>(levelHeight) + 3) / 4;
        if (blocksX > std::numeric_limits<std::size_t>::max() / blocksY ||
            blocksX * blocksY > std::numeric_limits<std::size_t>::max() / blockSize ||
            requiredSize >
                std::numeric_limits<std::size_t>::max() - blocksX * blocksY * blockSize) {
            return false;
        }
        requiredSize += blocksX * blocksY * blockSize;
        levelWidth = std::max(1U, levelWidth / 2);
        levelHeight = std::max(1U, levelHeight / 2);
    }
    input.seekg(0, std::ios::end);
    const std::streamoff fileSize = input.tellg();
    return fileSize >= 0 && static_cast<std::uintmax_t>(fileSize) >= requiredSize;
}

bool TextureLoader::isDxt5Dds(const std::filesystem::path& path) {
    TraceScope trace("texture", "isDxt5Dds");
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return false;
    }
    std::array<char, 4> fourCC = {};
    input.seekg(84);
    input.read(fourCC.data(), static_cast<std::streamsize>(fourCC.size()));
    return input && std::string(fourCC.data(), fourCC.size()) == "DXT5";
}

bool TextureLoader::isDdsTextureArray(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return false;
    }
    std::array<std::uint8_t, 128> header = {};
    input.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
    if (!input || std::string(header.data() + 84, header.data() + 88) != "DX10") {
        return false;
    }
    std::array<std::uint8_t, 20> dx10 = {};
    input.read(reinterpret_cast<char*>(dx10.data()), static_cast<std::streamsize>(dx10.size()));
    if (!input) {
        return false;
    }
    const std::uint32_t arraySize =
        static_cast<std::uint32_t>(dx10[12]) | (static_cast<std::uint32_t>(dx10[13]) << 8) |
        (static_cast<std::uint32_t>(dx10[14]) << 16) | (static_cast<std::uint32_t>(dx10[15]) << 24);
    return arraySize > 1;
}

bool TextureLoader::readDxt5CompressedDds(const std::filesystem::path& path, CompressedDds& image) {
    TraceScope trace("texture", "readDxt5CompressedDds");
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return false;
    }
    input.seekg(0, std::ios::end);
    const std::streamoff length = input.tellg();
    input.seekg(0, std::ios::beg);
    if (length < 128) {
        return false;
    }
    std::vector<std::uint8_t> data(static_cast<std::size_t>(length));
    input.read(reinterpret_cast<char*>(data.data()), length);
    if (!input || std::string(data.begin(), data.begin() + 4) != "DDS " ||
        std::string(data.begin() + 84, data.begin() + 88) != "DXT5") {
        return false;
    }
    const std::uint32_t width = readU32(data, 16);
    const std::uint32_t height = readU32(data, 12);
    const std::uint32_t flags = readU32(data, 8);
    const std::uint32_t declaredLevels = readU32(data, 28);
    const std::size_t levelCount = (flags & 0x20000U) != 0 ? declaredLevels : 1;
    if (width == 0 || height == 0 || levelCount == 0 || levelCount > 32 ||
        width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        return false;
    }
    image.width = static_cast<int>(width);
    image.height = static_cast<int>(height);
    image.levels.clear();
    image.levels.reserve(levelCount);
    std::size_t sourceOffset = 128;
    std::uint32_t levelWidth = width;
    std::uint32_t levelHeight = height;
    for (std::size_t level = 0; level < levelCount; ++level) {
        const std::size_t blocksX = (static_cast<std::size_t>(levelWidth) + 3) / 4;
        const std::size_t blocksY = (static_cast<std::size_t>(levelHeight) + 3) / 4;
        if (blocksX > std::numeric_limits<std::size_t>::max() / blocksY ||
            blocksX * blocksY > std::numeric_limits<std::size_t>::max() / 16) {
            return false;
        }
        const std::size_t levelSize = blocksX * blocksY * 16;
        if (sourceOffset > data.size() || levelSize > data.size() - sourceOffset) {
            return false;
        }
        image.levels.emplace_back(data.begin() + sourceOffset,
                                  data.begin() + sourceOffset + levelSize);
        sourceOffset += levelSize;
        levelWidth = std::max(1U, levelWidth / 2);
        levelHeight = std::max(1U, levelHeight / 2);
    }
    return true;
}

} // namespace openbus::rendering
