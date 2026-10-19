#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace openbus::rendering {

constexpr std::size_t MAX_TEXTURE_BUFFER_BYTES = 256U * 1024U * 1024U;

bool checkedTextureBufferSize(std::size_t width, std::size_t height, std::size_t bytesPerPixel,
                              std::size_t& size);

struct Image {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> rgba;
};

struct CompressedDds {
    int width = 0;
    int height = 0;
    std::vector<std::vector<std::uint8_t>> levels;
};

class TextureLoader {
  public:
    static bool readImage(const std::filesystem::path& path, Image& image);
    static bool isSafeCompressedDds(const std::filesystem::path& path);
    static bool isDxt5Dds(const std::filesystem::path& path);
    static bool isDdsTextureArray(const std::filesystem::path& path);
    static bool readDxt5CompressedDds(const std::filesystem::path& path, CompressedDds& image);
};

} // namespace openbus::rendering
