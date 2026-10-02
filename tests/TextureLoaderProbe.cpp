#include "TextureLoader.h"

#include "Logger.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

Logger gameLog("TextureLoaderProbe");

namespace {

void writeU32(std::array<std::uint8_t, 128>& header, std::size_t offset,
              std::uint32_t value) {
    header[offset + 0] = static_cast<std::uint8_t>(value & 0xffU);
    header[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xffU);
    header[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xffU);
    header[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xffU);
}

bool require(bool condition, const char* description) {
    if (!condition) {
        std::cerr << "failed: " << description << '\n';
        return false;
    }
    return true;
}

} // namespace

int main() {
    bool valid = true;
    std::size_t bufferSize = 0;
    valid &= require(openbus::rendering::checkedTextureBufferSize(64, 32, 4, bufferSize) &&
                         bufferSize == 8192,
                     "valid texture buffer size");
    valid &= require(!openbus::rendering::checkedTextureBufferSize(
                         std::numeric_limits<std::size_t>::max(), 2, 4, bufferSize),
                     "texture buffer overflow");

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "openbus_invalid_dimensions.dds";
    std::array<std::uint8_t, 128> header = {};
    header[0] = 'D';
    header[1] = 'D';
    header[2] = 'S';
    header[3] = ' ';
    writeU32(header, 12, 1);
    writeU32(header, 16, std::numeric_limits<std::uint32_t>::max());
    header[84] = 'D';
    header[85] = 'X';
    header[86] = 'T';
    header[87] = '5';
    {
        std::ofstream output(path, std::ios::binary);
        output.write(reinterpret_cast<const char*>(header.data()),
                     static_cast<std::streamsize>(header.size()));
    }
    openbus::rendering::Image image;
    valid &= require(!openbus::rendering::TextureLoader::readImage(path, image),
                     "DDS dimensions outside Image range");
    std::filesystem::remove(path);
    return valid ? 0 : 1;
}
