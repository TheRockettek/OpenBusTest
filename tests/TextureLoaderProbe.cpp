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

    const std::filesystem::path alphaMaskPath =
        std::filesystem::temp_directory_path() / "openbus_alpha_mask.dds";
    std::array<std::uint8_t, 128> alphaMaskHeader = {};
    alphaMaskHeader[0] = 'D';
    alphaMaskHeader[1] = 'D';
    alphaMaskHeader[2] = 'S';
    alphaMaskHeader[3] = ' ';
    writeU32(alphaMaskHeader, 12, 1);
    writeU32(alphaMaskHeader, 16, 2);
    writeU32(alphaMaskHeader, 80, 0x2);
    writeU32(alphaMaskHeader, 88, 8);
    writeU32(alphaMaskHeader, 104, 0xff);
    {
        std::ofstream output(alphaMaskPath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(alphaMaskHeader.data()),
                     static_cast<std::streamsize>(alphaMaskHeader.size()));
        constexpr std::array<std::uint8_t, 2> mask = {255, 0};
        output.write(reinterpret_cast<const char*>(mask.data()),
                     static_cast<std::streamsize>(mask.size()));
    }
    valid &= require(openbus::rendering::TextureLoader::readImage(alphaMaskPath, image),
                     "DDS alpha-only ground-texture mask loads");
    valid &= require(image.width == 2 && image.height == 1 && image.rgba.size() == 8 &&
                         image.rgba[0] == 255 && image.rgba[1] == 255 &&
                         image.rgba[2] == 255 && image.rgba[3] == 255 && image.rgba[7] == 0,
                     "DDS alpha-only mask preserves coverage values");
    std::filesystem::remove(alphaMaskPath);

    const std::filesystem::path dxt1Path =
        std::filesystem::temp_directory_path() / "openbus_dxt1_implicit_alpha.dds";
    std::array<std::uint8_t, 128> dxt1Header = {};
    dxt1Header[0] = 'D';
    dxt1Header[1] = 'D';
    dxt1Header[2] = 'S';
    dxt1Header[3] = ' ';
    writeU32(dxt1Header, 12, 4);
    writeU32(dxt1Header, 16, 4);
    writeU32(dxt1Header, 80, 0x4); // FourCC flag only; some OMSI DDS omit alpha-pixels.
    dxt1Header[84] = 'D';
    dxt1Header[85] = 'X';
    dxt1Header[86] = 'T';
    dxt1Header[87] = '1';
    {
        std::ofstream output(dxt1Path, std::ios::binary);
        output.write(reinterpret_cast<const char*>(dxt1Header.data()),
                     static_cast<std::streamsize>(dxt1Header.size()));
        constexpr std::array<std::uint8_t, 8> block = {
            0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
        output.write(reinterpret_cast<const char*>(block.data()),
                     static_cast<std::streamsize>(block.size()));
    }
    valid &= require(openbus::rendering::TextureLoader::readImage(dxt1Path, image),
                     "DXT1 DDS with omitted alpha flag loads");
    valid &= require(image.rgba.size() == 4 * 4 * 4 && image.rgba[3] == 0 &&
                         image.rgba[image.rgba.size() - 1] == 0,
                     "DXT1 three-colour selector preserves implicit transparency");
    std::filesystem::remove(dxt1Path);

    const std::filesystem::path dxt4Path =
        std::filesystem::temp_directory_path() / "openbus_dxt4_alpha.dds";
    std::array<std::uint8_t, 128> dxt4Header = {};
    dxt4Header[0] = 'D';
    dxt4Header[1] = 'D';
    dxt4Header[2] = 'S';
    dxt4Header[3] = ' ';
    writeU32(dxt4Header, 12, 4);
    writeU32(dxt4Header, 16, 4);
    writeU32(dxt4Header, 80, 0x4);
    dxt4Header[84] = 'D';
    dxt4Header[85] = 'X';
    dxt4Header[86] = 'T';
    dxt4Header[87] = '4';
    {
        std::ofstream output(dxt4Path, std::ios::binary);
        output.write(reinterpret_cast<const char*>(dxt4Header.data()),
                     static_cast<std::streamsize>(dxt4Header.size()));
        constexpr std::array<std::uint8_t, 16> block = {
            255, 0, 0, 0, 0, 0, 0, 0, 0x00, 0xf8, 0xe0, 0x07, 0, 0, 0, 0};
        output.write(reinterpret_cast<const char*>(block.data()),
                     static_cast<std::streamsize>(block.size()));
    }
    valid &= require(openbus::rendering::TextureLoader::readImage(dxt4Path, image),
                     "DXT4 interpolated-alpha DDS loads");
    valid &= require(image.rgba.size() == 4 * 4 * 4 && image.rgba[3] == 255,
                     "DXT4 interpolated alpha is decoded");
    std::filesystem::remove(dxt4Path);

    const std::filesystem::path tgaPath =
        std::filesystem::temp_directory_path() / "openbus_tga_alpha_without_descriptor.tga";
    std::array<std::uint8_t, 18> tgaHeader = {};
    tgaHeader[2] = 2;
    tgaHeader[12] = 1;
    tgaHeader[14] = 1;
    tgaHeader[16] = 32;
    {
        std::ofstream output(tgaPath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(tgaHeader.data()),
                     static_cast<std::streamsize>(tgaHeader.size()));
        constexpr std::array<std::uint8_t, 4> pixel = {0, 255, 0, 0};
        output.write(reinterpret_cast<const char*>(pixel.data()),
                     static_cast<std::streamsize>(pixel.size()));
    }
    valid &= require(openbus::rendering::TextureLoader::readImage(tgaPath, image),
                     "32-bit TGA with zero descriptor attribute bits loads");
    valid &= require(image.rgba.size() == 4 && image.rgba[1] == 255 && image.rgba[3] == 0,
                     "32-bit TGA alpha channel is preserved when descriptor bits are unset");
    std::filesystem::remove(tgaPath);
    return valid ? 0 : 1;
}
