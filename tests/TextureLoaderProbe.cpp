#include "assets/TextureLoader.h"

#include "core/Logger.h"
#include "ProbeTempDirectory.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>


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

bool writeBytes(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes) {
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
    return output.good();
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
    openbus::probe::removeProbeFile(path);

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
    openbus::probe::removeProbeFile(alphaMaskPath);

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
    openbus::probe::removeProbeFile(dxt1Path);

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
    openbus::probe::removeProbeFile(dxt4Path);

    const std::filesystem::path dxt5Path =
        std::filesystem::temp_directory_path() / "openbus_dxt5_alpha.dds";
    std::array<std::uint8_t, 128> dxt5Header = {};
    dxt5Header[0] = 'D';
    dxt5Header[1] = 'D';
    dxt5Header[2] = 'S';
    dxt5Header[3] = ' ';
    writeU32(dxt5Header, 12, 4);
    writeU32(dxt5Header, 16, 4);
    writeU32(dxt5Header, 80, 0x4);
    dxt5Header[84] = 'D';
    dxt5Header[85] = 'X';
    dxt5Header[86] = 'T';
    dxt5Header[87] = '5';
    std::vector<std::uint8_t> dxt5Bytes(dxt5Header.begin(), dxt5Header.end());
    constexpr std::array<std::uint8_t, 16> dxt5Block = {
        200, 100, 0x3a, 0, 0, 0, 0, 0, 0x00, 0xf8, 0xe0, 0x07, 0, 0, 0, 0};
    dxt5Bytes.insert(dxt5Bytes.end(), dxt5Block.begin(), dxt5Block.end());
    valid &= require(writeBytes(dxt5Path, dxt5Bytes), "write DXT5 DDS fixture");
    valid &= require(openbus::rendering::TextureLoader::readImage(dxt5Path, image),
                     "DXT5 interpolated-alpha DDS loads");
    valid &= require(image.width == 4 && image.height == 4 && image.rgba.size() == 64 &&
                         image.rgba[0] == 255 && image.rgba[1] == 0 &&
                         image.rgba[3] == 185 && image.rgba[7] == 114,
                     "DXT5 alpha selectors decode interpolated values");
    openbus::rendering::CompressedDds compressedDds;
    valid &= require(openbus::rendering::TextureLoader::isSafeCompressedDds(dxt5Path) &&
                         openbus::rendering::TextureLoader::isDxt5Dds(dxt5Path) &&
                         openbus::rendering::TextureLoader::readDxt5CompressedDds(
                             dxt5Path, compressedDds),
                     "DXT5 DDS compressed texture path accepts a valid block");
    valid &= require(compressedDds.width == 4 && compressedDds.height == 4 &&
                         compressedDds.levels.size() == 1 &&
                         compressedDds.levels[0] ==
                             std::vector<std::uint8_t>(dxt5Block.begin(), dxt5Block.end()),
                     "DXT5 compressed block data is retained");
    openbus::probe::removeProbeFile(dxt5Path);

    const std::filesystem::path mislabeledBmpPath =
        std::filesystem::temp_directory_path() / "openbus_palettized_bmp_payload.dds";
    constexpr std::uint32_t bmpPixelOffset = 14 + 124 + 256 * 4;
    std::vector<std::uint8_t> bmpHeader(bmpPixelOffset, 0);
    const auto writeBmpU32 = [&bmpHeader](std::size_t offset, std::uint32_t value) {
        bmpHeader[offset + 0] = static_cast<std::uint8_t>(value & 0xffU);
        bmpHeader[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xffU);
        bmpHeader[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xffU);
        bmpHeader[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xffU);
    };
    bmpHeader[0] = 'B';
    bmpHeader[1] = 'M';
    writeBmpU32(2, bmpPixelOffset + 4);
    writeBmpU32(10, bmpPixelOffset);
    writeBmpU32(14, 124); // BITMAPV5HEADER used by the installed London textures.
    writeBmpU32(18, 1);
    writeBmpU32(22, 1);
    bmpHeader[26] = 1;
    bmpHeader[28] = 8;
    writeBmpU32(34, 4);
    writeBmpU32(46, 256);
    bmpHeader[138 + 4] = 10; // Palette entry 1: blue.
    bmpHeader[138 + 5] = 20;
    bmpHeader[138 + 6] = 30;
    {
        std::ofstream output(mislabeledBmpPath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(bmpHeader.data()),
                     static_cast<std::streamsize>(bmpHeader.size()));
        constexpr std::array<std::uint8_t, 4> pixel = {1, 0, 0, 0};
        output.write(reinterpret_cast<const char*>(pixel.data()),
                     static_cast<std::streamsize>(pixel.size()));
    }
    valid &= require(openbus::rendering::TextureLoader::readImage(mislabeledBmpPath, image),
                     "8-bit BMP payload with a DDS filename loads");
    valid &= require(image.width == 1 && image.height == 1 && image.rgba.size() == 4 &&
                         image.rgba[0] == 30 && image.rgba[1] == 20 && image.rgba[2] == 10 &&
                         image.rgba[3] == 255,
                     "palettized BMP palette is expanded to opaque RGBA");
    openbus::probe::removeProbeFile(mislabeledBmpPath);

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
    openbus::probe::removeProbeFile(tgaPath);

    const std::filesystem::path pngPath =
        std::filesystem::temp_directory_path() / "openbus_rgba.png";
    const std::vector<std::uint8_t> pngBytes = {
         0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
         0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
         0x08, 0x06, 0x00, 0x00, 0x00, 0x1f, 0x15, 0xc4, 0x89, 0x00, 0x00, 0x00,
         0x0d, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9c, 0x63, 0xe0, 0x51, 0xb2, 0xf0,
         0x03, 0x00, 0x01, 0x59, 0x00, 0xb5, 0x46, 0xcb, 0x2a, 0x07, 0x00, 0x00,
         0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82};
    valid &= require(writeBytes(pngPath, pngBytes), "write PNG fixture");
    valid &= require(openbus::rendering::TextureLoader::readImage(pngPath, image),
                     "PNG image loads");
    valid &= require(image.width == 1 && image.height == 1 && image.rgba.size() == 4 &&
                         image.rgba[0] == 12 && image.rgba[1] == 34 && image.rgba[2] == 56 &&
                         image.rgba[3] == 78,
                     "PNG RGBA channels decode unchanged");
    openbus::probe::removeProbeFile(pngPath);

    const std::filesystem::path jpegPath =
        std::filesystem::temp_directory_path() / "openbus_grayscale.jpg";
    const std::vector<std::uint8_t> jpegBytes = {
        0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x00,
        0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xff, 0xdb, 0x00, 0x43, 0x00, 0x03, 0x02, 0x02,
        0x03, 0x02, 0x02, 0x03, 0x03, 0x03, 0x03, 0x04, 0x03, 0x03, 0x04, 0x05, 0x08, 0x05,
        0x05, 0x04, 0x04, 0x05, 0x0a, 0x07, 0x07, 0x06, 0x08, 0x0c, 0x0a, 0x0c, 0x0c, 0x0b,
        0x0a, 0x0b, 0x0b, 0x0d, 0x0e, 0x12, 0x10, 0x0d, 0x0e, 0x11, 0x0e, 0x0b, 0x0b, 0x10,
        0x16, 0x10, 0x11, 0x13, 0x14, 0x15, 0x15, 0x15, 0x0c, 0x0f, 0x17, 0x18, 0x16, 0x14,
        0x18, 0x12, 0x14, 0x15, 0x14, 0xff, 0xc0, 0x00, 0x0b, 0x08, 0x00, 0x04, 0x00, 0x04,
        0x01, 0x01, 0x11, 0x00, 0xff, 0xc4, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc4,
        0x00, 0x14, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xda, 0x00, 0x08, 0x01, 0x01, 0x00, 0x00,
        0x3f, 0x00, 0x2a, 0x7f, 0xff, 0xd9};
    valid &= require(writeBytes(jpegPath, jpegBytes), "write JPEG fixture");
    valid &= require(openbus::rendering::TextureLoader::readImage(jpegPath, image),
                     "JPEG image loads");
    valid &= require(image.width == 4 && image.height == 4 && image.rgba.size() == 64 &&
                         image.rgba[0] > 80 && image.rgba[0] < 112 &&
                         image.rgba[0] == image.rgba[1] && image.rgba[1] == image.rgba[2] &&
                         image.rgba[3] == 255,
                     "JPEG decodes to opaque grayscale RGBA pixels");
    openbus::probe::removeProbeFile(jpegPath);

    const std::filesystem::path indexedTgaPath =
        std::filesystem::temp_directory_path() / "openbus_indexed_tga.tga";
    std::vector<std::uint8_t> indexedTga(18, 0);
    indexedTga[1] = 1;
    indexedTga[2] = 1;
    indexedTga[5] = 2;
    indexedTga[7] = 24;
    indexedTga[12] = 2;
    indexedTga[14] = 1;
    indexedTga[16] = 8;
    indexedTga[17] = 0x20;
    indexedTga.insert(indexedTga.end(), {0, 0, 255, 0, 255, 0, 0, 1});
    valid &= require(writeBytes(indexedTgaPath, indexedTga), "write indexed TGA fixture");
    valid &= require(openbus::rendering::TextureLoader::readImage(indexedTgaPath, image),
                     "indexed TGA loads");
    valid &= require(image.width == 2 && image.height == 1 && image.rgba.size() == 8 &&
                         image.rgba[0] == 255 && image.rgba[1] == 0 && image.rgba[4] == 0 &&
                         image.rgba[5] == 255,
                     "indexed TGA expands color-map entries");
    openbus::probe::removeProbeFile(indexedTgaPath);

    const std::filesystem::path rleIndexedTgaPath =
        std::filesystem::temp_directory_path() / "openbus_rle_indexed_tga.tga";
    std::vector<std::uint8_t> rleIndexedTga(18, 0);
    rleIndexedTga[1] = 1;
    rleIndexedTga[2] = 9;
    rleIndexedTga[5] = 2;
    rleIndexedTga[7] = 24;
    rleIndexedTga[12] = 2;
    rleIndexedTga[14] = 1;
    rleIndexedTga[16] = 8;
    rleIndexedTga[17] = 0x20;
    rleIndexedTga.insert(rleIndexedTga.end(), {0, 0, 255, 0, 255, 0, 0x81, 1});
    valid &= require(writeBytes(rleIndexedTgaPath, rleIndexedTga), "write RLE indexed TGA fixture");
    valid &= require(openbus::rendering::TextureLoader::readImage(rleIndexedTgaPath, image),
                     "RLE indexed TGA loads");
    valid &= require(image.width == 2 && image.height == 1 && image.rgba.size() == 8 &&
                         image.rgba[1] == 255 && image.rgba[5] == 255 && image.rgba[3] == 255 &&
                         image.rgba[7] == 255,
                     "RLE indexed TGA repeats palette indices");
    openbus::probe::removeProbeFile(rleIndexedTgaPath);
    return valid ? 0 : 1;
}
