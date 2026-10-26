#include "assets/TextureAssetLoader.h"

#include "core/Logger.h"
#include "core/Utf8Path.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <exception>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace openbus::rendering {

bool loadTextureAsset(const std::filesystem::path& path, TextureAsset& asset) {
    if (path.empty()) {
        return false;
    }
    std::string extension = path.extension().string();
    std::transform(
        extension.begin(), extension.end(), extension.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    if (extension == ".dds" && TextureLoader::isSafeCompressedDds(path)) {
        if (TextureLoader::isDxt5Dds(path) && !TextureLoader::isDdsTextureArray(path)) {
            auto compressedDds = std::make_shared<CompressedDds>();
            if (TextureLoader::readDxt5CompressedDds(path, *compressedDds)) {
                asset.compressedDds = std::move(compressedDds);
                return true;
            }
        } else {
            try {
                std::ifstream input(path, std::ios::binary | std::ios::ate);
                if (!input) {
                    throw std::runtime_error("could not open file");
                }
                const std::streamoff fileSize = input.tellg();
                if (fileSize <= 0 ||
                    static_cast<std::uintmax_t>(fileSize) > MAX_TEXTURE_BUFFER_BYTES) {
                    throw std::runtime_error("file size is invalid or exceeds the texture limit");
                }
                std::vector<char> fileContents(static_cast<std::size_t>(fileSize));
                input.seekg(0, std::ios::beg);
                if (!input.read(fileContents.data(), static_cast<std::streamsize>(fileSize))) {
                    throw std::runtime_error("could not read complete file");
                }
                gli::texture loadedTexture = gli::load(fileContents.data(), fileContents.size());
                if (!loadedTexture.empty() && gli::is_compressed(loadedTexture.format())) {
                    asset.compressedTexture =
                        std::make_shared<gli::texture>(std::move(loadedTexture));
                    return true;
                }
            } catch (const std::exception& error) {
                gameLog.Log("GLI failed to load compressed texture " + openbus::pathToUtf8(path) +
                            ": " + error.what());
            }
        }
    }
    return TextureLoader::readImage(path, asset.image);
}

} // namespace openbus::rendering
