#include "TextureAssetLoader.h"

#include "Logger.h"

#include <algorithm>
#include <cctype>
#include <exception>

extern Logger gameLog;

namespace openbus::rendering {

bool loadTextureAsset(const std::filesystem::path& path, TextureAsset& asset) {
    if (path.empty()) {
        return false;
    }
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    if (extension == ".dds" && TextureLoader::isSafeCompressedDds(path)) {
        if (TextureLoader::isDxt5Dds(path) && !TextureLoader::isDdsTextureArray(path)) {
            auto compressedDds = std::make_shared<CompressedDds>();
            if (TextureLoader::readDxt5CompressedDds(path, *compressedDds)) {
                asset.compressedDds = std::move(compressedDds);
                return true;
            }
        } else {
            try {
                gli::texture loadedTexture = gli::load(path.string());
                if (!loadedTexture.empty() && gli::is_compressed(loadedTexture.format())) {
                    asset.compressedTexture =
                        std::make_shared<gli::texture>(std::move(loadedTexture));
                    return true;
                }
            } catch (const std::exception& error) {
                gameLog.Log("GLI failed to load compressed texture " + path.generic_string() +
                            ": " + error.what());
            }
        }
    }
    return TextureLoader::readImage(path, asset.image);
}

} // namespace openbus::rendering
