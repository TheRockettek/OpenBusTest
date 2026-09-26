#pragma once

#include "TextureLoader.h"

#include <gli/gli.hpp>
#include <memory>

namespace openbus::rendering {

struct TextureAsset {
    Image image;
    std::shared_ptr<gli::texture> compressedTexture;
    std::shared_ptr<CompressedDds> compressedDds;
};

bool loadTextureAsset(const std::filesystem::path& path, TextureAsset& asset);

} // namespace openbus::rendering
