#pragma once

#include "config/ModelConfigTypes.h"
#include "config/TextureAddressMode.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>

namespace openbus::map {

// CPU-side scenery attributes. GPU buffers and textures are owned by the renderer.
struct MapSceneryMaterial {
    bool textured = false;
    bool flipTextureY = false;
    bool weatherLitDynamicTexture = false;
    bool whiteAmbient = false;
    std::array<float, 3> specularColor = {};
    std::array<float, 3> emissionColor = {};
    float specularPower = 8.0F;
    float lightmapStrength = 0.0F;
    float bumpmapStrength = 0.0F;
    float environmentStrength = 0.0F;
    TextureAddressMode addressU = TextureAddressMode::Repeat;
    TextureAddressMode addressV = TextureAddressMode::Repeat;
};

struct MapSceneryBatch {
    std::size_t vertexCount = 0;
    double boundingRadius = 0.0;
    std::size_t sceneryLodIndex = 0;
    MapSceneryMaterial material;
    int scriptTextureIndex = -1;
    int textTextureIndex = -1;
    std::string freeTextureName;
    std::string freeTextureVariable;
    std::string texcoordTransXVariable;
    std::string texcoordTransYVariable;
    std::string lightmapStrengthVariable;
    std::string alphaScaleVariable;
    std::optional<ModelMaterialState> auxiliaryMaterialSelection;
    std::filesystem::path textureObjectRoot;
    std::filesystem::path textureModelRoot;
    std::array<double, 3> color = {1.0, 1.0, 1.0};
    double alpha = 1.0;
    int alphaMode = 0;
    int renderType = 2;
    bool noZwrite = false;
    bool noZcheck = false;
    std::string visibilityVariable;
    int visibilityValue = 0;
    bool cullBackFaces = false;
};

} // namespace openbus::map
