#pragma once

#include "assets/ObjLoader.h"
#include "config/ModelConfigTypes.h"
#include "config/SceneryObjectConfigLoader.h"
#include "core/Variables.h"
#include "map/MapSceneryBatch.h"
#include "render/ModelVertex.h"
#include "render/map/SharedMapTextureImages.h"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace openbus::rendering {

struct PreparedSceneryMesh {
    std::shared_ptr<ParsedObj> parsed;
    std::unordered_map<int, std::vector<ModelVertex>> verticesByMaterial;
    std::size_t invalidPositionIndices = 0;
    std::exception_ptr preparationError;
};

struct PreparedSceneryAuxTexture {
    std::string name;
    std::filesystem::path path;
    std::string preparationError;
};

struct PreparedSceneryBatch {
    std::string meshKey;
    int materialIndex = 0;
    openbus::map::MapSceneryBatch batch;
    std::string textureName;
    std::string transmapTextureName;
    std::string freeTextureName;
    std::filesystem::path texturePath;
    std::shared_ptr<const SharedMapTextureImages::SeasonalPaths> seasonTexturePaths;
    std::filesystem::path transmapTexturePath;
    PreparedSceneryAuxTexture lightmap;
    PreparedSceneryAuxTexture nightmap;
    std::vector<PreparedSceneryAuxTexture> itemLightmaps;
    std::vector<PreparedSceneryAuxTexture> itemNightmaps;
    PreparedSceneryAuxTexture bumpmap;
    PreparedSceneryAuxTexture environment;
    std::filesystem::path freeTexturePath;
    bool hasMaterialState = false;
    int stateAlphaMode = 0;
};

struct PreparedSceneryTreeBatch {
    std::vector<ModelVertex> vertices;
    std::string textureName;
    std::filesystem::path texturePath;
    std::shared_ptr<const SharedMapTextureImages::SeasonalPaths> seasonTexturePaths;
    openbus::map::MapSceneryBatch batch;
};

struct PreparedSceneryAsset {
    SceneryObjectConfig objectConfiguration;
    ModelConfig modelConfiguration;
    openbus::scripting::SceneryObject variables;
    std::unordered_map<std::string, PreparedSceneryMesh> meshes;
    std::vector<PreparedSceneryBatch> batches;
    std::vector<PreparedSceneryTreeBatch> treeBatches;
    std::unordered_map<std::string, std::filesystem::path> resolvedTexturePaths;
    std::unordered_map<std::string, std::string> auxiliaryTexturePreparationErrors;
    std::unordered_map<std::string, std::shared_ptr<Image>> decodedTextures;
    std::size_t textureLookups = 0;
    std::size_t textureResolutions = 0;
    std::filesystem::path modelConfigPath;
    std::filesystem::path modelRoot;
    std::filesystem::path externalModelConfigPath;
    bool usesExternalModelConfig = false;
    bool externalModelConfigMissing = false;
    bool hasModelConfiguration = false;
    std::exception_ptr preparationError;
};

// CPU-only preparation; images and geometry are never uploaded to a graphics API.
[[nodiscard]] PreparedSceneryAsset prepareSceneryAsset(const std::filesystem::path& configPath,
                                                       const std::filesystem::path& omsiRoot,
                                                       SharedMapTextureImages& textureImages);

} // namespace openbus::rendering
