#pragma once

#include "config/ModelConfigTypes.h"
#include "assets/ObjLoader.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace openbus::map {

inline std::string sceneryTextureBasename(const std::string& name) {
    std::string normalized = name;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    std::string basename = std::filesystem::path(normalized).filename().string();
    std::transform(basename.begin(), basename.end(), basename.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    return basename;
}

// Bind SCO [matl] records to source O3D/X material slots. Some OMSI scenery configs
// number repeated texture overrides in their own list rather than by the source O3D slot.
// Prefer matching duplicate texture names in authored order; only use numeric slots when
// no source slot carries that texture name. This avoids applying (for example) a fence
// texture/alpha mode to unrelated material slots 0 and 1 when fence surfaces are slots 4/5.
inline std::unordered_map<int, const ModelMaterialState*>
resolveSceneryMaterialStates(const openbus::rendering::ParsedObj& parsed, const ModelPart& part) {
    std::unordered_map<int, const ModelMaterialState*> statesByIndex;
    std::unordered_map<std::string, std::vector<const ModelMaterialState*>> statesByTexture;
    for (const ModelMaterialState& state : part.materialStatesInOrder) {
        statesByIndex.try_emplace(state.materialIndex, &state);
        const std::string key = sceneryTextureBasename(state.textureName);
        if (!key.empty()) {
            statesByTexture[key].push_back(&state);
        }
    }

    std::vector<const openbus::rendering::ObjMaterial*> sourceMaterials;
    sourceMaterials.reserve(parsed.materials.size());
    std::unordered_set<std::string> sourceTextureNames;
    for (const auto& [key, material] : parsed.materials) {
        static_cast<void>(key);
        sourceMaterials.push_back(&material);
        const std::string texture = sceneryTextureBasename(material.textureName);
        if (!texture.empty()) {
            sourceTextureNames.insert(texture);
        }
    }
    std::sort(sourceMaterials.begin(), sourceMaterials.end(),
              [](const openbus::rendering::ObjMaterial* left,
                 const openbus::rendering::ObjMaterial* right) {
                  return left->materialIndex < right->materialIndex;
              });

    std::unordered_map<int, const ModelMaterialState*> result;
    std::unordered_map<std::string, std::size_t> sourceTextureOrdinals;
    for (const openbus::rendering::ObjMaterial* source : sourceMaterials) {
        const std::string sourceTexture = sceneryTextureBasename(source->textureName);
        std::size_t ordinal = 0;
        if (!sourceTexture.empty()) {
            ordinal = sourceTextureOrdinals[sourceTexture]++;
        }
        const auto matchingStates = statesByTexture.find(sourceTexture);
        if (!sourceTexture.empty() && matchingStates != statesByTexture.end()) {
            const auto& candidates = matchingStates->second;
            if (ordinal < candidates.size()) {
                result[source->materialIndex] = candidates[ordinal];
            } else if (candidates.size() == 1) {
                result[source->materialIndex] = candidates.front();
            }
            continue;
        }

        const auto direct = statesByIndex.find(source->materialIndex);
        if (direct == statesByIndex.end()) {
            continue;
        }
        const std::string directTexture = sceneryTextureBasename(direct->second->textureName);
        if (!directTexture.empty() && directTexture != sourceTexture &&
            sourceTextureNames.contains(directTexture)) {
            continue; // This is another source slot's override, not this slot's material.
        }
        result[source->materialIndex] = direct->second;
    }
    return result;
}

} // namespace openbus::map
