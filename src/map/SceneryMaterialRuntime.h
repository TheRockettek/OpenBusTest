#pragma once

#include "config/ModelConfigTypes.h"

#include <optional>
#include <string>

class Variables;

namespace openbus::map {

struct SceneryMaterialRuntimeConfig {
    const std::string& lightmapStrengthVariable;
    const std::string& visibilityVariable;
    const std::string& texcoordTransXVariable;
    const std::string& texcoordTransYVariable;
    const std::string& alphaScaleVariable;
    const ModelMaterialState* auxiliaryMaterialSelection;
    float lightmapStrength;
    float nightmapStrength;
};

struct SceneryMaterialRuntimeState {
    // Non-owning pointer to an item owned by the supplied selection; null means base maps.
    const ModelMaterialState* selectedItem = nullptr;
    float lightmapStrength = 0.0F;
    float nightmapStrength = 0.0F;
    std::optional<float> texcoordOffsetX;
    std::optional<float> texcoordOffsetY;
    double alphaScale = 1.0;
};

// Resolves script and weather inputs for the rendered instance, without GPU resources.
[[nodiscard]] SceneryMaterialRuntimeState
resolveSceneryMaterialRuntime(const SceneryMaterialRuntimeConfig& config,
                              const Variables* variables, std::optional<double> weatherBrightness);

} // namespace openbus::map
