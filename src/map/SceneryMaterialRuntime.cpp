#include "map/SceneryMaterialRuntime.h"

#include "core/Variables.h"

#include <algorithm>

namespace openbus::map {

SceneryMaterialRuntimeState
resolveSceneryMaterialRuntime(const SceneryMaterialRuntimeConfig& config,
                              const Variables* variables, std::optional<double> weatherBrightness) {
    SceneryMaterialRuntimeState result;
    result.lightmapStrength = config.lightmapStrength;
    result.nightmapStrength =
        weatherBrightness ? static_cast<float>(1.0 - *weatherBrightness) : config.nightmapStrength;
    result.alphaScale = config.alphaScaleVariable.empty() ? 1.0 : 0.0;
    if (variables == nullptr) {
        return result;
    }

    const std::string* lightmapVariable = &config.lightmapStrengthVariable;
    if (config.auxiliaryMaterialSelection != nullptr) {
        const ModelMaterialState& selected = selectModelMaterial(
            *config.auxiliaryMaterialSelection,
            [variables](const std::string& name) { return variables->get(name); });
        if (&selected != config.auxiliaryMaterialSelection) {
            result.selectedItem = &selected;
            lightmapVariable = &selected.lightmapStrengthVariable;
        }
    }
    if (result.selectedItem != nullptr || !config.visibilityVariable.empty()) {
        result.nightmapStrength = 1.0F;
    } else if (variables->hasExplicitSceneryNightlight()) {
        result.nightmapStrength = std::clamp(variables->get("nightlighta"), 0.0F, 1.0F);
    }
    if (!config.texcoordTransXVariable.empty()) {
        result.texcoordOffsetX = variables->get(config.texcoordTransXVariable);
    }
    if (!config.texcoordTransYVariable.empty()) {
        result.texcoordOffsetY = variables->get(config.texcoordTransYVariable);
    }
    result.lightmapStrength = lightmapVariable->empty()
                                  ? 1.0F
                                  : std::clamp(variables->get(*lightmapVariable), 0.0F, 1.0F);
    if (!config.alphaScaleVariable.empty()) {
        result.alphaScale = std::clamp(variables->get(config.alphaScaleVariable), 0.0F, 1.0F);
    }
    return result;
}

} // namespace openbus::map
