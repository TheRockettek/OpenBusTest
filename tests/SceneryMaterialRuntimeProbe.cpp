#include "core/Variables.h"
#include "map/SceneryMaterialRuntime.h"

#include <iostream>
#include <memory>

int main() {
    std::string light = "lamp";
    std::string visibility;
    std::string x = "offset_x";
    std::string y;
    std::string alpha = "opacity";
    ModelMaterialState selection;
    auto item = std::make_shared<ModelMaterialState>();
    item->lightmapStrengthVariable = "item_lamp";
    ModelMaterialState::TextureChange change;
    change.activationVariable = "variant";
    change.items.push_back(item);
    selection.textureChanges.push_back(change);
    const openbus::map::SceneryMaterialRuntimeConfig config{light, visibility, x,     y,
                                                            alpha, &selection, 0.25F, 0.4F};
    const auto fail = [](const char* message) {
        std::cerr << message << '\n';
        return 1;
    };

    auto result = openbus::map::resolveSceneryMaterialRuntime(config, nullptr, std::nullopt);
    if (result.selectedItem || result.lightmapStrength != 0.25F ||
        result.nightmapStrength != 0.4F || result.alphaScale != 0.0 || result.texcoordOffsetX ||
        result.texcoordOffsetY) {
        return fail("No-script material defaults were not retained");
    }
    result = openbus::map::resolveSceneryMaterialRuntime(config, nullptr, 0.75);
    if (result.nightmapStrength != 0.25F || result.lightmapStrength != 0.25F) {
        return fail("Weather did not override night strength without script state");
    }
    alpha.clear();
    result = openbus::map::resolveSceneryMaterialRuntime(config, nullptr, std::nullopt);
    if (result.alphaScale != 1.0) {
        return fail("Uncontrolled alpha was not opaque");
    }

    alpha = "opacity";
    openbus::scripting::SceneryObject variables;
    variables.set("lamp", -1.0F);
    variables.set("offset_x", -0.5F);
    variables.set("opacity", 2.0F);
    result = openbus::map::resolveSceneryMaterialRuntime(config, &variables, 0.75);
    if (result.selectedItem || result.nightmapStrength != 0.25F ||
        result.lightmapStrength != 0.0F || result.alphaScale != 1.0 ||
        result.texcoordOffsetX != -0.5F || result.texcoordOffsetY) {
        return fail("Script variables, clamping, or implicit nightlight were incorrect");
    }
    variables.set("nightlighta", 1.5F);
    y = "offset_y";
    variables.set("offset_y", 0.75F);
    variables.set("opacity", -1.0F);
    result = openbus::map::resolveSceneryMaterialRuntime(config, &variables, 0.75);
    if (result.nightmapStrength != 1.0F || result.alphaScale != 0.0 ||
        result.texcoordOffsetY != 0.75F) {
        return fail("Explicit nightlight or second offset was not resolved correctly");
    }
    variables.set("nightlighta", -1.0F);
    result = openbus::map::resolveSceneryMaterialRuntime(config, &variables, std::nullopt);
    if (result.nightmapStrength != 0.0F) {
        return fail("Explicit negative nightlight was not clamped");
    }
    visibility = "visible";
    result = openbus::map::resolveSceneryMaterialRuntime(config, &variables, 1.0);
    if (result.nightmapStrength != 1.0F) {
        return fail("Visibility variable did not override explicit nightlight");
    }
    visibility.clear();
    variables.set("variant", 1.0F);
    variables.set("item_lamp", 2.0F);
    result = openbus::map::resolveSceneryMaterialRuntime(config, &variables, 1.0);
    if (result.selectedItem != item.get() || result.nightmapStrength != 1.0F ||
        result.lightmapStrength != 1.0F) {
        return fail("Selected item did not override lightmap and nightmap controls");
    }
    item->lightmapStrengthVariable.clear();
    result = openbus::map::resolveSceneryMaterialRuntime(config, &variables, std::nullopt);
    if (result.lightmapStrength != 1.0F) {
        return fail("Uncontrolled item lightmap was not fully enabled");
    }
    variables.set("variant", 0.0F);
    light.clear();
    result = openbus::map::resolveSceneryMaterialRuntime(config, &variables, std::nullopt);
    if (result.selectedItem || result.lightmapStrength != 1.0F) {
        return fail("Base material was not restored");
    }
    return 0;
}
