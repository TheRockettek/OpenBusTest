#include "app/vehicle/VehicleMaterialSelection.h"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

int main() {
    const std::vector<ModelCtcTexture> modelTextures = {
        {"Body", "body.dds"}, {"Interior", "window.bmp"}};
    const std::unordered_map<std::string, std::string> appearanceReplacements = {
        {"BODY", "C:/Vehicles/Bus/Texture/Paints/red_body.dds"}};
    const auto ctcReplacements = openbus::app::vehicle::buildCtcTextureReplacements(
        modelTextures, appearanceReplacements);
    if (openbus::app::vehicle::resolveCtcTextureName("BODY.DDS", ctcReplacements) !=
            "C:/Vehicles/Bus/Texture/Paints/red_body.dds" ||
        openbus::app::vehicle::resolveCtcTextureName("window.bmp", ctcReplacements) !=
            "window.bmp") {
        std::cerr << "CTI slots were not mapped to model CTC texture names\n";
        return 1;
    }
    const std::vector<ModelCtcTexture> pathTextures = {
        {"Body", "Texture\\body.dds"}};
    const auto pathReplacements = openbus::app::vehicle::buildCtcTextureReplacements(
        pathTextures, appearanceReplacements);
    if (openbus::app::vehicle::resolveCtcTextureName("texture/body.DDS", pathReplacements) !=
        "C:/Vehicles/Bus/Texture/Paints/red_body.dds") {
        std::cerr << "CTC texture paths were not matched across separator styles\n";
        return 1;
    }
    Variables variables(ScriptObjectKind::Vehicle);
    variables.declare("visibility_red");
    variables.set("visibility_red", 0.0F);
    variables.set("unrelated_variable", 7.0F);
    openbus::app::vehicle::applyCtiVariableReplacements(
        variables, {{"visibility_red", 1.0F}, {"paint_variant", 0.5F}});
    if (variables.get("visibility_red") != 1.0F || variables.get("paint_variant") != 0.5F ||
        variables.get("unrelated_variable") != 7.0F) {
        std::cerr << "CTI setvar values did not override the selected vehicle variables\n";
        return 1;
    }
    bool invalidCtiValueRejected = false;
    const std::unordered_map<std::string, float> invalidReplacements = {
        {"visibility_red", 0.0F},
        {"invalid_variable", std::numeric_limits<float>::infinity()}};
    try {
        openbus::app::vehicle::applyCtiVariableReplacements(variables, invalidReplacements);
    } catch (const std::invalid_argument&) {
        invalidCtiValueRejected = true;
    }
    if (!invalidCtiValueRejected || variables.has("invalid_variable") ||
        variables.get("visibility_red") != 1.0F) {
        std::cerr << "Invalid CTI setvar values were not rejected safely\n";
        return 1;
    }

    const std::unordered_map<std::string, std::string> replacements = {
        {"body.dds", "custom_body.dds"}, {"window.bmp", "custom_window.bmp"}};
    if (openbus::app::vehicle::resolveCtcTextureName("BODY.DDS", replacements) !=
            "custom_body.dds" ||
        openbus::app::vehicle::resolveCtcTextureName("unmapped.dds", replacements) !=
            "unmapped.dds") {
        std::cerr << "CTC texture replacement was not resolved case-insensitively\n";
        return 1;
    }

    struct LayerCase {
        bool textureArray;
        int requested;
        std::size_t available;
        int expected;
    };
    constexpr LayerCase cases[] = {
        {false, 2, 4, 0}, {true, -1, 4, 0}, {true, 0, 4, 0}, {true, 1, 4, 1},
        {true, 3, 4, 3},  {true, 8, 4, 3},  {true, 8, 0, 0}};
    for (const LayerCase& test : cases) {
        const int actual = openbus::app::vehicle::resolveTextureArrayLayer(
            test.textureArray, test.requested, test.available);
        if (actual != test.expected) {
            std::cerr << "texture-array layer selection did not clamp as expected\n";
            return 1;
        }
    }
    return 0;
}
