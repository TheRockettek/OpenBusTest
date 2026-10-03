#include "ModelConfigLoader.h"
#include "Variables.h"

#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus_model_config_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    const std::filesystem::path configPath = root / "display.cfg";
    std::ofstream mesh(root / "display.obj");
    mesh << "# parser fixture\n";
    mesh.close();
    std::ofstream config(configPath);
    config << "[scripttexture]\n64\n16\n0\n"
              "[CTC]\nColorscheme\nTexture\\Advert\n0\n"
              "[CTCTexture]\nFarbschema_Tex1\n3803_l.tga\n"
              "[mesh]\ndisplay.obj\n"
              "[mouseevent]\nRouteDisplay\n"
              "[matl]\ndisplay.bmp\n0\n"
              "[matl_change]\ndisplay.bmp\n0\nCockpit_Lights\n"
              "[usescripttexture]\n0\n"
              "[illumination_interior]\n2\n-1\n4\n5\n"
              "[interiorlight]\nCockpit_Lights\n1\n2\n3\n0.1\n0.2\n0.3\n4\n"
              "[texttexture]\nroute_display\nfont\n128\n32\n"
              "[usetexttexture]\n0\n";
    config.close();

    Variables variables(ScriptObjectKind::Vehicle);
    const ModelConfig result =
        loadModelConfig(configPath, root, ModelConfigKind::Bus, variables);
    std::ofstream variants(configPath);
    variants << "[mesh]\ndisplay.obj\n"
                "[matl]\ndisplay.bmp\n0\n"
                "[matl_alpha]\n2\n[usetexttexture]\n7\n"
                "[matl_change]\ndisplay.bmp\n0\nDisplayMode\n"
                "[matl_item]\n[matl_nightmap]\non.bmp\n"
                "[matl_item]\n[matl_lightmap]\nwarning.bmp\nBrightness\n"
                "[matl_nozwrite]\n"
                "[matl]\nother.bmp\n0\n[matl_alpha]\n1\n"
                "[matl_change]\ndisplay.bmp\n0\nAlternateMode\n"
                "[matl_item]\n[matl_nightmap]\nalternate.bmp\n"
                "[matl_change]\nstandalone.bmp\n2\nStandaloneMode\n"
                "[matl_item]\n[matl_nightmap]\nstandalone_lit.bmp\n";
    variants.close();
    const ModelConfig conditional =
        loadModelConfig(configPath, root, ModelConfigKind::Bus, variables);
    if (conditional.diagnostics.hasErrors() ||
        conditional.parts[0].materialStatesInOrder.size() != 3) {
        std::cerr << "conditional material fixture failed to parse\n";
        return 1;
    }
    const auto& base = conditional.parts[0].materialStatesInOrder[0];
    const auto selected = [&](double mode) -> const ModelMaterialState& {
        variables.set("displaymode", mode);
        return selectModelMaterial(base, [&](const std::string& name) { return variables.get(name); });
    };
    if (!base.nightmapTextureName.empty() || !base.lightmapTextureName.empty() ||
        &selected(0) != &base || selected(1).nightmapTextureName != "on.bmp" ||
        selected(1).textTextureIndex != 7 || selected(1).alphaMode != 2 ||
        selected(0.9).nightmapTextureName != "on.bmp" ||
        !selected(2).nightmapTextureName.empty() ||
        selected(2).lightmapTextureName != "warning.bmp" ||
        selected(2).lightmapStrengthVariable != "brightness" || !selected(2).noZwrite ||
        &selected(-1) != &base || &selected(3) != &base || &selected(0) != &base ||
        conditional.parts[0].materialStatesInOrder[1].alphaMode != 1) {
        std::cerr << "material items leaked into the base or were not selected generically\n";
        return 1;
    }
    variables.set("alternatemode", 1.0);
    if (selected(0).nightmapTextureName != "alternate.bmp" ||
        conditional.parts[0].materialStatesInOrder[1].textureChanges.size() != 0 ||
        conditional.parts[0].materialStatesInOrder[2].textureChanges.size() != 1) {
        std::cerr << "material changes were attached to the preceding rather than named material\n";
        return 1;
    }
    std::filesystem::remove_all(root);

    if (result.diagnostics.hasErrors() || result.parts.size() != 1 ||
        result.scriptTextures.size() != 1 || result.scriptTextures[0].slot != 0 ||
        result.scriptTextures[0].width != 64 || result.scriptTextures[0].height != 16 ||
        result.textTextures.size() != 1 || result.textTextures[0].slot != 0 ||
        result.ctcTemplates.size() != 1 || result.ctcTemplates[0].name != "Colorscheme" ||
        result.ctcTemplates[0].texturePath != "Texture\\Advert" ||
        result.ctcTextures.size() != 1 || result.ctcTextures[0].slot != "farbschema_tex1" ||
        result.ctcTextures[0].textureName != "3803_l.tga" ||
        result.parts[0].mouseEvent != "routedisplay" ||
        result.parts[0].interiorLightIndexes != std::array<int, 4>{2, -1, 4, 5} ||
        result.interiorLights.size() != 1 ||
        result.interiorLights[0].controller != "cockpit_lights" ||
        result.interiorLights[0].parameters[6] != 4.0 ||
        result.parts[0].materialStatesInOrder.size() != 1 ||
        result.parts[0].materialStatesInOrder[0].scriptTextureIndex != 0 ||
        result.parts[0].materialStatesInOrder[0].textTextureIndex != 0 ||
        result.parts[0].materialStatesInOrder[0].textureChanges.size() != 1 ||
        result.parts[0].materialStatesInOrder[0].textureChanges[0].activationVariable !=
            "cockpit_lights") {
        std::cerr << "model texture records were not retained and bound\n";
        return 1;
    }
    return 0;
}
