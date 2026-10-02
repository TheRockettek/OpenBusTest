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
              "[mesh]\ndisplay.obj\n"
              "[matl]\ndisplay.bmp\n0\n"
              "[usescripttexture]\n0\n"
              "[texttexture]\nroute_display\nfont\n128\n32\n"
              "[usetexttexture]\n0\n";
    config.close();

    Variables variables(ScriptObjectKind::Vehicle);
    const ModelConfig result =
        loadModelConfig(configPath, root, ModelConfigKind::Bus, variables);
    std::filesystem::remove_all(root);

    if (result.diagnostics.hasErrors() || result.parts.size() != 1 ||
        result.scriptTextures.size() != 1 || result.scriptTextures[0].slot != 0 ||
        result.scriptTextures[0].width != 64 || result.scriptTextures[0].height != 16 ||
        result.textTextures.size() != 1 || result.textTextures[0].slot != 0 ||
        result.parts[0].materialStatesInOrder.size() != 1 ||
        result.parts[0].materialStatesInOrder[0].scriptTextureIndex != 0 ||
        result.parts[0].materialStatesInOrder[0].textTextureIndex != 0) {
        std::cerr << "model texture records were not retained and bound\n";
        return 1;
    }
    return 0;
}
