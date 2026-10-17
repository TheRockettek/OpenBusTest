#include "Logger.h"
#include "ModelConfigLoader.h"
#include "SceneryObjectConfigLoader.h"
#include "ScriptRuntime.h"
#include "Variables.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

Logger gameLog = Logger("Game");

namespace {

void writeWhiteFontBitmap(const std::filesystem::path& path) {
    constexpr int width = 4;
    constexpr int height = 2;
    constexpr int rowStride = 12;
    constexpr std::uint32_t pixelBytes = rowStride * height;
    std::array<std::uint8_t, 54> header = {};
    header[0] = 'B';
    header[1] = 'M';
    const auto write32 = [&header](std::size_t offset, std::uint32_t value) {
        for (std::size_t byte = 0; byte < 4; ++byte) {
            header[offset + byte] = static_cast<std::uint8_t>(value >> (byte * 8));
        }
    };
    header[2] = static_cast<std::uint8_t>(54 + pixelBytes);
    write32(10, 54);
    write32(14, 40);
    write32(18, width);
    write32(22, height);
    header[26] = 1;
    header[28] = 24;
    write32(34, pixelBytes);
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(header.data()), header.size());
    const std::array<std::uint8_t, rowStride> row = {255, 255, 255, 255, 255, 255,
                                                     255, 255, 255, 0,   0,   0};
    output.write(reinterpret_cast<const char*>(row.data()), row.size());
    output.write(reinterpret_cast<const char*>(row.data()), row.size());
}

} // namespace

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus_map_placement_label_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    std::ofstream mesh(root / "display.o3d");
    mesh << "fixture";
    mesh.close();
    const std::filesystem::path sceneryPath = root / "label_sign.sco";
    std::ofstream scenery(sceneryPath);
    scenery << "[mesh]\ndisplay.o3d\n[texttexture]\n0\nProbeFont\n92\n64\n0\n"
               "35\n31\n32\n[matl]\nText_Field.bmp\n0\n"
               "[usetexttexture]\n0\n";
    scenery.close();
    const SceneryObjectConfig sceneryConfiguration = loadSceneryObjectFile(sceneryPath);
    openbus::scripting::SceneryObject modelVariables;
    const ModelConfig model = loadModelConfig(sceneryPath, root, ModelConfigKind::SceneryObject,
                                              modelVariables);
    if (sceneryConfiguration.diagnostics.hasErrors() || model.parts.size() != 1 ||
        model.textTextures.size() != 1 || !model.textTextures[0].placementLabel ||
        model.parts[0].materialStatesInOrder.empty() ||
        model.parts[0].materialStatesInOrder[0].textTextureIndex != 0) {
        std::cerr << "inline SCO label text texture did not parse and bind to its material\n";
        return 1;
    }

    VehicleConfig configuration;
    configuration.sourcePath = sceneryPath;
    SimulationState sharedState;
    openbus::scripting::SceneryObject firstPlacement;
    openbus::scripting::SceneryObject secondPlacement;
    ScriptRuntime firstRuntime(configuration, firstPlacement, sharedState);
    ScriptRuntime secondRuntime(configuration, secondPlacement, sharedState);
    firstRuntime.configureTextTextures(model.textTextures, {"90"});
    secondRuntime.configureTextTextures(model.textTextures, {"50"});
    firstRuntime.initialize();
    secondRuntime.initialize();

    ScriptRuntime::ScriptTextureSnapshot firstTexture;
    ScriptRuntime::ScriptTextureSnapshot secondTexture;
    const bool copied = firstRuntime.copyTextTexture(0, firstTexture) &&
                        secondRuntime.copyTextTexture(0, secondTexture);
    if (!copied || firstTexture.width != 92 || firstTexture.height != 64 ||
        firstTexture.pixels == secondTexture.pixels) {
        std::cerr << "different map placement labels did not generate distinct text textures\n";
        return 1;
    }

    const std::filesystem::path fonts = root / "Fonts";
    std::filesystem::create_directories(fonts);
    writeWhiteFontBitmap(fonts / "fixture.bmp");
    std::ofstream font(fonts / "delayed.oft");
    font << "Documentation preamble\n[newfont]\n{font name}\n{bitmap}\n{alpha}\n"
            "{height}\n{spacing}\n[char]\n{character}\n{left}\n{right}\n{top}\n"
            "[newfont]\nDelayedFont\nfixture.bmp\nfixture.bmp\n2\n0\n"
            "[char]\n5\n0\n1\n0\n[char]\n0\n1\n3\n0\n";
    font.close();
    openbus::scripting::SceneryObject fontPlacement;
    fontPlacement.declareString("sign_text");
    fontPlacement.setString("sign_text", "50");
    ScriptRuntime fontRuntime(configuration, fontPlacement, sharedState);
    fontRuntime.configureTextTextures(
        {ModelTextTexture{1, false,
                          {"sign_text", "DelayedFont", "8", "4", "0", "255", "255", "255"}}});
    fontRuntime.initialize();
    ScriptRuntime::ScriptTextureSnapshot fontTexture;
    if (!fontRuntime.copyTextTexture(1, fontTexture)) {
        std::cerr << "could not retrieve text rendered with the fixture bitmap font\n";
        return 1;
    }
    std::size_t opaquePixels = 0;
    for (std::size_t alpha = 3; alpha < fontTexture.pixels.size(); alpha += 4) {
        opaquePixels += fontTexture.pixels[alpha] != 0 ? 1U : 0U;
    }
    std::filesystem::remove_all(root);
    if (opaquePixels != 6) {
        std::cerr << "font lookup stopped at the documentation [newfont] block; expected six "
                     "pixels from the custom 1x2 and 2x2 digit glyphs, got "
                  << opaquePixels << "\n";
        return 1;
    }
    return 0;
}
