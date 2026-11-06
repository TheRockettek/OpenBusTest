#include "core/Logger.h"
#include "config/ModelConfigLoader.h"
#include "config/SceneryObjectConfigLoader.h"
#include "scripting/ScriptRuntime.h"
#include "core/Variables.h"
#include "map/MapConfigLoader.h"
#include "ProbeTempDirectory.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>


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

bool checkFontPixels(const VehicleConfig& configuration, SimulationState& sharedState,
                     const std::string& fontName, std::size_t expected) {
    openbus::scripting::SceneryObject placement;
    placement.declareString("sign_text");
    placement.setString("sign_text", "50");
    ScriptRuntime runtime(configuration, placement, sharedState);
    runtime.configureTextTextures(
        {ModelTextTexture{1, false,
                          {"sign_text", fontName, "8", "4", "0", "255", "255", "255"}}});
    runtime.initialize();
    ScriptRuntime::ScriptTextureSnapshot texture;
    if (!runtime.copyTextTexture(1, texture)) {
        std::cerr << "could not retrieve text rendered with font " << fontName << "\n";
        return false;
    }
    std::size_t opaquePixels = 0;
    for (std::size_t alpha = 3; alpha < texture.pixels.size(); alpha += 4) {
        opaquePixels += texture.pixels[alpha] != 0 ? 1U : 0U;
    }
    if (opaquePixels != expected) {
        std::cerr << "font " << fontName << ": expected " << expected
                  << " opaque pixels, got " << opaquePixels << "\n";
        return false;
    }
    return true;
}

} // namespace

int main() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_map_placement_label_probe");
    std::filesystem::create_directories(root);
    std::filesystem::create_directories(root / "script");
    std::ofstream stringVariables(root / "script" / "sign_variables.txt");
    stringVariables << "sign_texture\n";
    stringVariables.close();
    std::ofstream mesh(root / "display.o3d");
    mesh << "fixture";
    mesh.close();
    const std::filesystem::path sceneryPath = root / "label_sign.sco";
    std::ofstream scenery(sceneryPath);
    scenery << "[stringvarnamelist]\n1\nscript\\sign_variables.txt\n"
               "[mesh]\ndisplay.o3d\n[texttexture]\n0\nProbeFont\n92\n64\n0\n"
               "35\n31\n32\n[matl]\nText_Field.bmp\n0\n"
               "[usetexttexture]\n0\n";
    scenery.close();
    const SceneryObjectConfig sceneryConfiguration = loadSceneryObjectFile(sceneryPath);
    openbus::scripting::SceneryObject modelVariables;
    const ModelConfig model = loadModelConfig(sceneryPath, root, ModelConfigKind::SceneryObject,
                                              modelVariables);
    if (sceneryConfiguration.diagnostics.hasErrors() || model.parts.size() != 1 ||
        sceneryConfiguration.scriptConfiguration.stringVariables !=
            std::vector<std::string>{"sign_texture"} ||
        model.textTextures.size() != 1 || !model.textTextures[0].placementLabel ||
        model.parts[0].materialStatesInOrder.empty() ||
        model.parts[0].materialStatesInOrder[0].textTextureIndex != 0) {
        std::cerr << "inline SCO label text texture did not parse and bind to its material\n";
        return 1;
    }

    VehicleConfig configuration = sceneryConfiguration.scriptConfiguration;
    std::ofstream tileFile(root / "tile.map", std::ios::binary);
    tileFile << "[version]\n14\n[terrain]\n"
                "[splineAttachement]\n0\nlabel_sign.sco\n1\n0\n0\n0\n0\n"
                "0\n0\n0\n0\n0\n0\n3\n90\n\n[object]\n"
                "[splineAttachement_repeater]\n0\n0\n0\nlabel_sign.sco\n2\n0\n0\n0\n0\n"
                "0\n0\n0\n0\n0\n0\n3\n50\n\n[spline]\n";
    tileFile.close();
    openbus::map::MapTileReference tileReference;
    tileReference.textPath = root / "tile.map";
    const auto tile = openbus::map::loadMapTile(tileReference);
    if (tile.splineAttachments.size() != 2 ||
        tile.splineAttachments[0].labels != std::vector<std::string>{"90", "", "[object]"} ||
        tile.splineAttachments[1].labels != std::vector<std::string>{"50", "", "[spline]"}) {
        std::cerr << "spline attachment labels were not preserved in order\n";
        return 1;
    }
    SimulationState sharedState;
    openbus::scripting::SceneryObject firstPlacement;
    openbus::scripting::SceneryObject secondPlacement;
    ScriptRuntime firstRuntime(configuration, firstPlacement, sharedState);
    ScriptRuntime secondRuntime(configuration, secondPlacement, sharedState);
    firstRuntime.configureTextTextures(model.textTextures, tile.splineAttachments[0].labels);
    secondRuntime.configureTextTextures(model.textTextures, tile.splineAttachments[1].labels);
    firstRuntime.initialize();
    secondRuntime.initialize();

    ScriptRuntime::ScriptTextureSnapshot firstTexture;
    ScriptRuntime::ScriptTextureSnapshot secondTexture;
    const bool copied = firstRuntime.copyTextTexture(0, firstTexture) &&
                        secondRuntime.copyTextTexture(0, secondTexture);
    if (!copied || firstTexture.width != 92 || firstTexture.height != 64 ||
        firstTexture.pixels == secondTexture.pixels ||
        firstPlacement.getString("sign_texture") != "90" ||
        secondPlacement.getString("sign_texture") != "50") {
        std::cerr << "different map placement labels did not generate distinct text textures\n";
        return 1;
    }
    openbus::scripting::SceneryObject logoPlacement;
    ScriptRuntime logoRuntime(configuration, logoPlacement, sharedState);
    logoRuntime.configureTextTextures({}, {"Campus_logos\\Cotterell.bmp"});
    if (logoPlacement.getString("sign_texture") != "Campus_logos\\Cotterell.bmp") {
        std::cerr << "attached map label was not assigned to the SCO string-variable slot\n";
        return 1;
    }
    ScriptRuntime::ScriptTextureSnapshot unchangedTexture;
    if (!firstRuntime.copyTextTextureIfChanged(0, firstTexture.revision, unchangedTexture) ||
        unchangedTexture.revision != firstTexture.revision ||
        unchangedTexture.width != firstTexture.width || !unchangedTexture.pixels.empty()) {
        std::cerr << "unchanged placement text texture copied pixel bytes unnecessarily\n";
        return 1;
    }
    ScriptRuntime::ScriptTextureSnapshot changedTexture;
    if (!firstRuntime.copyTextTextureIfChanged(0, firstTexture.revision + 1U, changedTexture) ||
        changedTexture.revision != firstTexture.revision ||
        changedTexture.pixels != firstTexture.pixels) {
        std::cerr << "placement text texture snapshot omitted pixels for a stale revision\n";
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
    if (!checkFontPixels(configuration, sharedState, "DelayedFont", 6)) {
        return 1;
    }
    std::ofstream addedFont(fonts / "added.oft");
    addedFont << "[newfont]\nAddedFont\nfixture.bmp\nfixture.bmp\n2\n0\n"
                 "[char]\n5\n0\n1\n0\n[char]\n0\n1\n3\n0\n";
    addedFont.close();
    std::filesystem::last_write_time(fonts, std::filesystem::file_time_type::clock::now() +
                                                std::chrono::seconds(10));
    if (!checkFontPixels(configuration, sharedState, "AddedFont", 6)) {
        return 1;
    }
    std::filesystem::remove(fonts / "added.oft");
    std::filesystem::last_write_time(fonts, std::filesystem::file_time_type::clock::now() +
                                                std::chrono::seconds(20));
    if (!checkFontPixels(configuration, sharedState, "AddedFont", 15)) {
        return 1;
    }

    const auto upperRoot = root / "CASE";
    const auto lowerRoot = root / "case";
    std::filesystem::create_directories(upperRoot / "Fonts");
    std::filesystem::create_directories(lowerRoot / "Fonts");
    if (!std::filesystem::equivalent(upperRoot, lowerRoot)) {
        std::filesystem::copy_file(fonts / "fixture.bmp", lowerRoot / "Fonts" / "fixture.bmp");
        std::filesystem::copy_file(fonts / "delayed.oft", lowerRoot / "Fonts" / "delayed.oft");
        VehicleConfig upperConfiguration = configuration;
        VehicleConfig lowerConfiguration = configuration;
        upperConfiguration.sourcePath = upperRoot / "label.sco";
        lowerConfiguration.sourcePath = lowerRoot / "label.sco";
        if (!checkFontPixels(upperConfiguration, sharedState, "DelayedFont", 6)) {
            return 1;
        }
        std::filesystem::remove(fonts / "delayed.oft");
        std::filesystem::last_write_time(fonts, std::filesystem::file_time_type::clock::now() +
                                                    std::chrono::seconds(30));
        if (!checkFontPixels(lowerConfiguration, sharedState, "DelayedFont", 6)) {
            return 1;
        }
    }
    openbus::probe::removeProbeDirectory(root);
    return 0;
}
