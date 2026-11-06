#include "scripting/ScriptRuntime.h"
#include "core/Logger.h"
#include "osc/OscConverter.h"
#include "ProbeTempDirectory.h"
#include "config/ScriptTextureLimits.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#ifdef _WIN32
#include <cstdlib>
#else
#include <stdlib.h>
#endif
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

bool setScriptBackend(const char* backend) {
#ifdef _WIN32
    return _putenv_s("OPENBUS_SCRIPT_BACKEND", backend == nullptr ? "" : backend) == 0;
#else
    return backend == nullptr ? unsetenv("OPENBUS_SCRIPT_BACKEND") == 0
                              : setenv("OPENBUS_SCRIPT_BACKEND", backend, 1) == 0;
#endif
}

std::string readTextFile(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void writeTestBmp(const std::filesystem::path& path, int width, int height,
                  const std::vector<std::uint8_t>& redValues) {
    const int rowStride = (width * 3 + 3) & ~3;
    const std::uint32_t pixelBytes = static_cast<std::uint32_t>(rowStride * height);
    const std::uint32_t fileSize = 54 + pixelBytes;
    std::array<std::uint8_t, 54> header = {};
    header[0] = 'B';
    header[1] = 'M';
    auto write32 = [&header](std::size_t offset, std::uint32_t value) {
        header[offset] = static_cast<std::uint8_t>(value);
        header[offset + 1] = static_cast<std::uint8_t>(value >> 8);
        header[offset + 2] = static_cast<std::uint8_t>(value >> 16);
        header[offset + 3] = static_cast<std::uint8_t>(value >> 24);
    };
    write32(2, fileSize);
    write32(10, 54);
    write32(14, 40);
    write32(18, static_cast<std::uint32_t>(width));
    write32(22, static_cast<std::uint32_t>(height));
    header[26] = 1;
    header[28] = 24;
    write32(34, pixelBytes);
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(header.data()), header.size());
    std::vector<std::uint8_t> row(static_cast<std::size_t>(rowStride), 0);
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            const std::uint8_t value = redValues[static_cast<std::size_t>(y * width + x)];
            row[static_cast<std::size_t>(x) * 3] = value;
            row[static_cast<std::size_t>(x) * 3 + 1] = value;
            row[static_cast<std::size_t>(x) * 3 + 2] = value;
        }
        output.write(reinterpret_cast<const char*>(row.data()), row.size());
    }
}

bool checkAuthoredScriptDrawing(const std::filesystem::path& root) {
    using openbus::scripting::scriptTextureReferenceIndex;
    if (scriptTextureReferenceIndex("\\S:1") != 1 ||
        scriptTextureReferenceIndex("\\s:255") != 255 ||
        scriptTextureReferenceIndex("\\S:256") != -1 ||
        scriptTextureReferenceIndex("\\S:-1") != -1 ||
        scriptTextureReferenceIndex("\\S:1.bmp") != -1 ||
        scriptTextureReferenceIndex("mask.bmp") != -1) {
        std::cerr << "script texture reference parsing failed\n";
        return false;
    }
    const auto path = root / "authored_drawing.osc";
    std::ofstream source(path);
    source << "{init}\n"
              "\"ProbeFont\" (M.V.GetFontIndex) (S.L.font_index)\n"
              "\"UnitFont\" (M.V.GetFontIndex) (S.L.other_font_index)\n"
              "\"ProbeFont\" (M.V.GetFontIndex) (S.L.repeated_font_index)\n"
              "(L.L.font_index) \"AB\" (M.V.TextLength) (S.L.text_width)\n"
              "(L.L.font_index) \"A@B\" (M.V.TextLength) (S.L.multiline_width)\n"
              "(L.L.font_index) (L.$.multiline_input) (M.V.TextLength) (S.L.newline_width)\n"
              "(L.L.font_index) \"a\" (M.V.TextLength) (S.L.lowercase_width)\n"
              "(L.L.other_font_index) \"\xc2\xb0"
              "c\" (M.V.TextLength) (S.L.degree_width)\n"
              "(L.L.other_font_index) \"" "\xc3\xb1"
              "\" (M.V.TextLength) (S.L.latin1_width)\n"
              "(L.L.other_font_index) \"" "\xf0\x9f\x98\x80"
              "\" (M.V.TextLength) (S.L.unicode_fallback_width)\n"
              "1 (M.V.STNewTex)\n"
              "1 (M.V.STLock)\n"
              "1 128 10 20 30 (M.V.STSetColor)\n"
              "1 0 0 7 7 (M.V.STDrawRect)\n"
              "1 255 40 50 60 (M.V.STSetColor)\n"
              "1 6 6 10 10 (M.V.STDrawRect)\n"
              "1 255 0 0 0 (M.V.STSetColor)\n"
              "1 0 0 (L.L.font_index) 0 0 \"A\" (M.V.STTextOut)\n"
              "1 3 0 (L.L.font_index) 1 0 \"B\" (M.V.STTextOut)\n"
              "1 6 6 (M.V.STDrawPixel)\n"
              "1 (M.V.STUnlock)\n"
              "{end}\n"
              "{trigger:reset_surface}\n"
              "1 (M.V.STNewTex)\n"
              "{end}\n"
              "{trigger:draw_font}\n"
              "1 (M.V.STNewTex)\n"
              "1 255 0 0 0 (M.V.STSetColor)\n"
              "1 0 0 (L.L.font_index) 0 0 \"A\" (M.V.STTextOut)\n"
              "(L.L.font_index) \"AB\" (M.V.TextLength) (S.L.text_width)\n"
              "(L.L.font_index) \"A@B\" (M.V.TextLength) (S.L.multiline_width)\n"
              "{end}\n";
    source.close();
    std::string error;
    if (!convertOscToLua(path, generatedLuaPath(path), error)) {
        std::cerr << error << '\n';
        return false;
    }
    ScriptRuntime::prepareFontSearch(root / "drawing.bus",
                                     {"ProbeFont", "UnitFont", "GridFont", "SpacingFont"});
    const auto probeFontPath = root / "Fonts" / "probe.oft";
    const auto fontDirectoryTime = std::filesystem::last_write_time(root / "Fonts");
    int fontTimestampOffset = 60;
    const auto rewriteProbeFont = [&](int aLeft, int aRight, int horizontalGap) {
        std::ofstream font(probeFontPath, std::ios::trunc);
        font << "[newfont]\nProbeFont\nprobe.bmp\nprobe_alpha.bmp\n2\n" << horizontalGap
             << "\n[char]\nA\n" << aLeft << '\n' << aRight << "\n0\n"
                "[char]\nB\n2\n4\n0\n[char]\n0\n0\n2\n0\n[char]\n1\n2\n4\n0\n"
                "[char]\n?\n2\n4\n0\n";
        font.close();
        if (font.fail()) {
            return false;
        }
        std::error_code stampError;
        std::filesystem::last_write_time(
            probeFontPath, std::filesystem::file_time_type::clock::now() +
                               std::chrono::seconds(fontTimestampOffset),
            stampError);
        if (stampError) {
            return false;
        }
        fontTimestampOffset += 60;
        std::filesystem::last_write_time(root / "Fonts", fontDirectoryTime, stampError);
        return !stampError;
    };
    for (const char* backend : {"native", "lua"}) {
        if (!setScriptBackend(backend)) {
            return false;
        }
        VehicleConfig config;
        config.sourcePath = root / "drawing.bus";
        config.scripts.push_back(path.filename().string());
        Variables variables(ScriptObjectKind::Vehicle);
        SimulationState simulation;
        ScriptRuntime runtime(config, variables, simulation);
        runtime.configureScriptTextures({ModelScriptTexture{1, 8, 8, {}}});
        variables.setString("multiline_input", "A\r\nB");
        runtime.initialize();
        ScriptRuntime::ScriptTextureSnapshot snapshot;
        if (!runtime.valid() || !runtime.errors().empty() ||
            !runtime.copyScriptTexture(1, snapshot) || snapshot.width != 8 ||
            variables.get("font_index") == variables.get("other_font_index") ||
            variables.get("font_index") != variables.get("repeated_font_index") ||
            variables.get("text_width") != 4.0F || variables.get("lowercase_width") != 2.0F ||
            variables.get("degree_width") != 3.0F || variables.get("latin1_width") != 1.0F ||
            variables.get("unicode_fallback_width") != 1.0F ||
            variables.get("multiline_width") != 2.0F || variables.get("newline_width") != 2.0F) {
            std::cerr << backend << ": authored font selection/drawing failed\n";
            return false;
        }
        const auto pixel = [&](int x, int y) {
            const auto offset = static_cast<std::size_t>(y * snapshot.width + x) * 4;
            return std::array<std::uint8_t, 4>{
                snapshot.pixels[offset], snapshot.pixels[offset + 1],
                snapshot.pixels[offset + 2], snapshot.pixels[offset + 3]};
        };
        if (pixel(7, 7) != std::array<std::uint8_t, 4>{40, 50, 60, 255} ||
            pixel(0, 0) != std::array<std::uint8_t, 4>{0, 0, 0, 255} ||
            pixel(3, 0) != std::array<std::uint8_t, 4>{80, 80, 80, 128} ||
            pixel(6, 6) != std::array<std::uint8_t, 4>{0, 0, 0, 255}) {
            std::cerr << backend << ": clipped drawing, RGBA order or full-colour flag failed\n";
            return false;
        }
        if (!rewriteProbeFont(2, 4, 1)) {
            std::cerr << backend << ": could not update the OFT font in place\n";
            return false;
        }
        runtime.invokeEntryPoint("trigger_draw_font");
        ScriptRuntime::ScriptTextureSnapshot reloadedFontTexture;
        if (!runtime.copyScriptTexture(1, reloadedFontTexture) ||
            reloadedFontTexture.pixels[3] != 128 || variables.get("text_width") != 5.0F ||
            variables.get("multiline_width") != 2.0F) {
            std::cerr << backend << ": existing script font index did not reload edited OFT data"
                      << " (alpha="
                      << (reloadedFontTexture.pixels.size() > 3
                              ? static_cast<int>(reloadedFontTexture.pixels[3])
                              : -1)
                      << ", text_width=" << variables.get("text_width") << ")\n";
            return false;
        }
        if (!rewriteProbeFont(0, 2, 0)) {
            std::cerr << backend << ": could not restore the OFT font after refresh check\n";
            return false;
        }
        runtime.invokeEntryPoint("trigger_draw_font");
        if (!runtime.copyScriptTexture(1, reloadedFontTexture) ||
            reloadedFontTexture.pixels[3] != 255 || variables.get("text_width") != 4.0F ||
            variables.get("multiline_width") != 2.0F) {
            std::cerr << backend << ": existing script font index did not reload restored OFT data\n";
            return false;
        }
        for (int reset = 0; reset < 6; ++reset) {
            const auto previousRevision = snapshot.revision;
            runtime.invokeMouseEvent("reset_surface");
            if (!runtime.copyScriptTextureIfChanged(1, previousRevision, snapshot) ||
                snapshot.revision <= previousRevision ||
                std::any_of(snapshot.pixels.begin(), snapshot.pixels.end(),
                            [](std::uint8_t value) { return value != 0; })) {
                std::cerr << backend << ": reset script surface retained a stale GPU revision\n";
                return false;
            }
        }
    }
    return setScriptBackend(nullptr);
}

int main() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_script_texture_probe");
    std::filesystem::create_directories(root);
    const std::filesystem::path fonts = root / "Fonts";
    std::filesystem::create_directories(fonts);
    writeTestBmp(fonts / "probe.bmp", 4, 2, {255, 255, 80, 80, 255, 255, 80, 80});
    writeTestBmp(fonts / "probe_alpha.bmp", 4, 2, {255, 255, 128, 128, 255, 255, 128, 128});
    writeTestBmp(fonts / "unit.bmp", 6, 2,
                 {255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255});
    writeTestBmp(fonts / "unit_alpha.bmp", 6, 2,
                 {255, 255, 255, 255, 128, 255, 255, 255, 255, 255, 128, 255});
    writeTestBmp(fonts / "grid.bmp", 4, 2, {255, 255, 255, 255, 255, 255, 255, 255});
    writeTestBmp(fonts / "grid_alpha.bmp", 4, 2, {0, 0, 0, 0, 255, 255, 255, 255});
    std::ofstream font(fonts / "probe.oft");
    font << "[newfont]\nProbeFont\nprobe.bmp\nprobe_alpha.bmp\n2\n0\n"
            "[char]\nA\n0\n2\n0\n"
            "[char]\nB\n2\n4\n0\n";
    font << "[char]\n0\n0\n2\n0\n"
            "[char]\n1\n2\n4\n0\n"
            "[char]\n?\n2\n4\n0\n";
    font.close();
    std::ofstream unitFont(fonts / "unit.oft");
    unitFont << "[newfont]\nUnitFont\nunit.bmp\nunit_alpha.bmp\n2\n0\n"
                "[char]\nA\n0\n1\n0\n"
                "[char]\nc\n1\n2\n0\n"
                "[char]\n\xc2\xb0\n2\n4\n0\n"
                "[char]\n\xc3\xb1\n4\n5\n0\n";
    unitFont.close();
    std::ofstream legacyFont(fonts / "legacy.oft");
    legacyFont << "[newfont]\nLegacyFont\nunit.bmp\nunit_alpha.bmp\n2\n0\n"
                  "[char]\nA\n0\n1\n0\n"
                  "[char]\nc\n1\n2\n0\n";
    legacyFont.close();
    std::ofstream gridFont(fonts / "grid.oft");
    gridFont << "[newfont]\nGridFont\ngrid.bmp\ngrid_alpha.bmp\n2\n0\n"
                "[char]\nA\n0\n2\n0\n";
    gridFont.close();
    std::ofstream spacingFont(fonts / "spacing.oft");
    spacingFont << "[newfont]\nSpacingFont\nprobe.bmp\nprobe_alpha.bmp\n2\n1\n"
                   "[char]\nA\n0\n2\n0\n"
                   "[char]\nB\n2\n4\n0\n";
    spacingFont.close();
    if (!checkAuthoredScriptDrawing(root)) {
        openbus::probe::removeProbeDirectory(root);
        return 1;
    }
    const std::filesystem::path scriptPath = root / "probe.osc";
    std::ofstream source(scriptPath);
    source << "{InIt}\n"
              "(L.L.MixedNumeric)\n"
              "(S.L.MixedNumericOut)\n"
              "(L.$.MixedString)\n"
              "(S.$.MixedStringOut)\n"
              "(M.L.MixedMacro)\n"
              "{end}\n"
              "{MaCrO:TextureSafety}\n"
              "3 2147483648 0 (M.V.stdrawpixel)\n"
              "3 0 0 4095 4095 (M.V.stdrawrect)\n"
              "2147483648 (M.V.stnewtex)\n"
              "{end}\n"
              "{MaCrO:MixedMacro}\n"
              "(M.V.MixedSystemMacro)\n"
              "(M.L.AnotherMacro)\n"
              "{end}\n"
              "{TrIgGeR:MixedTrigger}\n"
              "(L.L.TriggerVariable)\n"
              "{end}\n";
    source.close();

    OscProgram parsedProgram;
    std::string parseError;
    if (!compileOscToBytecode(scriptPath, parsedProgram, parseError) ||
        parsedProgram.functions.find("init") == parsedProgram.functions.end() ||
        parsedProgram.functions.find("macro_mixedmacro") == parsedProgram.functions.end() ||
        parsedProgram.functions.find("trigger_mixedtrigger") == parsedProgram.functions.end()) {
        std::cerr << "mixed-case OSC blocks did not normalize at parse time: " << parseError
                  << "\n";
        return 1;
    }
    const auto& initCode = parsedProgram.functions.at("init");
    const std::vector<std::string> expectedNames = {
        "mixednumeric", "mixednumericout", "mixedstring", "mixedstringout", "macro_mixedmacro"};
    std::size_t nameIndex = 0;
    for (const OscInstruction& instruction : initCode) {
        if (instruction.name.empty()) {
            continue;
        }
        if (nameIndex >= expectedNames.size() || instruction.name != expectedNames[nameIndex++]) {
            std::cerr << "OSC identifier was not normalized at parse time: " << instruction.name
                      << "\n";
            return 1;
        }
    }
    if (nameIndex != expectedNames.size() ||
        parsedProgram.functions.find("macro_mixedmacro") == parsedProgram.functions.end()) {
        std::cerr << "not all mixed-case OSC identifiers were validated\n";
        return 1;
    }
    const std::filesystem::path strayEndifScript = root / "stray_endif.osc";
    {
        std::ofstream strayEndifSource(strayEndifScript);
        strayEndifSource << "{macro:legacy}\n"
                            "1 (S.L.before_stray_endif)\n"
                            "{endif}\n"
                            "2 (S.L.after_stray_endif)\n"
                            "{end}\n";
    }
    OscProgram strayEndifProgram;
    std::string strayEndifError;
    if (!compileOscToBytecode(strayEndifScript, strayEndifProgram, strayEndifError)) {
        std::cerr << "legacy extra {endif} stopped OSC compilation: " << strayEndifError << "\n";
        return 1;
    }
    const auto& strayEndifCode = strayEndifProgram.functions.at("macro_legacy");
    if (strayEndifCode.size() != 4 || strayEndifCode[1].name != "before_stray_endif" ||
        strayEndifCode[3].name != "after_stray_endif") {
        std::cerr << "OSC code after a legacy extra {endif} was not compiled\n";
        return 1;
    }
    std::string strayEndifLuaError;
    if (!convertOscToLua(strayEndifScript, generatedLuaPath(strayEndifScript),
                         strayEndifLuaError)) {
        std::cerr << "legacy extra {endif} failed Lua conversion: " << strayEndifLuaError << "\n";
        return 1;
    }
    const std::string strayEndifLuaSource = readTextFile(generatedLuaPath(strayEndifScript));
    if (strayEndifLuaSource.find("after_stray_endif") == std::string::npos) {
        std::cerr << "Lua conversion truncated a macro at an unmatched {endif}\n";
        return 1;
    }
    const std::filesystem::path missingBlockEndScript = root / "missing_block_end.osc";
    {
        std::ofstream missingBlockEndSource(missingBlockEndScript);
        missingBlockEndSource << "{macro:antrieb_frame}\n"
                                 "(L.L.antrieb_failure_general) !\n"
                                 "{if}\n"
                                 "(M.L.antrieb_getr_chkNxtGear)\n"
                                 "{endif}\n"
                                 "{macro:antrieb_getr_chkNxtGear}\n"
                                 "1 (S.L.gear_macro_body)\n"
                                 "{end}\n";
    }
    OscProgram missingBlockEndProgram;
    std::string missingBlockEndError;
    if (!compileOscToBytecode(missingBlockEndScript, missingBlockEndProgram,
                              missingBlockEndError) ||
        missingBlockEndProgram.functions.find("macro_antrieb_frame") ==
            missingBlockEndProgram.functions.end() ||
        missingBlockEndProgram.functions.find("macro_antrieb_getr_chknxtgear") ==
            missingBlockEndProgram.functions.end()) {
        std::cerr << "new OSC block did not recover from a missing preceding {end}: "
                  << missingBlockEndError << "\n";
        return 1;
    }
    const auto& recoveredFrameCode =
        missingBlockEndProgram.functions.at("macro_antrieb_frame");
    if (std::none_of(recoveredFrameCode.begin(), recoveredFrameCode.end(),
                     [](const OscInstruction& instruction) {
                         return instruction.opcode == OscOpcode::CallFunction &&
                                instruction.name == "macro_antrieb_getr_chknxtgear";
                     }) ||
        missingBlockEndProgram.functions.at("macro_antrieb_getr_chknxtgear").size() != 2) {
        std::cerr << "recovered OSC blocks did not retain the call and macro body\n";
        return 1;
    }
    std::string missingBlockEndLuaError;
    if (!convertOscToLua(missingBlockEndScript, generatedLuaPath(missingBlockEndScript),
                         missingBlockEndLuaError)) {
        std::cerr << "Lua conversion did not recover from a missing preceding {end}: "
                  << missingBlockEndLuaError << "\n";
        return 1;
    }
    const std::string recoveredLuaSource = readTextFile(generatedLuaPath(missingBlockEndScript));
    if (recoveredLuaSource.find("function macro_antrieb_getr_chknxtgear()") ==
            std::string::npos ||
        recoveredLuaSource.find("macro_antrieb_getr_chknxtgear()") == std::string::npos ||
        recoveredLuaSource.find("gear_macro_body") == std::string::npos) {
        std::cerr << "Lua conversion did not emit the recovered macro declaration and call\n";
        return 1;
    }
    for (const char* backend : {"native", "lua"}) {
        if (!setScriptBackend(backend)) {
            return 1;
        }
        VehicleConfig configuration;
        configuration.sourcePath = root / "missing_block_end.bus";
        configuration.scripts = {missingBlockEndScript.filename().string()};
        Variables variables(ScriptObjectKind::Vehicle);
        SimulationState simulation;
        ScriptRuntime runtime(configuration, variables, simulation);
        runtime.initialize();
        if (!runtime.valid() || !runtime.errors().empty() ||
            !runtime.hasScriptEntryPoint("macro_antrieb_getr_chknxtgear")) {
            std::cerr << backend << ": recovered gear macro was not registered\n";
            return 1;
        }
        runtime.invokeEntryPoint("macro_antrieb_frame");
        if (variables.get("gear_macro_body") != 1.0F) {
            std::cerr << backend << ": recovered gear macro was not invoked by its caller\n";
            return 1;
        }
    }
    if (!setScriptBackend(nullptr)) {
        return 1;
    }
    const std::filesystem::path stringCompareScript = root / "string_compare.osc";
    {
        std::ofstream stringCompareSource(stringCompareScript);
        stringCompareSource << "{init}\n\"¹\" \"¹\" $=>\n{end}\n";
    }
    OscProgram stringCompareProgram;
    std::string stringCompareError;
    if (!compileOscToBytecode(stringCompareScript, stringCompareProgram, stringCompareError)) {
        std::cerr << "OMSI string >= operator spelling was rejected: " << stringCompareError
                  << "\n";
        return 1;
    }
    const auto& stringCompareCode = stringCompareProgram.functions.at("init");
    if (stringCompareCode.size() != 3 ||
        stringCompareCode[2].opcode != OscOpcode::StringGreaterEqual) {
        std::cerr << "OMSI string >= spelling did not compile as StringGreaterEqual\n";
        return 1;
    }
    std::string stringCompareLuaError;
    if (!convertOscToLua(stringCompareScript, generatedLuaPath(stringCompareScript),
                         stringCompareLuaError)) {
        std::cerr << "OMSI string >= spelling failed Lua conversion: " << stringCompareLuaError
                  << "\n";
        return 1;
    }
    const std::string generatedCompareSource = readTextFile(generatedLuaPath(stringCompareScript));
    if (generatedCompareSource.find("a>=b and 1 or 0") == std::string::npos) {
        std::cerr << "Lua converter did not emit string >= comparison for $=>\n";
        return 1;
    }
    const auto& macroCode = parsedProgram.functions.at("macro_mixedmacro");
    if (macroCode.size() != 2 || macroCode[0].name != "mixedsystemmacro" ||
        macroCode[1].name != "macro_anothermacro") {
        std::cerr << "macro identifiers were not normalized at parse time\n";
        return 1;
    }
    const auto& triggerCode = parsedProgram.functions.at("trigger_mixedtrigger");
    if (triggerCode.size() != 1 || triggerCode[0].name != "triggervariable") {
        std::cerr << "trigger identifiers were not normalized at parse time\n";
        return 1;
    }
    std::ofstream generated(generatedLuaPath(scriptPath));
    generated << "function init()\n"
                 "_pushf(3)\n"
                 "sys_macro_stnewtex()\n"
                 "_pushf(3)\n"
                 "_pushf(255)\n"
                 "_pushf(255)\n"
                 "_pushf(0)\n"
                 "_pushf(0)\n"
                 "sys_macro_stsetcolor()\n"
                 "_pushf(3)\n"
                 "_pushf(1)\n"
                 "_pushf(2)\n"
                 "sys_macro_stdrawpixel()\n"
                 "_pushf(3)\n"
                 "_pushf(1e30)\n"
                 "_pushf(0)\n"
                 "sys_macro_stdrawpixel()\n"
                 "_pushf(3)\n"
                 "_pushf(0)\n"
                 "_pushf(0)\n"
                 "_pushf(4095)\n"
                 "_pushf(4095)\n"
                 "sys_macro_stdrawrect()\n"
                 "_pushf(1e30)\n"
                 "sys_macro_stnewtex()\n"
                 "end\n"
                 "function trigger_routedisplay()\n"
                 "set_local_var(\"mouse_click_seen\", 1)\n"
                 "end\n"
                 "function trigger_routedisplay_drag()\n"
                 "set_local_var(\"mouse_drag_seen\", 1)\n"
                 "end\n"
                 "function trigger_routedisplay_off()\n"
                 "set_local_var(\"mouse_release_seen\", 1)\n"
                 "end\n"
                 "function trigger_ticketergimble_drag()\n"
                 "set_local_var(\"ticketer_drag_seen\", 1)\n"
                 "local value = get_local_var(\"ticketer_pos\") + get_sys_var(\"mouse_y\") / 250\n"
                 "if value < -0.2 then value = -0.2 end\n"
                 "if value > 1.5 then value = 1.5 end\n"
                 "set_local_var(\"ticketer_pos\", value)\n"
                 "end\n"
                 "function trigger_kw_m_enginestart()\n"
                 "set_local_var(\"key_binding_seen\", 1)\n"
                 "end\n"
                 "function trigger_kw_m_enginestart_off()\n"
                 "set_local_var(\"key_binding_release_seen\", 1)\n"
                 "end\n";
    generated.close();

    VehicleConfig configuration;
    configuration.sourcePath = root / "probe.bus";
    configuration.selectedVehicleNumber = "Fleet 01";
    configuration.selectedRegistration = "AB12 CDE";
    configuration.scripts.push_back(scriptPath.filename().string());
    if (!setScriptBackend(nullptr)) {
        std::cerr << "could not clear script backend override for native-default test\n";
        return 1;
    }
    {
        Variables nativeVariables(ScriptObjectKind::Vehicle);
        SimulationState nativeSimulation;
        nativeVariables.set("mixednumeric", 42.0);
        ScriptRuntime nativeRuntime(configuration, nativeVariables, nativeSimulation);
        nativeRuntime.configureScriptTextures({ModelScriptTexture{3, 4, 5, {"4", "5"}}});
        nativeRuntime.initialize();
        nativeRuntime.invokeEntryPoint("macro_texturesafety");
        ScriptRuntime::ScriptTextureSnapshot textureAfterNativeSafety;
        if (nativeVariables.get("mixednumericout") != 42.0) {
            std::cerr << "native OSC backend was not selected by default\n";
            return 1;
        }
        if (!nativeRuntime.copyScriptTexture(3, textureAfterNativeSafety) ||
            textureAfterNativeSafety.width != 4 || textureAfterNativeSafety.height != 5) {
            std::cerr << "native texture macros accepted unsafe drawing arguments\n";
            return 1;
        }
    }
    const std::filesystem::path fallbackScriptPath = root / "fallback.osc";
    {
        std::ofstream invalidSource(fallbackScriptPath);
        invalidSource << "not valid OSC bytecode input\n";
        invalidSource.close();
        std::ofstream fallbackLua(generatedLuaPath(fallbackScriptPath));
        fallbackLua << "function init()\n"
                       "set_local_var(\"lua_fallback_seen\", 1)\n"
                       "end\n";
    }
    VehicleConfig fallbackConfiguration;
    fallbackConfiguration.sourcePath = root / "fallback.bus";
    fallbackConfiguration.scripts.push_back(fallbackScriptPath.filename().string());
    if (!setScriptBackend("native")) {
        std::cerr << "could not select native backend for fallback probe\n";
        return 1;
    }
    {
        Variables fallbackVariables(ScriptObjectKind::Vehicle);
        SimulationState fallbackSimulation;
        ScriptRuntime fallbackRuntime(fallbackConfiguration, fallbackVariables, fallbackSimulation);
        fallbackRuntime.initialize();
        if (fallbackVariables.get("lua_fallback_seen") != 1.0) {
            std::cerr << "native compilation failure did not fall back to Lua\n";
            return 1;
        }
    }
    const std::filesystem::path numericScriptPath = root / "numeric.osc";
    {
        std::ofstream numericSource(numericScriptPath);
        numericSource << "{init}\n"
                         "16777216 1 + (S.L.float32_sum)\n"
                         "0 2 - (S.L.truthy_input)\n"
                         "(L.L.truthy_input)\n"
                         "{if}\n"
                         "  1 (S.L.boolean_branch)\n"
                         "{else}\n"
                         "  0 (S.L.boolean_branch)\n"
                         "{endif}\n"
                         "(L.L.truthy_input) ! (S.L.boolean_not)\n"
                         "0 ! (S.L.boolean_from_zero)\n"
                         "(L.L.truthy_input) -3 && (S.L.boolean_and)\n"
                         "(L.L.truthy_input) 0 || (S.L.boolean_or)\n"
                         "2 1 > (S.L.boolean_compare)\n"
                         "{end}\n";
    }
    VehicleConfig numericConfiguration;
    numericConfiguration.sourcePath = root / "numeric.bus";
    numericConfiguration.scripts.push_back(numericScriptPath.filename().string());
    if (!setScriptBackend("native")) {
        std::cerr << "could not select native backend for numeric semantics probe\n";
        return 1;
    }
    {
        Variables numericVariables(ScriptObjectKind::Vehicle);
        SimulationState numericSimulation;
        ScriptRuntime numericRuntime(numericConfiguration, numericVariables, numericSimulation);
        numericRuntime.initialize();
        if (numericVariables.get("float32_sum") != 16777216.0 ||
            numericVariables.get("boolean_branch") != 1.0 ||
            numericVariables.get("boolean_not") != 0.0 ||
            numericVariables.get("boolean_from_zero") != 1.0 ||
            numericVariables.get("boolean_and") != 1.0 ||
            numericVariables.get("boolean_or") != 1.0 ||
            numericVariables.get("boolean_compare") != 1.0) {
            std::cerr << "native OSC numeric or boolean semantics mismatch: sum="
                      << numericVariables.get("float32_sum")
                      << " branch=" << numericVariables.get("boolean_branch")
                      << " not=" << numericVariables.get("boolean_not")
                      << " zero-not=" << numericVariables.get("boolean_from_zero")
                      << " and=" << numericVariables.get("boolean_and")
                      << " or=" << numericVariables.get("boolean_or")
                      << " compare=" << numericVariables.get("boolean_compare") << "\n";
            return 1;
        }
    }
    const std::filesystem::path registerScriptPath = root / "register9.osc";
    {
        std::ofstream registerScript(registerScriptPath);
        registerScript << "{init}\n"
                          "42 s9 l9 (S.L.register_nine)\n"
                          "0 (S.L.frame_count)\n"
                          "{end}\n"
                          "{frame}\n"
                          "(L.L.frame_count) 1 + (S.L.frame_count)\n"
                          "{end}\n";
    }
    VehicleConfig registerConfiguration;
    registerConfiguration.sourcePath = root / "register.bus";
    registerConfiguration.scripts = {registerScriptPath.filename().string(),
                                     registerScriptPath.filename().string()};
    registerConfiguration.floatVariables = {"register_nine", "frame_count"};
    if (!setScriptBackend("native")) {
        std::cerr << "could not select native backend for OSC register 9 probe\n";
        return 1;
    }
    {
        Variables registerVariables(ScriptObjectKind::Vehicle);
        SimulationState registerSimulation;
        ScriptRuntime registerRuntime(registerConfiguration, registerVariables,
                                      registerSimulation);
        registerRuntime.initialize();
        registerRuntime.update(false);
        if (registerVariables.get("register_nine") != 42.0 ||
            registerVariables.get("frame_count") != 1.0) {
            std::cerr << "native OSC register 9 or duplicate-script deduplication failed\n";
            return 1;
        }
    }
    std::string registerLuaError;
    if (!convertOscToLua(registerScriptPath, generatedLuaPath(registerScriptPath),
                         registerLuaError)) {
        std::cerr << "could not generate Lua register 9 probe: " << registerLuaError << "\n";
        return 1;
    }
    std::string numericLuaError;
    if (!convertOscToLua(numericScriptPath, generatedLuaPath(numericScriptPath), numericLuaError)) {
        std::cerr << "could not generate numeric semantics Lua probe: " << numericLuaError << "\n";
        return 1;
    }
    if (!setScriptBackend("lua")) {
        std::cerr << "could not select Lua backend for Lua-specific runtime probe\n";
        return 1;
    }
    {
        Variables numericVariables(ScriptObjectKind::Vehicle);
        SimulationState numericSimulation;
        ScriptRuntime numericRuntime(numericConfiguration, numericVariables, numericSimulation);
        numericRuntime.initialize();
        if (numericVariables.get("float32_sum") != 16777216.0 ||
            numericVariables.get("boolean_branch") != 1.0 ||
            numericVariables.get("boolean_not") != 0.0 ||
            numericVariables.get("boolean_from_zero") != 1.0 ||
            numericVariables.get("boolean_and") != 1.0 ||
            numericVariables.get("boolean_or") != 1.0 ||
            numericVariables.get("boolean_compare") != 1.0) {
            std::cerr << "generated Lua numeric or boolean semantics mismatch: sum="
                      << numericVariables.get("float32_sum")
                      << " branch=" << numericVariables.get("boolean_branch")
                      << " not=" << numericVariables.get("boolean_not")
                      << " zero-not=" << numericVariables.get("boolean_from_zero")
                      << " and=" << numericVariables.get("boolean_and")
                      << " or=" << numericVariables.get("boolean_or")
                      << " compare=" << numericVariables.get("boolean_compare") << "\n";
            return 1;
        }
    }
    {
        Variables registerVariables(ScriptObjectKind::Vehicle);
        SimulationState registerSimulation;
        ScriptRuntime registerRuntime(registerConfiguration, registerVariables,
                                      registerSimulation);
        registerRuntime.initialize();
        registerRuntime.update(false);
        if (registerVariables.get("register_nine") != 42.0 ||
            registerVariables.get("frame_count") != 1.0) {
            std::cerr << "generated Lua register 9 or duplicate-script handling failed\n";
            return 1;
        }
    }
    const std::filesystem::path toggleScriptPath = root / "toggle.osc";
    {
        std::ofstream toggleSource(toggleScriptPath);
        toggleSource << "{trigger:toggle}\n"
                        "(L.L.blinkgeber) ! (S.L.blinkgeber)\n"
                        "{if}\n"
                        "  (C.L.blinkertime_on) (S.L.interval)\n"
                        "{else}\n"
                        "  (C.L.blinkertime_off) (S.L.interval)\n"
                        "{endif}\n"
                        "{end}\n";
    }
    std::string luaConversionError;
    if (!convertOscToLua(toggleScriptPath, generatedLuaPath(toggleScriptPath),
                         luaConversionError)) {
        std::cerr << "could not generate Lua toggle probe: " << luaConversionError << "\n";
        return 1;
    }
    VehicleConfig toggleConfiguration;
    toggleConfiguration.sourcePath = root / "toggle.bus";
    toggleConfiguration.scripts.push_back(toggleScriptPath.filename().string());
    toggleConfiguration.constants["blinkertime_on"] = 0.6;
    toggleConfiguration.constants["blinkertime_off"] = 0.4;
    Variables toggleVariables(ScriptObjectKind::Vehicle);
    SimulationState toggleSimulation;
    toggleVariables.set("blinkgeber", 0.0);
    ScriptRuntime toggleRuntime(toggleConfiguration, toggleVariables, toggleSimulation);
    toggleRuntime.invokeEntryPoint("trigger_toggle");
    if (toggleVariables.get("blinkgeber") != 1.0 ||
        toggleVariables.get("interval") != static_cast<double>(static_cast<float>(0.6))) {
        std::cerr << "generated Lua ON transition mismatch: blinkgeber="
                  << toggleVariables.get("blinkgeber")
                  << " interval=" << toggleVariables.get("interval") << "\n";
        return 1;
    }
    toggleRuntime.invokeEntryPoint("trigger_toggle");
    if (toggleVariables.get("blinkgeber") != 0.0 ||
        toggleVariables.get("interval") != static_cast<double>(static_cast<float>(0.4))) {
        std::cerr << "generated Lua did not select the OFF interval after toggling off\n";
        return 1;
    }

    const std::filesystem::path sceneryScriptPath = root / "scenery.osc";
    {
        std::ofstream sceneryScript(sceneryScriptPath);
        sceneryScript << "{init}\n0 (S.L.frame_count)\n{end}\n"
                         "{frame}\n(L.L.frame_count) 1 + (S.L.frame_count)\n"
                         "(L.L.frame_count) (S.L.lamp_visible)\n{end}\n";
    }
    VehicleConfig sceneryConfiguration;
    sceneryConfiguration.sourcePath = root / "object.sco";
    sceneryConfiguration.scripts.push_back(sceneryScriptPath.filename().string());
    sceneryConfiguration.floatVariables = {"frame_count", "lamp_visible"};
    if (!setScriptBackend("native")) {
        std::cerr << "could not select the native backend for scenery runtime probe\n";
        return 1;
    }
    {
        openbus::scripting::SceneryObject firstObject;
        openbus::scripting::SceneryObject secondObject;
        SimulationState scenerySimulation;
        ScriptRuntime firstRuntime(sceneryConfiguration, firstObject, scenerySimulation);
        ScriptRuntime secondRuntime(sceneryConfiguration, secondObject, scenerySimulation);
        firstObject.declareString("sign_text");
        secondObject.declareString("sign_text");
        firstObject.setString("sign_text", "A");
        secondObject.setString("sign_text", "B");
        const std::vector<ModelTextTexture> signText = {ModelTextTexture{
            0, false, {"sign_text", "ProbeFont", "6", "4", "0", "255", "255", "255"}}};
        firstRuntime.configureTextTextures(signText);
        secondRuntime.configureTextTextures(signText);
        firstRuntime.initialize();
        secondRuntime.initialize();
        ScriptRuntime::ScriptTextureSnapshot firstSign;
        ScriptRuntime::ScriptTextureSnapshot secondSign;
        if (!firstRuntime.copyTextTexture(0, firstSign) ||
            !secondRuntime.copyTextTexture(0, secondSign) ||
            firstSign.pixels == secondSign.pixels) {
            std::cerr << "scenery text textures were not rendered from per-placement strings\n";
            return 1;
        }
        firstRuntime.update(false);
        if (firstObject.get("frame_count") != 1.0 || secondObject.get("frame_count") != 0.0 ||
            firstObject.get("lamp_visible") != 1.0 || secondObject.get("lamp_visible") != 0.0) {
            std::cerr << "scenery script state was not isolated per placed object\n";
            return 1;
        }
        secondRuntime.update(false);
        firstRuntime.update(false);
        if (firstObject.get("frame_count") != 2.0 || secondObject.get("frame_count") != 1.0 ||
            firstObject.get("lamp_visible") != 2.0 || secondObject.get("lamp_visible") != 1.0) {
            std::cerr << "scenery frame scripts did not tick independently\n";
            return 1;
        }
    }
    if (!setScriptBackend("lua")) {
        std::cerr << "could not restore the Lua backend for following runtime probes\n";
        return 1;
    }

    Variables variables(ScriptObjectKind::Vehicle);
    SimulationState simulation;
    ScriptRuntime runtime(configuration, variables, simulation);

    simulation.sharedVariables().updateFrame(0.016, 1.0, 900.0, 700.0);
    if (simulation.sharedVariables().get("mouse_x") != 0.0 ||
        simulation.sharedVariables().get("mouse_y") != 0.0 ||
        simulation.sharedVariables().get("mouse_cursor_x") != 900.0 ||
        simulation.sharedVariables().get("mouse_cursor_y") != 700.0) {
        std::cerr << "system cursor position was not separated from drag motion\n";
        return 1;
    }
    runtime.invokeInputEvent("W", true);
    if (variables.get("key_pressed") != 1.0 || variables.getString("key_name") != "w") {
        std::cerr << "legacy key event was not exposed to the script state\n";
        return 1;
    }
    runtime.invokeKeyBinding("kw_m_enginestart", true);
    runtime.invokeKeyBinding("kw_m_enginestart", false);
    runtime.invokeMouseEvent("RouteDisplay");
    runtime.invokeMouseDrag("RouteDisplay", 3.0, -2.0, 120.0, 80.0);
    runtime.invokeMouseRelease("RouteDisplay");
    if (variables.getString("mouse_event") != "routedisplay" ||
        variables.get("mouse_click_seen") != 1.0 || variables.get("mouse_drag_seen") != 1.0 ||
        variables.get("mouse_release_seen") != 1.0 || variables.get("key_binding_seen") != 1.0 ||
        variables.get("key_binding_release_seen") != 1.0 || variables.get("mouse_drag_x") != 3.0 ||
        variables.get("mouse_drag_y") != -2.0 || variables.get("mouse_cursor_x") != 120.0 ||
        variables.get("mouse_cursor_y") != 80.0 ||
        simulation.sharedVariables().get("mouse_x") != 0.0 ||
        simulation.sharedVariables().get("mouse_y") != 0.0) {
        std::cerr << "input events were not exposed to the script state\n";
        return 1;
    }
    runtime.invokeMouseDrag("TicketerGimble", 3.0, -2.0, 120.0, 80.0);
    if (variables.get("ticketer_drag_seen") != 1.0 || variables.get("mouse_drag_x") != 3.0 ||
        variables.get("mouse_drag_y") != -2.0 ||
        simulation.sharedVariables().get("mouse_x") != 0.0 ||
        simulation.sharedVariables().get("mouse_y") != 0.0) {
        std::cerr << "ticketer vertical drag did not use the expected Y direction\n";
        return 1;
    }
    variables.set("ticketer_pos", 0.0);
    runtime.invokeMouseDrag("TicketerGimble", 0.0, -500.0, 120.0, 80.0);
    if (variables.get("ticketer_pos") != static_cast<double>(static_cast<float>(-0.2))) {
        std::cerr << "upward ticketer drag did not clamp at the lower limit\n";
        return 1;
    }
    runtime.invokeMouseDrag("TicketerGimble", 0.0, 500.0, 120.0, 80.0);
    if (variables.get("ticketer_pos") != 1.5) {
        std::cerr << "downward ticketer drag did not clamp at the upper limit\n";
        return 1;
    }

    runtime.configureScriptTextures(
        {ModelScriptTexture{3, 4, 5, {"4", "5"}}, ModelScriptTexture{4, 4097, 1, {"4097", "1"}}});
    runtime.configureTextTextures(
        {ModelTextTexture{0, false, {"display", "probe-font", "32", "16", "0", "255", "0", "0"}},
         ModelTextTexture{
             1, true, {"bitmapdisplay", "ProbeFont", "6", "4", "0", "10", "20", "30", "2"}},
         ModelTextTexture{
             2, true, {"centerdisplay", "ProbeFont", "6", "8", "1", "10", "20", "30", "0"}},
         ModelTextTexture{
             3, true, {"blockcolordisplay", "ProbeFont", "6", "6", "0", "1", "1", "1", "0"}},
         ModelTextTexture{
             4, true, {"leftdisplay", "ProbeFont", "8", "4", "1", "10", "20", "30", "1"}},
         ModelTextTexture{5, false, {"ident", "probe-font", "32", "16", "0", "255", "255", "255"}},
         ModelTextTexture{
             6,
             true,
             {"gridcenterdisplay", "ProbeFont", "8", "4", "0", "10", "20", "30", "3", "1"}},
         ModelTextTexture{
             7, true, {"gridleftdisplay", "ProbeFont", "8", "4", "0", "10", "20", "30", "4", "1"}},
         ModelTextTexture{
             8, true, {"gridrightdisplay", "ProbeFont", "8", "4", "0", "10", "20", "30", "5", "0"}},
         ModelTextTexture{
             9, true, {"unitdisplay", "UnitFont", "12", "4", "0", "10", "20", "30", "0"}},
         ModelTextTexture{
             10, true, {"legacyunitdisplay", "LegacyFont", "12", "4", "0", "10", "20", "30", "0"}},
         ModelTextTexture{
             11, true, {"unitreference", "UnitFont", "12", "4", "0", "10", "20", "30", "0"}},
         ModelTextTexture{
             12, true, {"legacyreference", "LegacyFont", "12", "4", "0", "10", "20", "30", "0"}},
         ModelTextTexture{
             13, true, {"gridboxdisplay", "GridFont", "4", "6", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             14, true, {"gridpixelsdisplay", "GridFont", "4", "6", "0", "255", "255", "255", "0", "1"}},
         ModelTextTexture{
             15, true, {"leftbiasdisplay", "ProbeFont", "7", "4", "0", "255", "255", "255", "4", "0"}},
         ModelTextTexture{
             16, true, {"rightbiasdisplay", "ProbeFont", "7", "4", "0", "255", "255", "255", "5", "0"}},
         ModelTextTexture{
             17, true, {"fallbackdisplay", "ProbeFont", "6", "4", "0", "10", "20", "30", "1", "0"}},
         ModelTextTexture{
             18, true, {"fallbackunicodedisplay", "MissingFont", "24", "20", "0", "255", "255", "255", "1", "0"}},
         ModelTextTexture{
             19, true, {"fallbackpunctuationdisplay", "MissingFont", "40", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             20, true, {"lowercasefallbackdisplay", "MissingFont", "160", "10", "0", "255", "255", "255", "1", "0"}},
         ModelTextTexture{
             21, true, {"uppercasefallbackdisplay", "MissingFont", "160", "10", "0", "255", "255", "255", "1", "0"}},
         ModelTextTexture{
             22, true, {"fallbackmultilinedisplay", "MissingFont", "24", "20", "0", "255", "255", "255", "1", "0"}},
         ModelTextTexture{
             23, true, {"lowercaseauthored", "ProbeFont", "6", "8", "1", "10", "20", "30", "0"}},
         ModelTextTexture{
             24, true, {"fallbackgermandisplay", "MissingFont", "44", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             25, true, {"fallbackquestiondisplay", "MissingFont", "44", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             26, true, {"fallbacklatin1display", "MissingFont", "110", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             27, true, {"fallbacklatin1questiondisplay", "MissingFont", "110", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             28, true, {"unitlatin1display", "UnitFont", "6", "8", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             29, true, {"unitunicodefallbackdisplay", "UnitFont", "6", "8", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             30, true, {"unitfallbackreferencedisplay", "UnitFont", "6", "8", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             31, true, {"spacingcenterdisplay", "SpacingFont", "10", "4", "0", "255", "255", "255", "3", "0"}},
         ModelTextTexture{
             32, true, {"spacingrightdisplay", "SpacingFont", "10", "4", "0", "255", "255", "255", "2", "0"}},
         ModelTextTexture{
             33, true, {"spacingleftbiasdisplay", "SpacingFont", "10", "4", "0", "255", "255", "255", "4", "0"}},
         ModelTextTexture{
             34, true, {"spacingrightbiasdisplay", "SpacingFont", "10", "4", "0", "255", "255", "255", "5", "0"}},
         ModelTextTexture{
             35, true, {"typographicfallbackdisplay", "MissingFont", "62", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             36, true, {"typographicquestiondisplay", "MissingFont", "62", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             37, true, {"fallbackcurrencydisplay", "MissingFont", "62", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             38, true, {"fallbackcurrencyquestiondisplay", "MissingFont", "62", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             39, true, {"fallbackmathdisplay", "MissingFont", "20", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             40, true, {"fallbackmathquestiondisplay", "MissingFont", "20", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             41, true, {"fallbackarrowsdisplay", "MissingFont", "26", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             42, true, {"fallbackarrowsquestiondisplay", "MissingFont", "26", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             43, true, {"fallbackdotdisplay", "MissingFont", "14", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             44, true, {"fallbackdotquestiondisplay", "MissingFont", "14", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             45, true, {"fallbackspacesdisplay", "MissingFont", "20", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             46, true, {"fallbackspacesreferencedisplay", "MissingFont", "20", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             47, true, {"fallbackgreekdisplay", "MissingFont", "32", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             48, true, {"fallbackgreekquestiondisplay", "MissingFont", "32", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             49, true, {"fallbacknordicdisplay", "MissingFont", "50", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             50, true, {"fallbacknordicquestiondisplay", "MissingFont", "50", "10", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             51, false, {"malformedwidth", "MissingFont", "6junk", "4", "0", "255", "255", "255"}},
         ModelTextTexture{
             52, false, {"malformedheight", "MissingFont", "6", "4junk", "0", "255", "255", "255"}},
         ModelTextTexture{
             53, false, {"malformedcolor", "MissingFont", "6", "8", "0", "128junk", "255", "255"}},
         ModelTextTexture{
             54, false, {"referencewhite", "MissingFont", "6", "8", "0", "255", "255", "255"}},
         ModelTextTexture{
             55, true, {"malformedalignment", "MissingFont", "8", "8", "0", "255", "255", "255", "1junk", "0"}},
         ModelTextTexture{
             56, true, {"referencecenter", "MissingFont", "8", "8", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             57, true, {"outofrangealignment", "MissingFont", "8", "8", "0", "255", "255", "255", "9", "0"}},
         ModelTextTexture{
             58, true, {"referencecenteroutofrange", "MissingFont", "8", "8", "0", "255", "255", "255", "0", "0"}},
         ModelTextTexture{
             59, false, {"outofrangedimension", "MissingFont", "1000000", "4", "0", "255", "255", "255"}},
         ModelTextTexture{
             60, false, {"malformedcolormode", "MissingFont", "6", "8", "0junk", "255", "255", "255"}},
         ModelTextTexture{
             61, false, {"referencecolormode", "MissingFont", "6", "8", "0", "255", "255", "255"}},
         ModelTextTexture{
             64, true, {"malformedpixelmode", "GridFont", "4", "6", "0", "255", "255", "255", "0", "1junk"}},
         ModelTextTexture{
             65, true, {"referencepixelmode", "GridFont", "4", "6", "0", "255", "255", "255", "0", "0"}},
         });
    variables.setString("display", "HELLO\n123");
    variables.setString("bitmapdisplay", "A@AB");
    variables.setString("centerdisplay", "A");
    variables.setString("blockcolordisplay", "A");
    variables.setString("leftdisplay", "A");
    variables.setString("gridcenterdisplay", "AB");
    variables.setString("gridleftdisplay", "AB");
    variables.setString("gridrightdisplay", "AB");
    variables.setString("unitdisplay", std::string("\xef\xbf\xbd"
                                                   "c",
                                                   4));
    variables.setString("legacyunitdisplay", std::string("\xef\xbf\xbd"
                                                         "c",
                                                         4));
    variables.setString("unitreference", std::string("\xc2\xb0"
                                                     "c",
                                                     3));
    variables.setString("unitlatin1display", std::string("\xc3\xb1", 2));
    variables.setString("unitunicodefallbackdisplay", std::string("\xf0\x9f\x98\x80", 4));
    variables.setString("unitfallbackreferencedisplay", "A");
    variables.setString("spacingcenterdisplay", "AB");
    variables.setString("spacingrightdisplay", "AB");
    variables.setString("spacingleftbiasdisplay", "AB");
    variables.setString("spacingrightbiasdisplay", "AB");
    variables.setString(
        "typographicfallbackdisplay",
        std::string("\xc2\xa0\xc2\xa1\xc2\xbf\xc2\xab\xc2\xbb"
                    "\xe2\x80\x93\xe2\x80\x94\xe2\x80\x98\xe2\x80\x99\xe2\x80\xa6",
                    25));
    variables.setString("typographicquestiondisplay", "??????????");
    variables.setString("fallbackcurrencydisplay",
                        std::string("\xc2\xa2\xc2\xa3\xc2\xa5\xe2\x82\xac\xc2\xa9\xc2\xae\xe2\x84\xa2"
                                    "\xc2\xb5\xc2\xa7\xc2\xb6",
                                    22));
    variables.setString("fallbackcurrencyquestiondisplay", "??????????");
    variables.setString("fallbackmathdisplay", std::string("\xc2\xb1\xc3\x97\xc3\xb7", 6));
    variables.setString("fallbackmathquestiondisplay", "???");
    variables.setString("fallbackarrowsdisplay",
                        std::string("\xe2\x86\x90\xe2\x86\x91\xe2\x86\x92\xe2\x86\x93", 12));
    variables.setString("fallbackarrowsquestiondisplay", "????");
    variables.setString("fallbackdotdisplay", std::string("\xc2\xb7\xe2\x80\xa2", 5));
    variables.setString("fallbackdotquestiondisplay", "??");
    variables.setString("fallbackspacesdisplay", std::string("\xe2\x80\x83\xe2\x80\xaf\xe3\x80\x80", 9));
    variables.setString("fallbackspacesreferencedisplay", "   ");
    variables.setString("fallbackgreekdisplay",
                        std::string("\xce\xb1\xce\xb2\xce\xbc\xcf\x80\xce\xa9", 10));
    variables.setString("fallbackgreekquestiondisplay", "?????");
    variables.setString("fallbacknordicdisplay",
                        std::string("\xc3\x98\xc3\xb8\xc3\x86\xc3\xa6\xc3\x90\xc3\xb0\xc3\x9e\xc3\xbe",
                                    16));
    variables.setString("fallbacknordicquestiondisplay", "????????");
    variables.setString("malformedcolor", "A");
    variables.setString("referencewhite", "A");
    variables.setString("malformedalignment", "A");
    variables.setString("referencecenter", "A");
    variables.setString("outofrangealignment", "A");
    variables.setString("referencecenteroutofrange", "A");
    variables.setString("malformedcolormode", "A");
    variables.setString("referencecolormode", "A");
    variables.setString("malformedpixelmode", "A");
    variables.setString("referencepixelmode", "A");
    variables.setString("legacyreference", "AAAc");
    variables.setString("gridboxdisplay", "A");
    variables.setString("gridpixelsdisplay", "A");
    variables.setString("leftbiasdisplay", "AB");
    variables.setString("rightbiasdisplay", "AB");
    variables.setString("fallbackdisplay", "X");
    variables.setString("fallbackunicodedisplay", std::string("\xc2\xb0" "C@A", 5));
    variables.setString("fallbackpunctuationdisplay", "!?;()");
    variables.setString("lowercasefallbackdisplay", "abcdefghijklmnopqrstuvwxyz");
    variables.setString("uppercasefallbackdisplay", "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
    variables.setString("fallbackmultilinedisplay", "A@A");
    variables.setString("lowercaseauthored", "a");
    variables.setString("fallbackgermandisplay",
                        std::string("\xc3\x84\xc3\x96\xc3\x9c\xc3\xa4\xc3\xb6\xc3\xbc\xc3\x9f",
                                    14));
    variables.setString("fallbackquestiondisplay", "???????");
    variables.setString("fallbacklatin1display",
                        std::string("\xc3\x81\xc3\x89\xc3\x8d\xc3\x93\xc3\x9a"
                                    "\xc3\x83\xc3\x95\xc3\x91\xc3\x87"
                                    "\xc3\xa1\xc3\xa9\xc3\xad\xc3\xb3\xc3\xba"
                                    "\xc3\xa3\xc3\xb5\xc3\xb1\xc3\xa7",
                                    36));
    variables.setString("fallbacklatin1questiondisplay", "??????????????????");
    runtime.initialize();
    ScriptRuntime::ScriptTextureSnapshot initialText;
    ScriptRuntime::ScriptTextureSnapshot initialPlateText;
    const bool copiedInitialText = runtime.copyTextTexture(0, initialText);
    const bool copiedInitialPlateText = runtime.copyTextTexture(5, initialPlateText);
    bool hasInitialText = false;
    bool hasInitialPlateText = false;
    for (std::size_t offset = 3; copiedInitialText && offset < initialText.pixels.size();
         offset += 4) {
        hasInitialText = hasInitialText || initialText.pixels[offset] != 0;
    }
    for (std::size_t offset = 3; copiedInitialPlateText && offset < initialPlateText.pixels.size();
         offset += 4) {
        hasInitialPlateText = hasInitialPlateText || initialPlateText.pixels[offset] != 0;
    }
    if (!copiedInitialText || !hasInitialText || initialText.pixels.size() != 32U * 16U * 4U ||
        !copiedInitialPlateText || !hasInitialPlateText ||
        initialPlateText.pixels.size() != 32U * 16U * 4U ||
        variables.getString("ident") != "AB12 CDE" || variables.getString("number") != "Fleet 01") {
        std::cerr << "initial registration text was not bound and rasterized from vehicle data\n";
        return 1;
    }
    variables.set("refresh_strings", 1.0);
    runtime.update(false);
    if (variables.get("refresh_strings") != 0.0) {
        std::cerr << "text refresh request was not consumed\n";
        return 1;
    }

    ScriptRuntime::ScriptTextureSnapshot scriptTexture;
    ScriptRuntime::ScriptTextureSnapshot textTexture;
    ScriptRuntime::ScriptTextureSnapshot plateTextTexture;
    ScriptRuntime::ScriptTextureSnapshot bitmapTextTexture;
    ScriptRuntime::ScriptTextureSnapshot centerTextTexture;
    ScriptRuntime::ScriptTextureSnapshot blockColorTextTexture;
    ScriptRuntime::ScriptTextureSnapshot leftTextTexture;
    ScriptRuntime::ScriptTextureSnapshot gridCenterTextTexture;
    ScriptRuntime::ScriptTextureSnapshot gridLeftTextTexture;
    ScriptRuntime::ScriptTextureSnapshot gridRightTextTexture;
    ScriptRuntime::ScriptTextureSnapshot repairedUnitTexture;
    ScriptRuntime::ScriptTextureSnapshot expectedUnitTexture;
    ScriptRuntime::ScriptTextureSnapshot unchangedLegacyUnitTexture;
    ScriptRuntime::ScriptTextureSnapshot expectedLegacyTexture;
    ScriptRuntime::ScriptTextureSnapshot gridBoxAlignedText;
    ScriptRuntime::ScriptTextureSnapshot gridPixelAlignedText;
    ScriptRuntime::ScriptTextureSnapshot leftBiasText;
    ScriptRuntime::ScriptTextureSnapshot rightBiasText;
    ScriptRuntime::ScriptTextureSnapshot fallbackGlyphText;
    ScriptRuntime::ScriptTextureSnapshot fallbackUnicodeText;
    ScriptRuntime::ScriptTextureSnapshot fallbackPunctuationText;
    ScriptRuntime::ScriptTextureSnapshot lowercaseFallbackText;
    ScriptRuntime::ScriptTextureSnapshot uppercaseFallbackText;
    ScriptRuntime::ScriptTextureSnapshot fallbackMultilineText;
    ScriptRuntime::ScriptTextureSnapshot lowercaseAuthoredText;
    ScriptRuntime::ScriptTextureSnapshot germanFallbackText;
    ScriptRuntime::ScriptTextureSnapshot questionFallbackText;
    ScriptRuntime::ScriptTextureSnapshot latin1FallbackText;
    ScriptRuntime::ScriptTextureSnapshot latin1QuestionFallbackText;
    ScriptRuntime::ScriptTextureSnapshot unitLatin1Text;
    ScriptRuntime::ScriptTextureSnapshot unitUnicodeFallbackText;
    ScriptRuntime::ScriptTextureSnapshot unitFallbackReferenceText;
    ScriptRuntime::ScriptTextureSnapshot spacingCenterText;
    ScriptRuntime::ScriptTextureSnapshot spacingRightText;
    ScriptRuntime::ScriptTextureSnapshot spacingLeftBiasText;
    ScriptRuntime::ScriptTextureSnapshot spacingRightBiasText;
    ScriptRuntime::ScriptTextureSnapshot typographicFallbackText;
    ScriptRuntime::ScriptTextureSnapshot typographicQuestionText;
    ScriptRuntime::ScriptTextureSnapshot currencyFallbackText;
    ScriptRuntime::ScriptTextureSnapshot currencyQuestionText;
    ScriptRuntime::ScriptTextureSnapshot mathFallbackText;
    ScriptRuntime::ScriptTextureSnapshot mathQuestionText;
    ScriptRuntime::ScriptTextureSnapshot arrowsFallbackText;
    ScriptRuntime::ScriptTextureSnapshot arrowsQuestionText;
    ScriptRuntime::ScriptTextureSnapshot dotFallbackText;
    ScriptRuntime::ScriptTextureSnapshot dotQuestionText;
    ScriptRuntime::ScriptTextureSnapshot unicodeSpacesText;
    ScriptRuntime::ScriptTextureSnapshot asciiSpacesText;
    ScriptRuntime::ScriptTextureSnapshot greekFallbackText;
    ScriptRuntime::ScriptTextureSnapshot greekQuestionText;
    ScriptRuntime::ScriptTextureSnapshot nordicFallbackText;
    ScriptRuntime::ScriptTextureSnapshot nordicQuestionText;
    if (!runtime.copyScriptTexture(3, scriptTexture) || scriptTexture.width != 4 ||
        scriptTexture.height != 5 || scriptTexture.pixels.size() != 4U * 5U * 4U ||
        !runtime.copyTextTexture(0, textTexture) || textTexture.width != 32 ||
        textTexture.height != 16 || textTexture.pixels.size() != 32U * 16U * 4U ||
        !runtime.copyTextTexture(5, plateTextTexture) || plateTextTexture.width != 32 ||
        plateTextTexture.height != 16 || plateTextTexture.pixels.size() != 32U * 16U * 4U ||
        !runtime.copyTextTexture(1, bitmapTextTexture) || bitmapTextTexture.width != 6 ||
        bitmapTextTexture.height != 4 || bitmapTextTexture.pixels.size() != 6U * 4U * 4U ||
        !runtime.copyTextTexture(2, centerTextTexture) || centerTextTexture.width != 6 ||
        centerTextTexture.height != 8 || centerTextTexture.pixels.size() != 6U * 8U * 4U ||
        !runtime.copyTextTexture(3, blockColorTextTexture) || blockColorTextTexture.width != 6 ||
        blockColorTextTexture.height != 6 || blockColorTextTexture.pixels.size() != 6U * 6U * 4U ||
        !runtime.copyTextTexture(4, leftTextTexture) || leftTextTexture.width != 8 ||
        leftTextTexture.height != 4 || leftTextTexture.pixels.size() != 8U * 4U * 4U ||
        !runtime.copyTextTexture(6, gridCenterTextTexture) ||
        !runtime.copyTextTexture(7, gridLeftTextTexture) ||
        !runtime.copyTextTexture(8, gridRightTextTexture) ||
        !runtime.copyTextTexture(9, repairedUnitTexture) ||
        !runtime.copyTextTexture(10, unchangedLegacyUnitTexture) ||
        !runtime.copyTextTexture(11, expectedUnitTexture) ||
        !runtime.copyTextTexture(12, expectedLegacyTexture) ||
        !runtime.copyTextTexture(13, gridBoxAlignedText) ||
        !runtime.copyTextTexture(14, gridPixelAlignedText) ||
        !runtime.copyTextTexture(15, leftBiasText) ||
        !runtime.copyTextTexture(16, rightBiasText) ||
        !runtime.copyTextTexture(17, fallbackGlyphText) ||
        !runtime.copyTextTexture(18, fallbackUnicodeText) ||
        !runtime.copyTextTexture(19, fallbackPunctuationText) ||
        !runtime.copyTextTexture(20, lowercaseFallbackText) ||
        !runtime.copyTextTexture(21, uppercaseFallbackText) ||
        !runtime.copyTextTexture(22, fallbackMultilineText) ||
        !runtime.copyTextTexture(23, lowercaseAuthoredText) ||
        !runtime.copyTextTexture(24, germanFallbackText) ||
        !runtime.copyTextTexture(25, questionFallbackText) ||
        !runtime.copyTextTexture(26, latin1FallbackText) ||
        !runtime.copyTextTexture(27, latin1QuestionFallbackText) ||
        !runtime.copyTextTexture(28, unitLatin1Text) ||
        !runtime.copyTextTexture(29, unitUnicodeFallbackText) ||
        !runtime.copyTextTexture(30, unitFallbackReferenceText) ||
        !runtime.copyTextTexture(31, spacingCenterText) ||
        !runtime.copyTextTexture(32, spacingRightText) ||
        !runtime.copyTextTexture(33, spacingLeftBiasText) ||
        !runtime.copyTextTexture(34, spacingRightBiasText) ||
        !runtime.copyTextTexture(35, typographicFallbackText) ||
        !runtime.copyTextTexture(36, typographicQuestionText) ||
        !runtime.copyTextTexture(37, currencyFallbackText) ||
        !runtime.copyTextTexture(38, currencyQuestionText) ||
        !runtime.copyTextTexture(39, mathFallbackText) ||
        !runtime.copyTextTexture(40, mathQuestionText) ||
        !runtime.copyTextTexture(41, arrowsFallbackText) ||
        !runtime.copyTextTexture(42, arrowsQuestionText) ||
        !runtime.copyTextTexture(43, dotFallbackText) ||
        !runtime.copyTextTexture(44, dotQuestionText) ||
        !runtime.copyTextTexture(45, unicodeSpacesText) ||
        !runtime.copyTextTexture(46, asciiSpacesText) ||
        !runtime.copyTextTexture(47, greekFallbackText) ||
        !runtime.copyTextTexture(48, greekQuestionText) ||
        !runtime.copyTextTexture(49, nordicFallbackText) ||
        !runtime.copyTextTexture(50, nordicQuestionText)) {
        std::cerr << "configured texture surfaces were not created\n";
        return 1;
    }
    if (repairedUnitTexture.pixels != expectedUnitTexture.pixels ||
        unchangedLegacyUnitTexture.pixels != expectedLegacyTexture.pixels) {
        std::cerr << "legacy replacement-byte Celsius suffix was not conditionally repaired\n";
        return 1;
    }
    ScriptRuntime::ScriptTextureSnapshot malformedTextTexture;
    if (runtime.copyTextTexture(51, malformedTextTexture) ||
        runtime.copyTextTexture(52, malformedTextTexture) ||
        runtime.copyTextTexture(59, malformedTextTexture)) {
        std::cerr << "text texture dimensions accepted numeric prefixes with trailing data\n";
        return 1;
    }
    ScriptRuntime::ScriptTextureSnapshot malformedColorText;
    ScriptRuntime::ScriptTextureSnapshot referenceWhiteText;
    ScriptRuntime::ScriptTextureSnapshot malformedAlignmentText;
    ScriptRuntime::ScriptTextureSnapshot referenceCenterText;
    ScriptRuntime::ScriptTextureSnapshot outOfRangeAlignmentText;
    ScriptRuntime::ScriptTextureSnapshot referenceCenterOutOfRangeText;
    ScriptRuntime::ScriptTextureSnapshot malformedColorModeText;
    ScriptRuntime::ScriptTextureSnapshot referenceColorModeText;
    ScriptRuntime::ScriptTextureSnapshot malformedPixelModeText;
    ScriptRuntime::ScriptTextureSnapshot referencePixelModeText;
    if (!runtime.copyTextTexture(53, malformedColorText) ||
        !runtime.copyTextTexture(54, referenceWhiteText) ||
        !runtime.copyTextTexture(55, malformedAlignmentText) ||
        !runtime.copyTextTexture(56, referenceCenterText) ||
        !runtime.copyTextTexture(57, outOfRangeAlignmentText) ||
        !runtime.copyTextTexture(58, referenceCenterOutOfRangeText) ||
        !runtime.copyTextTexture(60, malformedColorModeText) ||
        !runtime.copyTextTexture(61, referenceColorModeText) ||
        !runtime.copyTextTexture(64, malformedPixelModeText) ||
        !runtime.copyTextTexture(65, referencePixelModeText) ||
        malformedColorText.pixels != referenceWhiteText.pixels ||
        malformedAlignmentText.pixels != referenceCenterText.pixels ||
        outOfRangeAlignmentText.pixels != referenceCenterOutOfRangeText.pixels ||
        malformedColorModeText.pixels != referenceColorModeText.pixels ||
        malformedPixelModeText.pixels != referencePixelModeText.pixels) {
        std::cerr << "text texture color/alignment accepted numeric prefixes with trailing data\n";
        return 1;
    }
    if (unicodeSpacesText.pixels != asciiSpacesText.pixels) {
        std::cerr << "Unicode whitespace fallback did not render as blank cells\n";
        return 1;
    }
    if (unitLatin1Text.pixels[(3U * unitLatin1Text.width + 2U) * 4U + 3U] != 128 ||
        unitLatin1Text.pixels[(3U * unitLatin1Text.width + 3U) * 4U + 3U] != 0) {
        std::cerr << "authored UTF-8 Latin-1 glyph was not decoded as one OFT character: "
                  << static_cast<int>(
                         unitLatin1Text.pixels[(3U * unitLatin1Text.width + 2U) * 4U + 3U])
                  << ", "
                  << static_cast<int>(
                         unitLatin1Text.pixels[(3U * unitLatin1Text.width + 3U) * 4U + 3U])
                  << '\n';
        return 1;
    }
    if (unitUnicodeFallbackText.pixels != unitFallbackReferenceText.pixels) {
        std::cerr << "unsupported UTF-8 codepoint did not resolve as one fallback glyph\n";
        return 1;
    }
    if (spacingCenterText.pixels[(1U * 10U + 2U) * 4U + 3U] != 255 ||
        spacingCenterText.pixels[(1U * 10U + 3U) * 4U + 3U] != 255 ||
        spacingCenterText.pixels[(1U * 10U + 4U) * 4U + 3U] != 0 ||
        spacingCenterText.pixels[(1U * 10U + 5U) * 4U + 3U] != 128 ||
        spacingCenterText.pixels[(1U * 10U + 6U) * 4U + 3U] != 128 ||
        spacingRightText.pixels[(1U * 10U + 5U) * 4U + 3U] != 255 ||
        spacingRightText.pixels[(1U * 10U + 6U) * 4U + 3U] != 255 ||
        spacingRightText.pixels[(1U * 10U + 7U) * 4U + 3U] != 0 ||
        spacingRightText.pixels[(1U * 10U + 8U) * 4U + 3U] != 128 ||
        spacingRightText.pixels[(1U * 10U + 9U) * 4U + 3U] != 128 ||
        spacingLeftBiasText.pixels[(1U * 10U + 2U) * 4U + 3U] != 255 ||
        spacingLeftBiasText.pixels[(1U * 10U + 5U) * 4U + 3U] != 128 ||
        spacingRightBiasText.pixels[(1U * 10U + 3U) * 4U + 3U] != 255 ||
        spacingRightBiasText.pixels[(1U * 10U + 6U) * 4U + 3U] != 128) {
        std::cerr << "authored-font alignment did not include inter-glyph spacing\n";
        return 1;
    }
    ScriptRuntime::ScriptTextureSnapshot rejectedTexture;
    if (runtime.copyScriptTexture(4, rejectedTexture)) {
        std::cerr << "runtime accepted an oversized configured script texture\n";
        return 1;
    }

    const std::size_t pixel = (2U * 4U + 1U) * 4U;
    if (scriptTexture.pixels[pixel] != 255 || scriptTexture.pixels[pixel + 1] != 0 ||
        scriptTexture.pixels[pixel + 2] != 0 || scriptTexture.pixels[pixel + 3] != 255) {
        std::cerr << "script drawing did not produce the expected RGBA pixel\n";
        return 1;
    }

    bool hasRenderedPixel = false;
    for (std::size_t offset = 0; offset + 3 < textTexture.pixels.size(); offset += 4) {
        if (textTexture.pixels[offset + 3] != 0) {
            hasRenderedPixel = true;
            break;
        }
    }
    if (!hasRenderedPixel) {
        std::cerr << "text texture remained blank after string update\n";
        return 1;
    }
    bool hasSecondLine = false;
    for (int y = 8; y < textTexture.height; ++y) {
        for (int x = 0; x < textTexture.width; ++x) {
            const std::size_t offset = (static_cast<std::size_t>(y) * textTexture.width + x) * 4;
            if (textTexture.pixels[offset + 3] != 0) {
                hasSecondLine = true;
                break;
            }
        }
        if (hasSecondLine) {
            break;
        }
    }
    if (!hasSecondLine) {
        std::cerr << "multiline text did not render its second line\n";
        return 1;
    }
    const std::size_t firstBitmapPixel = 4U * 4U;
    const std::size_t secondBitmapPixel = (2U * 6U + 2U) * 4U;
    const std::size_t secondLineBPixel = (2U * 6U + 4U) * 4U;
    if (bitmapTextTexture.pixels[firstBitmapPixel] != 10 ||
        bitmapTextTexture.pixels[firstBitmapPixel + 1] != 20 ||
        bitmapTextTexture.pixels[firstBitmapPixel + 2] != 30 ||
        bitmapTextTexture.pixels[firstBitmapPixel + 3] != 255 ||
        bitmapTextTexture.pixels[secondBitmapPixel] != 10 ||
        bitmapTextTexture.pixels[secondBitmapPixel + 1] != 20 ||
        bitmapTextTexture.pixels[secondBitmapPixel + 2] != 30 ||
        bitmapTextTexture.pixels[secondBitmapPixel + 3] != 255 ||
        bitmapTextTexture.pixels[secondLineBPixel + 3] != 128) {
        std::cerr << "OFT glyph color, alpha, @ line layout, or right alignment was not applied\n";
        return 1;
    }
    const std::size_t centeredFontPixel = (3U * 6U + 2U) * 4U;
    const std::size_t centeredBlockColorPixel = (2U * 6U + 2U) * 4U;
    const std::size_t leftAlignedPixel = (1U * 8U) * 4U;
    const std::size_t gridCenterFirstPixel = (1U * 8U + 2U) * 4U;
    const std::size_t gridCenterSecondPixel = (1U * 8U + 4U) * 4U;
    const std::size_t gridLeftSecondPixel = (1U * 8U + 4U) * 4U;
    const std::size_t gridRightFirstPixel = (1U * 8U + 2U) * 4U;
    const std::size_t gridRightSecondPixel = (1U * 8U + 4U) * 4U;
    int blockMinX = 6;
    int blockMinY = 6;
    int blockMaxX = -1;
    int blockMaxY = -1;
    for (int y = 0; y < blockColorTextTexture.height; ++y) {
        for (int x = 0; x < blockColorTextTexture.width; ++x) {
            const std::size_t offset = (static_cast<std::size_t>(y) * 6U + x) * 4U;
            if (blockColorTextTexture.pixels[offset + 3] != 0) {
                blockMinX = std::min(blockMinX, x);
                blockMinY = std::min(blockMinY, y);
                blockMaxX = std::max(blockMaxX, x);
                blockMaxY = std::max(blockMaxY, y);
            }
        }
    }
    if (centerTextTexture.pixels[centeredFontPixel] != 255 ||
        centerTextTexture.pixels[centeredFontPixel + 1] != 255 ||
        centerTextTexture.pixels[centeredFontPixel + 2] != 255 ||
        centerTextTexture.pixels[centeredFontPixel + 3] != 255 ||
        centerTextTexture.pixels[3] != 0 ||
        blockColorTextTexture.pixels[centeredBlockColorPixel] != 1 ||
        blockColorTextTexture.pixels[centeredBlockColorPixel + 1] != 1 ||
        blockColorTextTexture.pixels[centeredBlockColorPixel + 2] != 1 ||
        blockColorTextTexture.pixels[centeredBlockColorPixel + 3] != 255 ||
        leftTextTexture.pixels[leftAlignedPixel] != 255 ||
        leftTextTexture.pixels[leftAlignedPixel + 1] != 255 ||
        leftTextTexture.pixels[leftAlignedPixel + 2] != 255 ||
        leftTextTexture.pixels[leftAlignedPixel + 3] != 255 ||
        leftTextTexture.pixels[(1U * 8U + 3U) * 4U + 3U] != 0 ||
        gridCenterTextTexture.pixels[gridCenterFirstPixel + 3] != 255 ||
        gridCenterTextTexture.pixels[gridCenterSecondPixel + 3] != 128 ||
        gridLeftTextTexture.pixels[gridLeftSecondPixel + 3] != 128 ||
        gridRightTextTexture.pixels[gridRightFirstPixel + 3] != 255 ||
        gridRightTextTexture.pixels[gridRightSecondPixel + 3] != 128 ||
        gridBoxAlignedText.pixels[(3U * 4U + 1U) * 4U + 3U] != 255 ||
        gridBoxAlignedText.pixels[(2U * 4U + 1U) * 4U + 3U] != 0 ||
        gridPixelAlignedText.pixels[(2U * 4U + 1U) * 4U + 3U] != 255 ||
        gridPixelAlignedText.pixels[(3U * 4U + 1U) * 4U + 3U] != 0 ||
        leftBiasText.pixels[(1U * 7U + 1U) * 4U + 3U] != 255 ||
        leftBiasText.pixels[(1U * 7U + 3U) * 4U + 3U] != 128 ||
        rightBiasText.pixels[(1U * 7U + 2U) * 4U + 3U] != 255 ||
        rightBiasText.pixels[(1U * 7U + 4U) * 4U + 3U] != 128 ||
        fallbackGlyphText.pixels[(1U * 6U) * 4U + 3U] != 128 ||
        fallbackUnicodeText.pixels[(3U * 24U + 1U) * 4U + 3U] != 255 ||
        fallbackUnicodeText.pixels[(3U * 24U + 3U) * 4U + 3U] != 255 ||
        fallbackUnicodeText.pixels[(4U * 24U + 6U) * 4U + 3U] != 255 ||
        fallbackUnicodeText.pixels[(10U * 24U + 1U) * 4U + 3U] != 255) {
        std::cerr << "OMSI text layout/color mismatch: centered font RGBA="
                  << static_cast<int>(centerTextTexture.pixels[centeredFontPixel]) << ','
                  << static_cast<int>(centerTextTexture.pixels[centeredFontPixel + 1]) << ','
                  << static_cast<int>(centerTextTexture.pixels[centeredFontPixel + 2]) << ','
                  << static_cast<int>(centerTextTexture.pixels[centeredFontPixel + 3])
                  << ", block-color RGBA="
                  << static_cast<int>(blockColorTextTexture.pixels[centeredBlockColorPixel]) << ','
                  << static_cast<int>(blockColorTextTexture.pixels[centeredBlockColorPixel + 1])
                  << ','
                  << static_cast<int>(blockColorTextTexture.pixels[centeredBlockColorPixel + 2])
                  << ','
                  << static_cast<int>(blockColorTextTexture.pixels[centeredBlockColorPixel + 3])
                  << ", block bounds=" << blockMinX << ',' << blockMinY << "-" << blockMaxX << ','
                  << blockMaxY << ", revision=" << blockColorTextTexture.revision << '\n';
        return 1;
    }
    std::size_t fallbackPunctuationPixelCount = 0;
    for (std::size_t offset = 3; offset < fallbackPunctuationText.pixels.size(); offset += 4) {
        fallbackPunctuationPixelCount += fallbackPunctuationText.pixels[offset] != 0 ? 1U : 0U;
    }
    if (fallbackPunctuationPixelCount != 33U) {
        std::cerr << "fallback text did not rasterize its common punctuation glyphs\n";
        return 1;
    }
    for (std::size_t glyphIndex = 0; glyphIndex < 10; ++glyphIndex) {
        bool differsFromQuestion = false;
        for (std::size_t row = 0; row < 7; ++row) {
            for (std::size_t column = 0; column < 5; ++column) {
                const std::size_t pixelOffset =
                    (row * germanFallbackText.width + glyphIndex * 6 + column) * 4 + 3;
                differsFromQuestion =
                    differsFromQuestion ||
                    germanFallbackText.pixels[pixelOffset] !=
                        questionFallbackText.pixels[pixelOffset];
            }
        }
        for (std::size_t glyphIndex = 0; glyphIndex < 3; ++glyphIndex) {
            bool differsFromQuestion = false;
            for (std::size_t row = 0; row < 7; ++row) {
                for (std::size_t column = 0; column < 5; ++column) {
                    const std::size_t pixelOffset =
                        (row * mathFallbackText.width + 1U + glyphIndex * 6U + column) * 4U + 3U;
                    differsFromQuestion =
                        differsFromQuestion ||
                        mathFallbackText.pixels[pixelOffset] != mathQuestionText.pixels[pixelOffset];
                }
            }
            for (std::size_t glyphIndex = 0; glyphIndex < 4; ++glyphIndex) {
                bool differsFromQuestion = false;
                for (std::size_t row = 0; row < 7; ++row) {
                    for (std::size_t column = 0; column < 5; ++column) {
                        const std::size_t pixelOffset =
                            (row * arrowsFallbackText.width + 1U + glyphIndex * 6U + column) * 4U + 3U;
                        differsFromQuestion =
                            differsFromQuestion ||
                            arrowsFallbackText.pixels[pixelOffset] !=
                                arrowsQuestionText.pixels[pixelOffset];
                    }
                }
                for (std::size_t glyphIndex = 0; glyphIndex < 2; ++glyphIndex) {
                    bool differsFromQuestion = false;
                    for (std::size_t row = 0; row < 7; ++row) {
                        for (std::size_t column = 0; column < 5; ++column) {
                            const std::size_t pixelOffset =
                                (row * dotFallbackText.width + 1U + glyphIndex * 6U + column) * 4U + 3U;
                            differsFromQuestion =
                                differsFromQuestion ||
                                dotFallbackText.pixels[pixelOffset] != dotQuestionText.pixels[pixelOffset];
                        }
                    }
                    for (std::size_t glyphIndex = 0; glyphIndex < 5; ++glyphIndex) {
                        bool differsFromQuestion = false;
                        for (std::size_t row = 0; row < 7; ++row) {
                            for (std::size_t column = 0; column < 5; ++column) {
                                const std::size_t pixelOffset =
                                    (row * greekFallbackText.width + 1U + glyphIndex * 6U + column) * 4U + 3U;
                                differsFromQuestion =
                                    differsFromQuestion ||
                                    greekFallbackText.pixels[pixelOffset] != greekQuestionText.pixels[pixelOffset];
                            }
                        }
                        for (std::size_t glyphIndex = 0; glyphIndex < 8; ++glyphIndex) {
                            bool differsFromQuestion = false;
                            for (std::size_t row = 0; row < 7; ++row) {
                                for (std::size_t column = 0; column < 5; ++column) {
                                    const std::size_t pixelOffset =
                                        (row * nordicFallbackText.width + 1U + glyphIndex * 6U + column) * 4U + 3U;
                                    differsFromQuestion =
                                        differsFromQuestion ||
                                        nordicFallbackText.pixels[pixelOffset] != nordicQuestionText.pixels[pixelOffset];
                                }
                            }
                            if (!differsFromQuestion) {
                                std::cerr << "fallback text substituted a Nordic/Latin-1 ligature with '?'\n";
                                return 1;
                            }
                        }
                        if (!differsFromQuestion) {
                            std::cerr << "fallback text substituted a Greek glyph with '?'\n";
                            return 1;
                        }
                    }
                    if (!differsFromQuestion) {
                        std::cerr << "fallback text substituted a punctuation dot with '?'\n";
                        return 1;
                    }
                }
                if (!differsFromQuestion) {
                    std::cerr << "fallback text substituted a directional arrow with '?'\n";
                    return 1;
                }
            }
            if (!differsFromQuestion) {
                std::cerr << "fallback text substituted a mathematical glyph with '?'\n";
                return 1;
            }
        }
        if (!differsFromQuestion) {
            std::cerr << "fallback text substituted a German glyph with '?'\n";
            return 1;
        }
    }
    for (std::size_t glyphIndex = 0; glyphIndex < 18; ++glyphIndex) {
        bool differsFromQuestion = false;
        for (std::size_t row = 0; row < 7; ++row) {
            for (std::size_t column = 0; column < 5; ++column) {
                const std::size_t pixelOffset =
                    (row * latin1FallbackText.width + glyphIndex * 6 + column) * 4 + 3;
                differsFromQuestion =
                    differsFromQuestion ||
                    latin1FallbackText.pixels[pixelOffset] !=
                        latin1QuestionFallbackText.pixels[pixelOffset];
            }
        }
        if (!differsFromQuestion) {
            std::cerr << "fallback text substituted an accented Latin-1 glyph with '?'\n";
            return 1;
        }
    }
    for (std::size_t glyphIndex = 0; glyphIndex < 10; ++glyphIndex) {
        bool differsFromQuestion = false;
        for (std::size_t row = 0; row < 7; ++row) {
            for (std::size_t column = 0; column < 5; ++column) {
                const std::size_t pixelOffset =
                    (row * typographicFallbackText.width + 1U + glyphIndex * 6U + column) * 4U +
                    3U;
                differsFromQuestion =
                    differsFromQuestion ||
                    typographicFallbackText.pixels[pixelOffset] !=
                        typographicQuestionText.pixels[pixelOffset];
            }
        }
        if (!differsFromQuestion) {
            std::cerr << "fallback text substituted typographic punctuation with '?'\n";
            return 1;
        }
    }
    for (std::size_t glyphIndex = 0; glyphIndex < 10; ++glyphIndex) {
        bool differsFromQuestion = false;
        for (std::size_t row = 0; row < 7; ++row) {
            for (std::size_t column = 0; column < 5; ++column) {
                const std::size_t pixelOffset =
                    (row * currencyFallbackText.width + 1U + glyphIndex * 6U + column) * 4U + 3U;
                differsFromQuestion =
                    differsFromQuestion ||
                    currencyFallbackText.pixels[pixelOffset] !=
                        currencyQuestionText.pixels[pixelOffset];
            }
        }
        if (!differsFromQuestion) {
            std::cerr << "fallback text substituted a currency/legal glyph with '?'\n";
            return 1;
        }
    }
    if (lowercaseFallbackText.pixels == uppercaseFallbackText.pixels ||
        lowercaseFallbackText.pixels[(3U * 160U + 1U) * 4U + 3U] != 255 ||
        lowercaseFallbackText.pixels[(1U * 160U + 1U) * 4U + 3U] != 0 ||
        uppercaseFallbackText.pixels[(1U * 160U + 1U) * 4U + 3U] != 255) {
        std::cerr << "fallback text did not preserve lowercase glyph shapes\n";
        return 1;
    }
    if (fallbackMultilineText.pixels[(2U * 24U + 1U) * 4U + 3U] != 255 ||
        fallbackMultilineText.pixels[(3U * 24U + 1U) * 4U + 3U] != 0 ||
        fallbackMultilineText.pixels[(10U * 24U + 1U) * 4U + 3U] != 255) {
        std::cerr << "fallback multiline text did not preserve glyph row and line spacing: "
                  << static_cast<int>(fallbackMultilineText.pixels[(2U * 24U + 1U) * 4U + 3U])
                  << ','
                  << static_cast<int>(fallbackMultilineText.pixels[(3U * 24U + 1U) * 4U + 3U])
                  << ','
                  << static_cast<int>(fallbackMultilineText.pixels[(10U * 24U + 1U) * 4U + 3U])
                  << '\n';
        return 1;
    }
    if (lowercaseAuthoredText.pixels != centerTextTexture.pixels) {
        std::cerr << "missing OFT lowercase glyph did not retry its uppercase form\n";
        return 1;
    }
    const std::uint64_t firstTextRevision = textTexture.revision;
    variables.set("refresh_strings", 1.0);
    runtime.update(false);
    ScriptRuntime::ScriptTextureSnapshot unchangedTextTexture;
    if (!runtime.copyTextTexture(0, unchangedTextTexture) ||
        unchangedTextTexture.revision != firstTextRevision ||
        variables.get("refresh_strings") != 0.0) {
        std::cerr << "unchanged text was regenerated or refresh flag was not consumed\n";
        return 1;
    }

    const std::uint64_t firstBitmapRevision = bitmapTextTexture.revision;
    variables.setString("bitmapdisplay", "01");
    runtime.update(false);
    if (!runtime.copyTextTexture(1, bitmapTextTexture) ||
        bitmapTextTexture.revision != firstBitmapRevision) {
        std::cerr << "text changed without refresh_strings being incorrectly rasterized\n";
        return 1;
    }
    variables.set("refresh_strings", 1.0);
    runtime.update(false);
    ScriptRuntime::ScriptTextureSnapshot digits;
    if (!runtime.copyTextTexture(1, digits) || digits.pixels[(1U * 6U + 2U) * 4U + 3U] != 255 ||
        digits.pixels[(1U * 6U + 4U) * 4U + 3U] != 128 || digits.revision == firstBitmapRevision) {
        std::cerr << "OFT digit glyphs, right alignment, or refresh gating failed\n";
        return 1;
    }
    variables.setString("display", "WORLD 456");
    runtime.update(false);
    ScriptRuntime::ScriptTextureSnapshot changedTextTexture;
    if (!runtime.copyTextTexture(0, changedTextTexture) ||
        changedTextTexture.revision != firstTextRevision) {
        std::cerr << "text changed without refresh_strings being incorrectly rasterized\n";
        return 1;
    }
    variables.set("refresh_strings", 1.0);
    runtime.update(false);
    if (!runtime.copyTextTexture(0, changedTextTexture) ||
        changedTextTexture.revision == firstTextRevision) {
        std::cerr << "changed text texture was not regenerated\n";
        return 1;
    }
    const auto probeFontPath = fonts / "probe.oft";
    const auto fontDirectoryTime = std::filesystem::last_write_time(fonts);
    const auto originalFontSize = std::filesystem::file_size(probeFontPath);
    std::ofstream updatedProbeFont(probeFontPath, std::ios::trunc);
    updatedProbeFont << "[newfont]\nProbeFont\nprobe.bmp\nprobe_alpha.bmp\n2\n0\n"
                        "[char]\nA\n0\n2\n0\n[char]\nB\n2\n4\n0\n"
                        "[char]\n0\n2\n4\n0\n[char]\n1\n2\n4\n0\n"
                        "[char]\n?\n2\n4\n0\n";
    updatedProbeFont.close();
    std::error_code fontUpdateError;
    std::filesystem::last_write_time(
        probeFontPath, std::filesystem::file_time_type::clock::now() +
                           std::chrono::seconds(300),
        fontUpdateError);
    if (fontUpdateError || std::filesystem::file_size(probeFontPath) != originalFontSize) {
        std::cerr << "could not prepare same-size OFT glyph update\n";
        return 1;
    }
    std::filesystem::last_write_time(fonts, fontDirectoryTime, fontUpdateError);
    if (fontUpdateError) {
        std::cerr << "could not preserve font directory timestamp during glyph update\n";
        return 1;
    }
    variables.set("refresh_strings", 1.0);
    runtime.update(false);
    ScriptRuntime::ScriptTextureSnapshot reloadedDigits;
    if (!runtime.copyTextTexture(1, reloadedDigits) ||
        reloadedDigits.revision <= digits.revision ||
        reloadedDigits.pixels[(1U * 6U + 2U) * 4U + 3U] != 128) {
        std::cerr << "unchanged text texture did not reload its edited OFT glyph\n";
        return 1;
    }
    const auto rewriteFontImage = [&](const std::filesystem::path& path,
                                      const std::vector<std::uint8_t>& pixels, int timestampOffset) {
        const auto originalSize = std::filesystem::file_size(path);
        writeTestBmp(path, 4, 2, pixels);
        if (std::filesystem::file_size(path) != originalSize) {
            return false;
        }
        std::error_code imageUpdateError;
        std::filesystem::last_write_time(
            path, std::filesystem::file_time_type::clock::now() +
                      std::chrono::seconds(timestampOffset),
            imageUpdateError);
        if (imageUpdateError) {
            return false;
        }
        std::filesystem::last_write_time(fonts, fontDirectoryTime, imageUpdateError);
        return !imageUpdateError;
    };
    const std::uint64_t beforeAlphaReload = reloadedDigits.revision;
    if (!rewriteFontImage(fonts / "probe_alpha.bmp",
                          {255, 255, 64, 64, 255, 255, 64, 64}, 360)) {
        std::cerr << "could not prepare same-size font alpha image update\n";
        return 1;
    }
    variables.set("refresh_strings", 1.0);
    runtime.update(false);
    if (!runtime.copyTextTexture(1, reloadedDigits) ||
        reloadedDigits.revision <= beforeAlphaReload ||
        reloadedDigits.pixels[(1U * 6U + 2U) * 4U + 3U] != 64) {
        std::cerr << "unchanged text texture did not reload its edited font alpha image\n";
        return 1;
    }
    ScriptRuntime::ScriptTextureSnapshot reloadedColorText;
    if (!rewriteFontImage(fonts / "probe.bmp", {200, 200, 90, 90, 200, 200, 90, 90}, 420)) {
        std::cerr << "could not prepare same-size font color image update\n";
        return 1;
    }
    variables.set("refresh_strings", 1.0);
    runtime.update(false);
    if (!runtime.copyTextTexture(2, reloadedColorText) ||
        reloadedColorText.pixels[centeredFontPixel] != 200 ||
        reloadedColorText.pixels[centeredFontPixel + 1] != 200 ||
        reloadedColorText.pixels[centeredFontPixel + 2] != 200 ||
        reloadedColorText.pixels[centeredFontPixel + 3] != 255) {
        std::cerr << "unchanged text texture did not reload its edited font color image\n";
        return 1;
    }
    openbus::probe::removeProbeDirectory(root);
    return 0;
}
