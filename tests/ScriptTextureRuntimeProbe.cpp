#include "ScriptRuntime.h"
#include "Logger.h"
#include "osc/OscConverter.h"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>

Logger gameLog = Logger("Game");

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus_script_texture_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    const std::filesystem::path scriptPath = root / "probe.osc";
    std::ofstream source(scriptPath);
    source << "{InIt}\n"
              "(L.L.MixedNumeric)\n"
              "(S.L.MixedNumericOut)\n"
              "(L.$.MixedString)\n"
              "(S.$.MixedStringOut)\n"
              "(M.L.MixedMacro)\n"
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
                 "function trigger_kw_m_enginestart()\n"
                 "set_local_var(\"key_binding_seen\", 1)\n"
                 "end\n"
                 "function trigger_kw_m_enginestart_off()\n"
                 "set_local_var(\"key_binding_release_seen\", 1)\n"
                 "end\n";
    generated.close();

    VehicleConfig configuration;
    configuration.sourcePath = root / "probe.bus";
    configuration.scripts.push_back(scriptPath.filename().string());
    Variables variables(ScriptObjectKind::Vehicle);
    SimulationState simulation;
    ScriptRuntime runtime(configuration, variables, simulation);

    simulation.sharedVariables().updateFrame(0.016, 1.0, 900.0, 700.0);
    if (simulation.sharedVariables().get("mouse_x") != 900.0 ||
        simulation.sharedVariables().get("mouse_y") != 700.0) {
        std::cerr << "system cursor frame update was not applied\n";
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
        variables.get("mouse_release_seen") != 1.0 ||
        variables.get("key_binding_seen") != 1.0 ||
        variables.get("key_binding_release_seen") != 1.0 ||
        variables.get("mouse_drag_x") != 3.0 || variables.get("mouse_drag_y") != -2.0 ||
        variables.get("mouse_cursor_x") != 120.0 || variables.get("mouse_cursor_y") != 80.0 ||
        simulation.sharedVariables().get("mouse_x") != 3.0 ||
        simulation.sharedVariables().get("mouse_y") != -2.0) {
        std::cerr << "input events were not exposed to the script state\n";
        return 1;
    }

    runtime.configureScriptTextures({ModelScriptTexture{3, 4, 5, {"4", "5"}}});
    runtime.configureTextTextures({
        ModelTextTexture{0, false, {"display", "probe-font", "32", "16", "0", "255", "0", "0"}}});
    variables.setString("display", "HELLO\n123");
    runtime.initialize();
    runtime.update(false);

    ScriptRuntime::ScriptTextureSnapshot scriptTexture;
    ScriptRuntime::ScriptTextureSnapshot textTexture;
    if (!runtime.copyScriptTexture(3, scriptTexture) || scriptTexture.width != 4 ||
        scriptTexture.height != 5 || scriptTexture.pixels.size() != 4U * 5U * 4U ||
        !runtime.copyTextTexture(0, textTexture) || textTexture.width != 32 ||
        textTexture.height != 16 || textTexture.pixels.size() != 32U * 16U * 4U) {
        std::cerr << "configured texture surfaces were not created\n";
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
    const std::uint64_t firstTextRevision = textTexture.revision;
    runtime.update(false);
    ScriptRuntime::ScriptTextureSnapshot unchangedTextTexture;
    if (!runtime.copyTextTexture(0, unchangedTextTexture) ||
        unchangedTextTexture.revision != firstTextRevision) {
        std::cerr << "unchanged text texture was regenerated\n";
        return 1;
    }
    variables.setString("display", "WORLD 456");
    runtime.update(false);
    ScriptRuntime::ScriptTextureSnapshot changedTextTexture;
    if (!runtime.copyTextTexture(0, changedTextTexture) ||
        changedTextTexture.revision == firstTextRevision) {
        std::cerr << "changed text texture was not regenerated\n";
        return 1;
    }
    std::filesystem::remove_all(root);
    return 0;
}
