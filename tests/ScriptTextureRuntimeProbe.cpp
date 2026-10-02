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
    source << "[probe]\n";
    source.close();
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
    variables.setString("display", "HELLO 123");
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
