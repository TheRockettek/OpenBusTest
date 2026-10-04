#include "ScriptRuntime.h"
#include "Logger.h"
#include "osc/OscConverter.h"

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
#include <vector>

Logger gameLog = Logger("Game");

bool setScriptBackend(const char* backend) {
#ifdef _WIN32
    return _putenv_s("OPENBUS_SCRIPT_BACKEND", backend == nullptr ? "" : backend) == 0;
#else
    return backend == nullptr ? unsetenv("OPENBUS_SCRIPT_BACKEND") == 0
                              : setenv("OPENBUS_SCRIPT_BACKEND", backend, 1) == 0;
#endif
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

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus_script_texture_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
        const std::filesystem::path fonts = root / "Fonts";
        std::filesystem::create_directories(fonts);
        writeTestBmp(fonts / "probe.bmp", 4, 2, {255, 255, 80, 80, 255, 255, 80, 80});
        writeTestBmp(fonts / "probe_alpha.bmp", 4, 2,
             {255, 255, 128, 128, 255, 255, 128, 128});
        std::ofstream font(fonts / "probe.oft");
        font << "[newfont]\nProbeFont\nprobe.bmp\nprobe_alpha.bmp\n2\n0\n"
            "[char]\nA\n0\n2\n0\n"
            "[char]\nB\n2\n4\n0\n";
            font << "[char]\n0\n0\n2\n0\n"
                "[char]\n1\n2\n4\n0\n";
        font.close();
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
    {
        Variables fallbackVariables(ScriptObjectKind::Vehicle);
        SimulationState fallbackSimulation;
        ScriptRuntime fallbackRuntime(fallbackConfiguration, fallbackVariables,
                                      fallbackSimulation);
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
    if (!setScriptBackend(nullptr)) {
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
    runtime.invokeMouseDrag("TicketerGimble", 3.0, -2.0, 120.0, 80.0);
    if (variables.get("ticketer_drag_seen") != 1.0 ||
        variables.get("mouse_drag_x") != 3.0 || variables.get("mouse_drag_y") != -2.0 ||
        simulation.sharedVariables().get("mouse_x") != 3.0 ||
        simulation.sharedVariables().get("mouse_y") != -2.0) {
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

    runtime.configureScriptTextures({ModelScriptTexture{3, 4, 5, {"4", "5"}},
                                     ModelScriptTexture{4, 4097, 1, {"4097", "1"}}});
    runtime.configureTextTextures({
        ModelTextTexture{0, false, {"display", "probe-font", "32", "16", "0", "255", "0", "0"}},
        ModelTextTexture{1, true, {"bitmapdisplay", "ProbeFont", "6", "4", "0", "10", "20", "30", "2"}},
        ModelTextTexture{2, true, {"centerdisplay", "ProbeFont", "6", "8", "1", "10", "20", "30", "0"}},
        ModelTextTexture{3, true, {"blockcolordisplay", "ProbeFont", "6", "6", "0", "1", "1", "1", "0"}},
        ModelTextTexture{4, true, {"leftdisplay", "ProbeFont", "8", "4", "1", "10", "20", "30", "1"}},
        ModelTextTexture{5, false, {"ident", "probe-font", "32", "16", "0", "255", "255", "255"}},
        ModelTextTexture{6, true, {"gridcenterdisplay", "ProbeFont", "8", "4", "0", "10", "20", "30", "3", "1"}},
        ModelTextTexture{7, true, {"gridleftdisplay", "ProbeFont", "8", "4", "0", "10", "20", "30", "4", "1"}},
        ModelTextTexture{8, true, {"gridrightdisplay", "ProbeFont", "8", "4", "0", "10", "20", "30", "5", "0"}}});
    variables.setString("display", "HELLO\n123");
    variables.setString("bitmapdisplay", "A@AB");
    variables.setString("centerdisplay", "A");
    variables.setString("blockcolordisplay", "A");
    variables.setString("leftdisplay", "A");
    variables.setString("gridcenterdisplay", "AB");
    variables.setString("gridleftdisplay", "AB");
    variables.setString("gridrightdisplay", "AB");
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
    for (std::size_t offset = 3;
         copiedInitialPlateText && offset < initialPlateText.pixels.size(); offset += 4) {
        hasInitialPlateText = hasInitialPlateText || initialPlateText.pixels[offset] != 0;
    }
    if (!copiedInitialText || !hasInitialText ||
        initialText.pixels.size() != 32U * 16U * 4U || !copiedInitialPlateText ||
        !hasInitialPlateText || initialPlateText.pixels.size() != 32U * 16U * 4U ||
        variables.getString("ident") != "AB12 CDE" ||
        variables.getString("number") != "Fleet 01") {
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
        !runtime.copyTextTexture(8, gridRightTextTexture)) {
        std::cerr << "configured texture surfaces were not created\n";
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
    const std::size_t gridCenterFirstPixel = (1U * 8U + 1U) * 4U;
    const std::size_t gridCenterSecondPixel = (1U * 8U + 4U) * 4U;
    const std::size_t gridLeftSecondPixel = (1U * 8U + 3U) * 4U;
    const std::size_t gridRightFirstPixel = (1U * 8U + 4U) * 4U;
    const std::size_t gridRightSecondPixel = (1U * 8U + 6U) * 4U;
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
        centerTextTexture.pixels[3] != 0 || blockColorTextTexture.pixels[centeredBlockColorPixel] != 1 ||
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
        gridRightTextTexture.pixels[gridRightSecondPixel + 3] != 128) {
        std::cerr << "OMSI text layout/color mismatch: centered font RGBA="
                  << static_cast<int>(centerTextTexture.pixels[centeredFontPixel]) << ','
                  << static_cast<int>(centerTextTexture.pixels[centeredFontPixel + 1]) << ','
                  << static_cast<int>(centerTextTexture.pixels[centeredFontPixel + 2]) << ','
                  << static_cast<int>(centerTextTexture.pixels[centeredFontPixel + 3])
                  << ", block-color RGBA="
                  << static_cast<int>(blockColorTextTexture.pixels[centeredBlockColorPixel])
                  << ','
                  << static_cast<int>(blockColorTextTexture.pixels[centeredBlockColorPixel + 1])
                  << ','
                  << static_cast<int>(blockColorTextTexture.pixels[centeredBlockColorPixel + 2])
                  << ','
                  << static_cast<int>(blockColorTextTexture.pixels[centeredBlockColorPixel + 3])
                  << ", block bounds=" << blockMinX << ',' << blockMinY << "-" << blockMaxX
                  << ',' << blockMaxY << ", revision=" << blockColorTextTexture.revision << '\n';
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
    if (!runtime.copyTextTexture(1, digits) ||
        digits.pixels[(1U * 6U + 2U) * 4U + 3U] != 255 ||
        digits.pixels[(1U * 6U + 4U) * 4U + 3U] != 128 ||
        digits.revision == firstBitmapRevision) {
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
    std::filesystem::remove_all(root);
    return 0;
}
