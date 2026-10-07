#include "Logger.h"
#include "MouseInteraction.h"
#include "ScriptRuntime.h"
#include "VehicleInputBindings.h"
#include "osc/OscConverter.h"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

Logger gameLog = Logger("Game");

namespace {

bool selectBackend(const char* backend) {
#ifdef _WIN32
    return _putenv_s("OPENBUS_SCRIPT_BACKEND", backend) == 0;
#else
    return setenv("OPENBUS_SCRIPT_BACKEND", backend, 1) == 0;
#endif
}

bool check(const VehicleConfig& configuration, const char* backend) {
    if (!selectBackend(backend)) {
        return false;
    }
    Variables local(ScriptObjectKind::Vehicle);
    SimulationState simulation;
    auto& system = simulation.sharedVariables();
    system.updateFrame(0.125F, 10.0F, 100.0F, 200.0F);
    ScriptRuntime runtime(configuration, local, simulation);
    bool previous = false;
    constexpr std::array<bool, 5> states = {false, true, true, false, false};
    for (bool pressed : states) {
        const bool changed = pressed != previous;
        if (openbus::input::shouldDispatchKeyBinding(0, pressed, previous)) {
            runtime.invokeKeyBinding("edge", pressed);
        }
        if (openbus::input::shouldDispatchKeyBinding(openbus::input::kBindingHeld, pressed,
                                                    previous)) {
            runtime.invokeKeyBinding("held", pressed, changed);
        }
        previous = pressed;
    }
    if (local.get("edge_press") != 1 || local.get("edge_release") != 1 ||
        local.get("held_press") != 2 || local.get("held_release") != 3 ||
        local.get("key_time") != 0.125F || local.get("key_mouse") != 0) {
        std::cerr << backend << ": edge/held delivery or current input values mismatch\n";
        return false;
    }

    openbus::input::MouseInteraction mouse;
    const auto press = [&runtime](const std::string& event) { runtime.invokeMouseEvent(event); };
    const auto release = [&runtime](const std::string& event) {
        runtime.invokeMouseRelease(event);
    };
    const auto drag = [&runtime](const std::string& event, double dx, double dy, double x, double y) {
        runtime.invokeMouseDrag(event, dx, dy, x, y);
    };
    mouse.press("control", 100, 200, press, release);
    mouse.update(true, true, 100, 200, drag, release);
    mouse.update(true, true, 100, 200, drag, release);
    if (local.get("press_count") != 1 || local.get("drag_count") != 2 ||
        local.get("drag_dx") != 0 || local.get("drag_dy") != 0 ||
        local.get("drag_time") != 0.125F) {
        std::cerr << backend << ": stationary holding must dispatch zero-motion _drag\n";
        return false;
    }
    mouse.update(true, true, 103, 198, drag, release);
    if (local.get("drag_count") != 3 || local.get("drag_dx") != 3 ||
        local.get("drag_dy") != -2 || system.get("mouse_x") != 0 ||
        system.get("mouse_y") != 0 || system.get("mouse_cursor_x") != 100 ||
        local.get("mouse_cursor_x") != 103 || local.get("mouse_cursor_y") != 198) {
        std::cerr << backend << ": drag direction, cursor separation or scoped motion mismatch\n";
        return false;
    }
    mouse.update(true, false, 900, 700, drag, release);
    mouse.update(true, false, 900, 700, drag, release);
    if (local.get("release_count") != 1 || local.get("drag_count") != 3 ||
        local.get("release_mouse") != 0) {
        std::cerr << backend << ": capture must release once, including outside its mesh\n";
        return false;
    }
    mouse.press("control", 10, 20, press, release);
    mouse.update(false, true, 10, 20, drag, release);
    mouse.update(false, true, 10, 20, drag, release);
    mouse.press("control", 10, 20, press, release);
    mouse.press("control", 10, 20, press, release);
    mouse.cancel(release);
    mouse.cancel(release);
    if (local.get("press_count") != 4 || local.get("release_count") != 4 ||
        local.get("drag_count") != 3) {
        std::cerr << backend << ": mode switch/replaced capture cancellation mismatch\n";
        return false;
    }
    // Missing optional handlers and script failures must not leak motion either.
    system.set("mouse_x", 7);
    system.set("mouse_y", 8);
    runtime.invokeMouseDrag("missing", 1, 2, 3, 4);
    if (system.get("mouse_x") != 7 || system.get("mouse_y") != 8) {
        return false;
    }
    if (std::string(backend) == "lua") {
        const auto errors = runtime.errors().size();
        runtime.invokeMouseDrag("failed", 1, 2, 3, 4);
        if (runtime.errors().size() != errors + 1 || system.get("mouse_x") != 7 ||
            system.get("mouse_y") != 8) {
            std::cerr << "Lua error did not restore handler-scoped motion\n";
            return false;
        }
    } else if (!runtime.valid()) {
        return false;
    }
    return true;
}

} // namespace

int main() {
    const auto root = std::filesystem::temp_directory_path() / "openbus_script_input_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    const auto script = root / "input.osc";
    {
        std::ofstream source(script);
        source << "{trigger:edge}\n(L.L.edge_press) 1 + (S.L.edge_press)\n{end}\n"
                  "{trigger:edge_off}\n(L.L.edge_release) 1 + (S.L.edge_release)\n{end}\n"
                  "{trigger:held}\n(L.L.held_press) 1 + (S.L.held_press)\n"
                  "(L.S.Timegap) (S.L.key_time)\n(L.S.mouse_x) (S.L.key_mouse)\n{end}\n"
                  "{trigger:held_off}\n(L.L.held_release) 1 + (S.L.held_release)\n{end}\n"
                  "{trigger:control}\n(L.L.press_count) 1 + (S.L.press_count)\n{end}\n"
                  "{trigger:control_drag}\n(L.L.drag_count) 1 + (S.L.drag_count)\n"
                  "(L.S.mouse_x) (S.L.drag_dx)\n(L.S.mouse_y) (S.L.drag_dy)\n"
                  "(L.S.Timegap) (S.L.drag_time)\n{end}\n"
                  "{trigger:control_off}\n(L.L.release_count) 1 + (S.L.release_count)\n"
                  "(L.S.mouse_y) (S.L.release_mouse)\n{end}\n";
    }
    std::string error;
    if (!convertOscToLua(script, generatedLuaPath(script), error)) {
        std::cerr << error << '\n';
        return 1;
    }
    {
        std::ofstream lua(generatedLuaPath(script), std::ios::app);
        lua << "\nfunction trigger_failed_drag() error('expected probe failure') end\n";
    }
    VehicleConfig configuration;
    configuration.sourcePath = root / "input.bus";
    configuration.scripts = {script.filename().string()};
    const bool nativePassed = check(configuration, "native");
    const bool luaPassed = check(configuration, "lua");
    std::filesystem::remove_all(root);
    return nativePassed && luaPassed ? 0 : 1;
}