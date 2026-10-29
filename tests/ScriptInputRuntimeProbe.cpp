#include "core/Logger.h"
#include "input/MouseInteraction.h"
#include "scripting/ScriptRuntime.h"
#include "input/VehicleInputBindings.h"
#include "osc/OscConverter.h"
#include "ProbeTempDirectory.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <tuple>

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
    openbus::scripting::Vehicle local;
    SimulationState simulation;
    auto& system = simulation.sharedVariables();
    system.updateFrame(0.125F, 10.0F, 100.0F, 200.0F);
    system.set("ticket_value_2", 7.5F);
    system.setString("ticket_name_2", "Network Day Pass");
    system.set("cash_coin_value_1", 0.05F);
    local.set("ticket_value_0", 2.5F);
    local.set("ticket_value_1", std::numeric_limits<float>::infinity());
    local.setString("ticket_name_0", "Single Ride");
    local.set("cash_coin_value_0", 2.5F);
    local.set("cash_coin_value_2", std::numeric_limits<float>::infinity());
    local.set("human_count_on_seat_2", 1.0F);
    local.set("human_count_on_path_link_7", 3.0F);
    local.setString("tt_line_string", "42");
    local.setString("tt_busstop_name", "Central Station");
    ScriptRuntime runtime(configuration, local, simulation);
    system.configureSimulationClock(2024, 12, 31, 86399.0);
    runtime.invokeEntryPoint("trigger_system_environment");
    if (local.get("environment_dayofyear") != 366.0F) {
        std::cerr << backend << ": leap-year day was not published to scripts\n";
        return false;
    }
    system.updateSimulationClock(1.0);
    runtime.invokeEntryPoint("trigger_system_environment");
    if (local.get("environment_dayofyear") != 1.0F) {
        std::cerr << backend << ": new-year day was not published to scripts\n";
        return false;
    }
    for (const auto& [type, rate, available] :
         {std::tuple{1, 0.75, true}, std::tuple{2, 0.25, false},
          std::tuple{0, 0.0, true}}) {
        system.setPrecipitation(type, rate);
        system.setSoundAvailable(available);
        local.updateEnvironment(system);
        runtime.invokeEntryPoint("trigger_system_environment");
        if (local.get("environment_preciptype") != static_cast<float>(type) ||
            local.get("environment_preciprate") != static_cast<float>(rate) ||
            local.get("local_preciptype") != static_cast<float>(type) ||
            local.get("local_preciprate") != static_cast<float>(rate) ||
            local.get("environment_nosound") != (available ? 0.0F : 1.0F)) {
            std::cerr << backend << ": environment system variables were not published\n";
            return false;
        }
    }
    runtime.invokeEntryPoint("trigger_passenger_ticketing");
    if (local.get("ticket_value_result") != 2.5F ||
        local.getString("ticket_name_result") != "Single Ride" ||
        local.get("shared_ticket_value_result") != 7.5F ||
        local.getString("shared_ticket_name_result") != "Network Day Pass" ||
        std::abs(local.get("cash_dispensed_total") - 2.55F) > 1.0e-5F ||
        local.get("cash_last_coin_value") != 0.05F ||
        local.get("cash_dispensed_coin_count") != 2.0F ||
        local.get("cash_last_coin_index") != 1.0F ||
        local.get("cash_coin_dispensed_count_0") != 1.0F ||
        local.get("cash_coin_dispensed_count_1") != 1.0F || local.get("seat_human_count") != 1.0F ||
        local.get("path_link_human_count") != 3.0F ||
        local.getString("timetable_line_result") != "42" ||
        local.getString("timetable_destination_result") != "Central Station" ||
        local.get("nonfinite_ticket_value_result") != 0.0F ||
        local.get("invalid_ticket_value_result") != 0.0F ||
        !local.getString("invalid_ticket_name_result").empty()) {
        std::cerr << backend << ": ticket, timetable macro or change-coin state mismatch"
                  << " sharedTicket=" << local.get("shared_ticket_value_result")
                  << " sharedName=" << local.getString("shared_ticket_name_result")
                  << " cash=" << local.get("cash_dispensed_total")
                  << " lastCoin=" << local.get("cash_last_coin_value")
                  << " coinCount=" << local.get("cash_dispensed_coin_count")
                  << " lastIndex=" << local.get("cash_last_coin_index")
                  << " coin0=" << local.get("cash_coin_dispensed_count_0")
                  << " coin1=" << local.get("cash_coin_dispensed_count_1") << '\n';
        return false;
    }
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
    const auto press = [&runtime](const std::string& event) {
        if (runtime.hasScriptEntryPoint("trigger_" + event)) {
            runtime.invokeMouseEvent(event);
        }
    };
    const auto release = [&runtime](const std::string& event) {
        runtime.invokeMouseRelease(event);
    };
    const auto drag = [&runtime](const std::string& event, double dx, double dy, double x, double y) {
        runtime.invokeMouseDrag(event, dx, dy, x, y);
    };
    runtime.invokeMouseEvent("conditional");
    if (local.get("condition_true") != 5 || local.get("condition_false") != 0 ||
        local.get("condition_folded") != 3 || local.get("condition_negated") != 0 ||
        local.get("condition_macro") != 7) {
        std::cerr << backend << ": conditionals must inspect, not consume, the float stack\n";
        return false;
    }
    mouse.press("bus_doorfront0", 40, 60, press, release);
    mouse.cancel(release);
    if (local.get("front_door_request") != 0) {
        std::cerr << backend << ": front door must remain interlocked before brake activation\n";
        return false;
    }
    const auto errorsBeforeBrake = runtime.errors().size();
    mouse.press("bus_20h-switch", 40, 60, press, release);
    mouse.update(true, true, 40, 60, drag, release);
    mouse.cancel(release);
    if (local.get("door_20h_sw") != 1 || local.get("brake_press_count") != 1 ||
        runtime.errors().size() != errorsBeforeBrake) {
        std::cerr << backend << ": hyphenated brake click must toggle once without optional handlers\n";
        return false;
    }
    if (!runtime.hasScriptEntryPoint("trigger_bus_20h-switch") ||
        !runtime.hasScriptEntryPoint("TRIGGER_BUS_20H-SWITCH") ||
        !runtime.hasScriptEntryPoint("trigger_bus_20h_switch") ||
        runtime.hasScriptEntryPoint("trigger_missing-switch")) {
        std::cerr << backend << ": authored and normalized entry-point lookups disagree\n";
        return false;
    }
    mouse.press("bus_doorfront0", 40, 60, press, release);
    if (local.get("front_door_request") != 1) {
        std::cerr << backend << ": brake activation must allow the front-door request\n";
        return false;
    }
    mouse.cancel(release);
    mouse.press("bus_20h-switch", 40, 60, press, release);
    mouse.cancel(release);
    if (local.get("door_20h_sw") != 0 || local.get("brake_press_count") != 2) {
        std::cerr << backend << ": second brake click must toggle off\n";
        return false;
    }
    mouse.press("dragonly", 40, 60, press, release);
    mouse.update(true, true, 40, 55, drag, release);
    if (local.get("dragonly_count") != 1 || local.get("dragonly_y") != -5) {
        std::cerr << backend << ": captured drag-only mouse event must dispatch _drag without a base trigger\n";
        return false;
    }
    mouse.cancel(release);
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
    const auto root = openbus::probe::makeProbeDirectory("openbus_script_input_probe");
    std::filesystem::create_directories(root);
    const auto script = root / "input.osc";
    {
        std::ofstream source(script);
        source << "{trigger:edge}\n(L.L.edge_press) 1 + (S.L.edge_press)\n{end}\n"
                  "{trigger:edge_off}\n(L.L.edge_release) 1 + (S.L.edge_release)\n{end}\n"
                  "{trigger:system_environment}\n"
                  "(L.S.PrecipType) (S.L.environment_preciptype)\n"
                  "(L.S.PrecipRate) (S.L.environment_preciprate)\n"
                  "(L.L.PrecipType) (S.L.local_preciptype)\n"
                  "(L.L.PrecipRate) (S.L.local_preciprate)\n"
                  "(L.S.NoSound) (S.L.environment_nosound)\n"
                  "(L.S.DayOfYear) (S.L.environment_dayofyear)\n{end}\n"
                  "{trigger:passenger_ticketing}\n"
                  "0 (M.V.getticketvalue) (S.L.ticket_value_result)\n"
                  "1 (M.V.getticketvalue) (S.L.nonfinite_ticket_value_result)\n"
                  "0 (M.V.getticketname) (S.$.ticket_name_result)\n"
                  "99 (M.V.getticketvalue) (S.L.invalid_ticket_value_result)\n"
                  "-1 (M.V.getticketname) (S.$.invalid_ticket_name_result)\n"
                  "2 (M.V.getticketvalue) (S.L.shared_ticket_value_result)\n"
                  "2 (M.V.getticketname) (S.$.shared_ticket_name_result)\n"
                  "2 (M.V.gethumancountonseat) (S.L.seat_human_count)\n"
                  "7 (M.V.gethumancountonpathlink) (S.L.path_link_human_count)\n"
                  "(M.V.getttlinestring) (S.$.timetable_line_result)\n"
                  "(M.V.getttbusstopname) (S.$.timetable_destination_result)\n"
                  "0 (M.V.givechangecoin)\n"
                  "1 (M.V.givechangecoin)\n"
                  "2 (M.V.givechangecoin)\n"
                  "1.5 (M.V.givechangecoin)\n"
                  "-1 (M.V.givechangecoin)\n{end}\n"
                  "{trigger:held}\n(L.L.held_press) 1 + (S.L.held_press)\n"
                  "(L.S.Timegap) (S.L.key_time)\n(L.S.mouse_x) (S.L.key_mouse)\n{end}\n"
                  "{trigger:held_off}\n(L.L.held_release) 1 + (S.L.held_release)\n{end}\n"
                  "{trigger:bus_20h-switch}\n"
                  "(L.L.door_20h_sw) ! (S.L.door_20h_sw)\n"
                  "(L.L.brake_press_count) 1 + (S.L.brake_press_count)\n{end}\n"
                  "{trigger:bus_doorfront0}\n"
                  "(L.L.door_20h_sw)\n{if}\n(M.L.request_front)\n{endif}\n{end}\n"
                  "{macro:request_front}\n1 &&\n{if}\n"
                  "1 (S.L.front_door_request)\n{endif}\n{end}\n"
                  "{trigger:bus_doorfront0_off}\n0 (S.L.front_door_request)\n{end}\n"
                  "{trigger:conditional}\n"
                  "5 {if} (S.L.condition_true) {endif}\n"
                  "0 {if} 99 {else} (S.L.condition_false) {endif}\n"
                  "1 2 + {if} (S.L.condition_folded) {endif}\n"
                  "1 ! {if} 99 {else} (S.L.condition_negated) {endif}\n"
                  "1 {if} (M.L.conditional_gate) {endif}\n{end}\n"
                  "{macro:conditional_gate}\n"
                  "1 && {if} 7 (S.L.condition_macro) {endif}\n{end}\n"
                  "{trigger:control}\n(L.L.press_count) 1 + (S.L.press_count)\n{end}\n"
                  "{trigger:control_drag}\n(L.L.drag_count) 1 + (S.L.drag_count)\n"
                  "(L.S.mouse_x) (S.L.drag_dx)\n(L.S.mouse_y) (S.L.drag_dy)\n"
                  "(L.S.Timegap) (S.L.drag_time)\n{end}\n"
                  "{trigger:dragonly_drag}\n(L.L.dragonly_count) 1 + (S.L.dragonly_count)\n"
                  "(L.S.mouse_y) (S.L.dragonly_y)\n{end}\n"
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
    openbus::probe::removeProbeDirectory(root);
    return nativePassed && luaPassed ? 0 : 1;
}