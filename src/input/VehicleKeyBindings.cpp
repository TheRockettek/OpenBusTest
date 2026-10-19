#include "input/VehicleKeyBindings.h"

#include "core/Environment.h"
#include "core/Logger.h"
#include "core/OmsiRoot.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

namespace openbus::input {

namespace {

const std::vector<VehicleKeyBinding>& defaultVehicleKeyBindings() {
    static const std::vector<VehicleKeyBinding> bindings = {
        {"throttle", GLFW_KEY_KP_8, kBindingHeld},
        {"brake", GLFW_KEY_KP_2, kBindingHeld},
        {"throttle_amplify", GLFW_KEY_KP_ADD, kBindingHeld},
        {"clutch", GLFW_KEY_TAB, kBindingHeld},
        {"steering_left", GLFW_KEY_KP_4, kBindingHeld},
        {"steering_neutral", GLFW_KEY_KP_5, kBindingHeld},
        {"steering_right", GLFW_KEY_KP_6, kBindingHeld},
        {"ticket_give", GLFW_KEY_T, 0},
        {"change_give", GLFW_KEY_T, kBindingControl},
        {"change_take", GLFW_KEY_T, kBindingShift},
        {"parking_brake_toggle", GLFW_KEY_PERIOD, 0},
        {"blinker_left_set", GLFW_KEY_KP_7, kBindingHeld},
        {"blinker_right_set", GLFW_KEY_KP_9, kBindingHeld},
        {"blinker_off", GLFW_KEY_KP_DECIMAL, 0},
        {"blinker_warn_toggle", GLFW_KEY_B, 0},
        {"kw_scheinwerfer_toggle", GLFW_KEY_L, 0},
        {"kw_standlicht_toggle", GLFW_KEY_L, kBindingShift},
        {"kw_fernlicht_toggle", GLFW_KEY_F, 0},
        {"kw_m_enginestart", GLFW_KEY_M, 0},
        {"kw_wipermode_up", GLFW_KEY_W, 0},
        {"horn", GLFW_KEY_H, kBindingHeld},
        {"cp_microphone", GLFW_KEY_Q, kBindingHeld},
        {"cp_fahrerlicht_toggle", GLFW_KEY_6, 0},
        {"cp_licht_untenrechts_toggle", GLFW_KEY_7, 0},
        {"cp_licht_oberdeck_toggle", GLFW_KEY_8, 0},
        {"cp_licht_unterdeck_toggle", GLFW_KEY_9, 0},
        {"kw_s_R", GLFW_KEY_R, 0},
        {"kw_s_N", GLFW_KEY_N, 0},
        {"kw_s_1", GLFW_KEY_1, 0},
        {"kw_s_2", GLFW_KEY_2, 0},
        {"kw_s_3", GLFW_KEY_3, 0},
        {"kw_s_4", GLFW_KEY_4, 0},
        {"kw_s_5", GLFW_KEY_5, 0},
        {"kw_s_6", GLFW_KEY_6, 0},
        {"automatic_1", GLFW_KEY_1, kBindingShift},
        {"automatic_2", GLFW_KEY_2, kBindingShift},
        {"automatic_D", GLFW_KEY_LEFT_SHIFT, kBindingShift},
        {"automatic_R", GLFW_KEY_LEFT_ALT, 0},
        {"automatic_N", GLFW_KEY_LEFT_CONTROL, kBindingControl},
        {"bus_doorfront0", GLFW_KEY_KP_DIVIDE, 0},
        {"bus_doorfront1", GLFW_KEY_KP_MULTIPLY, 0},
        {"bus_dooraft", GLFW_KEY_KP_SUBTRACT, 0},
        {"bus_20h-switch", GLFW_KEY_SCROLL_LOCK, 0},
        {"bus_doorfront5", GLFW_KEY_KP_ADD, 0},
        {"bus_linie_plus", GLFW_KEY_F8, 0},
        {"bus_linie_minus", GLFW_KEY_F5, 0},
        {"bus_ziel_plus", GLFW_KEY_F7, 0},
        {"bus_ziel_minus", GLFW_KEY_F6, 0},
        {"debug_dump_variables", GLFW_KEY_F9, 0},
        {"cp_wischer_intervall_toggle", GLFW_KEY_W, kBindingShift},
        {"cp_wischer_wascher_button", GLFW_KEY_W, kBindingControl},
        {"cp_batterietrennschalter_toggle", GLFW_KEY_E, 0},
        {"taster_nebelschluss", GLFW_KEY_F, kBindingControl},
        {"cp_schalter_kinderwagen", GLFW_KEY_F12, 0},
        {"kw_s_plus", GLFW_KEY_LEFT_BRACKET, 0},
        {"kw_s_minus", GLFW_KEY_SLASH, 0},
        {"mouse_control_toggle", GLFW_KEY_O, 0},
    };
    return bindings;
}

void inheritVehicleKeyBindingsFromOmsi(std::vector<VehicleKeyBinding>& bindings) {
    const std::filesystem::path configPath = omsiRootPath() / "Inputs" / "keyboard.cfg";
    std::ifstream input(configPath);
    if (!input) {
        gameLog.Log("OMSI keybindings not found, using OpenBus defaults: " +
                    configPath.generic_string());
        return;
    }

    const bool inherited = openbus::input::inheritVehicleKeyBindings(input, bindings);
    gameLog.Log(std::string(inherited ? "Inherited OMSI vehicle keybindings from "
                                      : "No supported OMSI vehicle keybindings in ") +
                configPath.generic_string());
}

} // namespace

const std::vector<VehicleKeyBinding>& vehicleKeyBindings() {
    static const std::vector<VehicleKeyBinding> bindings = [] {
        std::vector<VehicleKeyBinding> result = defaultVehicleKeyBindings();
        inheritVehicleKeyBindingsFromOmsi(result);
        return result;
    }();
    return bindings;
}

const VehicleKeyBinding* findVehicleKeyBinding(const char* action) {
    const auto& bindings = vehicleKeyBindings();
    const auto found =
        std::find_if(bindings.begin(), bindings.end(), [action](const VehicleKeyBinding& binding) {
            return std::string_view(binding.action) == action;
        });
    return found == bindings.end() ? nullptr : &*found;
}

bool vehicleBindingPressed(GLFWwindow* window, const VehicleKeyBinding& binding) {
    if (binding.key == GLFW_KEY_UNKNOWN || glfwGetKey(window, binding.key) != GLFW_PRESS) {
        return false;
    }
    const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    const bool control = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                         glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
    const bool alt = glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
    const bool primaryShift =
        binding.key == GLFW_KEY_LEFT_SHIFT || binding.key == GLFW_KEY_RIGHT_SHIFT;
    const bool primaryControl =
        binding.key == GLFW_KEY_LEFT_CONTROL || binding.key == GLFW_KEY_RIGHT_CONTROL;
    const bool primaryAlt = binding.key == GLFW_KEY_LEFT_ALT || binding.key == GLFW_KEY_RIGHT_ALT;
    return shift == (((binding.flags & kBindingShift) != 0) || primaryShift) &&
           control == (((binding.flags & kBindingControl) != 0) || primaryControl) &&
           alt == (((binding.flags & kBindingAlt) != 0) || primaryAlt);
}

} // namespace openbus::input
