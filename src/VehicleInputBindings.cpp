#include "VehicleInputBindings.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

#include <algorithm>
#include <charconv>
#include <istream>
#include <string_view>
#include <system_error>
#include <utility>

namespace openbus::input {
namespace {

struct KeyMapping {
    int code;
    int key;
};

constexpr KeyMapping keyMappings[] = {
    {0x01, GLFW_KEY_ESCAPE},
    {0x02, GLFW_KEY_1},
    {0x03, GLFW_KEY_2},
    {0x04, GLFW_KEY_3},
    {0x05, GLFW_KEY_4},
    {0x06, GLFW_KEY_5},
    {0x07, GLFW_KEY_6},
    {0x08, GLFW_KEY_7},
    {0x09, GLFW_KEY_8},
    {0x0A, GLFW_KEY_9},
    {0x0B, GLFW_KEY_0},
    {0x0C, GLFW_KEY_MINUS},
    {0x0D, GLFW_KEY_EQUAL},
    {0x0E, GLFW_KEY_BACKSPACE},
    {0x0F, GLFW_KEY_TAB},
    {0x10, GLFW_KEY_Q},
    {0x11, GLFW_KEY_W},
    {0x12, GLFW_KEY_E},
    {0x13, GLFW_KEY_R},
    {0x14, GLFW_KEY_T},
    {0x15, GLFW_KEY_Y},
    {0x16, GLFW_KEY_U},
    {0x17, GLFW_KEY_I},
    {0x18, GLFW_KEY_O},
    {0x19, GLFW_KEY_P},
    {0x1A, GLFW_KEY_LEFT_BRACKET},
    {0x1B, GLFW_KEY_RIGHT_BRACKET},
    {0x1C, GLFW_KEY_ENTER},
    {0x1D, GLFW_KEY_LEFT_CONTROL},
    {0x1E, GLFW_KEY_A},
    {0x1F, GLFW_KEY_S},
    {0x20, GLFW_KEY_D},
    {0x21, GLFW_KEY_F},
    {0x22, GLFW_KEY_G},
    {0x23, GLFW_KEY_H},
    {0x24, GLFW_KEY_J},
    {0x25, GLFW_KEY_K},
    {0x26, GLFW_KEY_L},
    {0x27, GLFW_KEY_SEMICOLON},
    {0x28, GLFW_KEY_APOSTROPHE},
    {0x29, GLFW_KEY_GRAVE_ACCENT},
    {0x2A, GLFW_KEY_LEFT_SHIFT},
    {0x2B, GLFW_KEY_BACKSLASH},
    {0x2C, GLFW_KEY_Z},
    {0x2D, GLFW_KEY_X},
    {0x2E, GLFW_KEY_C},
    {0x2F, GLFW_KEY_V},
    {0x30, GLFW_KEY_B},
    {0x31, GLFW_KEY_N},
    {0x32, GLFW_KEY_M},
    {0x33, GLFW_KEY_COMMA},
    {0x34, GLFW_KEY_PERIOD},
    {0x35, GLFW_KEY_SLASH},
    {0x36, GLFW_KEY_RIGHT_SHIFT},
    {0x37, GLFW_KEY_KP_MULTIPLY},
    {0x38, GLFW_KEY_LEFT_ALT},
    {0x39, GLFW_KEY_SPACE},
    {0x3A, GLFW_KEY_CAPS_LOCK},
    {0x3B, GLFW_KEY_F1},
    {0x3C, GLFW_KEY_F2},
    {0x3D, GLFW_KEY_F3},
    {0x3E, GLFW_KEY_F4},
    {0x3F, GLFW_KEY_F5},
    {0x40, GLFW_KEY_F6},
    {0x41, GLFW_KEY_F7},
    {0x42, GLFW_KEY_F8},
    {0x43, GLFW_KEY_F9},
    {0x44, GLFW_KEY_F10},
    {0x45, GLFW_KEY_NUM_LOCK},
    {0x46, GLFW_KEY_SCROLL_LOCK},
    {0x47, GLFW_KEY_KP_7},
    {0x48, GLFW_KEY_KP_8},
    {0x49, GLFW_KEY_KP_9},
    {0x4A, GLFW_KEY_KP_SUBTRACT},
    {0x4B, GLFW_KEY_KP_4},
    {0x4C, GLFW_KEY_KP_5},
    {0x4D, GLFW_KEY_KP_6},
    {0x4E, GLFW_KEY_KP_ADD},
    {0x4F, GLFW_KEY_KP_1},
    {0x50, GLFW_KEY_KP_2},
    {0x51, GLFW_KEY_KP_3},
    {0x52, GLFW_KEY_KP_0},
    {0x53, GLFW_KEY_KP_DECIMAL},
    {0x56, GLFW_KEY_WORLD_2}, // ISO non-US key between left Shift and Z.
    {0x57, GLFW_KEY_F11},
    {0x58, GLFW_KEY_F12},
    {0x8D, GLFW_KEY_KP_EQUAL},
    {0x9C, GLFW_KEY_KP_ENTER},
    {0x9D, GLFW_KEY_RIGHT_CONTROL},
    {0xB5, GLFW_KEY_KP_DIVIDE},
    {0xB7, GLFW_KEY_PRINT_SCREEN},
    {0xB8, GLFW_KEY_RIGHT_ALT},
    {0xC5, GLFW_KEY_PAUSE},
    {0xC7, GLFW_KEY_HOME},
    {0xC8, GLFW_KEY_UP},
    {0xC9, GLFW_KEY_PAGE_UP},
    {0xCB, GLFW_KEY_LEFT},
    {0xCD, GLFW_KEY_RIGHT},
    {0xCF, GLFW_KEY_END},
    {0xD0, GLFW_KEY_DOWN},
    {0xD1, GLFW_KEY_PAGE_DOWN},
    {0xD2, GLFW_KEY_INSERT},
    {0xD3, GLFW_KEY_DELETE},
    {0xDB, GLFW_KEY_LEFT_SUPER},
    {0xDC, GLFW_KEY_RIGHT_SUPER},
    {0xDD, GLFW_KEY_MENU},
};

std::string_view trimBindingText(std::string_view value) {
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool parseInteger(std::string_view text, int& value) {
    if (text.empty()) {
        return false;
    }
    if (text.front() == '+') {
        text.remove_prefix(1);
        if (text.empty() || text.front() < '0' || text.front() > '9') {
            return false;
        }
    }
    int parsed = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed, 10);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
        return false;
    }
    value = parsed;
    return true;
}

bool isReservedAction(std::string_view action) {
    return action == "debug_dump_variables" || action == "mouse_control_toggle";
}

} // namespace

int glfwKeyFromKeyboardConfigCode(int code) {
    for (const KeyMapping& mapping : keyMappings) {
        if (mapping.code == code) {
            return mapping.key;
        }
    }
    return GLFW_KEY_UNKNOWN;
}

bool inheritVehicleKeyBindings(std::istream& input, std::vector<VehicleKeyBinding>& bindings) {
    enum class EntryField { None, Action, Code, Flags };

    bool inVehiclesSection = false;
    bool accepted = false;
    EntryField field = EntryField::None;
    std::string action;
    int keyCode = 0;
    std::string line;
    while (std::getline(input, line)) {
        const std::string_view text = trimBindingText(line);
        if (text.empty()) {
            continue;
        }
        if (text.front() == '[' && text.back() == ']') {
            field = EntryField::None;
            if (text == "[vehicles]") {
                inVehiclesSection = true;
            } else if (text == "[entry]") {
                if (inVehiclesSection) {
                    field = EntryField::Action;
                }
            } else {
                inVehiclesSection = false;
            }
            continue;
        }
        switch (field) {
        case EntryField::None:
            break;
        case EntryField::Action:
            action.assign(text);
            field = EntryField::Code;
            break;
        case EntryField::Code:
            field = parseInteger(text, keyCode) ? EntryField::Flags : EntryField::None;
            break;
        case EntryField::Flags: {
            field = EntryField::None;
            int flags = 0;
            if (!parseInteger(text, flags) || isReservedAction(action)) {
                break;
            }
            const int key = glfwKeyFromKeyboardConfigCode(keyCode);
            if (key == GLFW_KEY_UNKNOWN) {
                break;
            }
            const auto found = std::find_if(
                bindings.begin(), bindings.end(),
                [&action](const VehicleKeyBinding& binding) { return binding.action == action; });
            if (found != bindings.end()) {
                found->key = key;
                found->flags = flags;
            } else {
                bindings.push_back({std::move(action), key, flags});
            }
            accepted = true;
            break;
        }
        }
    }
    return accepted;
}

} // namespace openbus::input