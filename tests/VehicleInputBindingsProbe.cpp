#include "VehicleInputBindings.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using openbus::input::VehicleKeyBinding;
using openbus::input::glfwKeyFromKeyboardConfigCode;
using openbus::input::inheritVehicleKeyBindings;
using openbus::input::kBindingAlt;
using openbus::input::kBindingControl;
using openbus::input::kBindingHeld;
using openbus::input::kBindingShift;
using openbus::input::shouldDispatchKeyBinding;

class Probe {
public:
    void expect(bool condition, std::string_view message) {
        if (!condition) {
            std::cerr << "FAIL: " << message << '\n';
            ++failures;
        }
    }

    void binding(const std::vector<VehicleKeyBinding>& bindings, std::string_view action,
                 int key, int flags) {
        const auto found = std::find_if(
            bindings.begin(), bindings.end(),
            [action](const VehicleKeyBinding& value) { return value.action == action; });
        if (found == bindings.end() || found->key != key || found->flags != flags) {
            std::cerr << "FAIL: binding " << action << " expected key=" << key
                      << " flags=" << flags << '\n';
            ++failures;
        }
    }

    int failures = 0;
};

// Model a caller which makes a single dispatch decision for each frame.
std::vector<bool> dispatchedStates(int flags, std::initializer_list<bool> frames) {
    std::vector<bool> calls;
    bool wasPressed = false;
    for (const bool pressed : frames) {
        if (shouldDispatchKeyBinding(flags, pressed, wasPressed)) {
            calls.push_back(pressed);
        }
        wasPressed = pressed;
    }
    return calls;
}

void testDispatch(Probe& probe) {
    probe.expect(kBindingHeld == 1 && kBindingShift == 2 && kBindingControl == 4 &&
                     kBindingAlt == 8,
                 "binding flag constants");
    // Also exercise the public function in constant evaluation without assert().
    constexpr bool heldReleased = shouldDispatchKeyBinding(kBindingHeld, false, false);
    constexpr bool modifierOnly = shouldDispatchKeyBinding(kBindingControl, true, true);
    probe.expect(heldReleased && !modifierOnly, "constexpr dispatch decisions");

    const auto edges = dispatchedStates(0, {false, true, true, false, false, true, false});
    probe.expect(edges == std::vector<bool>({true, false, true, false}),
                 "edge binding dispatches two presses and two releases, not repeats");
    probe.expect(std::count(edges.begin(), edges.end(), true) == 2 &&
                     std::count(edges.begin(), edges.end(), false) == 2,
                 "edge press/release counts");

    probe.expect(dispatchedStates(kBindingHeld, {true, true, false, false}) ==
                     std::vector<bool>({true, true, false, false}),
                 "held binding dispatches once per frame, including first press (no double)");
    probe.expect(dispatchedStates(kBindingHeld, {false, false, true, false}) ==
                     std::vector<bool>({false, false, true, false}),
                 "held binding dispatches released state, including first frame");
    probe.expect(dispatchedStates(kBindingHeld | kBindingShift | kBindingControl | kBindingAlt,
                                 {false, true, true, false}) ==
                     std::vector<bool>({false, true, true, false}),
                 "held plus modifiers still dispatches exactly once per frame");
    for (const int flags : {kBindingShift, kBindingControl, kBindingAlt,
                           kBindingShift | kBindingControl | kBindingAlt, 16}) {
        probe.expect(dispatchedStates(flags, {false, true, true, false, false, true, false}) ==
                         edges,
                     "modifier or unknown bit alone must not enable held dispatch");
    }
    for (int flags = 0; flags < 32; ++flags) {
        for (const bool pressed : {false, true}) {
            for (const bool wasPressed : {false, true}) {
                const bool expected = (flags & 1) != 0 || pressed != wasPressed;
                probe.expect(shouldDispatchKeyBinding(flags, pressed, wasPressed) == expected,
                             "dispatch truth table");
            }
        }
    }
}

void testKeyMapping(Probe& probe) {
    const std::array<int, 10> digits = {GLFW_KEY_1, GLFW_KEY_2, GLFW_KEY_3, GLFW_KEY_4,
                                      GLFW_KEY_5, GLFW_KEY_6, GLFW_KEY_7, GLFW_KEY_8,
                                      GLFW_KEY_9, GLFW_KEY_0};
    for (std::size_t index = 0; index < digits.size(); ++index) {
        probe.expect(glfwKeyFromKeyboardConfigCode(2 + static_cast<int>(index)) == digits[index],
                     "top-row digit mapping (code 11 must be digit 0)");
    }
    // Indexed alphabetically rather than in scan-code order.
    const std::array<int, 26> letterCodes = {
        30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38, 50,
        49, 24, 25, 16, 19, 31, 20, 22, 47, 17, 45, 21, 44};
    for (std::size_t index = 0; index < letterCodes.size(); ++index) {
        probe.expect(glfwKeyFromKeyboardConfigCode(letterCodes[index]) ==
                         GLFW_KEY_A + static_cast<int>(index),
                     "alphabetic mapping");
    }
    for (int index = 0; index < 10; ++index) {
        probe.expect(glfwKeyFromKeyboardConfigCode(59 + index) == GLFW_KEY_F1 + index,
                     "F1 through F10 mapping");
    }
    const std::pair<int, int> keys[] = {
        {1, GLFW_KEY_ESCAPE},
        {12, GLFW_KEY_MINUS},
        {13, GLFW_KEY_EQUAL},
        {14, GLFW_KEY_BACKSPACE},
        {15, GLFW_KEY_TAB},
        {26, GLFW_KEY_LEFT_BRACKET},
        {27, GLFW_KEY_RIGHT_BRACKET},
        {28, GLFW_KEY_ENTER},
        {29, GLFW_KEY_LEFT_CONTROL},
        {39, GLFW_KEY_SEMICOLON},
        {40, GLFW_KEY_APOSTROPHE},
        {41, GLFW_KEY_GRAVE_ACCENT},
        {42, GLFW_KEY_LEFT_SHIFT},
        {43, GLFW_KEY_BACKSLASH},
        {51, GLFW_KEY_COMMA},
        {52, GLFW_KEY_PERIOD},
        {53, GLFW_KEY_SLASH},
        {54, GLFW_KEY_RIGHT_SHIFT},
        {55, GLFW_KEY_KP_MULTIPLY},
        {56, GLFW_KEY_LEFT_ALT},
        {57, GLFW_KEY_SPACE},
        {58, GLFW_KEY_CAPS_LOCK},
        {69, GLFW_KEY_NUM_LOCK},
        {70, GLFW_KEY_SCROLL_LOCK},
        {71, GLFW_KEY_KP_7},
        {72, GLFW_KEY_KP_8},
        {73, GLFW_KEY_KP_9},
        {74, GLFW_KEY_KP_SUBTRACT},
        {75, GLFW_KEY_KP_4},
        {76, GLFW_KEY_KP_5},
        {77, GLFW_KEY_KP_6},
        {78, GLFW_KEY_KP_ADD},
        {79, GLFW_KEY_KP_1},
        {80, GLFW_KEY_KP_2},
        {81, GLFW_KEY_KP_3},
        {82, GLFW_KEY_KP_0},
        {83, GLFW_KEY_KP_DECIMAL},
        {86, GLFW_KEY_WORLD_2},
        {87, GLFW_KEY_F11},
        {88, GLFW_KEY_F12},
        {141, GLFW_KEY_KP_EQUAL},
        {156, GLFW_KEY_KP_ENTER},
        {157, GLFW_KEY_RIGHT_CONTROL},
        {181, GLFW_KEY_KP_DIVIDE},
        {183, GLFW_KEY_PRINT_SCREEN},
        {184, GLFW_KEY_RIGHT_ALT},
        {197, GLFW_KEY_PAUSE},
        {199, GLFW_KEY_HOME},
        {200, GLFW_KEY_UP},
        {201, GLFW_KEY_PAGE_UP},
        {203, GLFW_KEY_LEFT},
        {205, GLFW_KEY_RIGHT},
        {207, GLFW_KEY_END},
        {208, GLFW_KEY_DOWN},
        {209, GLFW_KEY_PAGE_DOWN},
        {210, GLFW_KEY_INSERT},
        {211, GLFW_KEY_DELETE},
        {219, GLFW_KEY_LEFT_SUPER},
        {220, GLFW_KEY_RIGHT_SUPER},
        {221, GLFW_KEY_MENU},
    };
    for (const auto& [code, key] : keys) {
        probe.expect(glfwKeyFromKeyboardConfigCode(code) == key,
                     "punctuation/navigation/keypad/modifier mapping");
    }
    for (const int code : {-1, 0, 84, 89, 100, 255, 256, 0xE01D,
                           std::numeric_limits<int>::min(), std::numeric_limits<int>::max()}) {
        probe.expect(glfwKeyFromKeyboardConfigCode(code) == GLFW_KEY_UNKNOWN,
                     "unsupported scan code");
    }
}

void testInheritanceAndOwnership(Probe& probe) {
    std::vector<VehicleKeyBinding> bindings = {
        {"throttle", GLFW_KEY_KP_8, kBindingHeld},
        {"default_alias", GLFW_KEY_M, kBindingHeld},
        {"debug_dump_variables", GLFW_KEY_F9, 0},
        {"mouse_control_toggle", GLFW_KEY_O, 0},
    };
    const std::string longAction = "arbitrary_vehicle_action_" + std::string(256, 'x');
    {
        std::istringstream input(
            " \t[vehicles] \r\n\n"
            " \t[entry]\r\n throttle \t\r\n 11 \r\n 3\r\n"
            "[entry]\nalias_one\n50\n0\n"
            "[entry]\nalias_two\n50\n0\n"
            "[entry]\n" + longAction + "\n+16\n+8\n"
            "[entry]\ndebug_dump_variables\n2\n15\n"
            "[entry]\nmouse_control_toggle\n3\n1\n");
        probe.expect(inheritVehicleKeyBindings(input, bindings), "valid entries accepted");
        // Replacing the stream contents must not invalidate stored action names.
        input.str(std::string(8192, 'z'));
    }
    probe.expect(bindings.size() == 7, "append arbitrary actions without replacing aliases");
    probe.binding(bindings, "throttle", GLFW_KEY_0, kBindingHeld | kBindingShift);
    probe.binding(bindings, "default_alias", GLFW_KEY_M, kBindingHeld);
    probe.binding(bindings, "alias_one", GLFW_KEY_M, 0);
    probe.binding(bindings, "alias_two", GLFW_KEY_M, 0);
    probe.binding(bindings, longAction, GLFW_KEY_Q, kBindingAlt);
    probe.binding(bindings, "debug_dump_variables", GLFW_KEY_F9, 0);
    probe.binding(bindings, "mouse_control_toggle", GLFW_KEY_O, 0);

    std::istringstream repeated(
        "[vehicles]\n[entry]\nalias_one\n17\n2\n"
        "[entry]\nalias_one\n18\n8\n"
        "[entry]\nalias_one\n999\n1\n");
    probe.expect(inheritVehicleKeyBindings(repeated, bindings), "repeated action accepted");
    probe.expect(bindings.size() == 7, "repeated action updates rather than appends");
    probe.binding(bindings, "alias_one", GLFW_KEY_E, kBindingAlt);
    probe.binding(bindings, "alias_two", GLFW_KEY_M, 0);

    std::istringstream unchanged("[vehicles]\n[entry]\nalias_two\n50\n0\n");
    probe.expect(inheritVehicleKeyBindings(unchanged, bindings),
                 "supported entry counts as accepted even if its values do not change");
    probe.expect(bindings.size() == 7, "unchanged entry must not append");

    std::istringstream reservedDefaults(
        "[vehicles]\n[entry]\ndebug_dump_variables\n2\n15\n"
        "[entry]\nmouse_control_toggle\n3\n1\n");
    probe.expect(!inheritVehicleKeyBindings(reservedDefaults, bindings),
                 "reserved-only entries must not count as accepted when defaults exist");
    probe.binding(bindings, "debug_dump_variables", GLFW_KEY_F9, 0);
    probe.binding(bindings, "mouse_control_toggle", GLFW_KEY_O, 0);

    std::vector<VehicleKeyBinding> noDefaults;
    std::istringstream reservedOnly(
        "[vehicles]\n[entry]\ndebug_dump_variables\n2\n0\n"
        "[entry]\nmouse_control_toggle\n3\n0\n");
    probe.expect(!inheritVehicleKeyBindings(reservedOnly, noDefaults) && noDefaults.empty(),
                 "reserved actions are neither appended nor counted as accepted");
}

void testInvalidEntries(Probe& probe) {
    const std::pair<std::string_view, std::string_view> invalidEntries[] = {
        {"999", "0"}, {"-1", "0"}, {"0", "0"},
        {"11junk", "0"}, {"11.0", "0"}, {"0x0B", "0"}, {"1 1", "0"},
        {"999999999999999999999999", "0"}, {"-999999999999999999999999", "0"},
        {"+", "0"}, {"+-11", "0"}, {"--11", "0"}, {"garbage", "0"},
        {"11", "3junk"}, {"11", "1.0"}, {"11", "1 2"}, {"11", "0x1"},
        {"11", "999999999999999999999999"}, {"11", "-999999999999999999999999"},
        {"11", "+"}, {"11", "+-1"}, {"11", "--1"}, {"11", "garbage"},
    };
    for (const auto& [code, flags] : invalidEntries) {
        std::vector<VehicleKeyBinding> bindings = {{"throttle", GLFW_KEY_KP_8, kBindingHeld}};
        std::istringstream input("[vehicles]\n[entry]\nthrottle\n" + std::string(code) +
                                 '\n' + std::string(flags) +
                                 "\n[entry]\ninvalid_new_action\n" + std::string(code) +
                                 '\n' + std::string(flags) + '\n');
        probe.expect(!inheritVehicleKeyBindings(input, bindings),
                     "unsupported/malformed entries must not count as accepted");
        probe.expect(bindings.size() == 1, "unsupported/malformed actions must not append");
        probe.binding(bindings, "throttle", GLFW_KEY_KP_8, kBindingHeld);
    }

    const std::string incompleteEntries[] = {
        "", "[vehicles]\n", "[vehicles]\n[entry]\n",
        "[vehicles]\n[entry]\nthrottle\n", "[vehicles]\n[entry]\nthrottle\n11\n",
        "[vehicles]\n[entry]\nthrottle\n\n\t\r\n",
        "[vehicles]\n[entry]\nthrottle\n11\n\n \r\n",
    };
    for (const std::string& text : incompleteEntries) {
        std::vector<VehicleKeyBinding> bindings = {{"throttle", GLFW_KEY_KP_8, kBindingHeld}};
        std::istringstream input(text);
        probe.expect(!inheritVehicleKeyBindings(input, bindings), "incomplete entry ignored");
        probe.binding(bindings, "throttle", GLFW_KEY_KP_8, kBindingHeld);
    }

    std::vector<VehicleKeyBinding> bindings = {{"throttle", GLFW_KEY_KP_8, kBindingHeld}};
    std::istringstream recovery(
        "[vehicles]\n[entry]\nthrottle\n11suffix\n0\n"
        "[entry]\nthrottle\n11\n0suffix\n"
        "[entry]\nthrottle\n11\n"
        "[entry]\nrecovered_action\n35\n1");
    probe.expect(inheritVehicleKeyBindings(recovery, bindings),
                 "next entry recovers after invalid/incomplete entry; final newline optional");
    probe.binding(bindings, "throttle", GLFW_KEY_KP_8, kBindingHeld);
    probe.binding(bindings, "recovered_action", GLFW_KEY_H, kBindingHeld);

    std::istringstream integerLimits(
        "[vehicles]\n[entry]\nminimum_flags\n16\n" +
        std::to_string(std::numeric_limits<int>::min()) +
        "\n[entry]\nmaximum_flags\n17\n" +
        std::to_string(std::numeric_limits<int>::max()) + '\n');
    probe.expect(inheritVehicleKeyBindings(integerLimits, bindings), "valid int limits accepted");
    probe.binding(bindings, "minimum_flags", GLFW_KEY_Q, std::numeric_limits<int>::min());
    probe.binding(bindings, "maximum_flags", GLFW_KEY_W, std::numeric_limits<int>::max());
}

void testSections(Probe& probe) {
    std::vector<VehicleKeyBinding> bindings = {{"throttle", GLFW_KEY_KP_8, kBindingHeld}};
    std::istringstream otherSections(
        "[entry]\nthrottle\n11\n0\n"
        "[general]\n[entry]\nnot_vehicle\n16\n0\n"
        "[vehicles]\nthrottle\n11\n0\n"
        "[entry]\nthrottle\n11\n"
        "[camera]\n0\n[entry]\nthrottle\n11\n0\n"
        "[Vehicles]\n[entry]\nwrong_case\n17\n0\n");
    probe.expect(!inheritVehicleKeyBindings(otherSections, bindings) && bindings.size() == 1,
                 "only complete entries inside the exact vehicles section are accepted");
    probe.binding(bindings, "throttle", GLFW_KEY_KP_8, kBindingHeld);

    std::istringstream reentry(
        "[vehicles]\n[entry]\nfirst_vehicle\n16\n0\n"
        "extra_action_without_entry\n17\n0\n"
        "[unknown]\n[entry]\nnot_vehicle\n18\n0\n"
        "[vehicles]\n[entry]\npartial_action\n11\n"
        "[vehicles]\n0\n"
        "[entry]\nsecond_vehicle\n17\n4\n");
    probe.expect(inheritVehicleKeyBindings(reentry, bindings), "vehicles section can be reentered");
    probe.expect(bindings.size() == 3, "headers reset partial entries; extra data is ignored");
    probe.binding(bindings, "first_vehicle", GLFW_KEY_Q, 0);
    probe.binding(bindings, "second_vehicle", GLFW_KEY_W, kBindingControl);
}

} // namespace

int main() {
    Probe probe;
    testDispatch(probe);
    testKeyMapping(probe);
    testInheritanceAndOwnership(probe);
    testInvalidEntries(probe);
    testSections(probe);
    if (probe.failures != 0) {
        std::cerr << "VehicleInputBindingsProbe: " << probe.failures << " failure(s)\n";
        return 1;
    }
    std::cout << "VehicleInputBindingsProbe passed\n";
    return 0;
}