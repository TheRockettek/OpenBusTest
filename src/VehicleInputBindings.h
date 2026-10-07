#pragma once

#include <iosfwd>
#include <string>
#include <vector>

namespace openbus::input {

struct VehicleKeyBinding {
    std::string action;
    int key;
    int flags;
};

inline constexpr int kBindingHeld = 1;
inline constexpr int kBindingShift = 2;
inline constexpr int kBindingControl = 4;
inline constexpr int kBindingAlt = 8;

// Call once per binding per frame, passing the current state to the handler.
// Held bindings dispatch even when released; modifier bits alone are edge-only.
[[nodiscard]] constexpr bool shouldDispatchKeyBinding(int flags, bool pressed,
                                                      bool wasPressed) noexcept {
    return (flags & kBindingHeld) != 0 || pressed != wasPressed;
}

// DirectInput codes use set-1 scan codes, with bit 7 for extended keys.
// Unsupported codes return GLFW_KEY_UNKNOWN; no GLFW initialization is needed.
[[nodiscard]] int glfwKeyFromKeyboardConfigCode(int code);

// Read [vehicles]/[entry] action/code/flags triples, ignoring blank lines.
// Integers must be complete signed decimal values (an optional '+' is allowed).
// Valid entries update matching actions or append owned names, not replace keys.
// Invalid entries leave defaults intact; internal debug/mouse actions are reserved.
// Returns true if at least one non-reserved, supported binding was accepted.
[[nodiscard]] bool inheritVehicleKeyBindings(std::istream& input,
                                             std::vector<VehicleKeyBinding>& bindings);

} // namespace openbus::input