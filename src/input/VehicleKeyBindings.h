#pragma once

// Runtime vehicle key bindings: OpenBus defaults merged with OMSI's
// Inputs/keyboard.cfg, plus GLFW polling helpers.

#include "input/VehicleInputBindings.h"

#include <vector>

struct GLFWwindow;

namespace openbus::input {

// Bindings are resolved once on first use and cached for the process lifetime.
[[nodiscard]] const std::vector<VehicleKeyBinding>& vehicleKeyBindings();

[[nodiscard]] const VehicleKeyBinding* findVehicleKeyBinding(const char* action);

[[nodiscard]] bool vehicleBindingPressed(GLFWwindow* window, const VehicleKeyBinding& binding);

} // namespace openbus::input
