#pragma once

// Shared includes and tuning constants for the RenderLoop implementation files.

#include "app/RenderLoop.h"
#include "app/ScriptTickScheduler.h"
#include "app/StatusHud.h"
#include "app/vehicle/Vehicle.h"
#include "input/VehicleKeyBindings.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

namespace openbus::app {

inline constexpr double DEFAULT_FIELD_OF_VIEW = 60.0;
inline constexpr double MIN_FIELD_OF_VIEW = 20.0;
inline constexpr double MAX_FIELD_OF_VIEW = 120.0;
inline constexpr double GROUND_SHADOW_OFFSET_Z = 0.015;

} // namespace openbus::app

using namespace openbus::app;
using openbus::input::findVehicleKeyBinding;
using openbus::input::kBindingAlt;
using openbus::input::kBindingControl;
using openbus::input::kBindingHeld;
using openbus::input::kBindingShift;
using openbus::input::vehicleBindingPressed;
using openbus::input::VehicleKeyBinding;
using openbus::input::vehicleKeyBindings;
