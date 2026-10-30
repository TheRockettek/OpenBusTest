#include "core/Variables.h"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

bool require(bool condition, const char* description) {
    if (!condition) {
        std::cerr << "failed: " << description << '\n';
        return false;
    }
    return true;
}

} // namespace

int main() {
    SimulationState simulation;
    openbus::scripting::Vehicle vehicle;
    openbus::scripting::Human human;
    openbus::scripting::SceneryObject sceneryObject;

    bool valid = true;
    valid &= require(simulation.sharedVariables().has("timegap"), "system timegap");
    valid &= require(!simulation.sharedVariables().has("throttle"), "system excludes throttle");
    valid &= require(simulation.sharedVariables().get("preciptype") == 1.0F,
                     "system precipitation defaults to the OMSI clear value");
    simulation.sharedVariables().configureSimulationClock(2024, 2, 28, 86398.0);
    simulation.sharedVariables().updateSimulationClock(3.0);
    valid &= require(simulation.sharedVariables().get("year") == 2024.0F &&
                         simulation.sharedVariables().get("month") == 2.0F &&
                         simulation.sharedVariables().get("day") == 29.0F &&
                         simulation.sharedVariables().get("dayofyear") == 60.0F &&
                         simulation.sharedVariables().get("time") == 1.0F,
                     "simulation clock advances across leap-day midnight");
    simulation.sharedVariables().configureSimulationClock(2024, 12, 31, 86399.0);
    valid &= require(simulation.sharedVariables().get("dayofyear") == 366.0F,
                     "leap year ends on day 366");
    simulation.sharedVariables().updateSimulationClock(1.0);
    valid &= require(simulation.sharedVariables().get("year") == 2025.0F &&
                         simulation.sharedVariables().get("month") == 1.0F &&
                         simulation.sharedVariables().get("day") == 1.0F &&
                         simulation.sharedVariables().get("dayofyear") == 1.0F &&
                         simulation.sharedVariables().get("time") == 0.0F,
                     "simulation clock advances across year boundary");
    simulation.sharedVariables().configureSimulationClock(2024, 12, 31, 86399.75);
    simulation.sharedVariables().updateSimulationClock(0.20);
    valid &= require(simulation.sharedVariables().get("day") == 31.0F &&
                         simulation.sharedVariables().get("time") == 86399.0F,
                     "fractional start remains on its original day before midnight");
    simulation.sharedVariables().updateSimulationClock(0.25);
    valid &= require(simulation.sharedVariables().get("year") == 2025.0F &&
                         simulation.sharedVariables().get("month") == 1.0F &&
                         simulation.sharedVariables().get("day") == 1.0F &&
                         simulation.sharedVariables().get("time") == 0.0F,
                     "fractional start and elapsed time carry across midnight");
    simulation.sharedVariables().configureSimulationClock(2025, 12, 31, 0.0);
    valid &= require(simulation.sharedVariables().get("dayofyear") == 365.0F,
                     "common year ends on day 365");
    simulation.sharedVariables().configureSimulationClock(2025, 2, 28, 86399.0);
    simulation.sharedVariables().updateSimulationClock(1.0);
    valid &= require(simulation.sharedVariables().get("month") == 3.0F &&
                         simulation.sharedVariables().get("day") == 1.0F &&
                         simulation.sharedVariables().get("dayofyear") == 60.0F,
                     "common-year March follows day 59");
    bool rejectedInvalidCalendar = false;
    try {
        simulation.sharedVariables().configureSimulationClock(2025, 2, 29, 0.0);
    } catch (const std::invalid_argument&) {
        rejectedInvalidCalendar = true;
    }
    valid &= require(rejectedInvalidCalendar, "invalid simulation calendar is rejected");

    valid &= require(vehicle.has("throttle"), "vehicle throttle");
    valid &= require(!vehicle.has("lastmoveddist"), "vehicle excludes human state");
    valid &= require(human.has("lastmoveddist"), "human lastmoveddist");
    valid &= require(!human.has("throttle"), "human excludes vehicle state");
    valid &= require(sceneryObject.has("nightlighta"), "scenery nightlighta");
    valid &= require(!sceneryObject.has("throttle"), "scenery excludes vehicle state");
    valid &= require(vehicle.supportsSystemMacro("getrouteindex"), "vehicle macro capability");
    valid &= require(!vehicle.supportsSystemMacro("getarrbusline"),
                     "vehicle excludes scenery macro");
    valid &= require(sceneryObject.supportsSystemMacro("getarrbusline"),
                     "scenery macro capability");
    valid &= require(!human.supportsSystemMacro("getrouteindex"), "human excludes vehicle macro");
    valid &= require(vehicle.supportsSystemTrigger("collision"), "vehicle trigger capability");
    valid &= require(vehicle.supportsSystemTrigger("AI_SCHEDULED_SETTARGET") &&
                         vehicle.supportsSystemTrigger("railbond_3"),
                     "system triggers are case-insensitive and include the listed rail axles");
    valid &= require(!vehicle.supportsSystemTrigger("railbond_4") &&
                         !vehicle.supportsSystemTrigger("railbond_invalid"),
                     "unsupported railbond indexes are rejected");
    valid &= require(!sceneryObject.supportsSystemTrigger("collision"),
                     "scenery excludes vehicle trigger");
    valid &= require(human.supportsSystemTrigger("int_haltewunsch") &&
                         !human.supportsSystemTrigger("ai_scheduled_settarget"),
                     "human trigger capabilities are object-kind scoped");

    simulation.sharedVariables().set("mouse_x", 9.0F);
    simulation.sharedVariables().set("mouse_y", -4.0F);
    simulation.sharedVariables().setPrecipitation(1, 0.75);
    simulation.sharedVariables().setSoundAvailable(true);
    simulation.sharedVariables().updateFrame(0.125F, 10.0F, 800.0F, 600.0F);
    valid &= require(simulation.sharedVariables().get("mouse_x") == 0.0F &&
                         simulation.sharedVariables().get("mouse_y") == 0.0F &&
                         simulation.sharedVariables().get("mouse_cursor_x") == 800.0F &&
                         simulation.sharedVariables().get("mouse_cursor_y") == 600.0F &&
                         simulation.sharedVariables().get("timegap") == 0.125F &&
                         simulation.sharedVariables().get("preciptype") == 2.0F &&
                         simulation.sharedVariables().get("preciprate") == 0.75F &&
                         simulation.sharedVariables().get("nosound") == 0.0F,
                     "frame refresh resets motion but preserves weather and cursor/time state");
    simulation.sharedVariables().setSoundAvailable(false);
    valid &= require(simulation.sharedVariables().get("nosound") == 1.0F,
                     "unavailable sound is published to scripts");
    const auto environment =
        openbus::scripting::Vehicle::captureEnvironment(simulation.sharedVariables());
    vehicle.updateEnvironment(environment);
    valid &= require(vehicle.get("preciptype") == 2.0F && vehicle.get("preciprate") == 0.75F,
                     "a shared frame environment publishes weather to a vehicle");
    vehicle.set("preciptype", 3.0F);
    vehicle.set("preciprate", 0.0F);
    vehicle.updateEnvironment(environment);
    valid &= require(vehicle.get("preciptype") == 2.0F && vehicle.get("preciprate") == 0.75F,
                     "a shared frame environment restores script-modified weather");
    simulation.sharedVariables().setPrecipitation(2, 0.25);
    vehicle.updateEnvironment(simulation.sharedVariables());
    valid &= require(vehicle.get("preciptype") == 3.0F && vehicle.get("preciprate") == 0.25F,
                     "direct environment updates still capture the current weather");
    simulation.sharedVariables().setPrecipitation(1, 0.75);
    for (const auto& [type, rate] :
         {std::pair{-1, 0.5}, std::pair{3, 0.5}, std::pair{1, -0.1},
          std::pair{2, 1.1}, std::pair{1, std::numeric_limits<double>::quiet_NaN()}}) {
        bool rejected = false;
        try {
            simulation.sharedVariables().setPrecipitation(type, rate);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        valid &= require(rejected && simulation.sharedVariables().get("preciptype") == 2.0F &&
                             simulation.sharedVariables().get("preciprate") == 0.75F,
                         "invalid weather is rejected without changing either variable");
    }

    vehicle.setString("ident", "probe");
    valid &= require(vehicle.hasString("ident") && vehicle.getString("ident") == "probe",
                     "vehicle string state");
    valid &= require(vehicle.numericValues().count("throttle") == 1,
                     "numeric variable snapshot");
    valid &= require(vehicle.stringValues().count("ident") == 1,
                     "string variable snapshot");
    vehicle.set("steeringwheeloffset", 0.25);
    vehicle.updateFrame();
    valid &= require(vehicle.get("steeringwheeloffset") == 0.25,
                     "local numeric state persists across frames");
    valid &= require(vehicle.getString("ident") == "probe",
                     "local string state persists across frames");

    vehicle.set("mixedcasevalue", 0.75);
    valid &= require(vehicle.get("mixedcasevalue") == 0.75,
                     "normalized numeric state uses lowercase keys");
    vehicle.setString("mixedcasestring", "normalized");
    valid &= require(vehicle.getString("mixedcasestring") == "normalized",
                     "normalized string state uses lowercase keys");
    return valid ? 0 : 1;
}