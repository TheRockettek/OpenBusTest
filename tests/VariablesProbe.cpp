#include "Variables.h"

#include <iostream>

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
    valid &= require(simulation.sharedVariables().has("Timegap"), "system Timegap");
    valid &= require(!simulation.sharedVariables().has("Throttle"), "system excludes Throttle");
    valid &= require(vehicle.has("Throttle"), "vehicle Throttle");
    valid &= require(!vehicle.has("LastMovedDist"), "vehicle excludes human state");
    valid &= require(human.has("LastMovedDist"), "human LastMovedDist");
    valid &= require(!human.has("Throttle"), "human excludes vehicle state");
    valid &= require(sceneryObject.has("NightlightA"), "scenery NightlightA");
    valid &= require(!sceneryObject.has("Throttle"), "scenery excludes vehicle state");
    valid &= require(vehicle.supportsSystemMacro("GetRouteIndex"), "vehicle macro capability");
    valid &= require(!vehicle.supportsSystemMacro("GetArrBusLine"),
                     "vehicle excludes scenery macro");
    valid &= require(sceneryObject.supportsSystemMacro("GetArrBusLine"),
                     "scenery macro capability");
    valid &= require(!human.supportsSystemMacro("GetRouteIndex"), "human excludes vehicle macro");
    valid &= require(vehicle.supportsSystemTrigger("collision"), "vehicle trigger capability");
    valid &= require(!sceneryObject.supportsSystemTrigger("collision"),
                     "scenery excludes vehicle trigger");

    vehicle.setString("ident", "probe");
    valid &= require(vehicle.hasString("ident") && vehicle.getString("ident") == "probe",
                     "vehicle string state");
    vehicle.set("SteeringWheelOffset", 0.25);
    vehicle.updateFrame();
    valid &= require(vehicle.get("SteeringWheelOffset") == 0.25,
                     "local numeric state persists across frames");
    valid &= require(vehicle.getString("ident") == "probe",
                     "local string state persists across frames");
    return valid ? 0 : 1;
}