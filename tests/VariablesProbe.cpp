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
    valid &= require(simulation.sharedVariables().has("timegap"), "system timegap");
    valid &= require(!simulation.sharedVariables().has("throttle"), "system excludes throttle");
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
    valid &= require(!sceneryObject.supportsSystemTrigger("collision"),
                     "scenery excludes vehicle trigger");

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