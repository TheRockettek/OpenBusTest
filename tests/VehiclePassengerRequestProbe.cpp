#include "app/vehicle/PassengerRequestVariables.h"

#include <iostream>
#include <string>

int main() {
    using openbus::app::vehicle::passengerRequestVariables;

    const auto waiting = passengerRequestVariables(0, std::nullopt, true, 2, false, false);
    if (!waiting.entries[0] || waiting.exits[0]) {
        std::cerr << "waiting passengers did not request the single configured entry\n";
        return 1;
    }

    const auto boarding = passengerRequestVariables(0, std::nullopt, true, 0, true, false);
    if (!boarding.entries[0]) {
        std::cerr << "an in-progress boarder did not keep the entry request active\n";
        return 1;
    }

    const auto alighting = passengerRequestVariables(0, 0, true, 0, false, true);
    if (alighting.entries[0] || !alighting.exits[0]) {
        std::cerr << "an in-progress alighter did not request the single configured exit\n";
        return 1;
    }

    const auto multipleDoors = passengerRequestVariables(0, 1, true, 2, true, true);
    const auto inactiveStop = passengerRequestVariables(0, 1, false, 2, true, true);
    if (!multipleDoors.entries[0] || multipleDoors.entries[1] || multipleDoors.exits[0] ||
        !multipleDoors.exits[1] || inactiveStop.entries[0] || inactiveStop.exits[1]) {
        std::cerr << "requests did not follow the assigned active doorways\n";
        return 1;
    }
    const auto unsupported = passengerRequestVariables(
        openbus::scripting::passengerDoorSlotCount, std::nullopt, true, 1, false, false);
    if (unsupported.entries[0]) {
        std::cerr << "unsupported door slot received a passenger request\n";
        return 1;
    }

    openbus::scripting::Vehicle variables;
    using openbus::app::vehicle::PassengerDoorKind;
    using openbus::app::vehicle::passengerDoorIsOpen;
    variables.set("pax_exit0_open", 1.0F);
    variables.set("pax_exit1_open", 0.0F);
    variables.set("pax_entry0_open", 1.0F);
    if (!passengerDoorIsOpen(variables, PassengerDoorKind::Entry, 0) ||
        !passengerDoorIsOpen(variables, PassengerDoorKind::Exit, 0) ||
        passengerDoorIsOpen(variables, PassengerDoorKind::Exit, 1) ||
        passengerDoorIsOpen(variables, PassengerDoorKind::Exit, std::nullopt) ||
        passengerDoorIsOpen(variables, PassengerDoorKind::Exit,
                            openbus::scripting::passengerDoorSlotCount)) {
        std::cerr << "door gating did not use the selected configured slot\n";
        return 1;
    }
    variables.set("door_0", 1.0F);
    variables.set("door_1", 0.0F);
    openbus::app::vehicle::synchronizePassengerDoorOpenVariables(variables);
    if (!passengerDoorIsOpen(variables, PassengerDoorKind::Entry, 0) ||
        !passengerDoorIsOpen(variables, PassengerDoorKind::Exit, 0) ||
        passengerDoorIsOpen(variables, PassengerDoorKind::Entry, 1) ||
        passengerDoorIsOpen(variables, PassengerDoorKind::Exit, 1)) {
        std::cerr << "vehicle door state was not synchronized to passenger door variables\n";
        return 1;
    }
    variables.set("door_0", 0.0F);
    openbus::app::vehicle::synchronizePassengerDoorOpenVariables(variables);
    if (passengerDoorIsOpen(variables, PassengerDoorKind::Entry, 0) ||
        passengerDoorIsOpen(variables, PassengerDoorKind::Exit, 0)) {
        std::cerr << "closed vehicle door left passenger door variables open\n";
        return 1;
    }
    const std::string unsupportedSuffix =
        std::to_string(openbus::scripting::passengerDoorSlotCount) + "_req";
    if (variables.has("pax_entry" + unsupportedSuffix) ||
        variables.has("pax_exit" + unsupportedSuffix)) {
        std::cerr << "unexpected predefined passenger request beyond the supported slots\n";
        return 1;
    }
    variables.set("pax_entry" + unsupportedSuffix, 1.0F);
    variables.set("pax_exit" + unsupportedSuffix, 1.0F);
    for (std::size_t index = 0; index < openbus::scripting::passengerDoorSlotCount; ++index) {
        const std::string suffix = std::to_string(index) + "_req";
        const std::string openSuffix = std::to_string(index) + "_open";
        if (!variables.has("pax_entry" + suffix) || !variables.has("pax_exit" + suffix) ||
            !variables.has("pax_entry" + openSuffix) ||
            !variables.has("pax_exit" + openSuffix)) {
            std::cerr << "predefined passenger door variables missing at index " << index << '\n';
            return 1;
        }
        variables.set("pax_entry" + suffix, 1.0F);
        variables.set("pax_exit" + suffix, 1.0F);
    }
    openbus::app::vehicle::publishPassengerRequestVariables(
        variables, openbus::app::vehicle::PassengerRequestVariables{});
    for (std::size_t index = 0; index < openbus::scripting::passengerDoorSlotCount; ++index) {
        const std::string suffix = std::to_string(index) + "_req";
        if (variables.get("pax_entry" + suffix) != 0.0F ||
            variables.get("pax_exit" + suffix) != 0.0F) {
            std::cerr << "configuration change left a stale passenger request at index "
                      << index << '\n';
            return 1;
        }
    }
    openbus::app::vehicle::publishPassengerRequestVariables(variables, waiting);
    if (variables.get("pax_entry0_req") != 1.0F) {
        std::cerr << "active passenger request was not published\n";
        return 1;
    }
    openbus::app::vehicle::publishPassengerRequestVariables(variables, multipleDoors);
    if (variables.get("pax_entry0_req") != 1.0F ||
        variables.get("pax_exit1_req") != 1.0F ||
        variables.get("pax_exit0_req") != 0.0F) {
        std::cerr << "selected multi-door requests were not published\n";
        return 1;
    }
    variables.set("pax_exit" +
                      std::to_string(openbus::scripting::passengerDoorSlotCount - 1) + "_req",
                  1.0F);
    openbus::app::vehicle::publishPassengerRequestVariables(
        variables, openbus::app::vehicle::PassengerRequestVariables{});
    for (std::size_t index = 0; index < openbus::scripting::passengerDoorSlotCount; ++index) {
        const std::string suffix = std::to_string(index) + "_req";
        if (variables.get("pax_entry" + suffix) != 0.0F ||
            variables.get("pax_exit" + suffix) != 0.0F) {
            std::cerr << "absent passenger service left a stale request at index " << index << '\n';
            return 1;
        }
    }
    if (variables.get("pax_entry" + unsupportedSuffix) != 1.0F ||
        variables.get("pax_exit" + unsupportedSuffix) != 1.0F) {
        std::cerr << "publishing requests changed script-defined unsupported slots\n";
        return 1;
    }
    return 0;
}
