#include "app/vehicle/PassengerRequestVariables.h"

#include <iostream>
#include <string>

int main() {
    using openbus::app::vehicle::passengerRequestVariables;

    const auto waiting = passengerRequestVariables(1, 1, true, 2, false, false);
    if (!waiting.entries[0] || waiting.exits[0]) {
        std::cerr << "waiting passengers did not request the single configured entry\n";
        return 1;
    }

    const auto boarding = passengerRequestVariables(1, 1, true, 0, true, false);
    if (!boarding.entries[0]) {
        std::cerr << "an in-progress boarder did not keep the entry request active\n";
        return 1;
    }

    const auto alighting = passengerRequestVariables(1, 1, true, 0, false, true);
    if (alighting.entries[0] || !alighting.exits[0]) {
        std::cerr << "an in-progress alighter did not request the single configured exit\n";
        return 1;
    }

    const auto multipleEntries = passengerRequestVariables(2, 1, true, 2, true, false);
    const auto multipleExits = passengerRequestVariables(1, 2, true, 0, false, true);
    const auto inactiveStop = passengerRequestVariables(1, 1, false, 2, true, true);
    if (multipleEntries.entries[0] || multipleExits.exits[0] ||
        inactiveStop.entries[0] || inactiveStop.exits[0]) {
        std::cerr << "requests were assigned without an unambiguous active doorway\n";
        return 1;
    }

    openbus::scripting::Vehicle variables;
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
    openbus::app::vehicle::publishPassengerRequestVariables(variables, multipleEntries);
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
