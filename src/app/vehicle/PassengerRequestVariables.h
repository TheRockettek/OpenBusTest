#pragma once

#include "core/PassengerVariables.h"
#include "core/Variables.h"
#include "map/MapTrafficRuntime.h"

#include <array>
#include <cstddef>
#include <optional>
#include <string>

namespace openbus::app::vehicle {

struct PassengerRequestVariables {
    std::array<bool, openbus::scripting::passengerDoorSlotCount> entries = {};
    std::array<bool, openbus::scripting::passengerDoorSlotCount> exits = {};
};

enum class PassengerDoorKind { Entry, Exit };

inline bool passengerDoorIsOpen(const Variables& variables, PassengerDoorKind kind,
                                std::optional<std::size_t> doorIndex) {
    if (!doorIndex || *doorIndex >= openbus::scripting::passengerDoorSlotCount) {
        return false;
    }
    const std::string prefix = kind == PassengerDoorKind::Entry ? "pax_entry" : "pax_exit";
    return variables.get(prefix + std::to_string(*doorIndex) + "_open") > 0.5F;
}

inline void synchronizePassengerDoorOpenVariables(Variables& variables) {
    for (std::size_t index = 0; index < openbus::scripting::passengerDoorSlotCount; ++index) {
        const float open = variables.get("door_" + std::to_string(index)) > 0.5F ? 1.0F : 0.0F;
        variables.set("pax_entry" + std::to_string(index) + "_open", open);
        variables.set("pax_exit" + std::to_string(index) + "_open", open);
    }
}

inline PassengerRequestVariables
passengerRequestVariables(std::optional<std::size_t> entryDoor,
                          std::optional<std::size_t> exitDoor, bool atPassengerStop,
                          std::size_t waitingPassengers, bool boardingPassenger,
                          bool alightingPassenger) {
    PassengerRequestVariables requests;
    if (!atPassengerStop) {
        return requests;
    }
    if (entryDoor && *entryDoor < requests.entries.size() &&
        (waitingPassengers > 0 || boardingPassenger)) {
        requests.entries[*entryDoor] = true;
    }
    if (exitDoor && *exitDoor < requests.exits.size() && alightingPassenger) {
        requests.exits[*exitDoor] = true;
    }
    return requests;
}

inline PassengerRequestVariables
passengerServiceRequestVariables(std::optional<std::size_t> entryDoor,
                                 std::optional<std::size_t> exitDoor,
                                 const openbus::map::MapPassengerServiceRuntime& service) {
    const int stopId = service.currentStopId();
    return passengerRequestVariables(
        entryDoor, exitDoor, service.dwelling() && stopId != 0,
        service.canAcceptWaitingPassenger() ? service.eligibleWaitingPassengers() : 0,
        service.boardingPassenger(), service.alightingPassenger());
}

inline void publishPassengerRequestVariables(Variables& variables,
                                             const PassengerRequestVariables& requests) {
    for (std::size_t index = 0; index < requests.entries.size(); ++index) {
        const std::string suffix = std::to_string(index) + "_req";
        variables.set("pax_entry" + suffix, requests.entries[index] ? 1.0F : 0.0F);
        variables.set("pax_exit" + suffix, requests.exits[index] ? 1.0F : 0.0F);
    }
}

} // namespace openbus::app::vehicle
