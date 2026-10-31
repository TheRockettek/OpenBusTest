#pragma once

#include "core/PassengerVariables.h"
#include "core/Variables.h"

#include <array>
#include <cstddef>
#include <string>

namespace openbus::app::vehicle {

struct PassengerRequestVariables {
    std::array<bool, openbus::scripting::passengerDoorSlotCount> entries = {};
    std::array<bool, openbus::scripting::passengerDoorSlotCount> exits = {};
};

inline PassengerRequestVariables
passengerRequestVariables(std::size_t entryCount, std::size_t exitCount, bool atPassengerStop,
                          std::size_t waitingPassengers, bool boardingPassenger,
                          bool alightingPassenger) {
    PassengerRequestVariables requests;
    if (!atPassengerStop) {
        return requests;
    }
    if (entryCount == 1 && (waitingPassengers > 0 || boardingPassenger)) {
        requests.entries[0] = true;
    }
    if (exitCount == 1 && alightingPassenger) {
        requests.exits[0] = true;
    }
    return requests;
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
