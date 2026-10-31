#pragma once

#include <array>
#include <cstddef>

namespace openbus::app::vehicle {

struct PassengerRequestVariables {
    std::array<bool, 8> entries = {};
    std::array<bool, 8> exits = {};
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

} // namespace openbus::app::vehicle
