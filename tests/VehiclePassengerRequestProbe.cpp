#include "app/vehicle/PassengerRequestVariables.h"

#include <iostream>

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
    return 0;
}
