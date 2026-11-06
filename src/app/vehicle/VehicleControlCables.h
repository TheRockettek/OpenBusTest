#pragma once

#include "config/VehicleConfigLoader.h"
#include "core/Variables.h"

#include <vector>

namespace openbus::app::vehicle {

inline bool controlCableSidesConnect(VehicleControlCableSide first,
                                     VehicleControlCableSide second) {
    return (first == VehicleControlCableSide::Center &&
            second == VehicleControlCableSide::Center) ||
           (first == VehicleControlCableSide::Left && second == VehicleControlCableSide::Right) ||
           (first == VehicleControlCableSide::Right && second == VehicleControlCableSide::Left);
}

inline void synchronizeControlCables(
    Variables& rearVehicleVariables, const std::vector<VehicleControlCable>& rearCables,
    Variables& frontVehicleVariables, const std::vector<VehicleControlCable>& frontCables) {
    struct ReadAssignment {
        Variables* variables;
        const std::string* name;
        float value;
    };

    std::vector<ReadAssignment> assignments;
    assignments.reserve(rearCables.size() + frontCables.size());

    for (const VehicleControlCable& rearCable : rearCables) {
        assignments.push_back({&rearVehicleVariables, &rearCable.readVariable, 0.0F});
    }
    for (const VehicleControlCable& frontCable : frontCables) {
        assignments.push_back({&frontVehicleVariables, &frontCable.readVariable, 0.0F});
    }

    for (const VehicleControlCable& rearCable : rearCables) {
        for (const VehicleControlCable& frontCable : frontCables) {
            if (rearCable.number != frontCable.number ||
                !controlCableSidesConnect(rearCable.side, frontCable.side) ||
                rearVehicleVariables.get(rearCable.couplingVariable) != 1.0F ||
                frontVehicleVariables.get(frontCable.couplingVariable) != 1.0F) {
                continue;
            }
            assignments.push_back(
                {&rearVehicleVariables, &rearCable.readVariable,
                 frontVehicleVariables.get(frontCable.writeVariable)});
            assignments.push_back(
                {&frontVehicleVariables, &frontCable.readVariable,
                 rearVehicleVariables.get(rearCable.writeVariable)});
        }
    }

    for (const ReadAssignment& assignment : assignments) {
        if (assignment.value == 0.0F) {
            assignment.variables->set(*assignment.name, 0.0F);
        }
    }
    for (const ReadAssignment& assignment : assignments) {
        if (assignment.value != 0.0F) {
            assignment.variables->set(*assignment.name, assignment.value);
        }
    }
}

} // namespace openbus::app::vehicle
