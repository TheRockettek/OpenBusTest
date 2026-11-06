#include "app/vehicle/VehicleControlCables.h"

#include <iostream>
#include <vector>

int main() {
    using openbus::app::vehicle::synchronizeControlCables;

    const std::vector<VehicleControlCable> rearCables = {
        {VehicleControlCableSide::Left, 0, "rear_read", "rear_write", "rear_coupled"},
        {VehicleControlCableSide::Center, 1, "rear_center_read", "rear_center_write",
         "rear_center_coupled"},
        {VehicleControlCableSide::Left, 4, "rear_unmatched_read", "rear_unmatched_write",
         "rear_unmatched_coupled"}};
    const std::vector<VehicleControlCable> frontCables = {
        {VehicleControlCableSide::Right, 0, "front_read", "front_write", "front_coupled"},
        {VehicleControlCableSide::Center, 1, "front_center_read", "front_center_write",
         "front_center_coupled"},
        {VehicleControlCableSide::Right, 5, "front_unmatched_read", "front_unmatched_write",
         "front_unmatched_coupled"}};

    Variables rearVariables;
    rearVariables.setValues({{"rear_coupled", 1.0F},
                             {"rear_write", 1.0F},
                             {"rear_read", 99.0F},
                             {"rear_center_coupled", 1.0F},
                             {"rear_center_write", 0.25F},
                             {"rear_center_read", 99.0F},
                             {"rear_unmatched_read", 99.0F}});
    Variables frontVariables;
    frontVariables.setValues({{"front_coupled", 1.0F},
                              {"front_write", 0.5F},
                              {"front_read", 99.0F},
                              {"front_center_coupled", 1.0F},
                              {"front_center_write", 0.75F},
                              {"front_center_read", 99.0F},
                              {"front_unmatched_read", 99.0F}});

    synchronizeControlCables(rearVariables, rearCables, frontVariables, frontCables);
    if (rearVariables.get("rear_read") != 0.5F ||
        frontVariables.get("front_read") != 1.0F ||
        rearVariables.get("rear_center_read") != 0.75F ||
        frontVariables.get("front_center_read") != 0.25F ||
        rearVariables.get("rear_unmatched_read") != 0.0F ||
        frontVariables.get("front_unmatched_read") != 0.0F) {
        std::cerr << "Control-cable values were not transferred to the matching read variables\n";
        return 1;
    }

    frontVariables.set("front_coupled", 0.0F);
    synchronizeControlCables(rearVariables, rearCables, frontVariables, frontCables);
    if (rearVariables.get("rear_read") != 0.0F ||
        frontVariables.get("front_read") != 0.0F) {
        std::cerr << "Inactive electrical coupling did not clear cable read variables\n";
        return 1;
    }

    return 0;
}
