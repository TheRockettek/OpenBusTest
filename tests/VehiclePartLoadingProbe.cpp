#include "app/vehicle/VehiclePartLoading.h"

#include <cmath>
#include <iostream>

int main() {
    using openbus::app::vehicle::cameraEyeFromModelView;
    using openbus::app::vehicle::vehiclePartLoadPriority;
    using openbus::app::vehicle::VehiclePartLoadPriority;

    if (vehiclePartLoadPriority(true, -1, 2) != VehiclePartLoadPriority::Required) {
        std::cerr << "parts outside [LOD] sections must load for every selected LOD\n";
        return 1;
    }
    if (vehiclePartLoadPriority(true, 1, 1) != VehiclePartLoadPriority::Required ||
        vehiclePartLoadPriority(true, 2, -1) != VehiclePartLoadPriority::Required) {
        std::cerr << "selected-LOD parts must be required\n";
        return 1;
    }
    if (vehiclePartLoadPriority(true, 0, 1) != VehiclePartLoadPriority::Prefetch) {
        std::cerr << "other LODs of the current view must be prefetched so LOD switches never "
                     "draw an unloaded vehicle\n";
        return 1;
    }
    if (vehiclePartLoadPriority(false, 1, 1) != VehiclePartLoadPriority::Skip ||
        vehiclePartLoadPriority(false, -1, 1) != VehiclePartLoadPriority::Skip) {
        std::cerr << "parts for another viewpoint must not be loaded\n";
        return 1;
    }

    // World-to-eye view rotated 90 degrees about Z, with the eye at (10, 20, 3).
    const openbus::rendering::Matrix4 view = {0.0,  1.0, 0.0, 0.0, -1.0, 0.0,  0.0, 0.0,
                                              0.0,  0.0, 1.0, 0.0, 20.0, -10.0, -3.0, 1.0};
    const auto eye = cameraEyeFromModelView(view);
    if (std::abs(eye[0] - 10.0) > 1.0e-9 || std::abs(eye[1] - 20.0) > 1.0e-9 ||
        std::abs(eye[2] - 3.0) > 1.0e-9) {
        std::cerr << "camera eye was not recovered from the view matrix\n";
        return 1;
    }
    return 0;
}
