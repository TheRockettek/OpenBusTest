#include "app/vehicle/VehiclePartLoading.h"

#include <cmath>
#include <iostream>

int main() {
    using openbus::app::vehicle::applyAnimationMeshOrigin;
    using openbus::app::vehicle::cameraEyeFromModelView;
    using openbus::app::vehicle::vehiclePartLoadPriority;
    using openbus::app::vehicle::VehiclePartLoadPriority;

    openbus::rendering::ParsedObj mesh;
    mesh.boundsCenter = {4.0, -2.0, 7.0};
    ModelAnimation boundsOriginAnimation;
    boundsOriginAnimation.originFromMesh = true;
    applyAnimationMeshOrigin(boundsOriginAnimation, mesh);
    if (!boundsOriginAnimation.hasMeshTransform ||
        boundsOriginAnimation.meshTransform[12] != mesh.boundsCenter[0] ||
        boundsOriginAnimation.meshTransform[13] != mesh.boundsCenter[1] ||
        boundsOriginAnimation.meshTransform[14] != mesh.boundsCenter[2]) {
        std::cerr << "origin_from_mesh did not use the parsed bounds center fallback\n";
        return 1;
    }

    mesh.hasTransform = true;
    mesh.transform[12] = 2.0;
    mesh.transform[13] = 5.0;
    mesh.transform[14] = 3.0;
    ModelAnimation transformOriginAnimation;
    transformOriginAnimation.originFromMesh = true;
    applyAnimationMeshOrigin(transformOriginAnimation, mesh);
    if (!transformOriginAnimation.hasMeshTransform ||
        transformOriginAnimation.meshTransform[12] != 5.0 ||
        transformOriginAnimation.meshTransform[13] != -2.0 ||
        transformOriginAnimation.meshTransform[14] != 3.0) {
        std::cerr << "origin_from_mesh did not preserve source transform metadata\n";
        return 1;
    }

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
