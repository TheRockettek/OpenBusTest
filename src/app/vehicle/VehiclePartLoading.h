#pragma once

#include "assets/ObjLoader.h"
#include "config/ModelConfigTypes.h"
#include "render/MatrixMath.h"

#include <array>

namespace openbus::app::vehicle {

enum class VehiclePartLoadPriority { Required, Prefetch, Skip };

// Parts outside every [LOD] section are drawn at all distances, so they are always required.
// Other LODs of the current view are prefetched: draw selects LOD per part from the real eye
// distance, which can differ from the load distance and must never find an unloaded LOD.
inline VehiclePartLoadPriority vehiclePartLoadPriority(bool viewpointMatches, int partLodIndex,
                                                       int selectedLodIndex) {
    if (!viewpointMatches) {
        return VehiclePartLoadPriority::Skip;
    }
    if (selectedLodIndex < 0 || partLodIndex < 0 || partLodIndex == selectedLodIndex) {
        return VehiclePartLoadPriority::Required;
    }
    return VehiclePartLoadPriority::Prefetch;
}

inline std::array<double, 3> cameraEyeFromModelView(const openbus::rendering::Matrix4& view) {
    return {-(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]),
            -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]),
            -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14])};
}

inline void applyAnimationMeshOrigin(ModelAnimation& animation,
                                     const openbus::rendering::ParsedObj& mesh) {
    if (!animation.originFromMesh) {
        return;
    }

    std::array<double, 3> origin = mesh.boundsCenter;
    std::array<double, 9> rotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    if (mesh.hasTransform) {
        origin = {mesh.transform[13], -mesh.transform[12], mesh.transform[14]};
        rotation = {mesh.transform[0], mesh.transform[1], mesh.transform[2],
                    mesh.transform[4], mesh.transform[5], mesh.transform[6],
                    mesh.transform[8], mesh.transform[9], mesh.transform[10]};
    }
    if (!animation.hasOrigin) {
        animation.origin = origin;
        animation.hasOrigin = true;
    }
    animation.meshRotation = rotation;
    animation.hasMeshRotation = mesh.hasTransform;
    animation.meshTransform = openbus::rendering::makeMeshTransform(rotation, origin);
    animation.hasMeshTransform = true;
}

} // namespace openbus::app::vehicle
