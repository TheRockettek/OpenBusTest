#include "BusConfiguration.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void writeBusConfig(const std::filesystem::path& path) {
    std::ofstream config(path);
    config << "[model]\nmodel.cfg\n"
              "[mass]\n12.0\n"
              "[boundingbox]\n2.4\n12.0\n3.2\n0\n0\n1.6\n"
              "[schwerpunkt]\n1.2\n"
              "[newachse]\nachse_long\n0\n"
              "achse_maxwidth\n2.2\n"
              "achse_minwidth\n1.8\n"
              "achse_raddurchmesser\n1.0\n"
              "achse_feder\n200\n"
              "achse_maxforce\n100\n"
              "achse_daempfer\n20\n"
              "achse_antrieb\n0\n";
}

bool nearlyEqual(double first, double second) {
    return std::abs(first - second) < 1.0e-9;
}

} // namespace

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus_bus_collision_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    writeBusConfig(root / "bus.bus");

    std::ofstream mesh(root / "body.obj");
    mesh << "v 0 0 0\n";
    mesh.close();
    std::ofstream collisionMesh(root / "collision.obj");
    collisionMesh << "v 2 0 -1\nv 2 0 1\nv -2 0 0\nf 1 2 3\n";
    collisionMesh.close();
    constexpr double visualWheelRadius = 0.52;
    std::ofstream wheelMesh(root / "wheel.obj");
    wheelMesh << "v -0.145 " << visualWheelRadius << " 0\n"
              << "v -0.145 0 " << visualWheelRadius << "\n"
              << "v -0.145 " << -visualWheelRadius << " 0\n"
              << "v -0.145 0 " << -visualWheelRadius << "\n"
              << "v 0.145 " << visualWheelRadius << " 0\n"
              << "v 0.145 0 " << visualWheelRadius << "\n"
              << "v 0.145 " << -visualWheelRadius << " 0\n"
              << "v 0.145 0 " << -visualWheelRadius << "\n"
                 "f 1 2 3\nf 5 6 7\n";
    wheelMesh.close();

    const std::filesystem::path modelPath = root / "model.cfg";
    auto writeModelConfig = [&](bool noCollision) {
        std::ofstream model(modelPath, std::ios::trunc);
        model << "[mesh]\nbody.obj\n"
                 "[mesh]\nwheel.obj\n"
                 "[mesh_ident]\nwheel_frontleft\n"
                 "[newanim]\n"
                 "anim_rot\nwheel_rotation_0_l\n1\n"
                 "[collision_mesh]\ncollision.obj\n";
        if (noCollision) {
            model << "[nocollision]\n";
        }
        model << "[boundingbox]\n2.5\n13.5\n3.66\n0.1\n-0.3\n2.2\n";
    };

    writeModelConfig(false);
    BusConfiguration meshConfiguration{};
    try {
        meshConfiguration = loadBusConfiguration(root / "bus.bus");
    } catch (const std::exception& error) {
        std::cerr << "Collision-mesh config load failed: " << error.what() << '\n';
        return 10;
    }
    const bool meshWasBuilt = meshConfiguration.hasCollisionMesh &&
                              meshConfiguration.collisionMeshVertices.size() == 9 &&
                              meshConfiguration.collisionMeshIndices == std::vector<int>{0, 1, 2} &&
                              nearlyEqual(meshConfiguration.collisionMeshVertices[0], -1.0) &&
                              nearlyEqual(meshConfiguration.collisionMeshVertices[1], -2.0) &&
                              nearlyEqual(meshConfiguration.collisionMeshVertices[3], 1.0) &&
                              nearlyEqual(meshConfiguration.collisionMeshVertices[4], -2.0) &&
                              nearlyEqual(meshConfiguration.collisionMeshVertices[6], 0.0) &&
                              nearlyEqual(meshConfiguration.collisionMeshVertices[7], 2.0);
    const bool modelBoundsApplied =
        nearlyEqual(meshConfiguration.collisionWidth, 2.5) &&
        nearlyEqual(meshConfiguration.collisionLength, 13.5) &&
        nearlyEqual(meshConfiguration.collisionHeight, 3.66) &&
        nearlyEqual(meshConfiguration.collisionOffsetX, -0.3) &&
        nearlyEqual(meshConfiguration.collisionOffsetY, -0.1) &&
        nearlyEqual(meshConfiguration.collisionOffsetZ, 1.0);
    const bool wheelColliderMatchesVisualEnvelope =
        nearlyEqual(meshConfiguration.axles[0].wheelDiameter, visualWheelRadius * 2.0) &&
        nearlyEqual(meshConfiguration.wheelRadius, visualWheelRadius);

    writeModelConfig(true);
    BusConfiguration noCollisionConfiguration{};
    try {
        noCollisionConfiguration = loadBusConfiguration(root / "bus.bus");
    } catch (const std::exception& error) {
        std::cerr << "No-collision config load failed: " << error.what() << '\n';
        return 11;
    }
    const bool boxFallbackApplied = !noCollisionConfiguration.hasCollisionMesh &&
                                    noCollisionConfiguration.collisionEnabled &&
                                    nearlyEqual(noCollisionConfiguration.collisionWidth, 2.5) &&
                                    nearlyEqual(noCollisionConfiguration.collisionLength, 13.5) &&
                                    nearlyEqual(noCollisionConfiguration.collisionHeight, 3.66);

    std::filesystem::remove_all(root);
    if (!meshWasBuilt || !modelBoundsApplied || !wheelColliderMatchesVisualEnvelope ||
        !boxFallbackApplied) {
        std::cerr << "Bus collision mesh, wheel collider fit, nocollision filtering, or model "
                     "bounding box failed\n";
        return 1;
    }
    return 0;
}
