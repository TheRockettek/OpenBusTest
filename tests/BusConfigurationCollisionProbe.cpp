#include "physics/BusConfiguration.h"
#include "config/ModelConfigTypes.h"
#include "config/VehicleConfigLoader.h"
#include "ProbeTempDirectory.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {

void writeBusConfig(const std::filesystem::path& path, const std::string& extra = {},
                    const std::string& centerOfGravity = "[schwerpunkt]\n1.2\n") {
    std::ofstream config(path);
    config << "[model]\nmodel.cfg\n"
              "[mass]\n12.0\n"
              "[boundingbox]\n2.4\n12.0\n3.2\n0\n0\n1.6\n"
              << centerOfGravity << "[newachse]\nachse_long\n0\n"
              "achse_maxwidth\n2.2\n"
              "achse_minwidth\n1.8\n"
              "achse_raddurchmesser\n1.0\n"
              "achse_feder\n200\n"
              "achse_maxforce\n100\n"
              "achse_daempfer\n20\n"
              "achse_antrieb\n0\n"
              << extra;
}

bool nearlyEqual(double first, double second) {
    return std::abs(first - second) < 1.0e-9;
}

} // namespace

int main() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_bus_collision_probe");
    std::filesystem::create_directories(root);
    writeBusConfig(root / "bus.bus", "[rollwiderstand]\n1500\n");

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
    const bool authoredResistanceWasTransferred =
        meshConfiguration.rollingResistanceForceNewtons.has_value() &&
        nearlyEqual(*meshConfiguration.rollingResistanceForceNewtons, 1500.0);
    std::ostringstream exportedConfiguration;
    writeBusConfigurationJson(exportedConfiguration, meshConfiguration);
    const bool authoredResistanceAppearsInExport =
        exportedConfiguration.str().find("\"rolling_resistance_force_N\": 1500") !=
        std::string::npos;

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

    writeBusConfig(root / "bus.bus", "[rollwiderstand]\n-1\n");
    bool negativeRollingResistanceRejected = false;
    try {
        static_cast<void>(loadBusConfiguration(root / "bus.bus"));
    } catch (const std::runtime_error&) {
        negativeRollingResistanceRejected = true;
    }

    const std::filesystem::path nestedModelRoot = root / "Model";
    const std::filesystem::path nestedConfigDirectory =
        nestedModelRoot / "Configuration Files" / "Regional";
    std::filesystem::create_directories(nestedConfigDirectory);
    {
        std::ofstream bus(root / "nested.bus");
        bus << "[model]\nmodel\\configuration files\\regional\\model.cfg\n"
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
    const std::filesystem::path nestedMeshPath = nestedModelRoot / "body.obj";
    {
        std::ofstream nestedMesh(nestedMeshPath);
        nestedMesh << "v 0 0 0\n";
    }
    {
        std::ofstream nestedModel(nestedConfigDirectory / "model.cfg");
        nestedModel << "[mesh]\nbody.obj\n";
    }
    const ModelConfig nestedModelConfiguration =
        loadBusModelConfiguration(root / "nested.bus");
    const std::filesystem::path resolvedNestedModelPath =
        modelConfigurationPathForBusFromConfig(root / "nested.bus");
    const bool nestedModelMeshResolved =
        resolvedNestedModelPath == nestedConfigDirectory / "model.cfg" &&
        !nestedModelConfiguration.diagnostics.hasErrors() &&
        nestedModelConfiguration.parts.size() == 1 &&
        nestedModelConfiguration.parts.front().objPath == nestedMeshPath;

    const auto hasCogDiagnostic = [](const VehicleConfig& config,
                                     ConfigurationDiagnostic::Severity severity,
                                     const std::string& message) {
        return std::any_of(config.diagnostics.entries.begin(), config.diagnostics.entries.end(),
                           [&](const ConfigurationDiagnostic& diagnostic) {
                               return diagnostic.keyword == "cog" &&
                                      diagnostic.severity == severity && diagnostic.line != 0 &&
                                      diagnostic.message.find(message) != std::string::npos;
                           });
    };
    const auto cogPath = root / "cog.bus";
    writeBusConfig(cogPath, "", "[cog]\n0.1\n-0.2\n1.4\n");
    const VehicleConfig cog = loadVehicleConfig(cogPath, VehicleFileKind::Bus, false);
    const BusConfiguration cogPhysics = loadBusConfiguration(cogPath);
    const bool cogApplied =
        !cog.diagnostics.hasErrors() && cog.centerOfGravity.has_value() &&
        *cog.centerOfGravity == std::array<double, 3>{0.1, -0.2, 1.4} &&
        nearlyEqual(cogPhysics.centerOfGravityHeight, 1.4) &&
        nearlyEqual(cogPhysics.collisionOffsetZ, 0.8) &&
        hasCogDiagnostic(cog, ConfigurationDiagnostic::Severity::Warning, "not applied");
    bool precedenceWorks = true;
    for (const std::string& records :
         {"[cog]\n0\n0\n1.4\n[schwerpunkt]\n1.2\n",
          "[schwerpunkt]\n1.2\n[cog]\n0\n0\n1.4\n"}) {
        writeBusConfig(cogPath, "", records);
        const VehicleConfig config = loadVehicleConfig(cogPath, VehicleFileKind::Bus, false);
        const BusConfiguration physics = loadBusConfiguration(cogPath);
        precedenceWorks = precedenceWorks && !config.diagnostics.hasErrors() &&
                          nearlyEqual(physics.centerOfGravityHeight, 1.2) &&
                          hasCogDiagnostic(config, ConfigurationDiagnostic::Severity::Warning,
                                           "takes precedence");
    }
    writeBusConfig(cogPath, "", "[cog]\n0\n0\n1.2\n[schwerpunkt]\n1.2\n");
    const VehicleConfig matchingCog = loadVehicleConfig(cogPath, VehicleFileKind::Bus, false);
    precedenceWorks = precedenceWorks && !matchingCog.diagnostics.hasErrors() &&
                      nearlyEqual(loadBusConfiguration(cogPath).centerOfGravityHeight, 1.2) &&
                      !hasCogDiagnostic(matchingCog, ConfigurationDiagnostic::Severity::Warning,
                                        "takes precedence");
    bool invalidCogRejected = true;
    for (const std::string& records :
         {"[cog]\n0\n0\n0\n", "[cog]\n0\nnan\n1.4\n",
          "[cog]\n0\n0\n[schwerpunkt]\n1.2\n"}) {
        writeBusConfig(cogPath, "", records);
        const VehicleConfig config = loadVehicleConfig(cogPath, VehicleFileKind::Bus, false);
        bool rejected = false;
        try {
            static_cast<void>(loadBusConfiguration(cogPath));
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        const std::string expectedMessage =
            records == "[cog]\n0\n0\n0\n" ? "positive" : "expected";
        invalidCogRejected = invalidCogRejected && rejected &&
                             hasCogDiagnostic(config, ConfigurationDiagnostic::Severity::Error,
                                              expectedMessage);
    }
    writeBusConfig(cogPath, "", "[cog]\n0\n0\n1.4\n[schwerpunkt]\nnan\n");
    try {
        static_cast<void>(loadBusConfiguration(cogPath));
        invalidCogRejected = false;
    } catch (const std::runtime_error&) {
    }
    openbus::probe::removeProbeDirectory(root);
    if (!meshWasBuilt || !modelBoundsApplied || !wheelColliderMatchesVisualEnvelope ||
        !authoredResistanceWasTransferred || !authoredResistanceAppearsInExport ||
        !boxFallbackApplied ||
        !negativeRollingResistanceRejected || !nestedModelMeshResolved || !cogApplied ||
        !precedenceWorks || !invalidCogRejected) {
        std::cerr << "Bus collision mesh, wheel collider fit, nocollision filtering, model "
                     "bounding box, or nested model-root resolution failed\n";
        return 1;
    }
    return 0;
}
