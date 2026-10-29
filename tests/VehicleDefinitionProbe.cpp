#include "app/vehicle/VehicleDefinition.h"
#include "core/Logger.h"
#include "map/MapTrafficRuntime.h"
#include "render/MatrixMath.h"
#include "ProbeTempDirectory.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <numbers>

int main() {
    using namespace openbus::rendering;
    for (const double heading : {0.0, 0.5, std::numbers::pi / 2.0, std::numbers::pi,
                                 -std::numbers::pi / 2.0}) {
        for (const double rise : {-1.0, 0.0, 1.0}) {
            for (const double rollDegrees : {-8.0, 0.0, 8.0}) {
                openbus::map::MapTrafficRouteSegment segment;
                segment.points = {{0.0, 0.0, 0.0},
                                  {10.0 * std::sin(heading), 10.0 * std::cos(heading), rise}};
                openbus::map::MapTrafficRoute route;
                route.segments = {segment};
                route.totalLength = std::hypot(10.0, rise);
                route.cumulativeEndDistances = {route.totalLength};
                const auto pose = openbus::map::sampleMapTrafficRoute(route, 5.0, 0.0);
                if (!pose) {
                    std::cerr << "Could not sample graded traffic route\n";
                    return 1;
                }
                const VehiclePlacement placement{
                    pose->position,
                    openbus::map::mapTrafficHeadingToVehicleYawDegrees(pose->headingRadians),
                    pose->pitchRadians * 180.0 / std::numbers::pi, rollDegrees};
                setModelViewMatrix(identityMatrix());
                applyVehiclePlacement(placement);
                const auto forward = transformPoint(modelViewMatrix(), {1.0, 0.0, 0.0, 0.0});
                const std::array<double, 3> expectedForward = {
                    10.0 * std::sin(heading) / route.totalLength,
                    10.0 * std::cos(heading) / route.totalLength, rise / route.totalLength};
                for (std::size_t axis = 0; axis < expectedForward.size(); ++axis) {
                    if (std::abs(forward[axis] - expectedForward[axis]) > 1.0e-8) {
                        std::cerr << "Rendered vehicle forward does not follow road grade: heading="
                                  << heading << " rise=" << rise << " roll=" << rollDegrees << '\n';
                        return 1;
                    }
                }
                const auto left = transformPoint(modelViewMatrix(), {0.0, 1.0, 0.0, 0.0});
                const double expectedLeftZ =
                    std::cos(pose->pitchRadians) *
                    std::sin(rollDegrees * std::numbers::pi / 180.0);
                if (std::abs(left[2] - expectedLeftZ) > 1.0e-8) {
                    std::cerr << "Rendered vehicle bank is not relative to its travel direction\n";
                    return 1;
                }
            }
        }
    }
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_vehicle_definition_probe");
    std::filesystem::create_directories(root / "model");
    const std::string logPath = (root / "diagnostics.log").string();
#ifdef _WIN32
    if (_putenv_s("OPENBUS_LOG_FILE", logPath.c_str()) != 0) {
#else
    if (setenv("OPENBUS_LOG_FILE", logPath.c_str(), 1) != 0) {
#endif
        std::cerr << "Could not configure diagnostic log path\n";
        return 1;
    }

    const std::filesystem::path vehicleConfigPath = root / "ai_car.ovh";
    std::ofstream vehicleConfig(vehicleConfigPath);
    vehicleConfig << "[friendlyname]\nTest\nAI car\nDefault\n"
                     "[model]\nmodel/model.cfg\n"
                     "[cog]\n0.1\n0\n1.2\n"
                     "[rot_pnt_long]\n-2\n"
                     "[rowdy_factor]\n0.5\n";
    vehicleConfig.close();

    std::ofstream modelConfig(root / "model/model.cfg");
    modelConfig << "[mesh]\ncar.o3d\n";
    modelConfig.close();
    std::ofstream mesh(root / "model/car.o3d", std::ios::binary);
    mesh.close();

    const std::filesystem::path modelConfigPath = root / "model/model.cfg";
    const std::shared_ptr<const SharedVehicleDefinition> definition =
        sharedVehicleDefinition(vehicleConfigPath, modelConfigPath, VehicleFileKind::Vehicle);
    const bool loaded = definition->vehicleConfiguration.friendlyVehicleName == "AI car" &&
                        definition->modelConfigPath == modelConfigPath &&
                        definition->modelRoot == root / "model" &&
                        !definition->modelConfiguration.diagnostics.hasErrors() &&
                        definition->modelConfiguration.parts.size() == 1 &&
                        definition->modelConfiguration.parts[0].objPath == root / "model/car.o3d" &&
                        definition->modelConfiguration.parts[0].color ==
                            std::array<double, 3>{1.0, 1.0, 1.0};
    Logger::Flush();
    std::ifstream logInput(logPath);
    const std::string log((std::istreambuf_iterator<char>(logInput)),
                          std::istreambuf_iterator<char>());
    const bool diagnosticsLogged =
        log.find("[cog]: horizontal center-of-gravity offsets are retained but not applied") !=
            std::string::npos &&
        log.find("[rot_pnt_long]: recognized and retained but not applied to vehicle physics") !=
            std::string::npos &&
        log.find("[rowdy_factor]: recognized but unsupported: family-specific ride behavior is "
                 "not implemented") != std::string::npos;
    logInput.close();
    openbus::probe::removeProbeDirectory(root);

    if (!loaded || !diagnosticsLogged) {
        std::cerr << "Shared vehicle definition did not load an OVH config and vehicle model\n";
        return 1;
    }
    return 0;
}
