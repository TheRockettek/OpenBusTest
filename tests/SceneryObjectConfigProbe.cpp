#include "config/SceneryObjectConfigLoader.h"
#include "ProbeTempDirectory.h"

#include "map/MapSplineGeometry.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

namespace {

bool require(bool condition, const char* description) {
    if (!condition) {
        std::cerr << "failed: " << description << '\n';
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    if (argc == 2) {
        const SceneryObjectConfig configuration = loadSceneryObjectFile(argv[1]);
        bool valid = require(configuration.trafficPaths.size() == 40,
                             "Bath University A SCO paths parse");
        valid &= require(configuration.onlyEditor,
                         "Bath University A is marked editor-only");
        valid &= require(configuration.absoluteHeight,
                         "Bath University A uses absolute height");
        if (configuration.trafficPaths.size() == 40) {
            const SceneryObjectTrafficPath& curve = configuration.trafficPaths[0];
            const SceneryObjectTrafficPath& continuation = configuration.trafficPaths[3];
            const auto samples = openbus::map::tessellateMapSpline(
                curve.startPosition[0], curve.startPosition[1], curve.headingDegrees,
                curve.length, curve.radius);
            valid &= require(!samples.empty() &&
                                 std::hypot(samples.back().x - continuation.startPosition[0],
                                            samples.back().y - continuation.startPosition[1]) <
                                     0.01,
                             "Bath University A curved path ends at its continuation");
        }
        return valid ? 0 : 1;
    }
    if (argc != 1) {
        return 2;
    }

    const std::filesystem::path directory =
        openbus::probe::makeProbeDirectory("openbus-scenery-object-config-probe");
    const std::filesystem::path path = directory / "north-parade-pulteney-road.sco";
    const std::filesystem::path singleStopPath = directory / "single-stop.sco";
    const std::filesystem::path remappedPath = directory / "remapped-light.sco";
    const std::filesystem::path bathRoundaboutPath = directory / "bath-roundabout.sco";
    const std::filesystem::path polePath = directory / "pole.sco";
    const std::filesystem::path invalidPolePath = directory / "invalid-pole.sco";
    const std::filesystem::path nonFinitePolePath = directory / "non-finite-pole.sco";
    {
        std::ofstream output(path);
        if (!output) {
            std::cerr << "failed: create scenery object fixture\n";
            openbus::probe::removeProbeDirectory(directory);
            return 1;
        }
        output << "[mesh]\nmesh.x\n"
                  "[trafficlight]\n"
                  "[traffic_lights_group]\n10\n"
                  "[traffic_light]\nRoad\n"
                  "[phase]\n0\n3\n"
                  "[phase]\n6\n7\n"
                  "[approachdist]\n5\n"
                  "[traffic_light]\nCrossing\n"
                  "[phase]\n3\n5\n"
                  "[phase]\n9\n5\n"
                  "[traffic_light]\nDoNotUse\n"
                  "[phase]\n0\n4\n"
                  "[phase]\n6\n4\n"
                  "[path]\n"
                  "1\n2\n3\n4\n5\n6\n7\n8\n0\n2.5\n0\n0\n"
                  "[use_traffic_light]\n0\n"
                  "[traffic_light_stop]\n0\n1\n2\n"
                  "[traffic_light_stop]\n1\n2\n2\n"
                  "[traffic_light_stop]\n2\n3\n2\n"
                  "[traffic_light_jump]\n0\n25\n6\n3\n"
                  "[path]\n"
                  "-1\n-2\n-3\n-4\n-5\n-6\n-7\n-8\n1\n1\n2\n0\n";
        output << "[use_traffic_light]\n1\n";
    }
    {
        std::ofstream output(singleStopPath);
        if (!output) {
            std::cerr << "failed: create single-stop scenery object fixture\n";
            openbus::probe::removeProbeDirectory(directory);
            return 1;
        }
        output << "[mesh]\nmesh.x\n"
                  "[trafficlight]\n"
                  "[traffic_lights_group]\n10\n"
                  "[traffic_light]\nRoad\n"
                  "[phase]\n0\n3\n"
                  "[phase]\n6\n7\n"
                  "[path]\n"
                  "1\n2\n3\n4\n5\n6\n7\n8\n0\n2.5\n0\n0\n"
                  "[use_traffic_light]\n0\n"
                  "[traffic_light_stop]\n0\n1\n2\n";
    }
    {
        std::ofstream output(remappedPath);
        if (!output) {
            std::cerr << "failed: create remapped scenery fixture\n";
            openbus::probe::removeProbeDirectory(directory);
            return 1;
        }
        output << "[mesh]\nmesh.x\n"
                  "[trafficlight]\n"
                  "[traffic_lights_group]\n10\n"
                  "[traffic_light]\nDoNotUse\n"
                  "[phase]\n0\n3\n"
                  "[phase]\n6\n7\n"
                  "[traffic_light]\nRoad\n"
                  "[phase]\n0\n3\n"
                  "[phase]\n6\n7\n"
                  "[path]\n"
                  "1\n2\n3\n4\n5\n6\n7\n8\n0\n2.5\n0\n0\n"
                  "[use_traffic_light]\n1\n"
                  "[traffic_light_stop]\n1\n1\n2\n"
                  "[traffic_light_stop]\n1\n2\n2\n";
    }
    {
        std::ofstream output(bathRoundaboutPath);
        if (!output) {
            std::cerr << "failed: create Bath Roundabout scenery fixture\n";
            openbus::probe::removeProbeDirectory(directory);
            return 1;
        }
        output << "[mesh]\nmesh.x\n"
                  "[trafficlight]\n"
                  "[traffic_lights_group]\n10\n"
                  "[traffic_light]\nUnreferenced roundabout light\n"
                  "[phase]\n0\n3\n"
                  "[phase]\n6\n7\n";
    }
    {
        std::ofstream output(polePath);
        output << "[crashmode_pole]\n0.25\n1.2\n";
    }
    {
        std::ofstream output(invalidPolePath);
        output << "[crashmode_pole]\n-1\n0\n";
    }
    {
        std::ofstream output(nonFinitePolePath);
        output << "[crashmode_pole]\nnan\n1.2\n";
    }
    const SceneryObjectConfig configuration = loadSceneryObjectFile(path);
    const SceneryObjectConfig singleStopConfiguration = loadSceneryObjectFile(singleStopPath);
    const SceneryObjectConfig remappedConfiguration = loadSceneryObjectFile(remappedPath);
    const SceneryObjectConfig bathRoundaboutConfiguration =
        loadSceneryObjectFile(bathRoundaboutPath);
    const SceneryObjectConfig poleConfiguration = loadSceneryObjectFile(polePath);
    const SceneryObjectConfig invalidPoleConfiguration = loadSceneryObjectFile(invalidPolePath);
    const SceneryObjectConfig nonFinitePoleConfiguration =
        loadSceneryObjectFile(nonFinitePolePath);
    openbus::probe::removeProbeDirectory(directory);

    bool valid = require(configuration.trafficPaths.size() == 2, "path sections parsed");
    valid &= require(singleStopConfiguration.trafficLightSchedules.size() == 1 &&
                         singleStopConfiguration.trafficLights.size() == 1 &&
                         singleStopConfiguration.trafficLightSchedules.front().has_value() &&
                         !singleStopConfiguration.trafficLightSchedules.front()
                              ->hasConditionalOverrides &&
                         !singleStopConfiguration.trafficLightSchedules.front()->hasBusPriority &&
                         singleStopConfiguration.trafficLightSchedules.front()->program == nullptr &&
                         openbus::map::mapTrafficSignalStateAtTime(
                             *singleStopConfiguration.trafficLightSchedules.front(), 3.0) == 6,
                     "a traffic-light group with one stop ignores its conditional program");
    valid &= require(remappedConfiguration.trafficLights.size() == 1 &&
                         remappedConfiguration.trafficLightSchedules.size() == 1 &&
                         remappedConfiguration.trafficLights.front().name == "Road" &&
                         remappedConfiguration.trafficLightSchedules.front().has_value() &&
                         remappedConfiguration.trafficLightSchedules.front()->lightIndex == 0 &&
                         remappedConfiguration.trafficLightSchedules.front()
                             ->hasConditionalOverrides &&
                         remappedConfiguration.trafficPaths.size() == 1 &&
                         remappedConfiguration.trafficPaths.front().trafficLightIndex == 0 &&
                         remappedConfiguration.trafficPaths.front().trafficSignal.has_value() &&
                         remappedConfiguration.trafficLightStops.size() == 2 &&
                         remappedConfiguration.trafficLightStops[0][0] == 0.0 &&
                         remappedConfiguration.trafficPaths.front()
                                 .trafficSignal->program->controlPoints[0]
                                 .lightIndex == 0,
                     "unreferenced preceding light and its stops are discarded and indices remapped");
    valid &= require(bathRoundaboutConfiguration.trafficLight &&
                         bathRoundaboutConfiguration.trafficLights.empty() &&
                         bathRoundaboutConfiguration.trafficLightSchedules.empty(),
                     "Bath Roundabout light with no referencing path is discarded");
    valid &= require(poleConfiguration.poleCrash.has_value() &&
                         poleConfiguration.poleCrash->resistanceRadiansPerTonneMeterPerSecond ==
                             0.25 &&
                         poleConfiguration.poleCrash->breakAngleRadians == 1.2,
                     "[crashmode_pole] retains resistance and break angle");
    valid &= require(!invalidPoleConfiguration.poleCrash.has_value() &&
                         !invalidPoleConfiguration.diagnostics.entries.empty(),
                     "[crashmode_pole] rejects non-positive resistance and break angle");
    valid &= require(!nonFinitePoleConfiguration.poleCrash.has_value() &&
                         !nonFinitePoleConfiguration.diagnostics.entries.empty(),
                     "[crashmode_pole] rejects non-finite values");
    valid &= require(configuration.trafficLight,
                     "standalone [trafficlight] model marker is recognized");
    if (configuration.trafficPaths.size() == 2) {
        valid &= require(configuration.trafficPaths[0].startPosition ==
                             std::array<double, 3>{1.0, 2.0, 3.0},
                         "first path start position");
        valid &= require(configuration.trafficPaths[0].headingDegrees == 4.0 &&
                             configuration.trafficPaths[0].radius == 5.0 &&
                             configuration.trafficPaths[0].length == 6.0 &&
                             configuration.trafficPaths[0].gradientStartPercent == 7.0 &&
                             configuration.trafficPaths[0].gradientEndPercent == 8.0 &&
                             configuration.trafficPaths[0].widthMeters == 2.5,
                         "path spline fields are preserved");
        valid &= require(configuration.trafficPaths[0].pathTypeCode == 0.0 &&
                             configuration.trafficPaths[1].pathTypeCode == 1.0,
                         "path categories preserved");
        const bool hasExpectedSignal =
            configuration.trafficLights.size() == 2 &&
            configuration.trafficLightSchedules.size() == 2 &&
            configuration.trafficLights[1].name == "Crossing" &&
            configuration.trafficLightSchedules[1].has_value() &&
            configuration.trafficLightSchedules[1]->lightIndex == 1 &&
            configuration.trafficPaths[0].trafficLightIndex == 0 &&
            configuration.trafficPaths[0].trafficSignal.has_value() &&
            configuration.trafficPaths[0].trafficSignal->cycleDurationSeconds == 10.0 &&
            configuration.trafficPaths[0].trafficSignal->phases.size() == 2 &&
            configuration.trafficPaths[0].trafficSignal->hasConditionalOverrides &&
            configuration.trafficPaths[1].trafficSignal.has_value() &&
            configuration.trafficPaths[1].trafficSignal->lightIndex == 1 &&
            configuration.trafficLights[0].approachDistanceMeters == 5.0;
        valid &= require(
            hasExpectedSignal,
            "traffic-light phases and path association are retained");
        valid &= require(configuration.trafficLightStops.size() == 2 &&
                             configuration.trafficLightBusPriorityRules.size() == 1,
                         "conditional records for discarded lights are removed");
        if (hasExpectedSignal) {
            const auto& program = configuration.trafficPaths[0].trafficSignal->program;
            valid &= require(program != nullptr && program->controlPoints.size() == 2 &&
                                 program->controlPoints[0].lightIndex == 0 &&
                                 program->controlPoints[0].timeSeconds == 1.0 &&
                                 program->controlPoints[0].ifRequest &&
                                 program->controlPoints[1].lightIndex == 1 &&
                                 program->controlPoints[1].timeSeconds == 2.0 &&
                                 program->controlPoints[1].ifRequest &&
                                 program->busPriorityRules.size() == 1 &&
                                 program->busPriorityRules[0].phaseToInterrupt == 0 &&
                                 program->busPriorityRules[0].triggerDistanceMeters == 25.0 &&
                                 program->busPriorityRules[0].targetPhase == 6 &&
                                 program->busPriorityRules[0].minimumGreenSeconds == 3.0,
                             "unreferenced signal stops are discarded while bus-preemption rules remain");
        }
        if (hasExpectedSignal) {
            valid &= require(
                openbus::map::mapTrafficSignalStateAtTime(
                    *configuration.trafficPaths[0].trafficSignal, 2.9) == 0 &&
                    openbus::map::mapTrafficSignalStateAtTime(
                        *configuration.trafficPaths[0].trafficSignal, 3.0) == 6,
                "parsed traffic-light cycle returns the active phase");
            valid &= require(
                openbus::map::mapTrafficSignalStateAtTime(
                    *configuration.trafficLightSchedules[1], 0.0) == 3 &&
                    openbus::map::mapTrafficSignalStateAtTime(
                        *configuration.trafficLightSchedules[1], 5.0) == 9,
                "all traffic-light groups retain independent phase schedules");
        }
        valid &= require(configuration.diagnostics.entries.empty(),
                         "valid conditional signal timing is not reported unsupported");
    }
    return valid ? 0 : 1;
}
