#include "config/SceneryObjectConfigLoader.h"
#include "config/ModelConfigLoader.h"
#include "core/Logger.h"
#include "core/Variables.h"
#include "ProbeTempDirectory.h"

#include "map/MapSplineGeometry.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
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
    if (argc == 4 && std::string(argv[1]) == "--editor-passenger-assets") {
        const SceneryObjectConfig stop = loadSceneryObjectFile(argv[2]);
        const SceneryObjectConfig passengers = loadSceneryObjectFile(argv[3]);
        bool valid = require(stop.onlyEditor && stop.busStop && stop.hasBoundingBox &&
                                 !stop.diagnostics.hasErrors(),
                             "installed editor bus stop is parsed");
        valid &= require(passengers.onlyEditor && passengers.hasPassengerCabin &&
                             passengers.passengerPositions.size() == 7 &&
                             !passengers.diagnostics.hasErrors(),
                         "installed editor passenger area has seven positions");
        return valid ? 0 : 1;
    }
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
    const std::filesystem::path cycleFallbackPath = directory / "cycle-fallback.sco";
    const std::filesystem::path remappedPath = directory / "remapped-light.sco";
    const std::filesystem::path bathRoundaboutPath = directory / "bath-roundabout.sco";
    const std::filesystem::path polePath = directory / "pole.sco";
    const std::filesystem::path invalidPolePath = directory / "invalid-pole.sco";
    const std::filesystem::path nonFinitePolePath = directory / "non-finite-pole.sco";
    const std::filesystem::path editorStopPath = directory / "editor-stop.sco";
    const std::filesystem::path editorPassengersPath = directory / "editor-passengers.sco";
    const std::filesystem::path carParkPath = directory / "car-park.sco";
    const std::filesystem::path entryPointPath = directory / "entry-point.sco";
    const std::filesystem::path missingPassengersPath = directory / "missing-passengers.sco";
    const std::filesystem::path textureAddressPath = directory / "texture-address.sco";
    const std::filesystem::path detailPath = directory / "detail.sco";
    const std::filesystem::path invalidDetailPath = directory / "invalid-detail.sco";
    const std::filesystem::path missingDetailPath = directory / "missing-detail.sco";
    const std::filesystem::path externalModelPath = directory / "external.cfg";
    const std::filesystem::path statusPath = directory / "status.sco";
    const std::filesystem::path statusLogPath = directory / "diagnostics.log";
    const std::string statusLogPathText = statusLogPath.string();
#ifdef _WIN32
    if (_putenv_s("OPENBUS_LOG_FILE", statusLogPathText.c_str()) != 0) {
#else
    if (setenv("OPENBUS_LOG_FILE", statusLogPathText.c_str(), 1) != 0) {
#endif
        std::cerr << "Could not configure scenery diagnostic log path\n";
        openbus::probe::removeProbeDirectory(directory);
        return 1;
    }
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
                  "[traffic_light_stop]\n0\n1\n1\n"
                  "[traffic_light_stop]\n1\n2\n1\n"
                  "[traffic_light_stop]\n2\n3\n1\n"
                  "[traffic_light_jump]\n0\n8\n1\n0\n"
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
                  "[traffic_light_stop]\n0\n1\n1\n";
    }
    {
        std::ofstream output(cycleFallbackPath);
        if (!output) {
            std::cerr << "failed: create zero-cycle scenery object fixture\n";
            openbus::probe::removeProbeDirectory(directory);
            return 1;
        }
        output << "[mesh]\nmesh.x\n"
                  "[trafficlight]\n"
                  "[traffic_lights_group]\n0\n"
                  "[traffic_light]\nRoad\n"
                  "[phase]\n0\n4\n"
                  "[phase]\n6\n6\n"
                  "[path]\n"
                  "1\n2\n3\n4\n5\n6\n7\n8\n0\n2.5\n0\n0\n"
                  "[use_traffic_light]\n0\n";
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
                  "[traffic_light_stop]\n1\n1\n1\n"
                  "[traffic_light_stop]\n1\n2\n1\n";
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
    {
        std::ofstream output(editorStopPath);
        output << "[onlyeditor]\n[mesh]\nstop.x\n[busstop]\n"
                  "[boundingbox]\n1\n2\n3\n0\n0\n1.5\n";
    }
    {
        std::ofstream output(editorPassengersPath);
        output << "[onlyeditor]\n[mesh]\npeople.x\n[passengercabin]\nwaiting.cfg\n";
        std::ofstream cabin(directory / "waiting.cfg");
        cabin << "[passpos]\n-0.2\n-1.5\n0\n0\n185\n"
                 "[passpos]\n0.5\n-1.4\n0.25\n0\n181\n";
    }
    {
        std::ofstream output(carParkPath);
        output << "[onlyeditor]\n[mesh]\nmarker.x\n[carpark_p]\n";
    }
    {
        std::ofstream output(entryPointPath);
        output << "[onlyeditor]\n[mesh]\nmarker.x\n[entrypoint]\n";
    }
    {
        std::ofstream output(missingPassengersPath);
        output << "[onlyeditor]\n[mesh]\npeople.x\n[passengercabin]\nmissing.cfg\n";
    }
    {
        std::ofstream output(textureAddressPath);
        output << "[mesh]\nmesh.x\n"
                  "[matl]\nborder.bmp\n0\n[matl_texadress_border]\n"
                  "[texcoordtransX]\nscroll_x\n[texcoordtransY]\nscroll_y\n"
                  "[alphascale]\nlamp_alpha\n[matl_lightmap]\nlamp_light.bmp\nlamp_light\n"
                  "[matl_bumpmap]\nbump.bmp\n0.25\n"
                  "[matl_envmap]\nenv.bmp\n0.5\n"
                  "[matl_nightmap]\nnight.bmp\n"
                  "[matl_change]\nborder.bmp\n0\nnight_selector\n"
                  "[matl_item]\n[matl_nightmap]\nitem_night.bmp\n"
                  "[matl_lightmap]\nitem_light.bmp\nitem_light_strength\n"
                  "[matl]\nclamp.bmp\n1\n[matl_texadress_clamp]\n"
                  "[matl]\nmirror.bmp\n2\n[matl_texadress_mirror]\n"
                  "[matl]\nonce.bmp\n3\n[matl_texadress_mirroronce]\n";
    }
    {
        std::ofstream output(detailPath);
        output << "[complexity]\n2\n[NightMapMode]\n1\n[mesh]\nmesh.x\n";
        std::ofstream invalid(invalidDetailPath);
        invalid << "[complexity]\n2x\n[NightMapMode]\ninvalid\n";
        std::ofstream missing(missingDetailPath);
        missing << "[complexity]\n[NightMapMode]\n[mesh]\nmesh.x\n";
        std::ofstream external(externalModelPath);
        external << "[complexity]\n2\n[NightMapMode]\n1\n[mesh]\nmesh.x\n";
        std::ofstream mesh(directory / "mesh.x");
        mesh << "xof 0303txt 0032\n";
    }
    {
        std::ofstream output(statusPath);
        output << "[mesh]\nmesh.x\n"
                  "[description]\neditor description\n[end]\n"
                  "[busstop]\n"
                  "[helparrow]\n"
                  "[groups]\n"
                  "[friendlyname]\neditor name\n"
                  "[complexity]\n2\n"
                  "[nightmapmode]\n1\n"
                  "[maplight]\n0\n0\n0\n0\n0\n0\n0\n"
                  "[unknown_probe_keyword]\n";
    }
    const SceneryObjectConfig configuration = loadSceneryObjectFile(path);
    const SceneryObjectConfig singleStopConfiguration = loadSceneryObjectFile(singleStopPath);
    const SceneryObjectConfig cycleFallbackConfiguration =
        loadSceneryObjectFile(cycleFallbackPath);
    const SceneryObjectConfig remappedConfiguration = loadSceneryObjectFile(remappedPath);
    const SceneryObjectConfig bathRoundaboutConfiguration =
        loadSceneryObjectFile(bathRoundaboutPath);
    const SceneryObjectConfig poleConfiguration = loadSceneryObjectFile(polePath);
    const SceneryObjectConfig invalidPoleConfiguration = loadSceneryObjectFile(invalidPolePath);
    const SceneryObjectConfig nonFinitePoleConfiguration =
        loadSceneryObjectFile(nonFinitePolePath);
    const SceneryObjectConfig editorStop = loadSceneryObjectFile(editorStopPath);
    const SceneryObjectConfig editorPassengers = loadSceneryObjectFile(editorPassengersPath);
    const SceneryObjectConfig carPark = loadSceneryObjectFile(carParkPath);
    const SceneryObjectConfig entryPoint = loadSceneryObjectFile(entryPointPath);
    const SceneryObjectConfig missingPassengers = loadSceneryObjectFile(missingPassengersPath);
    const SceneryObjectConfig textureAddressObject = loadSceneryObjectFile(textureAddressPath);
    const SceneryObjectConfig detail = loadSceneryObjectFile(detailPath);
    const SceneryObjectConfig invalidDetail = loadSceneryObjectFile(invalidDetailPath);
    const SceneryObjectConfig missingDetail = loadSceneryObjectFile(missingDetailPath);
    const SceneryObjectConfig statusConfiguration = loadSceneryObjectFile(statusPath);
    openbus::scripting::SceneryObject textureAddressVariables;
    const ModelConfig textureAddressModel =
        loadSceneryObjectConfig(textureAddressPath, directory, textureAddressVariables);
    const ModelConfig detailModel =
        loadModelConfig(detailPath, directory, ModelConfigKind::SceneryObject,
                        textureAddressVariables);
    const ModelConfig externalModel =
        loadModelConfig(externalModelPath, directory, ModelConfigKind::SceneryObject,
                        textureAddressVariables);
    Logger::Flush();
    std::ifstream statusLogInput(statusLogPath);
    const std::string statusLog((std::istreambuf_iterator<char>(statusLogInput)),
                                std::istreambuf_iterator<char>());
    const bool statusLogOpened = statusLogInput.is_open();
    statusLogInput.close();
    openbus::probe::removeProbeDirectory(directory);

    bool valid = require(configuration.trafficPaths.size() == 2, "path sections parsed");
    valid &= require(statusConfiguration.busStop && !statusConfiguration.diagnostics.hasErrors() &&
                         statusLogOpened,
                     "SCO configuration statuses distinguish active, ignored, and unknown tags");
    valid &= require(statusLog.find("Configuration tag [busstop] recognized and active") !=
                         std::string::npos,
                     "SCO active status is logged");
    valid &= require(statusLog.find("Configuration tag [description] recognized but ignored") !=
                         std::string::npos &&
                         statusLog.find("Configuration tag [helparrow] recognized but ignored") !=
                             std::string::npos &&
                         statusLog.find("Configuration tag [groups] recognized but ignored") !=
                             std::string::npos &&
                         statusLog.find("Configuration tag [friendlyname] recognized but ignored") !=
                             std::string::npos &&
                         statusLog.find("Configuration tag [complexity] recognized but ignored") !=
                             std::string::npos &&
                         statusLog.find("Configuration tag [nightmapmode] recognized but ignored") !=
                             std::string::npos &&
                         statusLog.find("Configuration tag [maplight] recognized but ignored") !=
                             std::string::npos,
                     "SCO ignored statuses are logged");
    valid &= require(statusLog.find("Configuration tag [unknown_probe_keyword] unknown") !=
                         std::string::npos,
                     "SCO unknown status is logged");
    valid &= require(detail.complexity == 2 && detail.nightMapMode == 1 &&
                         !detail.diagnostics.hasErrors() && detailModel.parts.size() == 1 &&
                         detailModel.diagnostics.entries.empty(),
                     "SCO detail and night-map values are retained and not reparsed as model tags");
    valid &= require(!configuration.complexity && !configuration.nightMapMode,
                     "absent detail and night-map fields remain unspecified");
    const auto detailErrors = [](const SceneryObjectConfig& config) {
        return std::count_if(config.diagnostics.entries.begin(), config.diagnostics.entries.end(),
                             [](const ConfigurationDiagnostic& diagnostic) {
                                 return diagnostic.severity == ConfigurationDiagnostic::Severity::Error &&
                                        (diagnostic.keyword == "complexity" ||
                                         diagnostic.keyword == "nightmapmode");
                             });
    };
    valid &= require(!invalidDetail.complexity && !invalidDetail.nightMapMode &&
                         detailErrors(invalidDetail) == 2,
                     "invalid detail and night-map values are diagnosed");
    valid &= require(!missingDetail.complexity && !missingDetail.nightMapMode &&
                         detailErrors(missingDetail) == 2,
                     "missing detail values do not consume following keywords");
    valid &= require(externalModel.diagnostics.entries.size() == 2,
                     "external model configs still diagnose SCO-only keywords");
    const auto& addressParts = textureAddressModel.parts;
    valid &= require(
        !textureAddressObject.diagnostics.hasErrors() &&
            std::none_of(textureAddressObject.diagnostics.entries.begin(),
                         textureAddressObject.diagnostics.entries.end(),
                         [](const ConfigurationDiagnostic& diagnostic) {
                             return diagnostic.keyword.find("matl_texadress_") == 0 ||
                                    diagnostic.keyword == "alphascale" ||
                                    diagnostic.keyword == "matl_lightmap" ||
                                    diagnostic.keyword == "matl_bumpmap" ||
                                    diagnostic.keyword == "matl_envmap" ||
                                    diagnostic.keyword == "matl_nightmap" ||
                                    diagnostic.keyword == "texcoordtransx" ||
                                    diagnostic.keyword == "texcoordtransy";
                         }) &&
            !textureAddressModel.diagnostics.hasErrors() && addressParts.size() == 1 &&
            addressParts[0].materialStatesInOrder.size() == 4 &&
            addressParts[0].materialStatesInOrder[0].texcoordTransXVariable == "scroll_x" &&
            addressParts[0].materialStatesInOrder[0].texcoordTransYVariable == "scroll_y" &&
            addressParts[0].materialStatesInOrder[0].alphaScaleVariable == "lamp_alpha" &&
            addressParts[0].materialStatesInOrder[0].lightmapTextureName == "lamp_light.bmp" &&
            addressParts[0].materialStatesInOrder[0].lightmapStrengthVariable == "lamp_light" &&
            addressParts[0].materialStatesInOrder[0].bumpmapTextureName == "bump.bmp" &&
            addressParts[0].materialStatesInOrder[0].bumpmapStrength == 0.25 &&
            addressParts[0].materialStatesInOrder[0].environmentTextureName == "env.bmp" &&
            addressParts[0].materialStatesInOrder[0].environmentStrength == 0.5 &&
            addressParts[0].materialStatesInOrder[0].nightmapTextureName == "night.bmp" &&
            addressParts[0].materialStatesInOrder[0].textureChanges.size() == 1 &&
            addressParts[0].materialStatesInOrder[0].textureChanges[0].items.size() == 1 &&
            addressParts[0].materialStatesInOrder[0].textureChanges[0].items[0] &&
            addressParts[0].materialStatesInOrder[0]
                    .textureChanges[0].items[0]->nightmapTextureName == "item_night.bmp" &&
            addressParts[0].materialStatesInOrder[0]
                    .textureChanges[0].items[0]->lightmapTextureName == "item_light.bmp" &&
            addressParts[0].materialStatesInOrder[0]
                    .textureChanges[0].items[0]->lightmapStrengthVariable ==
                "item_light_strength" &&
            textureAddressVariables.has("night_selector") &&
            textureAddressVariables.has("item_light_strength") &&
            textureAddressVariables.has("scroll_x") && textureAddressVariables.has("scroll_y") &&
            textureAddressVariables.has("lamp_alpha") && textureAddressVariables.has("lamp_light") &&
            addressParts[0].materialStatesInOrder[0].textureAddressS == TextureAddressMode::Border &&
            addressParts[0].materialStatesInOrder[0].textureAddressT == TextureAddressMode::Border &&
            addressParts[0].materialStatesInOrder[1].textureAddressS == TextureAddressMode::Clamp &&
            addressParts[0].materialStatesInOrder[2].textureAddressS == TextureAddressMode::Mirror &&
            addressParts[0].materialStatesInOrder[3].textureAddressS == TextureAddressMode::MirrorOnce,
        "inline SCO material address directives are handled without unknown-keyword warnings");
    valid &= require(editorStop.onlyEditor && editorStop.busStop && editorStop.hasBoundingBox &&
                         !editorStop.diagnostics.hasErrors(),
                     "editor bus stop and its bounding box are parsed");
    valid &= require(carPark.onlyEditor && carPark.carPark && !carPark.diagnostics.hasErrors(),
                     "editor car-park marker is parsed");
    valid &= require(entryPoint.onlyEditor && entryPoint.entryPoint &&
                         !entryPoint.diagnostics.hasErrors(),
                     "editor entrypoint marker is parsed");
    valid &= require(editorPassengers.onlyEditor && editorPassengers.hasPassengerCabin &&
                         editorPassengers.passengerPositions.size() == 2 &&
                         editorPassengers.passengerPositions[0].position[0] == -0.2 &&
                         editorPassengers.passengerPositions[1].position[2] == 0.25 &&
                         !editorPassengers.diagnostics.hasErrors(),
                     "editor passenger cabin positions are parsed");
    valid &= require(missingPassengers.hasPassengerCabin &&
                         missingPassengers.diagnostics.hasErrors(),
                     "missing editor passenger cabin is reported");
    valid &= require(singleStopConfiguration.trafficLightSchedules.size() == 1 &&
                         singleStopConfiguration.trafficLights.size() == 1 &&
                         singleStopConfiguration.trafficLightSchedules.front().has_value() &&
                         singleStopConfiguration.trafficLightSchedules.front()
                             ->hasConditionalOverrides &&
                         singleStopConfiguration.trafficLightSchedules.front()->program != nullptr &&
                         singleStopConfiguration.trafficLightSchedules.front()->program
                                 ->controlPoints.size() == 1 &&
                         openbus::map::mapTrafficSignalStateAtTime(
                             *singleStopConfiguration.trafficLightSchedules.front(), 3.0) == 6,
                     "a traffic-light group with one conditional stop retains its controller");
    valid &= require(cycleFallbackConfiguration.trafficLightsGroupDurationSeconds == 10.0 &&
                         cycleFallbackConfiguration.trafficLightSchedules.size() == 1 &&
                         cycleFallbackConfiguration.trafficLightSchedules.front().has_value() &&
                         cycleFallbackConfiguration.trafficLightSchedules.front()
                                 ->cycleDurationSeconds == 10.0 &&
                         !cycleFallbackConfiguration.diagnostics.hasErrors(),
                     "a zero group cycle falls back to the longest configured light schedule");
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
                             configuration.trafficLightJumps.size() == 1,
                         "conditional records for discarded lights are removed");
        if (hasExpectedSignal) {
            const auto& program = configuration.trafficPaths[0].trafficSignal->program;
            valid &= require(program != nullptr && program->controlPoints.size() == 3 &&
                                 program->controlPoints[0].lightIndex == 0 &&
                                 program->controlPoints[0].timeSeconds == 1.0 &&
                                 program->controlPoints[0].ifRequest &&
                                 program->controlPoints[1].lightIndex == 1 &&
                                 program->controlPoints[1].timeSeconds == 2.0 &&
                                 program->controlPoints[1].ifRequest &&
                                 program->controlPoints[2].lightIndex == 0 &&
                                 program->controlPoints[2].timeSeconds == 8.0 &&
                                 program->controlPoints[2].ifRequest &&
                                 program->controlPoints[2].jumpToSeconds == 0.0,
                             "unreferenced stops are discarded and SCO jumps retain their target");
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
