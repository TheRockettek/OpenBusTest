#include "config/VehicleConfigLoader.h"
#include "core/Variables.h"
#include "osc/OscConverter.h"
#include "physics/BusConfiguration.h"
#include "ProbeTempDirectory.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

int main() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_vehicle_config_probe");
    std::filesystem::create_directories(root);
    std::filesystem::create_directories(root / "model");
    std::filesystem::create_directories(root / "Script");
    const std::filesystem::path configPath = root / "bus.bus";
    const std::filesystem::path scriptPath = root / "Script" / "Engine.OSC";
    std::ofstream script(scriptPath);
    script << "{init}\n{end}\n";
    script.close();
    std::ofstream cabin(root / "model/cabin.cfg");
    cabin << "[entry]\n0\n"
             "[exit]\n4\n"
             "[drivpos]\n0.7\n4.8\n1.5\n0.4\n0\n"
             "[illumination_interior]\n1\n2\n3\n4\n"
             "[passpos]\n0.9\n3.9\n1.8\n0.3\n0\n";
    cabin.close();
    std::ofstream paths(root / "model/paths.cfg");
    paths << "[stepsoundpack]\n2\nstep_a.wav\nstep_b.wav\n"
             "0\n[pathpnt]\n0\n0\n0\n"
             "1\n[pathpnt]\n1\n0\n0\n"
             "[pathlink]\n0\n1\n"
             "[pathlink_oneway]\n1\n0\n"
             "[next_stepsound]\n0\n"
             "[next_roomheight]\n1.85\n";
    paths.close();
    std::ofstream numbers(root / "registrations.org");
    numbers << "BUS-001\nBUS-002\n";
    numbers.close();
    std::ofstream registrationList(root / "registration_list.org");
    registrationList << "BUS-002\nBUS-003\n";
    registrationList.close();
    std::ofstream lightsConstants(root / "lights_constfile.txt", std::ios::binary);
    lightsConstants.write("\xEF\xBB\xBF", 3);
    lightsConstants << "[const]\nlights_blinkertime_on\n0.6\n"
                       "[const]\nlights_blinkertime_off\n0.4\n";
    lightsConstants.close();
    std::ofstream config(configPath);
    config << "[friendlyname]\nMercedes-Benz\nCitaro E400\nNight blue\n"
              "[description]\nUrban service bus.\n\n Electric drivetrain. \n[end]\n"
              "[type]\n0\n"
              "[model]\nmodel.cfg\n"
              "[rollwiderstand]\n1250\n"
              "[script]\n1\nscript/engine.osc\n"
              "[boundingbox]\n2.4\n12.0\n3.2\n0.1\n-0.2\n1.4\n"
              "[sound]\nsound/player.cfg\n"
              "[sound_ai]\nsound/ai.cfg\n"
              "[ai_deltaheight]\n0.35\n"
              "[paths]\nmodel/paths.cfg\n"
              "[passengercabin]\nmodel/cabin.cfg\n"
              "[number]\nregistrations.org\n"
              "[registration_list]\nregistration_list.org\n"
              "[constfile]\n1\nlights_constfile.txt\n"
              "[registration_automatic]\nB-TEST \n"
              "[registration_free]\n"
              "[kmcounter_init]\n2026\n1234.5\n";
    config.close();

    const VehicleConfig result = loadVehicleConfig(configPath, VehicleFileKind::Bus);
    VehicleConfig aiSoundFallback = result;
    aiSoundFallback.soundAiConfigPath.clear();
    const VehiclePlacement testPlacement{{1.0, 2.0, 3.0}, 0.0};
    const bool caseMismatchedScriptConverted =
        result.scripts.size() == 1 && std::filesystem::exists(generatedLuaPath(scriptPath));
    VehicleConfig secondRegistration = result;
    selectRegistrationAtIndex(secondRegistration, 1);

    const auto writeResistanceConfig = [&](const std::filesystem::path& path,
                                           const std::string& directive) {
        std::ofstream output(path);
        output << "[model]\nmodel.cfg\n" << directive;
    };
    const std::vector<std::string> unsupportedKeywords = {
        "fixed",          "scriptshare",             "rowdy_factor",
        "boogies",        "ai_brakeperformance",    "ai_veh_type",
        "sinus",          "coupling_front",         "coupling_back",
        "coupling_front_character",                  "couple_back",
        "couple_front_open_for_sound",               "control_cable_front",
        "control_cable_back",                        "new_attachment",
        "contact_shoe",   "rail_body_osc",
    };
    const std::filesystem::path unsupportedPath = root / "unsupported.bus";
    {
        std::ofstream output(unsupportedPath);
        output << "[model]\nmodel.cfg\n";
        for (const std::string& keyword : unsupportedKeywords) {
            output << '[' << keyword << "]\npayload\n";
        }
        output << "[unknown_probe_keyword]\npayload\n[rollwiderstand]\n1250\n";
    }
    for (VehicleFileKind kind : {VehicleFileKind::Bus, VehicleFileKind::Vehicle}) {
        const VehicleConfig unsupported = loadVehicleConfig(unsupportedPath, kind, false);
        for (const std::string& keyword : unsupportedKeywords) {
            const auto diagnostic = std::find_if(
                unsupported.diagnostics.entries.begin(), unsupported.diagnostics.entries.end(),
                [&](const ConfigurationDiagnostic& entry) {
                    return entry.keyword == keyword &&
                           entry.severity == ConfigurationDiagnostic::Severity::Warning &&
                           entry.message.find("recognized but unsupported:") == 0;
                });
            if (diagnostic == unsupported.diagnostics.entries.end() || diagnostic->line == 0) {
                std::cerr << "Missing unsupported vehicle keyword diagnostic: " << keyword << '\n';
                return 1;
            }
        }
        const auto unknown = std::find_if(
            unsupported.diagnostics.entries.begin(), unsupported.diagnostics.entries.end(),
            [](const ConfigurationDiagnostic& entry) {
                return entry.keyword == "unknown_probe_keyword" &&
                       entry.message == "unknown vehicle keyword";
            });
        if (unsupported.diagnostics.hasErrors() || unknown == unsupported.diagnostics.entries.end() ||
            unsupported.rollingResistanceForceNewtons != 1250.0) {
            std::cerr << "Unsupported vehicle records disrupted later supported directives\n";
            return 1;
        }
    }
    const std::filesystem::path zeroResistancePath = root / "zero_roll.bus";
    writeResistanceConfig(zeroResistancePath, "[rollwiderstand]\n0\n");
    const VehicleConfig zeroResistance =
        loadVehicleConfig(zeroResistancePath, VehicleFileKind::Bus, false);
    const std::filesystem::path absentResistancePath = root / "absent_roll.bus";
    writeResistanceConfig(absentResistancePath, "");
    const VehicleConfig absentResistance =
        loadVehicleConfig(absentResistancePath, VehicleFileKind::Bus, false);
    const std::filesystem::path negativeResistancePath = root / "negative_roll.bus";
    writeResistanceConfig(negativeResistancePath, "[rollwiderstand]\n-1\n");
    const VehicleConfig negativeResistance =
        loadVehicleConfig(negativeResistancePath, VehicleFileKind::Bus, false);
    const std::filesystem::path reflectionCameraPath = root / "reflection_cameras.bus";
    writeResistanceConfig(reflectionCameraPath,
                          "[add_camera_reflexion]\n1\n2\n3\n0\n40\n10\n-5\n"
                          "[add_camera_reflexion_static]\n1\n2\n3\n0\n40\n10\n-5\n"
                          "[add_camera_reflexion_2]\n1\n2\n3\n0\n40\n10\n-5\n12\n"
                          "[add_camera_reflexion_2_static]\n1\n2\n3\n0\n40\n10\n-5\n12\n");
    const VehicleConfig reflectionCameras =
        loadVehicleConfig(reflectionCameraPath, VehicleFileKind::Bus, false);
    const std::filesystem::path inactiveRecordsPath = root / "inactive_records.bus";
    writeResistanceConfig(inactiveRecordsPath,
                          "[add_camera_driver]\n1\n2\n3\n0\n40\n10\n-5\n"
                          "[view_schedule]\n0\nSchedule view\n"
                          "[view_ticketselling]\n1\nTicket view\n"
                          "[rot_pnt_long]\n-2.5\n"
                          "[ai_deltaheight]\n0.4\n");
    const VehicleConfig inactiveRecords =
        loadVehicleConfig(inactiveRecordsPath, VehicleFileKind::Bus, false);
    const auto hasIgnoredWarning = [&](const std::string& keyword) {
        return std::any_of(inactiveRecords.diagnostics.entries.begin(),
                           inactiveRecords.diagnostics.entries.end(),
                           [&](const ConfigurationDiagnostic& diagnostic) {
                               return diagnostic.severity == ConfigurationDiagnostic::Severity::Warning &&
                                      diagnostic.keyword == keyword && diagnostic.line > 0 &&
                                      diagnostic.message.find("not ") != std::string::npos;
                           });
    };
    const bool inactiveRecordsReported =
        !inactiveRecords.diagnostics.hasErrors() &&
        inactiveRecords.longitudinalRotationPoint == -2.5 &&
        inactiveRecords.hasAiDeltaHeight && inactiveRecords.aiDeltaHeight == 0.4 &&
        hasIgnoredWarning("view_schedule") && hasIgnoredWarning("view_ticketselling") &&
        hasIgnoredWarning("rot_pnt_long");
    const std::filesystem::path axleLabelsPath = root / "axle_labels.bus";
    std::ofstream axleLabels(axleLabelsPath);
    axleLabels << "[model]\nmodel.cfg\n"
                  "[newachse]\n"
                  "Vorderachse:\n"
                  "Koordinate:\n"
                  "achse_long\n1.5\n"
                  "Breite der Auflagefläche beider Reifen\n"
                  "achse_maxwidth\n2.4\n"
                  "achse_minwidth\n1.8\n"
                  "Raddurchmesser der Räder dieser Achse\n"
                  "achse_raddurchmesser\n1.0\n"
                  "Federkonstante der Federung PRO SEITE in kN/m\n"
                  "achse_feder\n300\n"
                  "Höchstbelastung in kN\n"
                  "achse_maxforce\n80\n"
                  "Dämpferkonstante in kNs/m\n"
                  "achse_daempfer\n20\n"
                  "Ist diese Achse angetrieben?\n"
                  "achse_antrieb\n1\n";
    axleLabels.close();
    const VehicleConfig axleLabelsConfig =
        loadVehicleConfig(axleLabelsPath, VehicleFileKind::Bus, false);
    const std::filesystem::path ovhConfigPath = root / "ai_car.ovh";
    std::ofstream ovhConfig(ovhConfigPath);
    ovhConfig << "[friendlyname]\nTest\nAI car\nDefault\n"
                 "[model]\nmodel/model.cfg\n";
    ovhConfig.close();
    std::ofstream vehicleModelConfig(root / "model/model.cfg");
    vehicleModelConfig << "[mesh]\ncar.o3d\n";
    vehicleModelConfig.close();
    std::ofstream vehicleMesh(root / "model/car.o3d", std::ios::binary);
    vehicleMesh.close();
    const VehicleConfig ovhVehicle =
        loadVehicleConfig(ovhConfigPath, VehicleFileKind::Vehicle, false);
    const std::filesystem::path ovhModelConfigPath =
        modelConfigurationPathForVehicle(ovhConfigPath);
    openbus::scripting::Vehicle ovhModelVariables;
    const ModelConfig ovhModel = loadVehicleModelConfig(
        ovhModelConfigPath, root / "model", ovhModelVariables);
    const bool ovhModelLoaded =
        !ovhVehicle.diagnostics.hasErrors() && !ovhModel.diagnostics.hasErrors() &&
        ovhModelConfigPath == root / "model/model.cfg" &&
        ovhModel.parts.size() == 1 && ovhModel.parts[0].objPath == root / "model/car.o3d";
    const auto& reflections = reflectionCameras.cameras;
    const bool reflectionCameraKeywordsParsed =
        !reflectionCameras.diagnostics.hasErrors() && reflections.size() == 4 &&
        reflections[0].kind == VehicleCameraKind::Reflexion && reflections[0].followsViewer &&
        reflections[1].kind == VehicleCameraKind::Reflexion && !reflections[1].followsViewer &&
        reflections[2].kind == VehicleCameraKind::Reflexion2 && reflections[2].followsViewer &&
        reflections[3].kind == VehicleCameraKind::Reflexion2 && !reflections[3].followsViewer &&
        reflections[3].pan == 10.0 && reflections[3].tilt == -5.0;
    openbus::probe::removeProbeDirectory(root);

    if (!reflectionCameraKeywordsParsed || !ovhModelLoaded || !inactiveRecordsReported) {
        std::cerr << "Reflection camera parsing or OVH vehicle model loading failed\n";
        return 1;
    }
    if (axleLabelsConfig.diagnostics.hasErrors() || axleLabelsConfig.axles.size() != 1 ||
        axleLabelsConfig.axles.front().position != 1.5 ||
        axleLabelsConfig.axles.front().maxWidth != 2.4 ||
        axleLabelsConfig.axles.front().driven != true) {
        std::cerr << "Descriptive axle labels interfered with numeric axle records\n";
        return 1;
    }
    if (result.diagnostics.hasErrors() || !caseMismatchedScriptConverted ||
        result.friendlyManufacturer != "Mercedes-Benz" ||
        result.friendlyVehicleName != "Citaro E400" ||
        result.friendlyDefaultPaint != "Night blue" ||
        !result.rollingResistanceForceNewtons.has_value() ||
        *result.rollingResistanceForceNewtons != 1250.0 ||
        !zeroResistance.rollingResistanceForceNewtons.has_value() ||
        *zeroResistance.rollingResistanceForceNewtons != 0.0 ||
        absentResistance.rollingResistanceForceNewtons.has_value() ||
        !negativeResistance.diagnostics.hasErrors() ||
        result.description != "Urban service bus.\n\n Electric drivetrain. " ||
        !result.vehicleType.has_value() || *result.vehicleType != 0 ||
        !result.hasBoundingBox ||
        result.boundingBox != std::array<double, 6>{2.4, 12.0, 3.2, 0.1, -0.2, 1.4} ||
        result.soundConfigPath.generic_string() != (root / "sound/player.cfg").generic_string() ||
        result.soundAiConfigPath.generic_string() != (root / "sound/ai.cfg").generic_string() ||
        soundConfigPathForVehicle(result, false) != result.soundConfigPath ||
        soundConfigPathForVehicle(result, true) != result.soundAiConfigPath ||
        soundConfigPathForVehicle(aiSoundFallback, true) != aiSoundFallback.soundConfigPath ||
        !result.hasAiDeltaHeight || result.aiDeltaHeight != 0.35 ||
        placementForVehicle(result, testPlacement, false).position[2] != 3.0 ||
        placementForVehicle(result, testPlacement, true).position[2] != 3.35 ||
        result.pathsConfigPath.generic_string() != (root / "model/paths.cfg").generic_string() ||
        result.passengerCabinConfigPath.generic_string() != (root / "model/cabin.cfg").generic_string() ||
        result.numberConfigPath.generic_string() != (root / "registrations.org").generic_string() ||
        result.registrationListConfigPath.generic_string() !=
            (root / "registration_list.org").generic_string() ||
        !result.registrationAutomatic || result.registrationPrefix != "B-TEST" ||
        !result.registrationFree || !result.hasOdometerInitial ||
        result.odometerInitialYear != 2026 || result.odometerInitialKilometres != 1234.5 ||
        !result.passengerCabinLoaded || !result.driverPosition.has_value() ||
        result.passengerPositions.size() != 1 || result.passengerEntryPathPoints.size() != 1 ||
        result.passengerExitPathPoints.size() != 1 ||
        result.passengerPositions[0].interiorLightIndexes != std::array<int, 4>{1, 2, 3, 4} ||
        !result.passengerPathsLoaded || result.passengerPathPoints.size() != 2 ||
        result.passengerPathLinks.size() != 2 ||
        result.passengerPathLinks[0].from != 0 || result.passengerPathLinks[0].to != 1 ||
        result.passengerPathLinks[0].oneWay ||
        result.passengerPathLinks[1].from != 1 || result.passengerPathLinks[1].to != 0 ||
        !result.passengerPathLinks[1].oneWay ||
        result.passengerStepSoundPacks.size() != 1 ||
        result.passengerStepSoundPacks[0].size() != 2 ||
        result.passengerPathNextStepSound != 0 ||
        result.passengerPathNextRoomHeights.size() != 1 ||
        result.passengerPathNextRoomHeights[0] != 1.85 ||
        !result.registrationListsLoaded || result.vehicleNumbers.size() != 2 ||
        result.registrationNumbers.size() != 2 || result.selectedRegistrationIndex != 0 ||
        result.selectedVehicleNumber != "BUS-001" ||
        result.selectedRegistration != "BUS-002" ||
        secondRegistration.selectedRegistrationIndex != 1 ||
        secondRegistration.selectedVehicleNumber != "BUS-002" ||
        secondRegistration.selectedRegistration != "BUS-003") {
        std::cerr << "BUS metadata records were not retained\n";
        return 1;
    }
    const auto blinkOn = result.constants.find("lights_blinkertime_on");
    const auto blinkOff = result.constants.find("lights_blinkertime_off");
    if (blinkOn == result.constants.end() || blinkOn->second != 0.6F ||
        blinkOff == result.constants.end() || blinkOff->second != 0.4F) {
        std::cerr << "BOM-prefixed constant file did not retain its first constant\n";
        return 1;
    }
    const std::vector<PassengerPathPoint> cabinPoints = {
        {10, {0.0, 0.0, 0.0}}, {20, {1.0, 0.0, 0.0}}, {30, {2.0, 0.0, 0.0}}};
    const std::vector<PassengerPathLink> cabinLinks = {
        {10, 20, true}, {20, 30, false}};
    if (passengerPathRoute(cabinPoints, cabinLinks, 10, 30) !=
            std::vector<int>{10, 20, 30} ||
        !passengerPathRoute(cabinPoints, cabinLinks, 30, 10).empty() ||
        passengerPathRoute(cabinPoints, cabinLinks, 20, 20) != std::vector<int>{20}) {
        std::cerr << "Passenger cabin path routing did not preserve link direction\n";
        return 1;
    }
    return 0;
}
