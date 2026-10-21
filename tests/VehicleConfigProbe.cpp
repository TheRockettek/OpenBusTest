#include "config/VehicleConfigLoader.h"
#include "osc/OscConverter.h"
#include "ProbeTempDirectory.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>

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
    const auto& reflections = reflectionCameras.cameras;
    const bool reflectionCameraKeywordsParsed =
        !reflectionCameras.diagnostics.hasErrors() && reflections.size() == 4 &&
        reflections[0].kind == VehicleCameraKind::Reflexion && reflections[0].followsViewer &&
        reflections[1].kind == VehicleCameraKind::Reflexion && !reflections[1].followsViewer &&
        reflections[2].kind == VehicleCameraKind::Reflexion2 && reflections[2].followsViewer &&
        reflections[3].kind == VehicleCameraKind::Reflexion2 && !reflections[3].followsViewer &&
        reflections[3].pan == 10.0 && reflections[3].tilt == -5.0;
    openbus::probe::removeProbeDirectory(root);

    if (!reflectionCameraKeywordsParsed) {
        std::cerr << "Reflection camera keywords or their viewer-following flags were not parsed\n";
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
        result.passengerPathLinks.size() != 1 || result.passengerStepSoundPacks.size() != 1 ||
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
    return 0;
}
