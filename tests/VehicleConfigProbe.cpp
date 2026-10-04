#include "VehicleConfigLoader.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus_vehicle_config_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    std::filesystem::create_directories(root / "model");
    const std::filesystem::path configPath = root / "bus.bus";
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
    std::ofstream config(configPath);
    config << "[friendlyname]\nMercedes-Benz\nCitaro E400\nNight blue\n"
              "[description]\nUrban service bus.\n\n Electric drivetrain. \n[end]\n"
              "[type]\n0\n"
              "[model]\nmodel.cfg\n"
              "[sound]\nsound/player.cfg\n"
              "[sound_ai]\nsound/ai.cfg\n"
              "[paths]\nmodel/paths.cfg\n"
              "[passengercabin]\nmodel/cabin.cfg\n"
              "[number]\nregistrations.org\n"
              "[registration_list]\nregistration_list.org\n"
              "[registration_automatic]\nB-TEST \n"
              "[registration_free]\n"
              "[kmcounter_init]\n2026\n1234.5\n";
    config.close();

    const VehicleConfig result = loadVehicleConfig(configPath, VehicleFileKind::Bus);
    std::filesystem::remove_all(root);

    if (result.diagnostics.hasErrors() ||
        result.friendlyManufacturer != "Mercedes-Benz" ||
        result.friendlyVehicleName != "Citaro E400" ||
        result.friendlyDefaultPaint != "Night blue" ||
        result.description != "Urban service bus.\n\n Electric drivetrain. " ||
        !result.vehicleType.has_value() || *result.vehicleType != 0 ||
        result.soundConfigPath.generic_string() != (root / "sound/player.cfg").generic_string() ||
        result.soundAiConfigPath.generic_string() != (root / "sound/ai.cfg").generic_string() ||
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
        !result.registrationListsLoaded || result.registrationNumbers.size() != 3 ||
        result.selectedRegistration != "BUS-001") {
        std::cerr << "BUS metadata records were not retained\n";
        return 1;
    }
    return 0;
}
