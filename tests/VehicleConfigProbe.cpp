#include "VehicleConfigLoader.h"

#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus_vehicle_config_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    const std::filesystem::path configPath = root / "bus.bus";
    std::ofstream config(configPath);
    config << "[model]\nmodel.cfg\n"
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
        result.soundConfigPath.generic_string() != (root / "sound/player.cfg").generic_string() ||
        result.soundAiConfigPath.generic_string() != (root / "sound/ai.cfg").generic_string() ||
        result.pathsConfigPath.generic_string() != (root / "model/paths.cfg").generic_string() ||
        result.passengerCabinConfigPath.generic_string() != (root / "model/cabin.cfg").generic_string() ||
        result.numberConfigPath.generic_string() != (root / "registrations.org").generic_string() ||
        result.registrationListConfigPath.generic_string() !=
            (root / "registration_list.org").generic_string() ||
        !result.registrationAutomatic || result.registrationPrefix != "B-TEST" ||
        !result.registrationFree || !result.hasOdometerInitial ||
        result.odometerInitialYear != 2026 || result.odometerInitialKilometres != 1234.5) {
        std::cerr << "BUS metadata records were not retained\n";
        return 1;
    }
    return 0;
}
