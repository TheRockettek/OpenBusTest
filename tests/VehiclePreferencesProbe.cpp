#include "app/vehicle/VehiclePreferences.h"
#include "ProbeTempDirectory.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

bool checkOdometerModes(const std::filesystem::path& root) {
    using namespace openbus::app;
    using enum VehicleOdometerMode;
    if (parseVehicleOdometerMode() != Global || parseVehicleOdometerMode("global") != Global ||
        parseVehicleOdometerMode("vehicle") != Vehicle ||
        parseVehicleOdometerMode("vehicle-default") != VehicleDefault) {
        std::cerr << "Odometer mode names or the global default are incorrect\n";
        return false;
    }
    for (const char* invalid : {"", "unknown", "vehicles"}) {
        bool rejected = false;
        try {
            (void)parseVehicleOdometerMode(invalid);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected) {
            std::cerr << "Invalid odometer mode must be reported, not silently defaulted\n";
            return false;
        }
    }
    VehicleOdometerPreferences counters;
    const auto first = root / "Bus.bus";
    const auto second = root / "Other.bus";
    if (vehicleOdometerReading(counters, Global, first, 60000.0) != 0.0 ||
        vehicleOdometerReading(counters, Vehicle, first, 60000.0) != 0.0 ||
        vehicleOdometerReading(counters, VehicleDefault, first, 60000.0) != 60000000.0 ||
        vehicleOdometerReading(counters, VehicleDefault, second, 0.0) != 0.0) {
        std::cerr << "Fresh raw counters start at zero; only vehicle-default applies initial km\n";
        return false;
    }
    setVehicleOdometerReading(counters, Global, first, 60000.0, 100.25);
    if (vehicleOdometerReading(counters, Global, second, 70000.0) != 100.25) {
        std::cerr << "The global counter must follow the player across bus models\n";
        return false;
    }
    setVehicleOdometerReading(counters, Global, second, 70000.0, 200.5);
    setVehicleOdometerReading(counters, Vehicle, first, 60000.0, 10.25);
    if (vehicleOdometerReading(counters, Vehicle, second, 70000.0) != 0.0 ||
        vehicleOdometerReading(counters, VehicleDefault, first, 60000.0) != 60000010.25) {
        std::cerr << "Model counters must be independent and share raw mileage across modes\n";
        return false;
    }
    setVehicleOdometerReading(counters, VehicleDefault, first, 60000.0, 60000020.5);
    const auto savedPath = root / "mode-counters.txt";
    saveVehicleOdometerPreferences(savedPath, counters);
    counters = loadVehicleOdometerPreferences(savedPath);
    if (vehicleOdometerReading(counters, Global, first, 60000.0) != 200.5 ||
        vehicleOdometerReading(counters, Vehicle, first, 60000.0) != 20.5 ||
        vehicleOdometerReading(counters, VehicleDefault, first, 60000.0) != 60000020.5) {
        std::cerr << "Reloading or switching modes must not mix counters or double-add defaults\n";
        return false;
    }
    const auto unchanged = counters;
    for (const double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            setVehicleOdometerReading(counters, Global, first, 0.0, invalid);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected || counters != unchanged) {
            std::cerr << "Invalid readings must not modify existing counters\n";
            return false;
        }
    }
    return true;
}

int main() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus_vehicle_preferences_probe");
    const std::filesystem::path preferencesPath = root / "vehicle-preferences.txt";
    if (!checkOdometerModes(root)) {
        return 1;
    }
    if (!openbus::app::loadVehicleRegistrationPreferences(preferencesPath).empty()) {
        std::cerr << "A missing preferences file should load as empty\n";
        return 1;
    }

    const openbus::app::VehicleRegistrationPreferences expected = {
        {"/vehicles/Bus with spaces/MAN.bus", {"fleet \"12\"", "AB\\123"}},
        {"/vehicles/another.bus", {"", "CUSTOM"}},
    };
    openbus::app::saveVehicleRegistrationPreferences(preferencesPath, expected);
    if (openbus::app::loadVehicleRegistrationPreferences(preferencesPath) != expected) {
        std::cerr << "Vehicle preferences did not round-trip quoted values\n";
        return 1;
    }
    auto updatedPreferences = expected;
    updatedPreferences["/vehicles/Bus with spaces/MAN.bus"].registration = "updated";
    openbus::app::saveVehicleRegistrationPreferences(preferencesPath, updatedPreferences);
    if (openbus::app::loadVehicleRegistrationPreferences(preferencesPath) != updatedPreferences) {
        std::cerr << "Vehicle preferences did not atomically replace an existing file\n";
        return 1;
    }
    const auto odometerKey = openbus::app::vehicleOdometerPreferenceKey(root / "Bus.bus");
    if (odometerKey !=
        openbus::app::vehicleOdometerPreferenceKey(root / "unused" / ".." / "Bus.bus")) {
        std::cerr << "Equivalent model paths must share their odometer key\n";
        return 1;
    }
    const openbus::app::VehicleOdometerPreferences expectedOdometers = {
        .globalMetres = 750.5,
        .modelMetres = {{odometerKey, 123456.789},
                       {openbus::app::vehicleOdometerPreferenceKey(root / "Other.bus"), 0.0}},
    };
    const std::filesystem::path odometersPath = root / "vehicle-odometers.txt";
    if (openbus::app::loadVehicleOdometerPreferences(odometersPath) !=
        openbus::app::VehicleOdometerPreferences{}) {
        std::cerr << "Missing odometers must not seed authored mileage\n";
        return 1;
    }
    openbus::app::saveVehicleOdometerPreferences(odometersPath, expectedOdometers);
    if (openbus::app::loadVehicleOdometerPreferences(odometersPath) != expectedOdometers) {
        std::cerr << "Vehicle odometers did not round-trip per bus model\n";
        return 1;
    }
    {
        std::ifstream saved(odometersPath);
        std::string line;
        if (!std::getline(saved, line) || line != "OpenBus vehicle odometers 3" ||
            !std::getline(saved, line) || line != "global 750.5") {
            std::cerr << "Version-3 odometers must save a separate global counter\n";
            return 1;
        }
        std::size_t count = 0;
        while (std::getline(saved, line)) {
            std::istringstream record(line);
            std::string key;
            double metres = 0.0;
            std::string extra;
            if (!(record >> std::quoted(key) >> metres) || (record >> extra) ||
                !expectedOdometers.modelMetres.contains(key) ||
                expectedOdometers.modelMetres.at(key) != metres) {
                std::cerr << "Odometer records must contain only model path and metres\n";
                return 1;
            }
            ++count;
        }
        if (count != expectedOdometers.modelMetres.size()) {
            std::cerr << "Every model must have exactly one saved record\n";
            return 1;
        }
    }
    auto updatedOdometers = expectedOdometers;
    updatedOdometers.modelMetres[odometerKey] = 123999.5;
    openbus::app::saveVehicleOdometerPreferences(odometersPath, updatedOdometers);
    if (openbus::app::loadVehicleOdometerPreferences(odometersPath) != updatedOdometers) {
        std::cerr << "Vehicle odometers did not atomically replace an existing file\n";
        return 1;
    }
    bool negativeOdometerRejected = false;
    try {
        openbus::app::saveVehicleOdometerPreferences(
            odometersPath, {.modelMetres = {{odometerKey, -1.0}}});
    } catch (const std::invalid_argument&) {
        negativeOdometerRejected = true;
    }
    if (!negativeOdometerRejected ||
        openbus::app::loadVehicleOdometerPreferences(odometersPath) != updatedOdometers) {
        std::cerr << "Invalid odometer value was accepted or replaced valid saved state\n";
        return 1;
    }
    const auto legacyPath = root / "legacy-odometers.txt";
    {
        std::ofstream legacy(legacyPath);
        legacy << "OpenBus vehicle odometers 1\n"
               << std::quoted(odometerKey) << " \"3099\" \"OLD\" 60000000.076449774\n"
               << std::quoted(odometerKey) << " \"3100\" \"OTHER\" 70000000\n"
               << std::quoted(openbus::app::vehicleOdometerPreferenceKey(root / "Other.bus"))
               << " \"\" \"\" 1000\n\n";
    }
    const openbus::app::VehicleOdometerPreferences resetOdometers = {
        .modelMetres = {{odometerKey, 0.0},
                       {openbus::app::vehicleOdometerPreferenceKey(root / "Other.bus"), 0.0}},
    };
    auto migrated = openbus::app::loadVehicleOdometerPreferences(legacyPath);
    if (migrated != resetOdometers) {
        std::cerr << "Legacy fleet entries must collapse into zero-based model counters\n";
        return 1;
    }
    migrated.modelMetres[odometerKey] += 125.25;
    openbus::app::saveVehicleOdometerPreferences(legacyPath, migrated);
    if (openbus::app::loadVehicleOdometerPreferences(legacyPath) != migrated) {
        std::cerr << "Shared mileage must persist after migration without resetting again\n";
        return 1;
    }
    {
        std::ofstream previous(legacyPath);
        previous << "OpenBus vehicle odometers 2\n" << std::quoted(odometerKey) << " 125.25\n";
    }
    const auto previousVersion = openbus::app::loadVehicleOdometerPreferences(legacyPath);
    if (previousVersion.globalMetres != 0.0 ||
        previousVersion.modelMetres != std::map<std::string, double>{{odometerKey, 125.25}}) {
        std::cerr << "Version-2 model mileage must survive while the new global counter starts at zero\n";
        return 1;
    }
    const auto badOdometerPath = root / "bad-odometer.txt";
    for (const char* record : {"\"Bus.bus\" -1", "\"Bus.bus\" 1 extra",
                               "\"\" 1", "\"Bus.bus\" \"3099\" \"\" 1"}) {
        {
            std::ofstream malformed(badOdometerPath);
            malformed << "OpenBus vehicle odometers 2\n" << record << '\n';
        }
        bool rejected = false;
        try {
            (void)openbus::app::loadVehicleOdometerPreferences(badOdometerPath);
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        if (!rejected) {
            std::cerr << "Malformed model odometer record was accepted\n";
            return 1;
        }
    }
    for (const char* record : {"", "global -1", "global nan", "global 1 extra", "model 1"}) {
        {
            std::ofstream malformed(badOdometerPath);
            malformed << "OpenBus vehicle odometers 3\n" << record << '\n';
        }
        bool rejected = false;
        try {
            (void)openbus::app::loadVehicleOdometerPreferences(badOdometerPath);
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        if (!rejected) {
            std::cerr << "Malformed global odometer record was accepted\n";
            return 1;
        }
    }

    const std::filesystem::path malformedPath = root / "malformed.txt";
    {
        std::ofstream malformed(malformedPath);
        malformed << "wrong header\n";
    }
    try {
        (void)openbus::app::loadVehicleRegistrationPreferences(malformedPath);
    } catch (const std::runtime_error&) {
        openbus::probe::removeProbeDirectory(root);
        return 0;
    }
    std::cerr << "Malformed vehicle preferences were not rejected\n";
    return 1;
}
