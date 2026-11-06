#include "core/SolarLighting.h"
#include "core/Variables.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
    bool passed = true;
    const auto check = [&](bool condition, const char* message) {
        if (!condition) {
            std::cerr << message << '\n';
            passed = false;
        }
    };
    const auto noon = openbus::solarLighting(80, 43200, 0);
    const auto midnight = openbus::solarLighting(80, 0, 0);
    const auto morning = openbus::solarLighting(80, 21600, 0);
    const auto evening = openbus::solarLighting(80, 64800, 0);
    check(noon.altitudeDegrees > 89.9 && midnight.altitudeDegrees < -89.9,
          "equatorial equinox altitudes are incorrect");
    check(morning.direction[0] > 0.99 && evening.direction[0] < -0.99,
          "solar direction does not move east to west");
    check(openbus::solarLighting(172, 43200, 52.52).altitudeDegrees >
              openbus::solarLighting(355, 43200, 52.52).altitudeDegrees + 40,
          "seasonal sun altitude does not vary");
    check(openbus::solarLighting(172, 0, 80).altitudeDegrees > 0 &&
              openbus::solarLighting(355, 43200, 80).altitudeDegrees < 0,
          "polar day and night are incorrect");
    bool invalid = false;
    try {
        static_cast<void>(openbus::solarLighting(1, 0, std::numeric_limits<double>::quiet_NaN()));
    } catch (const std::invalid_argument&) {
        invalid = true;
    }
    check(invalid, "invalid latitude was accepted");

    SystemVariables system;
    system.configureSimulationClock(2024, 3, 20, 43200);
    system.configureDayNight(0);
    check(system.get("sunalt") > 89 && system.environmentBrightness().value() > 0.99,
          "clock did not publish midday sunlight");
    const auto dayDirection = system.sunDirection();
    system.updateSimulationClock(43200);
    check(system.get("sunalt") < -89 && system.environmentBrightness().value() < 0.07 &&
              system.sunDirection() != dayDirection,
          "simulation clock did not advance to night");
    const auto frozenBrightness = system.environmentBrightness();
    system.updateSimulationClock(43200);
    check(system.environmentBrightness() == frozenBrightness, "unchanged clock advanced daylight");
    system.configureSimulationClock(2024, 3, 20, 43200);
    openbus::WeatherState weather;
    weather.brightness = 0.5;
    system.setWeather(weather);
    check(std::abs(*system.environmentBrightness() - 0.5) < 0.001,
          "weather brightness did not scale daylight");
    openbus::scripting::Vehicle vehicle;
    vehicle.updateEnvironment(system);
    check(std::abs(vehicle.get("envir_brightness") - 0.5F) < 0.001F,
          "vehicle scripts did not receive daylight");
    system.setSunAltitude(-15);
    system.updateSimulationClock(20);
    check(system.get("sunalt") == -15 && *system.environmentBrightness() < 0.04,
          "manual sun altitude was not preserved");
    system.configureDayNight(0);
    check(system.get("sunalt") > 89, "reconfiguring daylight did not clear the altitude override");
    return passed ? 0 : 1;
}
