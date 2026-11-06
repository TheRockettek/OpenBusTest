#include "core/Variables.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

bool require(bool condition, const char* description) {
    if (!condition) {
        std::cerr << "failed: " << description << '\n';
        return false;
    }
    return true;
}

bool checkRoadWeatherEvolution() {
    bool valid = true;
    SystemVariables system;
    system.setWeather(1, 1.0, 10.0, 10.0);
    system.advanceRoadWeather(600.0);
    const double wet = system.roadWeather().water;
    valid &= require(wet > 0.5 && wet < 1.0 && system.roadWeather().snow == 0.0,
                     "rain accumulates road water gradually");
    system.setWeather(0, 0.5, 10.0, 10.0);
    system.advanceRoadWeather(0.0);
    valid &= require(system.roadWeather().water == wet,
                     "zero simulation time leaves road weather unchanged");
    system.advanceRoadWeather(600.0);
    valid &= require(system.roadWeather().water > 0.0 && system.roadWeather().water < wet,
                     "clear weather dries accumulated water gradually");

    SystemVariables transition;
    auto warmPreset = transition.weather();
    warmPreset.omsiGroundWetness = std::array<double, 3>{79.0, 0.0, 0.0};
    transition.setWeather(warmPreset);
    transition.seedRoadWeatherFromPreset();
    transition.setWeather(0, 0.0, -5.0, 10.0);
    transition.advanceRoadWeather(0.0);
    valid &= require(transition.roadWeather().freezeFactor == 0.0,
                     "temperature change and pause do not freeze a road instantaneously");
    transition.advanceRoadWeather(600.0);
    const double partiallyFrozen = transition.roadWeather().freezeFactor;
    valid &= require(std::abs(partiallyFrozen - (1.0 - std::exp(-1.0))) < 1.0e-12,
                     "cold wet roads freeze gradually over simulation time");
    transition.setWeather(0, 0.0, 5.0, 10.0);
    transition.advanceRoadWeather(0.0);
    valid &= require(transition.roadWeather().freezeFactor == partiallyFrozen,
                     "warming and pause do not thaw a road instantaneously");
    transition.advanceRoadWeather(300.0);
    valid &= require(std::abs(transition.roadWeather().freezeFactor -
                              partiallyFrozen * std::exp(-1.0)) < 1.0e-12,
                     "warm roads thaw gradually over simulation time");

    auto preset = system.weather();
    preset.omsiGroundWetness = std::array<double, 3>{39.5, 0.0, 0.0};
    preset.omsiSnowOnRoad = true;
    preset.temperatureCelsius = -5.0;
    system.setWeather(preset);
    system.seedRoadWeatherFromPreset();
    valid &= require(system.roadWeather().water == 0.5 && system.roadWeather().snow == 1.0 &&
                         system.roadWeather().freezeFactor == 1.0 &&
                         system.roadWeather().groundSnow == 0.0,
                     "freezing preset seeds road water, snow and cold exposure");
    auto groundOnlyPreset = preset;
    groundOnlyPreset.omsiSnow = true;
    groundOnlyPreset.omsiSnowOnRoad = false;
    const auto groundOnly = openbus::roadWeatherFromPreset(groundOnlyPreset);
    valid &= require(groundOnly.groundSnow == 1.0 && groundOnly.snow == 0.0,
                     "ground and road preset snow flags seed independently");
    auto groundThaw = groundOnly;
    groundOnlyPreset.temperatureCelsius = 5.0;
    openbus::advanceRoadWeather(groundThaw, groundOnlyPreset, 0.0);
    valid &= require(groundThaw.groundSnow == 1.0 && groundThaw.snow == 0.0,
                     "ground snow remains unchanged while paused");
    openbus::advanceRoadWeather(groundThaw, groundOnlyPreset, 600.0);
    valid &= require(groundThaw.groundSnow > 0.0 && groundThaw.groundSnow < 1.0 &&
                         groundThaw.snow == 0.0,
                     "ground snow thaws without adding road snow");
    preset.fogTopAboveGroundMeters = 50.0;
    system.advanceRoadWeather(100.0);
    const auto beforeFogChange = system.roadWeather();
    system.setWeather(preset);
    valid &= require(system.roadWeather().water == beforeFogChange.water &&
                         system.roadWeather().snow == beforeFogChange.snow &&
                         system.roadWeather().groundSnow == beforeFogChange.groundSnow &&
                         system.roadWeather().freezeFactor == beforeFogChange.freezeFactor,
                     "editing fog does not reset accumulated road conditions");
    preset.omsiGroundWetness.reset();
    preset.omsiSnowOnRoad = false;
    preset.temperatureCelsius = 10.0;
    system.setWeather(preset);
    system.advanceRoadWeather(600.0);
    valid &= require(system.roadWeather().snow < beforeFogChange.snow &&
                         system.roadWeather().water > beforeFogChange.water &&
                         system.roadWeather().freezeFactor > 0.0 &&
                         system.roadWeather().freezeFactor < beforeFogChange.freezeFactor,
                     "warm weather melts road snow into water");

    openbus::WeatherState snow;
    snow.temperatureCelsius = -5.0;
    snow.precipitationType = 2;
    snow.precipitationRate = 1.0;
    openbus::RoadWeatherConditions whole;
    openbus::RoadWeatherConditions split;
    openbus::advanceRoadWeather(whole, snow, 1200.0);
    for (int step = 0; step < 120; ++step) {
        openbus::advanceRoadWeather(split, snow, 10.0);
    }
    valid &= require(whole.snow > 0.5 && whole.groundSnow > 0.5 &&
                         std::abs(whole.snow - split.snow) < 1.0e-12 &&
                         std::abs(whole.groundSnow - split.groundSnow) < 1.0e-12 &&
                         std::abs(whole.water - split.water) < 1.0e-12 &&
                         std::abs(whole.freezeFactor - split.freezeFactor) < 1.0e-12,
                     "snow accumulation is consistent across time steps");
    snow.precipitationType = 0;
    snow.temperatureCelsius = 5.0;
    openbus::advanceRoadWeather(whole, snow, 600.0);
    for (int step = 0; step < 60; ++step) {
        openbus::advanceRoadWeather(split, snow, 10.0);
    }
    valid &= require(std::abs(whole.snow - split.snow) < 1.0e-12 &&
                         std::abs(whole.groundSnow - split.groundSnow) < 1.0e-12 &&
                         std::abs(whole.water - split.water) < 1.0e-12 &&
                         std::abs(whole.freezeFactor - split.freezeFactor) < 1.0e-12,
                     "snow melt and runoff are consistent across time steps");
    snow.temperatureCelsius = -5.0;
    openbus::RoadWeatherConditions frozen{0.5, 0.0};
    openbus::advanceRoadWeather(frozen, snow, 3600.0);
    valid &= require(frozen.water > 0.45 && frozen.water < 0.5 &&
                         frozen.freezeFactor > 0.99 && frozen.freezeFactor < 1.0,
                     "cold conditions preserve water and gradually increase freezing");
    openbus::advanceRoadWeather(frozen, snow, std::numeric_limits<double>::max());
    valid &= require(std::isfinite(frozen.water) && std::isfinite(frozen.snow) &&
                         std::isfinite(frozen.freezeFactor) && std::isfinite(frozen.groundSnow) &&
                         frozen.water >= 0.0 && frozen.water <= 1.0 &&
                         frozen.snow >= 0.0 && frozen.snow <= 1.0 &&
                         frozen.freezeFactor >= 0.0 && frozen.freezeFactor <= 1.0 &&
                         frozen.groundSnow >= 0.0 && frozen.groundSnow <= 1.0,
                     "extreme elapsed time remains finite and bounded");
    const auto beforeInvalid = system.roadWeather();
    bool rejected = false;
    try {
        system.advanceRoadWeather(std::numeric_limits<double>::infinity());
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    valid &= require(rejected && system.roadWeather().water == beforeInvalid.water &&
                         system.roadWeather().snow == beforeInvalid.snow &&
                         system.roadWeather().freezeFactor == beforeInvalid.freezeFactor,
                     "invalid road weather time is rejected atomically");
    rejected = false;
    try {
        system.advanceRoadWeather(-1.0);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    valid &= require(rejected && system.roadWeather().water == beforeInvalid.water &&
                         system.roadWeather().snow == beforeInvalid.snow &&
                         system.roadWeather().freezeFactor == beforeInvalid.freezeFactor,
                     "negative road weather time is rejected atomically");
    system.resetRoadWeather();
    valid &= require(system.roadWeather().water == 0.0 && system.roadWeather().snow == 0.0 &&
                         system.roadWeather().freezeFactor == 0.0 &&
                         system.roadWeather().groundSnow == 0.0,
                     "road weather reset clears accumulated conditions");
    return valid;
}

} // namespace

int main() {
    SimulationState simulation;
    openbus::scripting::Vehicle vehicle;
    openbus::scripting::Human human;
    openbus::scripting::SceneryObject sceneryObject;

    bool valid = checkRoadWeatherEvolution();
    valid &= require(!sceneryObject.hasExplicitSceneryNightlight(),
                     "default scenery nightlight is not an explicit controller");
    sceneryObject.set("nightlighta", 0.0F);
    valid &= require(sceneryObject.hasExplicitSceneryNightlight(),
                     "explicit zero scenery nightlight is tracked");
    valid &= require(simulation.sharedVariables().has("timegap"), "system timegap");
    valid &= require(!simulation.sharedVariables().has("throttle"), "system excludes throttle");
    valid &= require(simulation.sharedVariables().get("preciptype") == 0.0F,
                     "system precipitation defaults to the OMSI clear value");
    simulation.sharedVariables().setWeather(2, 0.25, 12.5, 8.75);
    valid &= require(simulation.sharedVariables().get("preciptype") == 2.0F &&
                         simulation.sharedVariables().get("preciprate") == 0.25F &&
                         simulation.sharedVariables().get("weather_temperature") == 12.5F &&
                         simulation.sharedVariables().get("weather_abshum") == 8.75F,
                     "weather settings publish precipitation and ambient conditions together");
    bool rejectedInvalidWeather = false;
    try {
        simulation.sharedVariables().setWeather(1, 0.5, -273.15, 3.0);
    } catch (const std::invalid_argument&) {
        rejectedInvalidWeather = true;
    }
    valid &= require(rejectedInvalidWeather &&
                         simulation.sharedVariables().get("preciptype") == 2.0F &&
                         simulation.sharedVariables().get("preciprate") == 0.25F &&
                         simulation.sharedVariables().get("weather_temperature") == 12.5F &&
                         simulation.sharedVariables().get("weather_abshum") == 8.75F,
                     "invalid weather settings are rejected atomically");
    openbus::WeatherState weather = simulation.sharedVariables().weather();
    valid &= require(!weather.visibilityMeters && !weather.windDirectionDegrees &&
                         !weather.cloudType && !weather.atmosphericPressureHectopascals &&
                         !simulation.sharedVariables().weatherVisibilityMeters() &&
                         !simulation.sharedVariables().effectiveWeatherVisibilityMeters(),
                     "unavailable weather observations are not inferred");
    weather.visibilityMeters = 5000.0;
    weather.brightness = 0.4;
    weather.windDirectionDegrees = 270.0;
    weather.windSpeedMetersPerSecond = 12.0;
    weather.cloudBaseAboveGroundMeters = 900.0;
    weather.fogTopAboveGroundMeters = 120.0;
    weather.cloudType = openbus::WeatherCloudType::Cumulus2;
    weather.atmosphericPressureHectopascals = 1013.25;
    simulation.sharedVariables().setWeather(weather);
    valid &= require(simulation.sharedVariables().weather().visibilityMeters == 5000.0 &&
                         simulation.sharedVariables().weatherVisibilityMeters() == 5000.0 &&
                         simulation.sharedVariables().weather().cloudType ==
                             openbus::WeatherCloudType::Cumulus2 &&
                         simulation.sharedVariables().get("preciptype") == 2.0F &&
                         simulation.sharedVariables().get("weather_abshum") == 8.75F,
                     "complete weather snapshot retains observations and publishes script values");
    const auto rejectsWeather = [&](const openbus::WeatherState& invalid) {
        try {
            simulation.sharedVariables().setWeather(invalid);
            return false;
        } catch (const std::invalid_argument&) {
            return simulation.sharedVariables().weather().visibilityMeters == 5000.0 &&
                   simulation.sharedVariables().weather().windDirectionDegrees == 270.0 &&
                   simulation.sharedVariables().get("preciptype") == 2.0F &&
                   simulation.sharedVariables().get("weather_temperature") == 12.5F;
        }
    };
    auto invalidSnapshot = weather;
    invalidSnapshot.visibilityMeters = -1.0;
    valid &= require(rejectsWeather(invalidSnapshot), "negative visibility is rejected atomically");
    invalidSnapshot = weather;
    invalidSnapshot.brightness = std::numeric_limits<double>::quiet_NaN();
    valid &= require(rejectsWeather(invalidSnapshot), "non-finite brightness is rejected");
    invalidSnapshot = weather;
    invalidSnapshot.windDirectionDegrees = 360.0;
    valid &= require(rejectsWeather(invalidSnapshot), "wind direction excludes 360 degrees");
    invalidSnapshot = weather;
    invalidSnapshot.windSpeedMetersPerSecond = -0.01;
    valid &= require(rejectsWeather(invalidSnapshot), "negative wind speed is rejected");
    invalidSnapshot = weather;
    invalidSnapshot.cloudBaseAboveGroundMeters = -1.0;
    valid &= require(rejectsWeather(invalidSnapshot), "negative cloud base is rejected");
    invalidSnapshot = weather;
    invalidSnapshot.fogTopAboveGroundMeters = std::numeric_limits<double>::infinity();
    valid &= require(rejectsWeather(invalidSnapshot), "non-finite fog height is rejected");
    invalidSnapshot.fogTopAboveGroundMeters = std::numeric_limits<double>::max();
    valid &= require(rejectsWeather(invalidSnapshot), "unrepresentable fog height is rejected");
    invalidSnapshot = weather;
    invalidSnapshot.omsiFogHeight = std::numeric_limits<double>::quiet_NaN();
    valid &= require(rejectsWeather(invalidSnapshot), "non-finite preset fog height is rejected");
    invalidSnapshot = weather;
    invalidSnapshot.omsiGroundWetness =
        std::array<double, 3>{0.0, std::numeric_limits<double>::infinity(), 0.0};
    valid &= require(rejectsWeather(invalidSnapshot), "non-finite ground wetness is rejected");
    invalidSnapshot = weather;
    invalidSnapshot.atmosphericPressureHectopascals = 0.0;
    valid &= require(rejectsWeather(invalidSnapshot), "non-positive pressure is rejected");
    invalidSnapshot = weather;
    invalidSnapshot.cloudType = static_cast<openbus::WeatherCloudType>(100);
    valid &= require(rejectsWeather(invalidSnapshot), "unknown cloud type is rejected");
    invalidSnapshot = weather;
    invalidSnapshot.temperatureCelsius = std::numeric_limits<double>::max();
    valid &=
        require(rejectsWeather(invalidSnapshot), "unrepresentable script temperature is rejected");
    simulation.sharedVariables().setPrecipitation(1, 0.5);
    valid &= require(simulation.sharedVariables().weather().cloudType ==
                             openbus::WeatherCloudType::Cumulus2 &&
                         simulation.sharedVariables().weather().visibilityMeters == 5000.0 &&
                         simulation.sharedVariables().get("preciptype") == 1.0F,
                     "legacy precipitation updates preserve other weather observations");
    simulation.sharedVariables().set("preciprate", 0.75F);
    simulation.sharedVariables().set("weather_temperature", -5.0F);
    valid &= require(simulation.sharedVariables().weather().precipitationRate == 0.75 &&
                         simulation.sharedVariables().weather().temperatureCelsius == -5.0,
                     "weather snapshots reflect direct system variable writes");
    simulation.sharedVariables().set("preciprate", 1.0F);
    for (const auto& [type, expected] :
         {std::pair{0, 5000.0}, std::pair{1, 100000.0 / 255.0},
          std::pair{2, 20000.0 / 255.0}}) {
        simulation.sharedVariables().set("preciptype", static_cast<float>(type));
        const auto visibility = simulation.sharedVariables().effectiveWeatherVisibilityMeters();
        valid &= require(visibility && std::abs(*visibility - expected) < 0.001,
                         "effective visibility applies OMSI's precipitation limit");
    }
    simulation.sharedVariables().set("preciprate", 0.0F);
    valid &= require(simulation.sharedVariables().effectiveWeatherVisibilityMeters() == 5000.0,
                     "zero precipitation leaves visibility unchanged");
    simulation.sharedVariables().set("preciprate", 0.01F);
    valid &= require(simulation.sharedVariables().effectiveWeatherVisibilityMeters() == 5000.0,
                     "precipitation does not extend an existing shorter visibility");
    for (const int type : {0, 1, 2}) {
        simulation.sharedVariables().set("preciptype", static_cast<float>(type));
        valid &= require(simulation.sharedVariables().weather().precipitationType == type,
                         "direct script precipitation values retain OMSI's 0-based type");
    }
    simulation.sharedVariables().set("preciptype", 1.5F);
    bool invalidSystemWeatherRejected = false;
    try {
        static_cast<void>(simulation.sharedVariables().weather());
    } catch (const std::invalid_argument&) {
        invalidSystemWeatherRejected = true;
    }
    bool invalidVisibilityRejected = false;
    try {
        static_cast<void>(simulation.sharedVariables().effectiveWeatherVisibilityMeters());
    } catch (const std::invalid_argument&) {
        invalidVisibilityRejected = true;
    }
    valid &=
        require(invalidSystemWeatherRejected && invalidVisibilityRejected &&
                    simulation.sharedVariables().weatherVisibilityMeters() == 5000.0 &&
                    simulation.sharedVariables().weatherCloudType() ==
                        openbus::WeatherCloudType::Cumulus2 &&
                    simulation.sharedVariables().weatherCloudBaseAboveGroundMeters() == 900.0 &&
                    simulation.sharedVariables().weatherWindDirectionDegrees() == 270.0 &&
                    simulation.sharedVariables().weatherWindSpeedMetersPerSecond() == 12.0,
                "invalid script precipitation does not hide rendering observations");
    simulation.sharedVariables().setWeather(weather);
    {
        SimulationState lightingState;
        openbus::scripting::Vehicle first;
        openbus::scripting::Vehicle second;
        first.set("envir_brightness", 0.8F);
        first.updateEnvironment(lightingState.sharedVariables());
        valid &= require(first.get("envir_brightness") == 0.8F,
                         "unknown weather brightness preserves vehicle-local behavior");
        auto lightingWeather = lightingState.sharedVariables().weather();
        lightingWeather.brightness = 0.25;
        lightingState.sharedVariables().setWeather(lightingWeather);
        const auto environment =
            openbus::scripting::Vehicle::captureEnvironment(lightingState.sharedVariables());
        first.updateEnvironment(environment);
        second.updateEnvironment(environment);
        valid &= require(first.get("envir_brightness") == 0.25F &&
                             second.get("envir_brightness") == 0.25F,
                         "configured brightness reaches each vehicle");
        auto invalidEnvironment = environment;
        invalidEnvironment.brightness = -1.0F;
        invalidEnvironment.precipitationRate = 0.5F;
        bool rejected = false;
        try {
            first.updateEnvironment(invalidEnvironment);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        valid &= require(rejected && first.get("envir_brightness") == 0.25F &&
                             first.get("preciprate") == 0.0F,
                         "invalid vehicle brightness is rejected atomically");
        first.set("envir_brightness", 0.7F);
        first.updateEnvironment(environment);
        valid &= require(first.get("envir_brightness") == 0.25F,
                         "configured weather owns brightness on each update");
        lightingWeather.brightness.reset();
        lightingState.sharedVariables().setWeather(lightingWeather);
        first.updateEnvironment(lightingState.sharedVariables());
        second.updateEnvironment(lightingState.sharedVariables());
        first.set("envir_brightness", 0.6F);
        first.updateEnvironment(lightingState.sharedVariables());
        valid &= require(first.get("envir_brightness") == 0.6F &&
                             second.get("envir_brightness") == 1.0F,
                         "removing brightness clears the override and restores local ownership");
    }
    simulation.sharedVariables().setAutoClutch(true);
    valid &= require(simulation.sharedVariables().get("autoclutch") == 1.0F,
                     "automatic clutch setting is published as enabled");
    simulation.sharedVariables().setAutoClutch(false);
    valid &= require(simulation.sharedVariables().get("autoclutch") == 0.0F,
                     "automatic clutch setting is published as disabled");
    simulation.sharedVariables().setSunAltitude(-12.5);
    valid &= require(simulation.sharedVariables().get("sunalt") == -12.5F,
                     "sun altitude setting is published to scripts");
    bool rejectedInvalidSunAltitude = false;
    try {
        simulation.sharedVariables().setSunAltitude(90.1);
    } catch (const std::invalid_argument&) {
        rejectedInvalidSunAltitude = true;
    }
    valid &= require(rejectedInvalidSunAltitude &&
                         simulation.sharedVariables().get("sunalt") == -12.5F,
                     "invalid sun altitude is rejected without changing the prior value");
    simulation.sharedVariables().setPaused(true);
    valid &= require(simulation.sharedVariables().get("pause") == 1.0F,
                     "paused state is exposed to scripts");
    simulation.sharedVariables().setPaused(false);
    valid &= require(simulation.sharedVariables().get("pause") == 0.0F,
                     "resumed state is exposed to scripts");
    simulation.sharedVariables().configureSimulationClock(2024, 2, 28, 86398.0);
    simulation.sharedVariables().updateSimulationClock(3.0);
    valid &= require(simulation.sharedVariables().get("year") == 2024.0F &&
                         simulation.sharedVariables().get("month") == 2.0F &&
                         simulation.sharedVariables().get("day") == 29.0F &&
                         simulation.sharedVariables().get("dayofyear") == 60.0F &&
                         simulation.sharedVariables().get("time") == 1.0F,
                     "simulation clock advances across leap-day midnight");
    simulation.sharedVariables().configureSimulationClock(2024, 12, 31, 86399.0);
    valid &= require(simulation.sharedVariables().get("dayofyear") == 366.0F,
                     "leap year ends on day 366");
    simulation.sharedVariables().updateSimulationClock(1.0);
    valid &= require(simulation.sharedVariables().get("year") == 2025.0F &&
                         simulation.sharedVariables().get("month") == 1.0F &&
                         simulation.sharedVariables().get("day") == 1.0F &&
                         simulation.sharedVariables().get("dayofyear") == 1.0F &&
                         simulation.sharedVariables().get("time") == 0.0F,
                     "simulation clock advances across year boundary");
    simulation.sharedVariables().configureSimulationClock(2024, 12, 31, 86399.75);
    simulation.sharedVariables().updateSimulationClock(0.20);
    valid &= require(simulation.sharedVariables().get("day") == 31.0F &&
                         simulation.sharedVariables().get("time") == 86399.0F,
                     "fractional start remains on its original day before midnight");
    simulation.sharedVariables().updateSimulationClock(0.25);
    valid &= require(simulation.sharedVariables().get("year") == 2025.0F &&
                         simulation.sharedVariables().get("month") == 1.0F &&
                         simulation.sharedVariables().get("day") == 1.0F &&
                         simulation.sharedVariables().get("time") == 0.0F,
                     "fractional start and elapsed time carry across midnight");
    simulation.sharedVariables().configureSimulationClock(2025, 12, 31, 0.0);
    valid &= require(simulation.sharedVariables().get("dayofyear") == 365.0F,
                     "common year ends on day 365");
    simulation.sharedVariables().configureSimulationClock(2025, 2, 28, 86399.0);
    simulation.sharedVariables().updateSimulationClock(1.0);
    valid &= require(simulation.sharedVariables().get("month") == 3.0F &&
                         simulation.sharedVariables().get("day") == 1.0F &&
                         simulation.sharedVariables().get("dayofyear") == 60.0F,
                     "common-year March follows day 59");
    bool rejectedInvalidCalendar = false;
    try {
        simulation.sharedVariables().configureSimulationClock(2025, 2, 29, 0.0);
    } catch (const std::invalid_argument&) {
        rejectedInvalidCalendar = true;
    }
    valid &= require(rejectedInvalidCalendar, "invalid simulation calendar is rejected");

    valid &= require(vehicle.has("throttle"), "vehicle throttle");
    valid &= require(!vehicle.has("lastmoveddist"), "vehicle excludes human state");
    valid &= require(human.has("lastmoveddist"), "human lastmoveddist");
    valid &= require(!human.has("throttle"), "human excludes vehicle state");
    valid &= require(sceneryObject.has("nightlighta"), "scenery nightlighta");
    valid &= require(!sceneryObject.has("throttle"), "scenery excludes vehicle state");
    valid &= require(vehicle.supportsSystemMacro("getrouteindex"), "vehicle macro capability");
    valid &= require(!vehicle.supportsSystemMacro("getarrbusline"),
                     "vehicle excludes scenery macro");
    valid &= require(sceneryObject.supportsSystemMacro("getarrbusline"),
                     "scenery macro capability");
    valid &= require(!human.supportsSystemMacro("getrouteindex"), "human excludes vehicle macro");
    valid &= require(vehicle.supportsSystemTrigger("collision"), "vehicle trigger capability");
    valid &= require(vehicle.supportsSystemTrigger("AI_SCHEDULED_SETTARGET") &&
                         vehicle.supportsSystemTrigger("railbond_3"),
                     "system triggers are case-insensitive and include the listed rail axles");
    valid &= require(!vehicle.supportsSystemTrigger("railbond_4") &&
                         !vehicle.supportsSystemTrigger("railbond_invalid"),
                     "unsupported railbond indexes are rejected");
    valid &= require(!sceneryObject.supportsSystemTrigger("collision"),
                     "scenery excludes vehicle trigger");
    valid &= require(human.supportsSystemTrigger("int_haltewunsch") &&
                         !human.supportsSystemTrigger("ai_scheduled_settarget"),
                     "human trigger capabilities are object-kind scoped");

    simulation.sharedVariables().set("mouse_x", 9.0F);
    simulation.sharedVariables().set("mouse_y", -4.0F);
    simulation.sharedVariables().setPrecipitation(1, 0.75);
    simulation.sharedVariables().setSoundAvailable(true);
    simulation.sharedVariables().updateFrame(0.125F, 10.0F, 800.0F, 600.0F);
    valid &= require(simulation.sharedVariables().get("mouse_x") == 0.0F &&
                         simulation.sharedVariables().get("mouse_y") == 0.0F &&
                         simulation.sharedVariables().get("mouse_cursor_x") == 800.0F &&
                         simulation.sharedVariables().get("mouse_cursor_y") == 600.0F &&
                         simulation.sharedVariables().get("timegap") == 0.125F &&
                         simulation.sharedVariables().get("preciptype") == 1.0F &&
                         simulation.sharedVariables().get("preciprate") == 0.75F &&
                         simulation.sharedVariables().get("nosound") == 0.0F,
                     "frame refresh resets motion but preserves weather and cursor/time state");
    simulation.sharedVariables().setSoundAvailable(false);
    valid &= require(simulation.sharedVariables().get("nosound") == 1.0F,
                     "unavailable sound is published to scripts");
    const auto environment =
        openbus::scripting::Vehicle::captureEnvironment(simulation.sharedVariables());
    vehicle.updateEnvironment(environment);
    valid &= require(vehicle.get("preciptype") == 1.0F && vehicle.get("preciprate") == 0.75F,
                     "a shared frame environment publishes weather to a vehicle");
    vehicle.set("preciptype", 3.0F);
    vehicle.set("preciprate", 0.0F);
    vehicle.updateEnvironment(environment);
    valid &= require(vehicle.get("preciptype") == 1.0F && vehicle.get("preciprate") == 0.75F,
                     "a shared frame environment restores script-modified weather");
    simulation.sharedVariables().setPrecipitation(2, 0.25);
    vehicle.updateEnvironment(simulation.sharedVariables());
    valid &= require(vehicle.get("preciptype") == 2.0F && vehicle.get("preciprate") == 0.25F,
                     "direct environment updates still capture the current weather");
    simulation.sharedVariables().setPrecipitation(1, 0.75);
    for (const auto& [type, rate] :
         {std::pair{-1, 0.5}, std::pair{3, 0.5}, std::pair{1, -0.1},
          std::pair{2, 1.1}, std::pair{1, std::numeric_limits<double>::quiet_NaN()}}) {
        bool rejected = false;
        try {
            simulation.sharedVariables().setPrecipitation(type, rate);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        valid &= require(rejected && simulation.sharedVariables().get("preciptype") == 1.0F &&
                             simulation.sharedVariables().get("preciprate") == 0.75F,
                         "invalid weather is rejected without changing either variable");
    }

    vehicle.setString("ident", "probe");
    valid &= require(vehicle.hasString("ident") && vehicle.getString("ident") == "probe",
                     "vehicle string state");
    valid &= require(vehicle.numericValues().count("throttle") == 1,
                     "numeric variable snapshot");
    valid &= require(vehicle.stringValues().count("ident") == 1,
                     "string variable snapshot");
    vehicle.set("steeringwheeloffset", 0.25);
    vehicle.updateFrame();
    valid &= require(vehicle.get("steeringwheeloffset") == 0.25,
                     "local numeric state persists across frames");
    valid &= require(vehicle.getString("ident") == "probe",
                     "local string state persists across frames");

    vehicle.set("mixedcasevalue", 0.75);
    valid &= require(vehicle.get("mixedcasevalue") == 0.75,
                     "normalized numeric state uses lowercase keys");
    vehicle.setString("mixedcasestring", "normalized");
    valid &= require(vehicle.getString("mixedcasestring") == "normalized",
                     "normalized string state uses lowercase keys");
    return valid ? 0 : 1;
}