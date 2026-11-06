#include "core/Variables.h"
#include "core/WeatherSelection.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--live") {
        openbus::WeatherSelection live;
        live.requestMetar(argv[2]);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        std::optional<openbus::WeatherState> observed;
        while (live.pending() && std::chrono::steady_clock::now() < deadline) {
            if (auto weather = live.poll())
                observed = weather;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!observed || live.failed()) {
            std::cerr << "Live METAR selection failed: " << live.status() << '\n';
            return 1;
        }
        SystemVariables variables;
        variables.setWeather(*observed);
        if (!variables.weatherVisibilityMeters() ||
            variables.weather().temperatureCelsius !=
                static_cast<float>(observed->temperatureCelsius)) {
            std::cerr << "Live METAR did not reach shared weather state\n";
            return 1;
        }
        std::cout << live.status() << '\n';
        return 0;
    }
    bool passed = true;
    const auto check = [&](bool condition, const char* message) {
        if (!condition) {
            std::cerr << message << '\n';
            passed = false;
        }
    };
    if (argc != 2) {
        std::cerr << "Station CSV path is required\n";
        return 1;
    }
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) {
        std::cerr << "Could not open station CSV\n";
        return 1;
    }
    const std::string csv((std::istreambuf_iterator<char>(input)),
                          std::istreambuf_iterator<char>());
    const auto allStations = openbus::parseMetarStations(csv);
    const auto embedded = openbus::metarStations();
    check(allStations.size() == embedded.size() && allStations.size() > 7000,
          "embedded catalogue does not contain all worldwide stations");
    check(std::equal(
              allStations.begin(), allStations.end(), embedded.begin(), embedded.end(),
              [](const auto& a, const auto& b) {
                  return a.code == b.code && a.name == b.name && a.country == b.country;
              }),
          "embedded catalogue does not match the CSV");
    check(std::any_of(embedded.begin(), embedded.end(),
                      [](const auto& station) { return station.code == "EGLL"; }) &&
              std::any_of(embedded.begin(), embedded.end(), [](const auto& station) {
                  return station.code == "KJFK" && station.country == "United States";
              }) &&
              std::any_of(embedded.begin(), embedded.end(), [](const auto& station) {
                  return station.code == "NCPL" && station.name.empty();
              }),
          "worldwide catalogue or station without metadata was omitted");
    const auto quoted =
        openbus::parseMetarStations("\xEF\xBB\xBF"
                                      "code,name,country\r\n"
                                      "EGLL,\"London, \"\"Heathrow\"\"\",United Kingdom\r\n"
                                      "KJFK,New York,United States\r\n");
    check(quoted.size() == 2 && quoted[0].name == "London, \"Heathrow\"" &&
              quoted[1].country == "United States",
          "quoted CSV fields or BOM were not parsed");
    for (const std::string invalid :
         {"code,name,country\nEGLL,\"bad,United Kingdom\n",
          "code,name,country\nEGLL,Name,United Kingdom\nEGLL,Other,United Kingdom\n",
          "code,name,country\nBAD,Name,United Kingdom\n", "code,name\nEGLL,Name\n",
          "code,name,country\nEGLL,Name,United Kingdom,extra\n"}) {
        bool rejected = false;
        try {
            static_cast<void>(openbus::parseMetarStations(invalid));
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        check(rejected, "invalid station catalogue was not rejected");
    }

    std::promise<void> release;
    const auto ready = release.get_future().share();
    openbus::WeatherSelection pending([ready](const std::string& station) {
        ready.wait();
        if (station != "EGLL")
            throw std::runtime_error("station normalization failed");
        openbus::WeatherState weather;
        weather.precipitationType = 2;
        weather.precipitationRate = 0.5;
        return weather;
    });
    pending.requestMetar("egll");
    const auto before = std::chrono::steady_clock::now();
    check(!pending.poll() && pending.pending(), "pending request completed before release");
    check(std::chrono::steady_clock::now() - before < std::chrono::milliseconds(100),
          "poll blocked the calling thread");
    bool duplicateRejected = false;
    try {
        pending.requestMetar("EGKK");
    } catch (const std::logic_error&) {
        duplicateRejected = true;
    }
    check(duplicateRejected, "overlapping METAR requests were not rejected");
    release.set_value();
    std::optional<openbus::WeatherState> result;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (pending.pending() && std::chrono::steady_clock::now() < deadline) {
        if (auto weather = pending.poll())
            result = weather;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(result && result->precipitationType == 2 && !pending.failed() &&
              pending.status().find("EGLL") != std::string::npos,
          "completed METAR did not return weather");

    std::promise<void> releaseDiscarded;
    const auto discardedReady = releaseDiscarded.get_future().share();
    openbus::WeatherSelection discarded([discardedReady](const std::string&) {
        discardedReady.wait();
        openbus::WeatherState weather;
        weather.precipitationType = 2;
        weather.precipitationRate = 0.5;
        return weather;
    });
    discarded.requestMetar("EGLL");
    SystemVariables current;
    openbus::WeatherState selectedCustom;
    selectedCustom.precipitationType = 1;
    selectedCustom.precipitationRate = 0.75;
    current.setWeather(openbus::customWeatherFromDraft(selectedCustom));
    discarded.discardPending();
    check(!discarded.activeStation(), "discarded request kept automatic refresh enabled");
    discarded.enableMetarRefresh("KSEA", current.weather());
    releaseDiscarded.set_value();
    bool appliedDiscarded = false;
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (discarded.pending() && std::chrono::steady_clock::now() < deadline) {
        if (auto oldWeather = discarded.poll()) {
            current.setWeather(*oldWeather);
            appliedDiscarded = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(!discarded.pending() && !appliedDiscarded && !discarded.failed() &&
              current.weather().precipitationType == 1 && discarded.activeStation() == "KSEA",
          "superseded METAR changed the newer weather or refresh station");
    current.setWeather(openbus::WeatherState{});

    openbus::WeatherState draft;
    draft.precipitationType = 2;
    draft.precipitationRate = 0.6;
    draft.temperatureCelsius = -4.5;
    draft.absoluteHumidityGramsPerCubicMeter = 3.5;
    draft.visibilityMeters = 2500.0;
    draft.brightness = 0.4;
    draft.windDirectionDegrees = 270.0;
    draft.windSpeedMetersPerSecond = 12.0;
    draft.cloudBaseAboveGroundMeters = 400.0;
    draft.fogTopAboveGroundMeters = 100.0;
    draft.cloudType = openbus::WeatherCloudType::Overcast1;
    draft.atmosphericPressureHectopascals = 995.0;
    draft.omsiFogHeight = 0.5;
    draft.omsiPrecipitationParameters = std::array<double, 3>{1.0, 2.0, 3.0};
    draft.omsiGroundWetness = std::array<double, 3>{79.0, 0.0, 0.0};
    draft.omsiSnow = true;
    draft.omsiSnowOnRoad = true;
    SystemVariables customVariables;
    customVariables.setWeather(draft);
    customVariables.seedRoadWeatherFromPreset();
    const auto previousRoad = customVariables.roadWeather();
    const auto custom = openbus::customWeatherFromDraft(draft);
    customVariables.setWeather(custom);
    const auto applied = customVariables.weather();
    check(applied.precipitationType == 2 &&
              std::abs(applied.precipitationRate - 0.6) < 1e-6 &&
              applied.temperatureCelsius == -4.5 &&
              applied.absoluteHumidityGramsPerCubicMeter == 3.5 &&
              applied.visibilityMeters == 2500.0 && applied.brightness == 0.4 &&
              applied.windDirectionDegrees == 270.0 &&
              applied.windSpeedMetersPerSecond == 12.0 &&
              applied.cloudBaseAboveGroundMeters == 400.0 &&
              applied.fogTopAboveGroundMeters == 100.0 &&
              applied.cloudType == openbus::WeatherCloudType::Overcast1 &&
              applied.atmosphericPressureHectopascals == 995.0 &&
              !applied.omsiFogHeight && !applied.omsiPrecipitationParameters &&
              !applied.omsiGroundWetness && !applied.omsiSnow && !applied.omsiSnowOnRoad &&
              customVariables.roadWeather().water == previousRoad.water &&
              customVariables.roadWeather().snow == previousRoad.snow &&
              customVariables.roadWeather().groundSnow == previousRoad.groundSnow,
          "custom weather did not preserve all inputs and accumulated surface conditions");
    check(customVariables.get("preciptype") == 2 &&
              customVariables.get("weather_temperature") == -4.5F &&
              customVariables.get("weather_abshum") == 3.5F &&
              customVariables.effectiveWeatherVisibilityMeters().value_or(2500.0) < 2500.0,
          "custom weather did not update script variables or precipitation visibility");
    auto clear = draft;
    clear.precipitationType = 0;
    clear.visibilityMeters.reset();
    clear.cloudType.reset();
    const auto customClear = openbus::customWeatherFromDraft(clear);
    check(customClear.precipitationRate == 0.0 && !customClear.visibilityMeters &&
              !customClear.cloudType,
          "clear custom weather or unspecified values were not preserved");
    for (const auto type : {openbus::WeatherCloudType::Cumulus1,
                            openbus::WeatherCloudType::Cumulus2,
                            openbus::WeatherCloudType::Cumulus3,
                            openbus::WeatherCloudType::Overcast1}) {
        auto candidate = draft;
        candidate.cloudType = type;
        check(openbus::customWeatherFromDraft(candidate).cloudType == type,
              "custom cloud selection was not retained");
    }
    for (const auto& invalid : std::vector<openbus::WeatherState>{
             [&] { auto value = draft; value.precipitationRate = 1.01; return value; }(),
             [&] { auto value = draft; value.temperatureCelsius = -273.15; return value; }(),
             [&] { auto value = draft; value.absoluteHumidityGramsPerCubicMeter = -1; return value; }(),
             [&] { auto value = draft; value.visibilityMeters = -1; return value; }(),
             [&] { auto value = draft; value.brightness = 1.1; return value; }(),
             [&] { auto value = draft; value.windDirectionDegrees = 360; return value; }(),
             [&] { auto value = draft; value.windSpeedMetersPerSecond = -1; return value; }(),
             [&] { auto value = draft; value.cloudBaseAboveGroundMeters = -1; return value; }(),
             [&] {
                 auto value = draft;
                 value.cloudBaseAboveGroundMeters = std::numeric_limits<float>::max();
                 return value;
             }(),
             [&] { auto value = draft; value.fogTopAboveGroundMeters = -1; return value; }(),
             [&] { auto value = draft; value.atmosphericPressureHectopascals = 0; return value; }(),
             [&] { auto value = draft; value.brightness = std::numeric_limits<double>::quiet_NaN(); return value; }(),
         }) {
        bool rejected = false;
        try {
            customVariables.setWeather(openbus::customWeatherFromDraft(invalid));
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        check(rejected &&
                  customVariables.weather().temperatureCelsius == applied.temperatureCelsius &&
                  customVariables.weather().precipitationType == applied.precipitationType,
              "invalid custom weather was accepted or changed active weather");
    }
    auto maximumCloud = draft;
    maximumCloud.cloudBaseAboveGroundMeters = openbus::MAX_CLOUD_BASE_ABOVE_GROUND_METERS;
    check(openbus::customWeatherFromDraft(maximumCloud).cloudBaseAboveGroundMeters ==
              openbus::MAX_CLOUD_BASE_ABOVE_GROUND_METERS,
          "maximum supported cloud height was rejected");
    maximumCloud.cloudBaseAboveGroundMeters = openbus::MAX_CLOUD_BASE_ABOVE_GROUND_METERS + 1.0;
    bool rejectedCloud = false;
    try {
        openbus::validateWeatherState(maximumCloud);
    } catch (const std::invalid_argument&) {
        rejectedCloud = true;
    }
    check(rejectedCloud, "shared weather validation accepted unsupported cloud height");

    openbus::WeatherSelection failure([](const std::string&) -> openbus::WeatherState {
        throw std::runtime_error("network unavailable");
    });
    failure.requestMetar("EGLL");
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (failure.pending() && std::chrono::steady_clock::now() < deadline) {
        check(!failure.poll(), "failed request returned weather");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(!failure.pending() && failure.failed() &&
              failure.status().find("network unavailable") != std::string::npos &&
              current.weather().precipitationType == 0,
          "METAR error was not surfaced");
    openbus::WeatherSelection invalidResult([](const std::string&) {
        openbus::WeatherState weather;
        weather.precipitationRate = -1;
        return weather;
    });
    invalidResult.requestMetar("EGLL");
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (invalidResult.pending() && std::chrono::steady_clock::now() < deadline) {
        check(!invalidResult.poll(), "invalid METAR result returned weather");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(!invalidResult.pending() && invalidResult.failed(),
          "invalid METAR weather was not reported");

    using Clock = openbus::WeatherSelection::Clock;
    auto simulatedNow = Clock::time_point{};
    std::atomic<int> fetches{0};
    openbus::WeatherSelection refresh(
        [&fetches](const std::string& station) {
            const int count = ++fetches;
            if (count == 2)
                throw std::runtime_error("temporary outage");
            if (station != "KJFK")
                throw std::runtime_error("wrong refresh station");
            openbus::WeatherState weather;
            weather.temperatureCelsius = static_cast<double>(count);
            return weather;
        },
        [&simulatedNow] { return simulatedNow; });
    const auto waitFor = [&](openbus::WeatherSelection& selection) {
        std::optional<openbus::WeatherState> value;
        const auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (selection.pending() && std::chrono::steady_clock::now() < limit) {
            if (auto result = selection.poll())
                value = result;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return value;
    };
    refresh.requestMetar("kjfk");
    auto first = waitFor(refresh);
    check(first && first->temperatureCelsius == 1.0 && refresh.activeStation() == "KJFK",
          "initial worldwide METAR request did not activate refresh");
    bool invalidStationRejected = false;
    try {
        refresh.requestMetar("ABCDE");
    } catch (const std::invalid_argument&) {
        invalidStationRejected = true;
    }
    check(invalidStationRejected && refresh.activeStation() == "KJFK" && fetches == 1,
          "invalid ICAO input changed the active station or started a request");
    simulatedNow += std::chrono::minutes(10) - std::chrono::milliseconds(1);
    check(!refresh.refreshIfDue() && fetches == 1,
          "METAR refreshed before ten minutes elapsed");
    simulatedNow += std::chrono::milliseconds(1);
    check(refresh.refreshIfDue() && refresh.pending(),
          "METAR did not refresh after exactly ten minutes");
    check(!refresh.refreshIfDue(), "overlapping refresh request was started");
    check(!waitFor(refresh) && refresh.failed() && refresh.activeStation() == "KJFK" &&
              refresh.status().find("temporary outage") != std::string::npos,
          "failed refresh was not reported or did not remain active");
    simulatedNow += std::chrono::minutes(10) - std::chrono::milliseconds(1);
    check(!refresh.refreshIfDue(), "failed refresh retried before ten minutes elapsed");
    simulatedNow += std::chrono::milliseconds(1);
    check(refresh.refreshIfDue(), "failed METAR refresh was not retried");
    auto retried = waitFor(refresh);
    check(retried && retried->temperatureCelsius == 3.0 && !refresh.failed() && fetches == 3,
          "METAR retry did not recover");
    refresh.discardPending();
    simulatedNow += std::chrono::minutes(30);
    check(!refresh.activeStation() && !refresh.refreshIfDue() && fetches == 3,
          "discarding METAR did not stop automatic refresh");

    openbus::WeatherState initialWeather;
    initialWeather.brightness = 0.4;
    initialWeather.cloudType = openbus::WeatherCloudType::Overcast1;
    initialWeather.fogTopAboveGroundMeters = 55.0;
    openbus::WeatherSelection startup(
        [](const std::string&) { return openbus::WeatherState{}; },
        [&simulatedNow] { return simulatedNow; });
    startup.enableMetarRefresh("egll", initialWeather);
    check(startup.activeStation() == "EGLL" && !startup.refreshIfDue(),
          "startup METAR did not schedule its first refresh");
    simulatedNow += std::chrono::minutes(10);
    check(startup.refreshIfDue(), "startup METAR did not refresh after ten minutes");
    const auto refreshedStartup = waitFor(startup);
    check(refreshedStartup && refreshedStartup->brightness == 0.4 &&
              refreshedStartup->cloudType == openbus::WeatherCloudType::Overcast1 &&
              refreshedStartup->fogTopAboveGroundMeters == 55.0,
          "startup METAR refresh lost manual lighting and fog overrides");
    startup.requestMetar("KJFK");
    const auto changedStation = waitFor(startup);
    check(changedStation && !changedStation->brightness && !changedStation->cloudType &&
              !changedStation->fogTopAboveGroundMeters && startup.activeStation() == "KJFK",
          "changing METAR station retained startup-only overrides");

    std::promise<void> releaseSlow;
    const auto slowReady = releaseSlow.get_future().share();
    std::atomic<int> slowFetches{0};
    openbus::WeatherSelection slow(
        [slowReady, &slowFetches](const std::string&) {
            ++slowFetches;
            slowReady.wait();
            return openbus::WeatherState{};
        },
        [&simulatedNow] { return simulatedNow; });
    slow.requestMetar("EGLL");
    simulatedNow += std::chrono::minutes(20);
    check(!slow.refreshIfDue(), "slow METAR request allowed an overlapping refresh");
    releaseSlow.set_value();
    check(waitFor(slow).has_value(), "slow METAR request did not complete");
    check(slow.refreshIfDue(), "overdue refresh did not start after slow request completed");
    check(waitFor(slow).has_value() && slowFetches == 2,
          "overdue refresh did not complete exactly once");
    return passed ? 0 : 1;
}
