#include "core/MetarNoaa.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

bool check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
    }
    return condition;
}

bool rejects(const auto& function) {
    try {
        function();
        return false;
    } catch (const std::invalid_argument&) {
        return true;
    }
}

} // namespace

int main(int argc, char** argv) {
    using namespace openbus;
    using namespace std::chrono;
    if (argc == 3 && std::string_view(argv[1]) == "--live") {
        try {
            const auto body = fetchNoaaMetar(argv[2]);
            const auto report = parseNoaaMetar(argv[2], body, system_clock::now());
            const auto weather = weatherFromMetar(report.observation);
            std::cout << "Fetched and applied NOAA report for " << report.observation.station
                      << "; temperature=" << weather.temperatureCelsius
                      << " C, precipitation type=" << weather.precipitationType << '\n';
            return 0;
        } catch (const std::exception& error) {
            std::cerr << "Live NOAA METAR probe failed: " << error.what() << '\n';
            return 1;
        }
    }
    if (argc == 3 && std::string_view(argv[1]) == "--expect-http-error") {
        try {
            static_cast<void>(fetchNoaaMetar(argv[2]));
            std::cerr << "A nonexistent station unexpectedly returned a report\n";
            return 1;
        } catch (const std::runtime_error& error) {
            if (std::string_view(error.what()).find("returned HTTP ") != std::string_view::npos) {
                std::cout << "NOAA returned an expected non-success HTTP status\n";
                return 0;
            }
            std::cerr << "Unexpected NOAA request error: " << error.what() << '\n';
            return 1;
        }
    }
    if (argc == 3 && std::string_view(argv[1]) == "--expect-stale") {
        try {
            const auto body = fetchNoaaMetar(argv[2]);
            static_cast<void>(parseNoaaMetar(argv[2], body, system_clock::now()));
            std::cerr << "A stale station unexpectedly passed freshness validation\n";
            return 1;
        } catch (const std::invalid_argument& error) {
            if (std::string_view(error.what()).find("older than two hours") !=
                std::string_view::npos) {
                std::cout << "NOAA stale report was rejected\n";
                return 0;
            }
            std::cerr << "Unexpected NOAA validation error: " << error.what() << '\n';
            return 1;
        }
    }
    bool passed = true;
    const sys_time<minutes> now = sys_days{year{2026}/11/6} + hours{20} + minutes{5};
    const std::string body =
        "2026/11/06 19:51\r\n"
        "KJFK 061951Z 34008KT 10SM BKN060 OVC120 18/16 A2979 RMK AO2\r\n";
    const auto report = parseNoaaMetar("kjfk", body, now);
    const auto clear = weatherFromMetar(report.observation);
    passed &= check(report.observation.station == "KJFK" &&
                        report.observedAtUtc == sys_days{year{2026}/11/6} + hours{19} + minutes{51} &&
                        clear.precipitationType == 0 && clear.precipitationRate == 0.0 &&
                        clear.temperatureCelsius == 18.0 &&
                        clear.absoluteHumidityGramsPerCubicMeter > 13.0 &&
                        clear.absoluteHumidityGramsPerCubicMeter < 14.0 &&
                        clear.visibilityMeters && std::abs(*clear.visibilityMeters - 16093.44) < 0.01 &&
                        clear.windDirectionDegrees == 340.0 &&
                        clear.windSpeedMetersPerSecond &&
                        std::abs(*clear.windSpeedMetersPerSecond - 8.0 * 0.5144444444) < 0.001 &&
                        clear.cloudBaseAboveGroundMeters &&
                        std::abs(*clear.cloudBaseAboveGroundMeters - 1828.8) < 0.01 &&
                        !clear.cloudType && !clear.atmosphericPressureHectopascals &&
                        !clear.brightness,
                    "NOAA timestamp or clear-weather observations were applied incorrectly");
    const auto reportAt = [&](std::string_view time, std::string_view metar) {
        return parseNoaaMetar("KJFK", std::string(time) + "\n" + std::string(metar) + "\n", now);
    };
    passed &= check(rejects([&] { static_cast<void>(noaaMetarStation("../KJFK")); }) &&
                        rejects([&] { static_cast<void>(noaaMetarStation("1234")); }) &&
                        noaaMetarStation("kjfk") == "KJFK",
                    "station normalization or path validation failed");
    passed &= check(rejects([&] { static_cast<void>(parseNoaaMetar("KLAX", body, now)); }) &&
                        rejects([&] { static_cast<void>(parseNoaaMetar("KJFK", "", now)); }) &&
                        rejects([&] { static_cast<void>(parseNoaaMetar("KJFK", body + "extra\n", now)); }) &&
                        rejects([&] { static_cast<void>(parseNoaaMetar("KJFK", std::string(8193, 'X'), now)); }) &&
                        rejects([&] { static_cast<void>(reportAt("2026/02/30 19:51",
                                                                "KJFK 301951Z 34008KT 10SM SKC 18/16")); }) &&
                        rejects([&] { static_cast<void>(reportAt("2026/11/06 19:50",
                                                                "KJFK 061951Z 34008KT 10SM SKC 18/16")); }),
                    "invalid NOAA body, station, calendar or timestamp was accepted");
    const auto makeTimed = [&](std::string_view time, std::string_view observationTime) {
        return reportAt(time, "KJFK " + std::string(observationTime) +
                                  " 00000KT 9999 SKC 18/16");
    };
    passed &= check(!rejects([&] { static_cast<void>(makeTimed("2026/11/06 18:05", "061805Z")); }) &&
                        rejects([&] { static_cast<void>(makeTimed("2026/11/06 18:04", "061804Z")); }) &&
                        !rejects([&] { static_cast<void>(makeTimed("2026/11/06 20:10", "062010Z")); }) &&
                        rejects([&] { static_cast<void>(makeTimed("2026/11/06 20:11", "062011Z")); }),
                    "NOAA report-age and clock-skew boundaries were not enforced");
    const auto derive = [&](const char* reportText) {
        return weatherFromMetar(parseMetar(reportText));
    };
    const auto light = derive("KJFK 061951Z 00000KT 1SM -RA BKN010 05/03");
    const auto rain = derive("KJFK 061951Z 00000KT 1SM SHRA BKN010 05/03");
    const auto snow = derive("KJFK 061951Z 00000KT M1/4SM +SN OVC010 M05/M08");
    const auto freezing = derive("KJFK 061951Z 00000KT 1SM FZRA OVC010 M02/M03");
    passed &= check(light.precipitationType == 1 && light.precipitationRate == 0.25 &&
                        rain.precipitationType == 1 && rain.precipitationRate == 0.5 &&
                        snow.precipitationType == 2 && snow.precipitationRate == 0.75 &&
                        snow.visibilityMeters && std::abs(*snow.visibilityMeters - 402.336) < 0.001 &&
                        freezing.precipitationType == 1 && freezing.precipitationRate == 0.5,
                    "precipitation or bounded visibility mapping failed");
    const auto rvr = derive("KJFK 061951Z 00000KT 1SM R04/2200FT BR OVC010 05/03");
    const auto cavok = derive("KJFK 061951Z VRB02KT CAVOK 05/03");
    const auto trend = derive("KJFK 061951Z 00000KT P6SM SKC 05/03 TEMPO +SN");
    passed &= check(rvr.precipitationType == 0 && cavok.precipitationType == 0 &&
                        !cavok.windDirectionDegrees && cavok.visibilityMeters == 10000.0 &&
                        trend.precipitationType == 0 && trend.visibilityMeters &&
                        std::abs(*trend.visibilityMeters - 6.0 * 1609.344) < 0.001,
                    "non-precipitation weather, CAVOK, trend or visibility bound was misapplied");
    auto invalidObservation = report.observation;
    invalidObservation.presentWeather.push_back("");
    passed &= check(rejects([&] { static_cast<void>(weatherFromMetar(invalidObservation)); }),
                    "empty weather group was accepted");
    for (const char* invalid : {
             "KJFK 061951Z 00000KT 1SM RASN BKN010 05/03",
             "KJFK 061951Z 00000KT 1SM UP BKN010 05/03",
             "KJFK 061951Z 00000KT 1SM VCSHRA BKN010 05/03",
             "KJFK 061951Z 00000KT 1SM BLSN BKN010 M05/M08",
             "KJFK 061951Z 00000KT 1SM GR BKN010 05/03",
             "KJFK 061951Z 00000KT 1SM UNKNOWN BKN010 05/03",
             "KJFK 061951Z 00000KT 1SM BKN010 05///",
             "KJFK 061951Z 00000KT BKN010 05/03",
             "KJFK 061951Z 1SM BKN010 05/03",
             "KJFK 061951Z 00000KT 1SM 05/03",
             "KJFK 061951Z 00000KT 1SM BKN010 M41/M42"}) {
        if (!rejects([&] { static_cast<void>(derive(invalid)); })) {
            std::cerr << "unsupported or incomplete METAR was accepted: " << invalid << '\n';
            passed = false;
        }
    }
    return passed ? 0 : 1;
}
