#include "core/Metar.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

bool check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
    }
    return condition;
}

bool near(double actual, double expected, double tolerance = 0.001) {
    return std::abs(actual - expected) <= tolerance;
}

bool rejects(const char* report) {
    try {
        static_cast<void>(openbus::parseMetar(report));
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

} // namespace

int main() {
    using namespace openbus;
    bool passed = true;
    const auto us = parseMetar(
        "METAR KJFK 121651Z 18012G21KT 1 1/2SM -RA BR BKN012 OVC030 "
        "18/16 A2992 RMK AO2 SLP134=");
    passed &= check(us.station == "KJFK" && us.day == 12 && us.hourUtc == 16 &&
                        us.minuteUtc == 51 && us.windDirectionDegrees == 180.0 &&
                        us.windSpeedMetersPerSecond &&
                        near(*us.windSpeedMetersPerSecond, 12.0 * 0.5144444444444444) &&
                        us.gustSpeedMetersPerSecond &&
                        near(*us.gustSpeedMetersPerSecond, 21.0 * 0.5144444444444444),
                    "US report header or wind was parsed incorrectly");
    passed &= check(us.visibility && near(us.visibility->meters, 1.5 * 1609.344) &&
                        us.visibility->bound == MetarVisibilityBound::Reported &&
                        us.presentWeather == std::vector<std::string>{"-RA", "BR"} &&
                        us.cloudLayers.size() == 2 &&
                        us.cloudLayers[0].coverage == MetarCloudCoverage::Broken &&
                        us.cloudLayers[0].heightAboveAerodromeMeters &&
                        near(*us.cloudLayers[0].heightAboveAerodromeMeters, 365.76) &&
                        us.cloudLayers[1].coverage == MetarCloudCoverage::Overcast &&
                        us.temperatureCelsius == 18.0 && us.dewPointCelsius == 16.0 &&
                        us.altimeter && us.altimeter->reportedUnit == MetarAltimeterUnit::InchesOfMercury &&
                        near(us.altimeter->hectopascals, 2992.0 * 0.338638866667) &&
                        us.unparsedGroups.empty() &&
                        us.trailingGroups == std::vector<std::string>{"RMK", "AO2", "SLP134"},
                    "US report body or remarks were parsed incorrectly");

    const auto international = parseMetar(
        "SPECI EGLL 312350Z COR 36005KT 9999 FEW020TCU BKN040 05/M01 Q1017 NOSIG");
    passed &= check(international.specialReport && international.corrected &&
                        international.windDirectionDegrees == 0.0 && international.visibility &&
                        international.visibility->meters == 10000.0 &&
                        international.visibility->bound == MetarVisibilityBound::AtLeast &&
                        international.cloudLayers.size() == 2 &&
                        international.cloudLayers[0].cloudSuffix == "TCU" &&
                        international.dewPointCelsius == -1.0 &&
                        international.altimeter &&
                        international.altimeter->reportedUnit == MetarAltimeterUnit::Hectopascals &&
                        international.altimeter->hectopascals == 1017.0 &&
                        international.trailingGroups == std::vector<std::string>{"NOSIG"},
                    "international report was parsed incorrectly");

    const auto variable = parseMetar(
        "METAR TEST 010005Z AUTO VRB03MPS CAVOK M05/M08 Q1001 TEMPO 0500 SN");
    passed &= check(variable.automated && variable.variableWindDirection &&
                        !variable.windDirectionDegrees &&
                        variable.windSpeedMetersPerSecond == 3.0 && variable.cavok &&
                        variable.visibility &&
                        variable.visibility->bound == MetarVisibilityBound::AtLeast &&
                        variable.cloudLayers.empty() && variable.presentWeather.empty() &&
                        variable.temperatureCelsius == -5.0 && variable.dewPointCelsius == -8.0 &&
                        variable.trailingGroups == std::vector<std::string>{"TEMPO", "0500", "SN"},
                    "variable wind, CAVOK, negative temperature or trend was misread");
    const auto range = parseMetar("TEST 020030Z 21036G54KMH 180V240 M1/4SM VV/// 00/// Q0999");
    passed &= check(range.windSpeedMetersPerSecond && near(*range.windSpeedMetersPerSecond, 10.0) &&
                        range.gustSpeedMetersPerSecond && near(*range.gustSpeedMetersPerSecond, 15.0) &&
                        range.variableWindRange && range.variableWindRange->fromDegrees == 180.0 &&
                        range.variableWindRange->toDegrees == 240.0 && range.visibility &&
                        near(range.visibility->meters, 1609.344 / 4.0) &&
                        range.visibility->bound == MetarVisibilityBound::LessThan &&
                        range.cloudLayers.size() == 1 &&
                        range.cloudLayers[0].coverage == MetarCloudCoverage::VerticalVisibility &&
                        !range.cloudLayers[0].heightAboveAerodromeMeters &&
                        range.temperatureCelsius == 0.0 && !range.dewPointCelsius,
                    "wind range, metric wind, bounded visibility or missing fields failed");
    const auto missing = parseMetar("TEST 030930Z /////KT P6SM ///// Q//// R04/2200FT");
    passed &= check(!missing.windDirectionDegrees && !missing.windSpeedMetersPerSecond &&
                        !missing.temperatureCelsius && !missing.dewPointCelsius &&
                        !missing.altimeter && missing.visibility &&
                        missing.visibility->bound == MetarVisibilityBound::GreaterThan &&
                        missing.unparsedGroups == std::vector<std::string>{"Q////", "R04/2200FT"},
                    "missing or unparsed fields were silently inferred");
    const auto calm = parseMetar("TEST 040000Z 00000KT 0000 SKC 10/05 Q1013");
    passed &= check(!calm.windDirectionDegrees && calm.windSpeedMetersPerSecond == 0.0 &&
                        calm.visibility && calm.visibility->meters == 50.0 &&
                        calm.visibility->bound == MetarVisibilityBound::LessThan &&
                        calm.clearSkyCode == "SKC",
                    "calm wind, minimum visibility or clear sky was misread");
    const auto weather = parseMetar("TEST 040000Z 00000KT 8000 -FZRASN TSRA SCT020CB");
    passed &= check(weather.presentWeather == std::vector<std::string>{"-FZRASN", "TSRA"} &&
                        weather.cloudLayers.size() == 1 &&
                        weather.cloudLayers[0].cloudSuffix == "CB",
                    "weather groups or convective cloud suffix were not retained");

    for (const char* report : {"", "TEST 000000Z", "TEST 322400Z", "TEST 012460Z",
                               "TEST NIL", "TEST 010000Z NIL",
                               "TEST 010000Z 36110KT",
                               "TEST 010000Z 00010KT",
                               "TEST 010000Z 1/0SM",
                               "TEST 010000Z 9999 1/2SM",
                               "TEST 010000Z 9999 CAVOK",
                               "TEST 010000Z CAVOK BKN020",
                               "TEST 010000Z -RA CAVOK",
                               "TEST 010000Z CAVOK -RA",
                               "TEST 010000Z SKC OVC010",
                               "TEST 010000Z 10/05 11/06",
                               "TEST 010000Z Q0000",
                               "TEST 010000Z Q1013 A2992"}) {
        if (!rejects(report)) {
            std::cerr << "invalid or contradictory METAR was accepted: " << report << '\n';
            passed = false;
        }
    }
    passed &= check(near(absoluteHumidityFromDewPoint(20.0, 10.0), 9.07, 0.04) &&
                        near(absoluteHumidityFromDewPoint(0.0, 0.0), 4.85, 0.04),
                    "absolute humidity approximation was outside expected bounds");
    const double omsiExampleHumidity = absoluteHumidityFromDewPoint(15.0, 5.0);
    const auto omsiExampleDewPoint = dewPointFromAbsoluteHumidity(15.0, omsiExampleHumidity);
    passed &= check(near(relativeHumidityFromAbsolute(15.0, omsiExampleHumidity), 51.1, 0.5) &&
                        omsiExampleDewPoint && near(*omsiExampleDewPoint, 5.0, 1e-8) &&
                        near(absoluteHumidityFromRelative(15.0, 51.0), omsiExampleHumidity, 0.1) &&
                        absoluteHumidityFromRelative(15.0, 0.0) == 0.0 &&
                        near(relativeHumidityFromAbsolute(15.0,
                                                         absoluteHumidityFromRelative(15.0, 100.0)),
                             100.0, 1e-8) &&
                        !dewPointFromAbsoluteHumidity(15.0, 0.0) &&
                        !dewPointFromAbsoluteHumidity(15.0, 100.0),
                    "relative humidity or dew-point conversion was incorrect");
    for (double invalid : {-1.0, 101.0, std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            static_cast<void>(absoluteHumidityFromRelative(15.0, invalid));
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        passed &= check(rejected, "invalid relative humidity was accepted");
    }
    for (double temperature : {-40.0, 50.0}) {
        const double humidity = absoluteHumidityFromRelative(temperature, 100.0);
        const auto dewPoint = dewPointFromAbsoluteHumidity(temperature, humidity);
        passed &= check(near(relativeHumidityFromAbsolute(temperature, humidity), 100.0) &&
                            dewPoint && near(*dewPoint, temperature),
                        "humidity approximation failed at a temperature boundary");
    }
    for (const auto& pair : {std::pair{-41.0, 1.0}, std::pair{51.0, 1.0},
                             std::pair{15.0, -1.0},
                             std::pair{15.0, std::numeric_limits<double>::infinity()},
                             std::pair{15.0, std::numeric_limits<double>::max()}}) {
        bool rejected = false;
        try {
            static_cast<void>(relativeHumidityFromAbsolute(pair.first, pair.second));
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        passed &= check(rejected, "invalid absolute-to-relative humidity input was accepted");
    }
    for (const auto& pair : {std::pair{-41.0, 0.0}, std::pair{0.0, 51.0},
                             std::pair{std::numeric_limits<double>::quiet_NaN(), 0.0}}) {
        bool rejected = false;
        try {
            static_cast<void>(absoluteHumidityFromDewPoint(pair.first, pair.second));
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        passed &= check(rejected, "invalid humidity approximation input was accepted");
    }
    return passed ? 0 : 1;
}
