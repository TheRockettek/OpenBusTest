#include "core/MetarNoaa.h"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace openbus {
namespace {

int number(std::string_view value) {
    int result = 0;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
        throw std::invalid_argument("Invalid NOAA METAR timestamp");
    }
    return result;
}

bool safeRunwayVisualRange(const std::string& value) {
    static const std::regex pattern("^R[0-9]{2}[LCR]?/[MP]?[0-9]{4}(V[MP]?[0-9]{4})?(FT)?[UDN]?$");
    return std::regex_match(value, pattern);
}

} // namespace

std::string noaaMetarStation(std::string_view station) {
    std::string result(station);
    for (char& character : result) {
        if (character >= 'a' && character <= 'z') {
            character = static_cast<char>(character - 'a' + 'A');
        }
    }
    if (result.size() != 4 || result[0] < 'A' || result[0] > 'Z' ||
        !std::all_of(result.begin() + 1, result.end(), [](char character) {
            return (character >= 'A' && character <= 'Z') || (character >= '0' && character <= '9');
        })) {
        throw std::invalid_argument("METAR station must be a four-character ICAO identifier");
    }
    return result;
}

NoaaMetarReport parseNoaaMetar(std::string_view station, std::string_view body,
                               std::chrono::system_clock::time_point now) {
    using namespace std::chrono;
    const std::string expectedStation = noaaMetarStation(station);
    if (body.empty() || body.size() > 8192) {
        throw std::invalid_argument("NOAA METAR response is empty or exceeds 8192 bytes");
    }
    std::istringstream stream{std::string(body)};
    std::string date, report, extra;
    if (!std::getline(stream, date) || !std::getline(stream, report)) {
        throw std::invalid_argument("NOAA METAR response must have a timestamp and report");
    }
    if (!date.empty() && date.back() == '\r') {
        date.pop_back();
    }
    if (!report.empty() && report.back() == '\r') {
        report.pop_back();
    }
    while (std::getline(stream, extra)) {
        if (extra.find_first_not_of(" \t\r") != std::string::npos) {
            throw std::invalid_argument("NOAA METAR response contains unexpected extra lines");
        }
    }
    static const std::regex datePattern("^([0-9]{4})/([0-9]{2})/([0-9]{2}) ([0-9]{2}):([0-9]{2})$");
    std::smatch match;
    if (!std::regex_match(date, match, datePattern)) {
        throw std::invalid_argument("NOAA METAR timestamp is invalid");
    }
    const year_month_day calendar{year{number(match[1].str())},
                                  month{static_cast<unsigned>(number(match[2].str()))},
                                  day{static_cast<unsigned>(number(match[3].str()))}};
    const int hour = number(match[4].str());
    const int minute = number(match[5].str());
    if (!calendar.ok() || hour > 23 || minute > 59) {
        throw std::invalid_argument("NOAA METAR timestamp is invalid");
    }
    const sys_time<minutes> observedAt = sys_days{calendar} + hours{hour} + minutes{minute};
    if (observedAt > now + minutes{5}) {
        throw std::invalid_argument("NOAA METAR report is more than five minutes in the future");
    }
    if (observedAt < now - hours{2}) {
        throw std::invalid_argument("NOAA METAR report is older than two hours");
    }
    MetarObservation observation = parseMetar(report);
    if (observation.station != expectedStation) {
        throw std::invalid_argument("NOAA METAR station does not match the requested station");
    }
    if (observation.day != static_cast<int>(static_cast<unsigned>(calendar.day())) ||
        observation.hourUtc != hour || observation.minuteUtc != minute) {
        throw std::invalid_argument("NOAA METAR observation time does not match its timestamp");
    }
    return {std::move(observation), observedAt};
}

WeatherState weatherFromMetar(const MetarObservation& observation) {
    if (!observation.temperatureCelsius || !observation.dewPointCelsius ||
        !observation.visibility || !observation.windSpeedMetersPerSecond ||
        (!observation.cavok && !observation.clearSkyCode && observation.cloudLayers.empty())) {
        throw std::invalid_argument(
            "METAR needs temperature, dew point, visibility, wind speed and sky condition");
    }
    for (const std::string& group : observation.unparsedGroups) {
        if (!safeRunwayVisualRange(group)) {
            throw std::invalid_argument("Unsupported METAR observation group: " + group);
        }
    }
    WeatherState weather;
    weather.temperatureCelsius = *observation.temperatureCelsius;
    weather.absoluteHumidityGramsPerCubicMeter =
        absoluteHumidityFromDewPoint(*observation.temperatureCelsius, *observation.dewPointCelsius);
    weather.visibilityMeters = observation.visibility->meters;
    weather.windDirectionDegrees = observation.windDirectionDegrees;
    weather.windSpeedMetersPerSecond = observation.windSpeedMetersPerSecond;
    for (const MetarCloudLayer& layer : observation.cloudLayers) {
        if (layer.coverage != MetarCloudCoverage::VerticalVisibility &&
            layer.heightAboveAerodromeMeters &&
            (!weather.cloudBaseAboveGroundMeters ||
             *layer.heightAboveAerodromeMeters < *weather.cloudBaseAboveGroundMeters)) {
            weather.cloudBaseAboveGroundMeters = layer.heightAboveAerodromeMeters;
        }
    }
    for (const std::string& code : observation.presentWeather) {
        std::string_view group(code);
        if (group.empty()) {
            throw std::invalid_argument("Empty METAR weather group");
        }
        double rate = 0.5;
        if (group.front() == '-' || group.front() == '+') {
            rate = group.front() == '-' ? 0.25 : 0.75;
            group.remove_prefix(1);
        }
        if (group.starts_with("VC")) {
            throw std::invalid_argument("METAR vicinity weather cannot be applied locally: " +
                                        code);
        }
        std::string_view selectedDescriptor;
        for (std::string_view descriptor : {"MI", "PR", "BC", "DR", "BL", "SH", "TS", "FZ"}) {
            if (group.starts_with(descriptor)) {
                selectedDescriptor = descriptor;
                group.remove_prefix(2);
                break;
            }
            if (group.empty() || group.size() % 2 != 0) {
                throw std::invalid_argument("Invalid METAR weather group: " + code);
            }
        }
        for (std::size_t offset = 0; offset < group.size(); offset += 2) {
            const std::string_view phenomenon = group.substr(offset, 2);
            int type = 0;
            if (phenomenon == "RA" || phenomenon == "DZ") {
                type = 1;
            } else if (phenomenon == "SN" || phenomenon == "SG") {
                if (selectedDescriptor == "BL" || selectedDescriptor == "DR") {
                    throw std::invalid_argument("Blowing or drifting snow is not snowfall: " +
                                                code);
                }
                type = 2;
            } else if (phenomenon == "IC" || phenomenon == "PL" || phenomenon == "GR" ||
                       phenomenon == "GS" || phenomenon == "UP") {
                throw std::invalid_argument("Unsupported METAR precipitation: " + code);
            } else if (phenomenon != "BR" && phenomenon != "FG" && phenomenon != "FU" &&
                       phenomenon != "VA" && phenomenon != "DU" && phenomenon != "SA" &&
                       phenomenon != "HZ" && phenomenon != "PY" && phenomenon != "PO" &&
                       phenomenon != "SQ" && phenomenon != "FC" && phenomenon != "SS" &&
                       phenomenon != "DS") {
                throw std::invalid_argument("Unsupported METAR weather group: " + code);
            }
            if (type != 0) {
                if (weather.precipitationType != 0 && weather.precipitationType != type) {
                    throw std::invalid_argument("Mixed METAR rain and snow cannot be represented");
                }
                weather.precipitationType = type;
                weather.precipitationRate = std::max(weather.precipitationRate, rate);
            }
        }
    }
    validateWeatherState(weather);
    return weather;
}

} // namespace openbus
