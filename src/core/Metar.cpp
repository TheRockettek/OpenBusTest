#include "core/Metar.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cctype>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace openbus {
namespace {

int integer(std::string_view value) {
    int result = 0;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
        throw std::invalid_argument("Invalid METAR number: " + std::string(value));
    }
    return result;
}

double direction(std::string_view value) {
    const int degrees = integer(value);
    if (degrees > 360) {
        throw std::invalid_argument("Invalid METAR wind direction");
    }
    return degrees == 360 ? 0.0 : static_cast<double>(degrees);
}

double temperature(std::string_view value) {
    return value.front() == 'M' ? -static_cast<double>(integer(value.substr(1)))
                                : static_cast<double>(integer(value));
}

template <typename T> void setOnce(std::optional<T>& target, T value, const char* field) {
    if (target) {
        throw std::invalid_argument(std::string("Duplicate METAR ") + field);
    }
    target = std::move(value);
}

} // namespace

MetarObservation parseMetar(std::string_view report) {
    if (report.empty() || report.size() > 8192) {
        throw std::invalid_argument("METAR report is empty or too long");
    }
    std::istringstream input{std::string(report)};
    std::vector<std::string> tokens;
    for (std::string token; input >> token;) {
        std::transform(token.begin(), token.end(), token.begin(), [](unsigned char character) {
            return static_cast<char>(std::toupper(character));
        });
        tokens.push_back(std::move(token));
    }
    if (!tokens.empty() && !tokens.back().empty() && tokens.back().back() == '=') {
        tokens.back().pop_back();
        if (tokens.back().empty()) {
            tokens.pop_back();
        }
    }
    static const std::regex stationPattern("^[A-Z][A-Z0-9]{3}$");
    static const std::regex timePattern("^([0-9]{2})([0-9]{2})([0-9]{2})Z$");
    std::size_t index = 0;
    MetarObservation result{};
    if (index < tokens.size() && (tokens[index] == "METAR" || tokens[index] == "SPECI")) {
        result.specialReport = tokens[index] == "SPECI";
        ++index;
    }
    if (index < tokens.size() && tokens[index] == "COR") {
        result.corrected = true;
        ++index;
    }
    if (index >= tokens.size() || !std::regex_match(tokens[index], stationPattern)) {
        throw std::invalid_argument("METAR station identifier is missing or invalid");
    }
    result.station = tokens[index++];
    std::smatch match;
    if (index >= tokens.size() || !std::regex_match(tokens[index++], match, timePattern)) {
        throw std::invalid_argument("METAR observation time is missing or invalid");
    }
    result.day = integer(match[1].str());
    result.hourUtc = integer(match[2].str());
    result.minuteUtc = integer(match[3].str());
    if (result.day < 1 || result.day > 31 || result.hourUtc > 23 || result.minuteUtc > 59) {
        throw std::invalid_argument("METAR observation time is outside its valid range");
    }

    static const std::regex windPattern(
        "^(VRB|///|[0-9]{3})(//|[0-9]{2,3})(G(//|[0-9]{2,3}))?(KT|MPS|KMH)$");
    static const std::regex windRangePattern("^([0-9]{3})V([0-9]{3})$");
    static const std::regex visibilityMilesPattern("^([MP]?)([0-9]+)(/([0-9]+))?SM$");
    static const std::regex wholeNumberPattern("^[0-9]+$");
    static const std::regex visibilityMetersPattern("^[0-9]{4}$");
    static const std::regex cloudPattern("^(FEW|SCT|BKN|OVC|VV)([0-9]{3}|///)(CB|TCU)?$");
    static const std::regex temperaturePattern("^(M?[0-9]{2}|//)/(M?[0-9]{2}|//)?$");
    static const std::regex pressurePattern("^(Q|A)([0-9]{4})$");
    static const std::regex weatherPattern(
        "^[+-]?(VC)?(MI|PR|BC|DR|BL|SH|TS|FZ)?"
        "(DZ|RA|SN|SG|IC|PL|GR|GS|UP|BR|FG|FU|VA|DU|SA|HZ|PY|PO|SQ|FC|SS|DS){1,3}$");
    bool windSeen = false;
    bool temperatureSeen = false;
    bool trailing = false;
    while (index < tokens.size()) {
        const std::string& token = tokens[index++];
        if (trailing || token == "RMK" || token == "NOSIG" || token == "TEMPO" ||
            token == "BECMG") {
            trailing = true;
            result.trailingGroups.push_back(token);
        } else if (token == "NIL") {
            throw std::invalid_argument("METAR observation is unavailable (NIL)");
        } else if (token == "AUTO") {
            result.automated = true;
        } else if (token == "COR") {
            result.corrected = true;
        } else if (std::regex_match(token, match, windPattern)) {
            if (windSeen) {
                throw std::invalid_argument("Duplicate METAR wind");
            }
            windSeen = true;
            const std::string directionCode = match[1].str();
            const std::string speedCode = match[2].str();
            const std::string units = match[5].str();
            const double factor = units == "KT"    ? 0.5144444444444444
                                  : units == "KMH" ? 1.0 / 3.6
                                                   : 1.0;
            if (directionCode == "VRB") {
                result.variableWindDirection = true;
            } else if (directionCode != "///") {
                const double degrees = direction(directionCode);
                if (directionCode != "000") {
                    result.windDirectionDegrees = degrees;
                } else if (speedCode != "//" && integer(speedCode) != 0) {
                    throw std::invalid_argument("METAR calm wind has a nonzero speed");
                }
            }
            if (speedCode != "//") {
                result.windSpeedMetersPerSecond = integer(speedCode) * factor;
            }
            if (match[4].matched && match[4].str() != "//") {
                result.gustSpeedMetersPerSecond = integer(match[4].str()) * factor;
            }
        } else if (std::regex_match(token, match, windRangePattern)) {
            setOnce(result.variableWindRange,
                    MetarWindRange{direction(match[1].str()), direction(match[2].str())},
                    "variable wind range");
        } else if (token == "CAVOK") {
            if (result.cavok || result.clearSkyCode || !result.cloudLayers.empty() ||
                !result.presentWeather.empty()) {
                throw std::invalid_argument("Contradictory METAR sky conditions");
            }
            setOnce(result.visibility, MetarVisibility{10000.0, MetarVisibilityBound::AtLeast},
                    "visibility");
            result.cavok = true;
        } else if (std::regex_match(token, match, visibilityMilesPattern) ||
                   (token.size() <= 2 && std::regex_match(token, wholeNumberPattern) &&
                    index < tokens.size() &&
                    std::regex_match(tokens[index], match, visibilityMilesPattern) &&
                    match[4].matched)) {
            const bool mixed = std::regex_match(token, wholeNumberPattern);
            const std::string bound = match[1].str();
            double miles = static_cast<double>(integer(match[2].str()));
            if (match[4].matched) {
                const int denominator = integer(match[4].str());
                if (denominator == 0) {
                    throw std::invalid_argument("Invalid METAR visibility fraction");
                }
                miles /= denominator;
            }
            if (mixed) {
                if (!match[4].matched || !bound.empty()) {
                    throw std::invalid_argument("Invalid METAR mixed visibility");
                }
                miles += integer(token);
                ++index;
            }
            const auto qualifier = bound == "M"   ? MetarVisibilityBound::LessThan
                                   : bound == "P" ? MetarVisibilityBound::GreaterThan
                                                  : MetarVisibilityBound::Reported;
            setOnce(result.visibility, MetarVisibility{miles * 1609.344, qualifier}, "visibility");
        } else if (std::regex_match(token, visibilityMetersPattern)) {
            const int meters = integer(token);
            const MetarVisibility visibility =
                meters == 9999 ? MetarVisibility{10000.0, MetarVisibilityBound::AtLeast}
                : meters == 0
                    ? MetarVisibility{50.0, MetarVisibilityBound::LessThan}
                    : MetarVisibility{static_cast<double>(meters), MetarVisibilityBound::Reported};
            setOnce(result.visibility, visibility, "visibility");
        } else if (token == "SKC" || token == "CLR" || token == "NSC" || token == "NCD") {
            if (result.cavok || result.clearSkyCode || !result.cloudLayers.empty()) {
                throw std::invalid_argument("Contradictory METAR sky conditions");
            }
            result.clearSkyCode = token;
        } else if (std::regex_match(token, match, cloudPattern)) {
            if (result.cavok || result.clearSkyCode) {
                throw std::invalid_argument("Contradictory METAR sky conditions");
            }
            const std::string coverage = match[1].str();
            const auto kind = coverage == "FEW"   ? MetarCloudCoverage::Few
                              : coverage == "SCT" ? MetarCloudCoverage::Scattered
                              : coverage == "BKN" ? MetarCloudCoverage::Broken
                              : coverage == "OVC" ? MetarCloudCoverage::Overcast
                                                  : MetarCloudCoverage::VerticalVisibility;
            const std::string height = match[2].str();
            result.cloudLayers.push_back(
                {kind,
                 height == "///" ? std::nullopt : std::optional<double>(integer(height) * 30.48),
                 match[3].str()});
        } else if (std::regex_match(token, match, temperaturePattern)) {
            if (temperatureSeen) {
                throw std::invalid_argument("Duplicate METAR temperature");
            }
            temperatureSeen = true;
            if (match[1].str() != "//") {
                result.temperatureCelsius = temperature(match[1].str());
            }
            if (match[2].matched && match[2].str() != "//") {
                result.dewPointCelsius = temperature(match[2].str());
            }
        } else if (std::regex_match(token, match, pressurePattern)) {
            const double raw = integer(match[2].str());
            if (raw <= 0.0) {
                throw std::invalid_argument("Invalid METAR pressure");
            }
            const bool hectopascals = match[1].str() == "Q";
            setOnce(result.altimeter,
                    MetarAltimeterSetting{hectopascals ? raw : raw * 0.338638866667,
                                          hectopascals ? MetarAltimeterUnit::Hectopascals
                                                       : MetarAltimeterUnit::InchesOfMercury},
                    "altimeter");
        } else if (std::regex_match(token, weatherPattern)) {
            if (result.cavok) {
                throw std::invalid_argument("Contradictory METAR weather and CAVOK");
            }
            result.presentWeather.push_back(token);
        } else {
            result.unparsedGroups.push_back(token);
        }
    }
    return result;
}

double absoluteHumidityFromDewPoint(double temperatureCelsius, double dewPointCelsius) {
    if (!std::isfinite(temperatureCelsius) || !std::isfinite(dewPointCelsius) ||
        temperatureCelsius < -40.0 || temperatureCelsius > 50.0 || dewPointCelsius < -40.0 ||
        dewPointCelsius > 50.0) {
        throw std::invalid_argument(
            "Humidity approximation requires temperatures from -40 to 50 C");
    }
    const double vaporPressure =
        6.112 * std::exp(17.67 * dewPointCelsius / (dewPointCelsius + 243.5));
    return 216.7 * vaporPressure / (temperatureCelsius + 273.15);
}

double relativeHumidityFromAbsolute(double temperatureCelsius, double absoluteHumidity) {
    if (!std::isfinite(absoluteHumidity) || absoluteHumidity < 0.0) {
        throw std::invalid_argument("Absolute humidity must be finite and non-negative");
    }
    const double relative =
        100.0 *
        (absoluteHumidity / absoluteHumidityFromDewPoint(temperatureCelsius, temperatureCelsius));
    if (!std::isfinite(relative)) {
        throw std::invalid_argument("Absolute humidity is too large to convert");
    }
    return relative;
}

double absoluteHumidityFromRelative(double temperatureCelsius, double relativeHumidity) {
    if (!std::isfinite(relativeHumidity) || relativeHumidity < 0.0 || relativeHumidity > 100.0) {
        throw std::invalid_argument("Relative humidity must be between 0 and 100 percent");
    }
    return absoluteHumidityFromDewPoint(temperatureCelsius, temperatureCelsius) *
           (relativeHumidity / 100.0);
}

std::optional<double> dewPointFromAbsoluteHumidity(double temperatureCelsius,
                                                   double absoluteHumidity) {
    const double relative = relativeHumidityFromAbsolute(temperatureCelsius, absoluteHumidity);
    if (relative <= 0.0 || relative > 100.0) {
        return std::nullopt;
    }
    const double pressure = absoluteHumidity * (temperatureCelsius + 273.15) / 216.7;
    const double logarithm = std::log(pressure / 6.112);
    const double dewPoint = 243.5 * logarithm / (17.67 - logarithm);
    if (!std::isfinite(dewPoint) || dewPoint < -40.0 || dewPoint > 50.0) {
        return std::nullopt;
    }
    return dewPoint;
}

} // namespace openbus
