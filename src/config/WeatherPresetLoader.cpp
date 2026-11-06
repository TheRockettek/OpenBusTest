#include "config/WeatherPresetLoader.h"

#include "config/ConfigurationParser.h"
#include "core/Metar.h"
#include "core/Utf8Path.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace openbus::config {
namespace {

std::string readPresetText(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot open weather preset " + pathToUtf8(path));
    const std::string bytes((std::istreambuf_iterator<char>(input)), {});
    if (input.bad())
        throw std::runtime_error("Cannot read weather preset " + pathToUtf8(path));
    const auto byte = [&](std::size_t offset) { return static_cast<unsigned char>(bytes[offset]); };
    if (bytes.size() < 2 || byte(0) != 0xff || byte(1) != 0xfe) {
        if (bytes.size() >= 2 && byte(0) == 0xfe && byte(1) == 0xff)
            throw std::runtime_error("UTF-16BE weather preset is not supported: " +
                                     pathToUtf8(path));
        return bytes;
    }
    if ((bytes.size() - 2) % 2 != 0)
        throw std::runtime_error("Odd-length UTF-16 weather preset: " + pathToUtf8(path));
    std::string output;
    for (std::size_t offset = 2; offset < bytes.size(); offset += 2) {
        std::uint32_t code = byte(offset) | (static_cast<std::uint32_t>(byte(offset + 1)) << 8);
        if (code >= 0xd800 && code <= 0xdbff) {
            if (offset + 3 >= bytes.size())
                throw std::runtime_error("Unpaired UTF-16 surrogate in " + pathToUtf8(path));
            const std::uint32_t low =
                byte(offset + 2) | (static_cast<std::uint32_t>(byte(offset + 3)) << 8);
            if (low < 0xdc00 || low > 0xdfff)
                throw std::runtime_error("Unpaired UTF-16 surrogate in " + pathToUtf8(path));
            code = 0x10000 + ((code - 0xd800) << 10) + low - 0xdc00;
            offset += 2;
        } else if (code >= 0xdc00 && code <= 0xdfff) {
            throw std::runtime_error("Unpaired UTF-16 surrogate in " + pathToUtf8(path));
        }
        if (code <= 0x7f)
            output.push_back(static_cast<char>(code));
        else if (code <= 0x7ff) {
            output.push_back(static_cast<char>(0xc0 | (code >> 6)));
            output.push_back(static_cast<char>(0x80 | (code & 0x3f)));
        } else if (code <= 0xffff) {
            output.push_back(static_cast<char>(0xe0 | (code >> 12)));
            output.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3f)));
            output.push_back(static_cast<char>(0x80 | (code & 0x3f)));
        } else {
            output.push_back(static_cast<char>(0xf0 | (code >> 18)));
            output.push_back(static_cast<char>(0x80 | ((code >> 12) & 0x3f)));
            output.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3f)));
            output.push_back(static_cast<char>(0x80 | (code & 0x3f)));
        }
    }
    return output;
}

std::string value(Reader& reader, const std::filesystem::path& path, const std::string& tag) {
    Line line;
    if (!reader.next(line) || line.isKeyword()) {
        throw std::runtime_error(pathToUtf8(path) + ": missing value for [" + tag + "]");
    }
    return line.text;
}

double number(Reader& reader, const std::filesystem::path& path, const std::string& tag) {
    const std::string text = value(reader, path, tag);
    double result = 0;
    if (!parseDouble(text, result)) {
        throw std::runtime_error(pathToUtf8(path) + ": invalid number for [" + tag + "]: " + text);
    }
    return result;
}

std::string localizedName(const std::filesystem::path& path) {
    std::filesystem::path localized = path.parent_path() / path.stem();
    localized += "_ENG.dsc";
    std::error_code error;
    const bool exists = std::filesystem::exists(localized, error);
    if (error)
        throw std::runtime_error("Cannot inspect " + pathToUtf8(localized) + ": " +
                                 error.message());
    if (!exists)
        return pathToUtf8(path.stem());
    std::istringstream input(readPresetText(localized));
    Reader reader(input);
    Line line;
    while (reader.next(line)) {
        if (line.isKeyword() && lower(line.keyword()) == "name") {
            const std::string name = value(reader, localized, "name");
            if (!name.empty() && std::all_of(name.begin(), name.end(),
                                             [](unsigned char c) { return c >= 0x20 && c < 0x7f; }))
                return name;
            return pathToUtf8(path.stem());
        }
    }
    return pathToUtf8(path.stem());
}

} // namespace

std::vector<WeatherPresetFile> discoverWeatherPresets(const std::filesystem::path& directory) {
    std::error_code error;
    std::filesystem::directory_iterator entry(directory, error), end;
    if (error)
        throw std::runtime_error("Cannot list weather presets in " + pathToUtf8(directory) + ": " +
                                 error.message());
    std::vector<WeatherPresetFile> presets;
    for (; entry != end; entry.increment(error)) {
        if (error)
            break;
        std::error_code fileError;
        const bool regular = entry->is_regular_file(fileError);
        if (fileError)
            throw std::runtime_error("Cannot inspect weather preset: " + fileError.message());
        if (regular && lower(pathToUtf8(entry->path().extension())) == ".owt") {
            presets.push_back({entry->path(), localizedName(entry->path())});
        }
    }
    if (error)
        throw std::runtime_error("Cannot list weather presets: " + error.message());
    std::sort(presets.begin(), presets.end(),
              [](const auto& a, const auto& b) { return a.path < b.path; });
    if (presets.empty())
        throw std::runtime_error("No .owt weather presets found in " + pathToUtf8(directory));
    return presets;
}

LoadedWeatherPreset loadWeatherPreset(const std::filesystem::path& path) {
    std::istringstream input(readPresetText(path));
    Reader reader(input);
    LoadedWeatherPreset result;
    std::set<std::string> seen;
    Line line;
    while (reader.next(line)) {
        if (!line.isKeyword())
            continue;
        const std::string tag = lower(line.keyword());
        if (tag == "description") {
            bool ended = false;
            while (reader.next(line)) {
                if (line.isKeyword() && lower(line.keyword()) == "end") {
                    ended = true;
                    break;
                }
            }
            if (!ended)
                throw std::runtime_error(pathToUtf8(path) + ": unterminated [description]");
            continue;
        }
        if (tag == "name" || tag == "end")
            continue;
        if (!seen.insert(tag).second) {
            throw std::runtime_error(pathToUtf8(path) + ": duplicate [" + tag + "]");
        }
        if (tag == "fog") {
            result.weather.visibilityMeters = number(reader, path, tag);
            if (*result.weather.visibilityMeters <= 0.0)
                result.weather.visibilityMeters = 100000.0;
            result.weather.omsiFogHeight = number(reader, path, tag);
            result.warnings.push_back("Fog height is not applied.");
        } else if (tag == "wind") {
            double direction = number(reader, path, tag);
            const double speed = number(reader, path, tag);
            if (direction == 360.0)
                direction = 0.0;
            result.weather.windDirectionDegrees = direction;
            result.weather.windSpeedMetersPerSecond = speed;
        } else if (tag == "temp") {
            const double temperature = number(reader, path, tag);
            const double dewPoint = number(reader, path, tag);
            result.weather.temperatureCelsius = temperature;
            result.weather.absoluteHumidityGramsPerCubicMeter =
                absoluteHumidityFromDewPoint(temperature, dewPoint);
            result.warnings.push_back("Absolute humidity is approximated from preset dew point.");
        } else if (tag == "press") {
            result.weather.atmosphericPressureHectopascals = number(reader, path, tag);
            result.warnings.push_back(
                "Atmospheric pressure is retained but has no simulation effects.");
        } else if (tag == "clouds") {
            const std::string type = lower(value(reader, path, tag));
            const double height = number(reader, path, tag);
            if (type == "cumulus 1")
                result.weather.cloudType = WeatherCloudType::Cumulus1;
            else if (type == "cumulus 2")
                result.weather.cloudType = WeatherCloudType::Cumulus2;
            else if (type == "cumulus 3")
                result.weather.cloudType = WeatherCloudType::Cumulus3;
            else if (type == "overcast 1")
                result.weather.cloudType = WeatherCloudType::Overcast1;
            else if (type != "-1")
                throw std::runtime_error(pathToUtf8(path) + ": unsupported cloud type " + type);
            if (result.weather.cloudType)
                result.weather.cloudBaseAboveGroundMeters = height;
            result.warnings.push_back(
                "Cloud height is treated as meters above ground; OMSI's height reference is "
                "unverified.");
        } else if (tag == "precip") {
            const double type = number(reader, path, tag);
            const double intensity = number(reader, path, tag);
            if (type < 0 || type > 2 || std::floor(type) != type || intensity < 0 ||
                intensity > 255 || std::floor(intensity) != intensity) {
                throw std::runtime_error(pathToUtf8(path) +
                                         ": invalid precipitation type or intensity");
            }
            result.weather.precipitationType = static_cast<int>(type);
            result.weather.precipitationRate = intensity / 255.0;
            Line optional;
            if (reader.nextIncludingBlank(optional) && !optional.text.empty()) {
                double first = 0.0;
                if (!optional.isKeyword()) {
                    if (!parseDouble(optional.text, first))
                        throw std::runtime_error(pathToUtf8(path) +
                                                 ": invalid number for [precip]: " + optional.text);
                    result.weather.omsiPrecipitationParameters = std::array<double, 3>{
                        first, number(reader, path, tag), number(reader, path, tag)};
                    result.warnings.push_back("Additional precipitation fields are not applied.");
                } else {
                    reader.pushBack(std::move(optional));
                }
            }
        } else if (tag == "groundwet") {
            result.weather.omsiGroundWetness = std::array<double, 3>{
                number(reader, path, tag), number(reader, path, tag), number(reader, path, tag)};
            result.warnings.push_back(
                "Ground wetness initializes approximate classified-road conditions; "
                "native accumulation and visual effects are not simulated.");
        } else if (tag == "snow" || tag == "snowonroad") {
            if (tag == "snow")
                result.weather.omsiSnow = true;
            else
                result.weather.omsiSnowOnRoad = true;
            result.warnings.push_back(
                "Snow-on-road initializes approximate classified-road conditions; "
                "native accumulation and visual effects are not simulated.");
        } else {
            result.warnings.push_back("Unsupported weather preset tag [" + tag + "].");
        }
    }
    for (const char* required : {"fog", "wind", "temp", "press", "clouds", "precip"}) {
        if (!seen.contains(required)) {
            throw std::runtime_error(pathToUtf8(path) + ": missing [" + required + "]");
        }
    }
    validateWeatherState(result.weather);
    return result;
}

} // namespace openbus::config
