#include "map/MapConfigLoader.h"

#include "config/ConfigurationParser.h"
#include "core/Logger.h"
#include "core/PerfTrace.h"
#include "core/Utf8Path.h"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace openbus::map {

namespace {

using openbus::pathToUtf8;

std::filesystem::path pathFromUtf8(std::string_view value) {
    std::u8string utf8;
    utf8.reserve(value.size());
    for (const unsigned char character : value) {
        utf8.push_back(static_cast<char8_t>(character));
    }
    return std::filesystem::path(utf8);
}

std::filesystem::path pathWithSuffix(const std::filesystem::path& path, std::string_view suffix) {
    std::filesystem::path result = path;
    result += suffix;
    return result;
}

std::string trim(std::string_view value) {
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return std::string(value.substr(first, last - first + 1));
}

void appendUtf8(std::string& output, std::uint32_t codepoint) {
    if (codepoint <= 0x7fU) {
        output.push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7ffU) {
        output.push_back(static_cast<char>(0xc0U | (codepoint >> 6U)));
        output.push_back(static_cast<char>(0x80U | (codepoint & 0x3fU)));
    } else if (codepoint <= 0xffffU) {
        output.push_back(static_cast<char>(0xe0U | (codepoint >> 12U)));
        output.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3fU)));
        output.push_back(static_cast<char>(0x80U | (codepoint & 0x3fU)));
    } else {
        output.push_back(static_cast<char>(0xf0U | (codepoint >> 18U)));
        output.push_back(static_cast<char>(0x80U | ((codepoint >> 12U) & 0x3fU)));
        output.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3fU)));
        output.push_back(static_cast<char>(0x80U | (codepoint & 0x3fU)));
    }
}

std::string decodeText(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not open map text file: " + pathToUtf8(path));
    }
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)),
                                           std::istreambuf_iterator<char>());
    if (bytes.size() >= 2 && bytes[0] == 0xffU && bytes[1] == 0xfeU) {
        if ((bytes.size() - 2) % 2 != 0) {
            throw std::runtime_error("Odd-length UTF-16LE map text file: " + pathToUtf8(path));
        }
        std::string output;
        for (std::size_t offset = 2; offset < bytes.size(); offset += 2) {
            std::uint32_t codepoint = static_cast<std::uint32_t>(bytes[offset]) |
                                      (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U);
            if (codepoint >= 0xd800U && codepoint <= 0xdbffU) {
                if (offset + 3 >= bytes.size()) {
                    throw std::runtime_error("Unpaired UTF-16 surrogate in: " + pathToUtf8(path));
                }
                const std::uint32_t low = static_cast<std::uint32_t>(bytes[offset + 2]) |
                                          (static_cast<std::uint32_t>(bytes[offset + 3]) << 8U);
                if (low < 0xdc00U || low > 0xdfffU) {
                    throw std::runtime_error("Unpaired UTF-16 surrogate in: " + pathToUtf8(path));
                }
                codepoint = 0x10000U + ((codepoint - 0xd800U) << 10U) + (low - 0xdc00U);
                offset += 2;
            } else if (codepoint >= 0xdc00U && codepoint <= 0xdfffU) {
                throw std::runtime_error("Unpaired UTF-16 surrogate in: " + pathToUtf8(path));
            }
            appendUtf8(output, codepoint);
        }
        return output;
    }
    if (bytes.size() >= 2 && bytes[0] == 0xfeU && bytes[1] == 0xffU) {
        throw std::runtime_error("UTF-16BE map text is not supported: " + pathToUtf8(path));
    }
    std::size_t offset =
        bytes.size() >= 3 && bytes[0] == 0xefU && bytes[1] == 0xbbU && bytes[2] == 0xbfU ? 3 : 0;
    return std::string(bytes.begin() + static_cast<std::ptrdiff_t>(offset), bytes.end());
}

std::vector<std::string> splitLines(const std::string& text) {
    std::vector<std::string> lines;
    std::size_t begin = 0;
    while (begin < text.size()) {
        const std::size_t end = text.find('\n', begin);
        const std::size_t lineEnd = end == std::string::npos ? text.size() : end;
        std::string line = text.substr(begin, lineEnd - begin);
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(std::move(line));
        if (end == std::string::npos) {
            return lines;
        }
        begin = end + 1;
    }
    if (!text.empty() && text.back() == '\n') {
        lines.emplace_back();
    }
    return lines;
}

std::string readField(const std::vector<std::string>& lines, std::size_t& cursor,
                      const std::filesystem::path& path, const std::string& section) {
    if (cursor >= lines.size()) {
        throw std::runtime_error("Truncated [" + section + "] record in " + pathToUtf8(path));
    }
    return trim(lines[cursor++]);
}

bool parseInteger(const std::string& value, int& result) {
    if (value.empty()) {
        return false;
    }
    const char* begin = value.data();
    const char* end = begin + value.size();
    if (*begin == '+') {
        ++begin;
    }
    const auto parsed = std::from_chars(begin, end, result);
    return begin != end && parsed.ec == std::errc{} && parsed.ptr == end;
}

std::optional<int> parseEditorObjectNumber(const std::string& label) {
    std::string normalized = trim(label);
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    constexpr std::string_view prefix = "object nr.";
    if (!normalized.starts_with(prefix)) {
        return std::nullopt;
    }
    int number = 0;
    if (!parseInteger(trim(normalized.substr(prefix.size())), number) || number < 0) {
        return std::nullopt;
    }
    return number;
}

bool parseSize(const std::string& value, std::size_t& result) {
    int parsed = 0;
    if (!parseInteger(value, parsed) || parsed < 0) {
        return false;
    }
    result = static_cast<std::size_t>(parsed);
    return true;
}

int checkedIndexToInt(std::size_t index, std::string_view description) {
    if (index > static_cast<std::size_t>((std::numeric_limits<int>::max)())) {
        throw std::overflow_error(std::string(description) +
                                  " exceeds the supported integer range");
    }
    return static_cast<int>(index);
}

bool parseReal(const std::string& value, double& result) {
    if (value.empty()) {
        return false;
    }
    const char* begin = value.data();
    const char* end = begin + value.size();
    const auto parsed = std::from_chars(begin, end, result);
    return parsed.ec == std::errc{} && parsed.ptr == end && std::isfinite(result);
}

std::string lowerAscii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::string canonicalAiRoadGroupName(std::string name) {
    name = lowerAscii(std::move(name));
    if (name == "normal" || name == "normalcar") {
        return "normalcars";
    }
    if (name == "commercial") {
        return "commercials";
    }
    return name;
}

bool isSupportedRoadAiGroup(std::string_view name) {
    const std::string normalized = canonicalAiRoadGroupName(std::string(name));
    return normalized == "normalcars" || normalized == "trucks" || normalized == "commercials" ||
           normalized == "ambulance" || normalized == "emergency" || normalized == "taxi";
}

bool isLegacySupportedRoadAiGroup(std::string_view name) {
    const std::string normalized = canonicalAiRoadGroupName(std::string(name));
    return normalized == "normalcars" || normalized == "trucks" || normalized == "commercials" ||
           normalized == "ambulance";
}

bool isSectionHeaderLine(const std::string& value) {
    return value.size() > 2 && value.front() == '[' && value.back() == ']';
}

bool isIgnoredAiListLine(const std::string& value) {
    return value.empty() || value.starts_with("#") || value.starts_with(";") ||
           value.starts_with("//");
}

bool hasExtension(const std::string& assetPath, std::string_view extension) {
    const std::string normalizedPath = lowerAscii(assetPath);
    const std::string normalizedExtension = lowerAscii(std::string(extension));
    return normalizedPath.size() >= normalizedExtension.size() &&
           normalizedPath.compare(normalizedPath.size() - normalizedExtension.size(),
                                  normalizedExtension.size(), normalizedExtension) == 0;
}

std::optional<std::string> parseAiVehicleAssetPath(const std::string& sourceLine,
                                                   bool allowOvh = true) {
    const std::string line = trim(sourceLine);
    if (isIgnoredAiListLine(line)) {
        return std::nullopt;
    }
    const std::string normalizedLine = lowerAscii(line);
    std::size_t extensionPosition = normalizedLine.find(".bus");
    std::size_t extensionLength = 4;
    const std::size_t ovhPosition = allowOvh ? normalizedLine.find(".ovh") : std::string::npos;
    if (ovhPosition != std::string::npos &&
        (extensionPosition == std::string::npos || ovhPosition < extensionPosition)) {
        extensionPosition = ovhPosition;
        extensionLength = 4;
    }
    if (extensionPosition == std::string::npos) {
        return std::nullopt;
    }
    const std::size_t assetEnd = extensionPosition + extensionLength;
    if (assetEnd < line.size() && line[assetEnd] != ' ' && line[assetEnd] != '\t') {
        return std::nullopt;
    }
    return trim(line.substr(0, assetEnd));
}

std::optional<int> parseAiBusTypeSection(const std::string& section) {
    constexpr std::string_view prefix = "[aigroup_depot_typgroup_";
    const std::string normalized = lowerAscii(section);
    if (!normalized.starts_with(prefix) || !normalized.ends_with(']')) {
        return std::nullopt;
    }
    const std::string typeText =
        normalized.substr(prefix.size(), normalized.size() - prefix.size() - 1);
    int type = 0;
    if (!parseInteger(typeText, type)) {
        return std::nullopt;
    }
    return type;
}

void parseAiListFile(const std::filesystem::path& path, MapAiTrafficData& aiTraffic) {
    const std::vector<std::string> lines = splitLines(decodeText(path));
    std::vector<MapAiVehicleEntry> aiListVehicles;
    std::optional<MapAiVehicleGroup> standardRoadGroup;
    std::optional<std::size_t> currentDepotIndex;
    for (std::size_t cursor = 0; cursor < lines.size();) {
        const std::string section = lowerAscii(trim(lines[cursor]));
        ++cursor;
        if (section == "[ailist]") {
            MapAiVehicleGroup group;
            group.name = "NormalCars";
            while (cursor < lines.size()) {
                const std::string& sourceLine = lines[cursor];
                if (isSectionHeaderLine(trim(sourceLine))) {
                    break;
                }
                ++cursor;
                const std::optional<std::string> assetPath = parseAiVehicleAssetPath(sourceLine);
                if (!assetPath) {
                    continue;
                }
                MapAiVehicleEntry entry{*assetPath, sourceLine};
                aiListVehicles.push_back(entry);
                group.rawLines.push_back(sourceLine);
                group.vehicles.push_back(std::move(entry));
            }
            if (!group.vehicles.empty()) {
                standardRoadGroup = std::move(group);
            }
            currentDepotIndex.reset();
            continue;
        }
        if (section == "[aigroup_2]") {
            currentDepotIndex.reset();
            MapAiVehicleGroup group;
            while (cursor < lines.size()) {
                const std::string& sourceLine = lines[cursor];
                const std::string line = trim(sourceLine);
                if (isSectionHeaderLine(line)) {
                    if (lowerAscii(line) == "[end]") {
                        ++cursor;
                    }
                    break;
                }
                ++cursor;
                group.rawLines.push_back(sourceLine);
                if (isIgnoredAiListLine(line)) {
                    continue;
                }
                const std::optional<std::string> assetPath = parseAiVehicleAssetPath(sourceLine);
                if (assetPath) {
                    group.vehicles.push_back({*assetPath, sourceLine});
                } else if (group.name.empty()) {
                    group.name = line;
                }
            }
            if (!group.vehicles.empty()) {
                aiTraffic.vehicleGroups.push_back(std::move(group));
            }
            continue;
        }

        if (section == "[aigroup_depot_typgroup]") {
            MapAiBusTypeDefinition definition;
            definition.depotIndex = currentDepotIndex;
            bool hasType = false;
            while (cursor < lines.size()) {
                const std::string& sourceLine = lines[cursor];
                const std::string line = trim(sourceLine);
                if (isSectionHeaderLine(line)) {
                    if (lowerAscii(line) == "[end]") {
                        ++cursor;
                    }
                    break;
                }
                ++cursor;
                definition.rawLines.push_back(sourceLine);
                if (isIgnoredAiListLine(line)) {
                    continue;
                }
                if (!hasType) {
                    hasType = parseInteger(line, definition.type);
                    if (hasType && definition.type >= 0 &&
                        static_cast<std::size_t>(definition.type) < aiListVehicles.size()) {
                        const std::string& assetPath =
                            aiListVehicles[static_cast<std::size_t>(definition.type)].assetPath;
                        if (hasExtension(assetPath, ".bus")) {
                            definition.assetPath = assetPath;
                        }
                    }
                }
            }
            if (!definition.assetPath.empty()) {
                const std::size_t definitionIndex = aiTraffic.busTypes.size();
                aiTraffic.busTypes.push_back(std::move(definition));
                if (currentDepotIndex.has_value()) {
                    aiTraffic.depots[*currentDepotIndex].busTypeDefinitionIndices.push_back(
                        definitionIndex);
                }
            }
            continue;
        }

        if (section == "[aigroup_depot]") {
            MapAiDepotDefinition depot;
            while (cursor < lines.size()) {
                const std::string& sourceLine = lines[cursor];
                const std::string line = trim(sourceLine);
                if (isSectionHeaderLine(line)) {
                    break;
                }
                ++cursor;
                depot.rawLines.push_back(sourceLine);
                if (isIgnoredAiListLine(line)) {
                    continue;
                }
                if (depot.name.empty()) {
                    depot.name = line;
                } else if (depot.hofName.empty()) {
                    depot.hofName = line;
                }
            }
            if (!depot.name.empty() || !depot.hofName.empty()) {
                aiTraffic.depots.push_back(std::move(depot));
                currentDepotIndex = aiTraffic.depots.size() - 1;
            } else {
                currentDepotIndex.reset();
            }
            continue;
        }

        const std::optional<int> busType = parseAiBusTypeSection(section);
        if (!busType) {
            currentDepotIndex.reset();
            continue;
        }
        MapAiBusTypeDefinition definition;
        definition.type = *busType;
        definition.depotIndex = currentDepotIndex;
        while (cursor < lines.size()) {
            const std::string& sourceLine = lines[cursor];
            const std::string line = trim(sourceLine);
            if (isSectionHeaderLine(line)) {
                if (lowerAscii(line) == "[end]") {
                    ++cursor;
                }
                break;
            }
            ++cursor;
            definition.rawLines.push_back(sourceLine);
            if (!isIgnoredAiListLine(line)) {
                const std::optional<std::string> assetPath =
                    parseAiVehicleAssetPath(sourceLine, false);
                if (assetPath && definition.assetPath.empty()) {
                    definition.assetPath = *assetPath;
                }
            }
        }
        if (!definition.assetPath.empty()) {
            const std::size_t definitionIndex = aiTraffic.busTypes.size();
            aiTraffic.busTypes.push_back(std::move(definition));
            if (currentDepotIndex.has_value()) {
                aiTraffic.depots[*currentDepotIndex].busTypeDefinitionIndices.push_back(
                    definitionIndex);
            }
        }
    }
    if (aiTraffic.vehicleGroups.empty() && standardRoadGroup.has_value()) {
        aiTraffic.vehicleGroups.push_back(std::move(*standardRoadGroup));
    }
}

void parseUnscheduledVehicleGroups(const std::filesystem::path& path, MapAiTrafficData& aiTraffic) {
    const std::vector<std::string> lines = splitLines(decodeText(path));
    for (std::size_t cursor = 0; cursor < lines.size(); ++cursor) {
        if (lowerAscii(trim(lines[cursor])) != "[group]") {
            continue;
        }
        while (++cursor < lines.size()) {
            const std::string name = trim(lines[cursor]);
            if (name.empty() || isIgnoredAiListLine(name)) {
                continue;
            }
            if (isSectionHeaderLine(name)) {
                --cursor;
                break;
            }
            if (name.size() >= 2 && name.front() == '<' && name.back() == '>') {
                while (++cursor < lines.size() && !isSectionHeaderLine(trim(lines[cursor]))) {
                }
                if (cursor < lines.size()) {
                    --cursor;
                }
                break;
            }
            aiTraffic.unscheduledVehicleGroupNames.push_back(name);
            break;
        }
    }
}

void parseUnscheduledTrafficDensities(const std::filesystem::path& path,
                                      MapAiTrafficData& aiTraffic,
                                      ConfigurationDiagnostics& diagnostics) {
    aiTraffic.hasUnscheduledTrafficDensityFile = false;
    const std::vector<std::string> lines = splitLines(decodeText(path));
    std::optional<std::size_t> currentGroup;
    std::optional<std::size_t> currentSchedule;
    bool hasDaySchedule = false;
    bool invalidDaySchedule = false;
    const auto readValue = [&](std::size_t& cursor) -> std::optional<std::string> {
        while (cursor < lines.size()) {
            const std::string value = trim(lines[cursor++]);
            if (isIgnoredAiListLine(value)) {
                continue;
            }
            if (isSectionHeaderLine(value)) {
                --cursor;
                return std::nullopt;
            }
            return value;
        }
        return std::nullopt;
    };

    for (std::size_t cursor = 0; cursor < lines.size();) {
        const std::size_t sectionLine = cursor;
        const std::string section = lowerAscii(trim(lines[cursor++]));
        if (section == "[group]") {
            currentGroup.reset();
            currentSchedule.reset();
            hasDaySchedule = false;
            invalidDaySchedule = false;
            const auto name = readValue(cursor);
            const auto factorText = readValue(cursor);
            double factor = 0.0;
            if (!name.has_value() || name->empty() || !factorText.has_value() ||
                !parseReal(*factorText, factor) || factor < 0.0) {
                diagnostics.warning(sectionLine + 1, "group",
                                    "expected a group name and finite non-negative factor");
                continue;
            }
            aiTraffic.unscheduledTrafficDensityGroups.push_back({*name, factor, {}});
            currentGroup = aiTraffic.unscheduledTrafficDensityGroups.size() - 1;
        } else if (section == "[set_day_of_week]") {
            hasDaySchedule = true;
            currentSchedule.reset();
            invalidDaySchedule = true;
            const auto maskText = readValue(cursor);
            int mask = 0;
            if (!currentGroup.has_value() || !maskText.has_value() ||
                !parseInteger(*maskText, mask) || mask < 0 || (mask & ~7) != 0) {
                diagnostics.warning(sectionLine + 1, "set_day_of_week",
                                    "expected a group and weekday mask from 0 through 7");
                continue;
            }
            aiTraffic.unscheduledTrafficDensityGroups[*currentGroup].schedules.push_back(
                {mask, {}});
            currentSchedule =
                aiTraffic.unscheduledTrafficDensityGroups[*currentGroup].schedules.size() - 1;
            invalidDaySchedule = false;
        } else if (section == "[trafficdensity]") {
            const auto hourText = readValue(cursor);
            const auto densityText = readValue(cursor);
            double hour = 0.0;
            double density = 0.0;
            if (!currentGroup.has_value() || !hourText.has_value() || !densityText.has_value() ||
                !parseReal(*hourText, hour) || hour < 0.0 || hour > 24.0 ||
                !parseReal(*densityText, density)) {
                diagnostics.warning(sectionLine + 1, "trafficdensity",
                                    "expected a group and finite hour/density values");
                continue;
            }
            if (!currentSchedule.has_value() && !hasDaySchedule) {
                aiTraffic.unscheduledTrafficDensityGroups[*currentGroup].schedules.push_back(
                    {0, {}});
                currentSchedule =
                    aiTraffic.unscheduledTrafficDensityGroups[*currentGroup].schedules.size() - 1;
            }
            if (invalidDaySchedule || !currentSchedule.has_value()) {
                diagnostics.warning(sectionLine + 1, "trafficdensity",
                                    "ignored density point after an invalid weekday mask");
                continue;
            }
            aiTraffic.unscheduledTrafficDensityGroups[*currentGroup]
                .schedules[*currentSchedule]
                .points.push_back({hour, std::max(0.0, density)});
        }
    }

    for (const MapAiTrafficDensityGroup& group : aiTraffic.unscheduledTrafficDensityGroups) {
        for (const MapAiTrafficDensitySchedule& schedule : group.schedules) {
            if (!schedule.points.empty() &&
                !mapTrafficDensityAtHour(schedule.points, schedule.points.front().hour)
                     .has_value()) {
                diagnostics.warning(0, "trafficdensity",
                                    "ignored an invalid or unordered curve for group '" +
                                        group.name + "'");
            }
        }
    }
    aiTraffic.hasUnscheduledTrafficDensityFile = !aiTraffic.unscheduledTrafficDensityGroups.empty();
}

struct TimetableSource {
    std::filesystem::path directory;
    std::vector<std::string> deactivatedLines;
};

void parseAiTimetableRoutes(const std::vector<TimetableSource>& sources,
                            MapAiTrafficData& aiTraffic, ConfigurationDiagnostics& diagnostics) {
    std::unordered_map<std::string, std::filesystem::path> trackFiles;
    std::unordered_map<std::string, std::filesystem::path> tripsByName;
    std::unordered_map<std::string, std::filesystem::path> linesByName;
    std::vector<std::filesystem::path> tripFiles;
    std::vector<std::filesystem::path> lineFiles;
    std::vector<std::filesystem::path> busStopsPaths;
    std::vector<std::filesystem::path> stationLinksPaths;
    for (const auto& source : sources) {
        for (const auto& line : source.deactivatedLines)
            linesByName.erase(lowerAscii(line));
        if (!std::filesystem::is_directory(source.directory))
            continue;
        for (const std::filesystem::directory_entry& entry :
             std::filesystem::directory_iterator(source.directory)) {
            if (!entry.is_regular_file())
                continue;
            const std::string extension = lowerAscii(pathToUtf8(entry.path().extension()));
            const std::string stem = lowerAscii(pathToUtf8(entry.path().stem()));
            if (extension == ".ttr") {
                trackFiles.insert_or_assign(stem, entry.path());
            } else if (extension == ".ttp") {
                tripsByName.insert_or_assign(stem, entry.path());
            } else if (extension == ".ttl") {
                linesByName.insert_or_assign(stem, entry.path());
            } else if (lowerAscii(pathToUtf8(entry.path().filename())) == "busstops.cfg") {
                busStopsPaths.push_back(entry.path());
            } else if (lowerAscii(pathToUtf8(entry.path().filename())) == "stnlinks.cfg") {
                stationLinksPaths.push_back(entry.path());
            }
        }
    }
    for (const auto& [name, path] : tripsByName)
        tripFiles.push_back(path);
    for (const auto& [name, path] : linesByName)
        lineFiles.push_back(path);
    std::sort(tripFiles.begin(), tripFiles.end());
    std::sort(lineFiles.begin(), lineFiles.end());

    for (const auto& busStopsPath : busStopsPaths) {
        const std::size_t oldCount = aiTraffic.busStops.size();
        const std::vector<std::string> lines = splitLines(decodeText(busStopsPath));
        for (std::size_t lineIndex = 0; lineIndex < lines.size(); ++lineIndex) {
            if (lowerAscii(trim(lines[lineIndex])) != "[busstop]") {
                continue;
            }
            std::array<std::string, 7> fields;
            bool complete = true;
            for (std::size_t fieldIndex = 0; fieldIndex < 6; ++fieldIndex) {
                std::string& field = fields[fieldIndex];
                if (++lineIndex >= lines.size()) {
                    complete = false;
                    break;
                }
                if (isSectionHeaderLine(trim(lines[lineIndex]))) {
                    --lineIndex;
                    complete = false;
                    break;
                }
                field = trim(lines[lineIndex]);
            }
            if (complete && lineIndex + 1 < lines.size() &&
                !isSectionHeaderLine(trim(lines[lineIndex + 1]))) {
                fields[6] = trim(lines[++lineIndex]);
            }
            int tileIndex = 0;
            int id = 0;
            bool parametersValid = complete;
            for (std::size_t fieldIndex = 3; parametersValid && fieldIndex < 6; ++fieldIndex) {
                double value = 0.0;
                parametersValid = parseReal(fields[fieldIndex], value);
            }
            if (complete && parametersValid && parseInteger(fields[1], tileIndex) &&
                tileIndex >= 0 && parseInteger(fields[2], id) && id != 0 && !fields[0].empty()) {
                MapAiBusStop stop;
                stop.name = fields[0];
                stop.tileIndex = tileIndex;
                stop.id = id;
                std::copy_n(fields.begin() + 3, stop.remainingFields.size(),
                            stop.remainingFields.begin());
                aiTraffic.busStops.push_back(std::move(stop));
            } else {
                diagnostics.warning(lineIndex + 1, "busstop",
                                    "invalid or incomplete bus-stop record in " +
                                        pathToUtf8(busStopsPath.filename()));
            }
        }
        std::unordered_set<int> overriddenIds;
        for (auto it = aiTraffic.busStops.begin() + static_cast<std::ptrdiff_t>(oldCount);
             it != aiTraffic.busStops.end(); ++it)
            overriddenIds.insert(it->id);
        const auto oldEnd = aiTraffic.busStops.begin() + static_cast<std::ptrdiff_t>(oldCount);
        aiTraffic.busStops.erase(
            std::remove_if(aiTraffic.busStops.begin(), oldEnd,
                           [&](const auto& stop) { return overriddenIds.contains(stop.id); }),
            oldEnd);
    }

    for (const auto& stationLinksPath : stationLinksPaths) {
        const std::size_t oldCount = aiTraffic.stationLinks.size();
        const std::vector<std::string> lines = splitLines(decodeText(stationLinksPath));
        for (std::size_t lineIndex = 0; lineIndex < lines.size(); ++lineIndex) {
            const std::string section = lowerAscii(trim(lines[lineIndex]));
            if (section == "[stnlink]") {
                std::array<std::string, 9> fields;
                bool complete = true;
                for (std::string& field : fields) {
                    if (++lineIndex >= lines.size()) {
                        complete = false;
                        break;
                    }
                    if (isSectionHeaderLine(trim(lines[lineIndex]))) {
                        --lineIndex;
                        complete = false;
                        break;
                    }
                    field = trim(lines[lineIndex]);
                }
                MapAiStationLink link;
                bool numericFieldsValid = complete;
                for (std::size_t fieldIndex = 0; numericFieldsValid && fieldIndex < fields.size();
                     ++fieldIndex) {
                    if (fieldIndex == 1 || fieldIndex == 2) {
                        continue;
                    }
                    double value = 0.0;
                    numericFieldsValid = parseReal(fields[fieldIndex], value);
                }
                if (complete && numericFieldsValid && parseInteger(fields[1], link.fromStopId) &&
                    link.fromStopId != 0 && parseInteger(fields[2], link.toStopId) &&
                    link.toStopId != 0) {
                    link.fields = fields;
                    aiTraffic.stationLinks.push_back(std::move(link));
                } else {
                    diagnostics.warning(lineIndex + 1, "StnLink",
                                        "invalid or incomplete station-link record in " +
                                            pathToUtf8(stationLinksPath.filename()));
                }
            } else if (section == "[stnlink_entry]") {
                std::array<std::string, 7> entryValues;
                bool complete = true;
                for (std::string& value : entryValues) {
                    if (++lineIndex >= lines.size()) {
                        complete = false;
                        break;
                    }
                    if (isSectionHeaderLine(trim(lines[lineIndex]))) {
                        --lineIndex;
                        complete = false;
                        break;
                    }
                    value = trim(lines[lineIndex]);
                }
                MapAiStationLink::Entry entry;
                bool numericEntryValid =
                    complete && parseInteger(entryValues[0], entry.splineId) &&
                    entry.splineId != 0 && parseInteger(entryValues[1], entry.pathIndex) &&
                    entry.pathIndex >= 0 && parseInteger(entryValues[2], entry.tileIndex) &&
                    entry.tileIndex >= 0 && parseReal(entryValues[3], entry.lengthMeters);
                for (std::size_t index = 4; numericEntryValid && index < entryValues.size();
                     ++index) {
                    double number = 0.0;
                    numericEntryValid = parseReal(entryValues[index], number);
                }
                if (complete && numericEntryValid && aiTraffic.stationLinks.size() > oldCount) {
                    std::copy(entryValues.begin() + 4, entryValues.end(),
                              entry.remainingFields.begin());
                    aiTraffic.stationLinks.back().entries.push_back(std::move(entry));
                } else {
                    diagnostics.warning(lineIndex + 1, "StnLink_entry",
                                        "invalid or orphaned station-link path entry in " +
                                            pathToUtf8(stationLinksPath.filename()));
                }
            }
        }
        std::set<std::pair<int, int>> overriddenLinks;
        for (auto it = aiTraffic.stationLinks.begin() + static_cast<std::ptrdiff_t>(oldCount);
             it != aiTraffic.stationLinks.end(); ++it)
            overriddenLinks.emplace(it->fromStopId, it->toStopId);
        const auto oldEnd = aiTraffic.stationLinks.begin() + static_cast<std::ptrdiff_t>(oldCount);
        aiTraffic.stationLinks.erase(std::remove_if(aiTraffic.stationLinks.begin(), oldEnd,
                                                    [&](const auto& link) {
                                                        return overriddenLinks.contains(
                                                            {link.fromStopId, link.toStopId});
                                                    }),
                                     oldEnd);
    }

    for (const std::filesystem::path& tripPath : tripFiles) {
        const std::vector<std::string> tripLines = splitLines(decodeText(tripPath));
        MapAiTimetableRoute route;
        MapAiPassengerTimetable passengerTimetable;
        route.timetableKey = lowerAscii(pathToUtf8(tripPath.stem()));
        passengerTimetable.timetableKey = route.timetableKey;
        std::size_t routeHeaderFields = 0;
        for (std::size_t lineIndex = 0; lineIndex < tripLines.size(); ++lineIndex) {
            const std::string section = lowerAscii(trim(tripLines[lineIndex]));
            if (section == "[trip]") {
                std::array<std::string*, 3> fields = {&route.tripName, &route.destinationName,
                                                      &route.lineName};
                std::size_t next = lineIndex + 1;
                for (std::string* field : fields) {
                    if (next >= tripLines.size() || isSectionHeaderLine(trim(tripLines[next]))) {
                        break;
                    }
                    *field = trim(tripLines[next++]);
                    ++routeHeaderFields;
                }
                lineIndex = next > 0 ? next - 1 : lineIndex;
            } else if (section == "[station]") {
                std::array<std::string, 8> stationFields;
                std::size_t next = lineIndex + 1;
                bool complete = true;
                for (std::string& field : stationFields) {
                    if (next >= tripLines.size() || isSectionHeaderLine(trim(tripLines[next]))) {
                        complete = false;
                        break;
                    }
                    field = trim(tripLines[next++]);
                }
                int stationId = 0;
                int trackEntryIndex = 0;
                int tileIndex = 0;
                if (complete && parseInteger(stationFields[0], stationId) && stationId != 0 &&
                    parseInteger(stationFields[1], trackEntryIndex) && trackEntryIndex >= 0 &&
                    !stationFields[2].empty() && parseInteger(stationFields[3], tileIndex) &&
                    tileIndex >= 0) {
                    ++route.stationCount;
                    MapAiTimetableRoute::Station station;
                    station.id = stationId;
                    station.trackEntryIndex = trackEntryIndex;
                    station.name = stationFields[2];
                    station.tileIndex = tileIndex;
                    std::copy_n(stationFields.begin() + 4, station.opaqueFields.size(),
                                station.opaqueFields.begin());
                    double positionOnPreviousTrackEntryMeters = 0.0;
                    if (parseReal(station.opaqueFields[2], positionOnPreviousTrackEntryMeters) &&
                        positionOnPreviousTrackEntryMeters >= 0.0) {
                        station.positionOnPreviousTrackEntryMeters =
                            positionOnPreviousTrackEntryMeters;
                    }
                    route.stations.push_back(std::move(station));
                    route.stationIds.push_back(stationId);
                    passengerTimetable.stationIds.push_back(stationId);
                    passengerTimetable.stationRecords.push_back(route.stations.back());
                } else {
                    diagnostics.warning(lineIndex + 1, "station",
                                        "invalid or truncated timetable station record in " +
                                            pathToUtf8(tripPath.filename()));
                }
                lineIndex = next > 0 ? next - 1 : lineIndex;
            } else if (section == "[station_typ2]") {
                const std::size_t valueLine = lineIndex + 1;
                int stationId = 0;
                if (valueLine < tripLines.size() &&
                    !isSectionHeaderLine(trim(tripLines[valueLine])) &&
                    parseInteger(trim(tripLines[valueLine]), stationId) && stationId != 0) {
                    ++route.stationCount;
                    route.stationIds.push_back(stationId);
                    passengerTimetable.stationIds.push_back(stationId);
                    passengerTimetable.stationRecords.push_back(std::nullopt);
                    lineIndex = valueLine;
                } else {
                    diagnostics.warning(lineIndex + 1, "station_typ2",
                                        "invalid or truncated timetable station ID in " +
                                            pathToUtf8(tripPath.filename()));
                }
            } else if (section == "[profile]") {
                std::size_t next = lineIndex + 1;
                while (next < tripLines.size() && trim(tripLines[next]).empty()) {
                    ++next;
                }
                const std::string profileName =
                    next < tripLines.size() && !isSectionHeaderLine(trim(tripLines[next]))
                        ? trim(tripLines[next++])
                        : std::string{};
                while (next < tripLines.size() && trim(tripLines[next]).empty()) {
                    ++next;
                }
                double durationMinutes = 0.0;
                if (next >= tripLines.size() || isSectionHeaderLine(trim(tripLines[next])) ||
                    !parseReal(trim(tripLines[next]), durationMinutes) ||
                    !std::isfinite(durationMinutes) || durationMinutes < 0.0) {
                    diagnostics.warning(lineIndex + 1, "profile",
                                        "invalid or truncated timetable profile in " +
                                            pathToUtf8(tripPath.filename()));
                } else {
                    route.profiles.push_back({profileName, durationMinutes, {}, {}, {}, {}});
                }
                if (next < tripLines.size() && !isSectionHeaderLine(trim(tripLines[next]))) {
                    lineIndex = next;
                } else if (next > 0) {
                    lineIndex = next - 1;
                }
            } else if (section == "[profile_man_arr_time]" || section == "[profile_man_dep_time]") {
                std::size_t next = lineIndex + 1;
                while (next < tripLines.size() && trim(tripLines[next]).empty()) {
                    ++next;
                }
                int stationIndex = -1;
                const bool validIndex = next < tripLines.size() &&
                                        !isSectionHeaderLine(trim(tripLines[next])) &&
                                        parseInteger(trim(tripLines[next]), stationIndex);
                if (validIndex) {
                    ++next;
                }
                while (next < tripLines.size() && trim(tripLines[next]).empty()) {
                    ++next;
                }
                double minutesAfterDeparture = 0.0;
                if (!validIndex || stationIndex < 0 || next >= tripLines.size() ||
                    isSectionHeaderLine(trim(tripLines[next])) ||
                    !parseReal(trim(tripLines[next]), minutesAfterDeparture) ||
                    !std::isfinite(minutesAfterDeparture) || minutesAfterDeparture < 0.0 ||
                    route.profiles.empty()) {
                    diagnostics.warning(
                        lineIndex + 1, section.substr(1, section.size() - 2),
                        "invalid, truncated, or orphaned timetable profile time in " +
                            pathToUtf8(tripPath.filename()));
                } else {
                    auto& times = section == "[profile_man_arr_time]"
                                      ? route.profiles.back().manualArrivalMinutes
                                      : route.profiles.back().manualDepartureMinutes;
                    times.emplace_back(stationIndex, minutesAfterDeparture);
                }
                if (next < tripLines.size() && !isSectionHeaderLine(trim(tripLines[next]))) {
                    lineIndex = next;
                } else if (next > 0) {
                    lineIndex = next - 1;
                }
            }
        }
        passengerTimetable.tripName = route.tripName;
        passengerTimetable.destinationName = route.destinationName;
        passengerTimetable.lineName = route.lineName;
        passengerTimetable.profiles = route.profiles;
        if (routeHeaderFields != 3 || route.destinationName.empty() || route.lineName.empty() ||
            route.stationCount == 0) {
            continue;
        }

        if (route.tripName.empty()) {
            if (!passengerTimetable.stationIds.empty()) {
                aiTraffic.passengerTimetables.push_back(std::move(passengerTimetable));
            }
            continue;
        }
        const auto trackFile = trackFiles.find(lowerAscii(route.tripName));
        if (trackFile == trackFiles.end()) {
            if (!passengerTimetable.stationIds.empty()) {
                aiTraffic.passengerTimetables.push_back(std::move(passengerTimetable));
            } else {
                diagnostics.warning(0, "trip",
                                    "no matching timetable track file for " + route.tripName +
                                        " in " + pathToUtf8(tripPath.filename()));
            }
            continue;
        }
        const std::vector<std::string> trackLines = splitLines(decodeText(trackFile->second));
        bool malformedTrackEntry = false;
        for (std::size_t lineIndex = 0; lineIndex < trackLines.size(); ++lineIndex) {
            if (lowerAscii(trim(trackLines[lineIndex])) != "[track_entry]") {
                continue;
            }
            std::size_t next = lineIndex + 1;
            while (next < trackLines.size() && trim(trackLines[next]).empty()) {
                ++next;
            }
            int splineId = 0;
            int pathIndex = 0;
            if (next >= trackLines.size() || isSectionHeaderLine(trim(trackLines[next])) ||
                !parseInteger(trim(trackLines[next]), splineId) || splineId == 0) {
                malformedTrackEntry = true;
                diagnostics.warning(lineIndex + 1, "track_entry",
                                    "invalid timetable spline ID in " +
                                        pathToUtf8(trackFile->second.filename()));
                continue;
            }
            std::array<std::string, 5> remainingFields;
            std::size_t selectorLine = next + 1;
            bool completeTrackEntry = true;
            for (std::string& field : remainingFields) {
                while (selectorLine < trackLines.size() && trim(trackLines[selectorLine]).empty()) {
                    ++selectorLine;
                }
                if (selectorLine >= trackLines.size() ||
                    isSectionHeaderLine(trim(trackLines[selectorLine]))) {
                    completeTrackEntry = false;
                    break;
                }
                field = trim(trackLines[selectorLine++]);
            }
            int tileIndex = 0;
            double lengthMeters = 0.0;
            if (!completeTrackEntry || !parseInteger(remainingFields[0], pathIndex) ||
                pathIndex < 0 || !parseInteger(remainingFields[1], tileIndex) || tileIndex < 0 ||
                !parseReal(remainingFields[3], lengthMeters) || lengthMeters < 0.0) {
                malformedTrackEntry = true;
                diagnostics.warning(lineIndex + 1, "track_entry",
                                    "invalid or truncated timetable path entry in " +
                                        pathToUtf8(trackFile->second.filename()));
                continue;
            }
            route.trackEntries.push_back({splineId, pathIndex, tileIndex, lengthMeters});
            if (route.splineIds.empty() || route.splineIds.back() != splineId) {
                route.splineIds.push_back(splineId);
            }
            lineIndex = selectorLine - 1;
        }
        if (malformedTrackEntry || route.splineIds.size() < 2) {
            if (!passengerTimetable.stationIds.empty())
                aiTraffic.passengerTimetables.push_back(std::move(passengerTimetable));
            continue;
        }
        aiTraffic.timetableRoutes.push_back(std::move(route));
        if (!passengerTimetable.stationIds.empty()) {
            aiTraffic.passengerTimetables.push_back(std::move(passengerTimetable));
        }
    }

    for (const std::filesystem::path& linePath : lineFiles) {
        const std::vector<std::string> lineLines = splitLines(decodeText(linePath));
        MapAiTimetableCourse course;
        course.name = pathToUtf8(linePath.stem());
        for (std::size_t lineIndex = 0; lineIndex < lineLines.size(); ++lineIndex) {
            const std::string section = lowerAscii(trim(lineLines[lineIndex]));
            if (section == "[userallowed]") {
                course.userAllowed = true;
                continue;
            }
            if (section != "[addtrip]") {
                continue;
            }
            std::array<std::string, 3> fields;
            std::size_t next = lineIndex + 1;
            bool complete = true;
            for (std::string& field : fields) {
                while (next < lineLines.size() && trim(lineLines[next]).empty()) {
                    ++next;
                }
                if (next >= lineLines.size() || isSectionHeaderLine(trim(lineLines[next]))) {
                    complete = false;
                    break;
                }
                field = trim(lineLines[next++]);
            }
            int tripIndex = 0;
            double departureMinutes = 0.0;
            if (!complete || fields[0].empty() || !parseInteger(fields[1], tripIndex) ||
                !parseReal(fields[2], departureMinutes) || departureMinutes < 0.0) {
                diagnostics.warning(lineIndex + 1, "addtrip",
                                    "invalid or truncated timetable departure in " +
                                        pathToUtf8(linePath.filename()));
                lineIndex = next > 0 ? next - 1 : lineIndex;
                continue;
            }
            const std::string timetableKey = lowerAscii(fields[0]);
            const auto route =
                std::find_if(aiTraffic.timetableRoutes.begin(), aiTraffic.timetableRoutes.end(),
                             [&](const MapAiTimetableRoute& candidate) {
                                 return candidate.timetableKey == timetableKey;
                             });
            auto passengerTimetable = std::find_if(
                aiTraffic.passengerTimetables.begin(), aiTraffic.passengerTimetables.end(),
                [&](const MapAiPassengerTimetable& candidate) {
                    return candidate.timetableKey == timetableKey;
                });
            if (route == aiTraffic.timetableRoutes.end() &&
                passengerTimetable == aiTraffic.passengerTimetables.end()) {
                diagnostics.warning(lineIndex + 1, "addtrip",
                                    "departure references missing trip " + fields[0] + " in " +
                                        pathToUtf8(linePath.filename()));
            } else {
                tripIndex = std::max(0, tripIndex);
                course.trips.push_back({timetableKey, tripIndex, departureMinutes});
                if (route != aiTraffic.timetableRoutes.end()) {
                    route->scheduledDepartureMinutes.push_back(departureMinutes);
                    route->scheduledDepartureProfileIndices.push_back(tripIndex);
                }
                if (passengerTimetable != aiTraffic.passengerTimetables.end()) {
                    passengerTimetable->scheduledDepartureMinutes.push_back(departureMinutes);
                    passengerTimetable->scheduledDepartureProfileIndices.push_back(tripIndex);
                }
            }
            lineIndex = next > 0 ? next - 1 : lineIndex;
        }
        if (!course.trips.empty()) {
            aiTraffic.timetableCourses.push_back(std::move(course));
        }
    }
    const auto sortDepartures = [](std::vector<double>& minutes, std::vector<int>& profileIndices) {
        if (minutes.size() != profileIndices.size()) {
            throw std::logic_error("Timetable departures and profile indices lost alignment");
        }
        std::vector<std::pair<double, int>> departures;
        departures.reserve(minutes.size());
        for (std::size_t index = 0; index < minutes.size(); ++index) {
            departures.emplace_back(minutes[index], profileIndices[index]);
        }
        std::stable_sort(
            departures.begin(), departures.end(),
            [](const auto& left, const auto& right) { return left.first < right.first; });
        for (std::size_t index = 0; index < departures.size(); ++index) {
            minutes[index] = departures[index].first;
            profileIndices[index] = departures[index].second;
        }
    };
    for (MapAiTimetableRoute& route : aiTraffic.timetableRoutes) {
        sortDepartures(route.scheduledDepartureMinutes, route.scheduledDepartureProfileIndices);
    }
    for (MapAiPassengerTimetable& timetable : aiTraffic.passengerTimetables) {
        sortDepartures(timetable.scheduledDepartureMinutes,
                       timetable.scheduledDepartureProfileIndices);
        if (!mapAiPassengerTimetableHasResolvedLinks(aiTraffic, timetable)) {
            diagnostics.warning(
                0, "passenger_timetable",
                "timetable " + timetable.timetableKey +
                    " has stop IDs without unique Busstops.cfg and directed StnLinks.cfg matches");
            continue;
        }
        const bool hasDrivingRoute =
            std::any_of(aiTraffic.timetableRoutes.begin(), aiTraffic.timetableRoutes.end(),
                        [&](const MapAiTimetableRoute& route) {
                            return route.timetableKey == timetable.timetableKey;
                        });
        if (timetable.scheduledDepartureMinutes.empty()) {
            continue;
        }

        const std::vector<MapAiPassengerStopCall> calls =
            mapAiPassengerStopCalls(aiTraffic, timetable);
        const auto drivingRoute =
            std::find_if(aiTraffic.timetableRoutes.begin(), aiTraffic.timetableRoutes.end(),
                         [&](const MapAiTimetableRoute& route) {
                             return route.timetableKey == timetable.timetableKey;
                         });
        if (hasDrivingRoute) {
            if (drivingRoute == aiTraffic.timetableRoutes.end() ||
                !drivingRoute->stations.empty()) {
                continue;
            }
            if (drivingRoute->stationIds != timetable.stationIds || calls.size() < 2 ||
                calls.size() != timetable.stationIds.size() || drivingRoute->trackEntries.empty()) {
                diagnostics.warning(
                    0, "passenger_timetable",
                    "timetable " + timetable.timetableKey +
                        " has passenger stop IDs that cannot be mapped to its driving route");
                continue;
            }

            std::vector<std::size_t> linkStartIndices;
            linkStartIndices.reserve(calls.size() - 1);
            std::size_t earliestTrackEntry = 0;
            bool completeRoute = true;
            for (std::size_t stationIndex = 0; stationIndex + 1 < calls.size(); ++stationIndex) {
                const std::optional<std::size_t> linkIndex =
                    calls[stationIndex].outgoingStationLinkIndex;
                if (!linkIndex || *linkIndex >= aiTraffic.stationLinks.size()) {
                    completeRoute = false;
                    break;
                }
                const MapAiStationLink& link = aiTraffic.stationLinks[*linkIndex];
                if (link.entries.empty()) {
                    completeRoute = false;
                    break;
                }
                std::optional<std::size_t> linkStartIndex;
                if (link.entries.size() <= drivingRoute->trackEntries.size()) {
                    const std::size_t lastStart =
                        drivingRoute->trackEntries.size() - link.entries.size();
                    for (std::size_t start = earliestTrackEntry; start <= lastStart; ++start) {
                        const auto trackEntry =
                            drivingRoute->trackEntries.begin() + static_cast<std::ptrdiff_t>(start);
                        if (std::equal(
                                link.entries.begin(), link.entries.end(), trackEntry,
                                [](const MapAiStationLink::Entry& stationEntry,
                                   const MapAiTimetableRoute::TrackEntry& timetableEntry) {
                                    return stationEntry.splineId == timetableEntry.splineId &&
                                           stationEntry.pathIndex == timetableEntry.pathIndex &&
                                           stationEntry.tileIndex == timetableEntry.tileIndex;
                                })) {
                            linkStartIndex = start;
                            break;
                        }
                    }
                }
                if (!linkStartIndex) {
                    completeRoute = false;
                    break;
                }
                linkStartIndices.push_back(*linkStartIndex);
                earliestTrackEntry = *linkStartIndex + link.entries.size() - 1;
            }
            if (!completeRoute || linkStartIndices.size() + 1 != calls.size()) {
                diagnostics.warning(
                    0, "passenger_timetable",
                    "timetable " + timetable.timetableKey +
                        " station links do not align with its driving route track entries");
                continue;
            }

            std::vector<MapAiTimetableRoute::Station> stations;
            stations.reserve(calls.size());
            for (std::size_t stationIndex = 0; stationIndex < calls.size(); ++stationIndex) {
                const MapAiPassengerStopCall& call = calls[stationIndex];
                const MapAiBusStop& busStop = aiTraffic.busStops[*call.busStopIndex];
                MapAiTimetableRoute::Station station;
                station.id = call.stationId;
                station.name = busStop.name;
                station.tileIndex = busStop.tileIndex;
                if (stationIndex + 1 < calls.size()) {
                    station.trackEntryIndex =
                        checkedIndexToInt(linkStartIndices[stationIndex], "Track-entry index");
                } else {
                    const MapAiStationLink& incomingLink =
                        aiTraffic.stationLinks[*calls[stationIndex - 1].outgoingStationLinkIndex];
                    const std::size_t previousEntryIndex =
                        linkStartIndices.back() + incomingLink.entries.size() - 1;
                    station.trackEntryIndex =
                        checkedIndexToInt(previousEntryIndex + 1, "Track-entry index");
                    station.positionOnPreviousTrackEntryMeters =
                        drivingRoute->trackEntries[previousEntryIndex].lengthMeters;
                }
                stations.push_back(std::move(station));
            }
            drivingRoute->stations = std::move(stations);
            continue;
        }

        MapAiTimetableRoute route;
        route.timetableKey = timetable.timetableKey;
        route.tripName = timetable.tripName.empty() ? timetable.timetableKey : timetable.tripName;
        route.destinationName =
            timetable.destinationName.empty() ? timetable.timetableKey : timetable.destinationName;
        route.lineName = timetable.lineName.empty() ? timetable.timetableKey : timetable.lineName;
        route.stationCount = calls.size();
        route.stationIds = timetable.stationIds;
        route.scheduledDepartureMinutes = timetable.scheduledDepartureMinutes;
        route.scheduledDepartureProfileIndices = timetable.scheduledDepartureProfileIndices;
        route.profiles = timetable.profiles;
        bool completeRoute = true;
        for (std::size_t stationIndex = 0; stationIndex < calls.size(); ++stationIndex) {
            const MapAiPassengerStopCall& call = calls[stationIndex];
            const MapAiBusStop& busStop = aiTraffic.busStops[*call.busStopIndex];
            MapAiTimetableRoute::Station station;
            station.id = call.stationId;
            station.name = busStop.name;
            station.tileIndex = busStop.tileIndex;

            if (stationIndex + 1 < calls.size()) {
                const MapAiStationLink& link =
                    aiTraffic.stationLinks[*call.outgoingStationLinkIndex];
                if (link.entries.empty()) {
                    completeRoute = false;
                    break;
                }
                station.trackEntryIndex =
                    checkedIndexToInt(route.trackEntries.size(), "Track-entry index");
                for (const MapAiStationLink::Entry& entry : link.entries) {
                    route.trackEntries.push_back(
                        {entry.splineId, entry.pathIndex, entry.tileIndex, entry.lengthMeters});
                    if (route.splineIds.empty() || route.splineIds.back() != entry.splineId) {
                        route.splineIds.push_back(entry.splineId);
                    }
                }
            } else if (!route.trackEntries.empty()) {
                station.trackEntryIndex =
                    checkedIndexToInt(route.trackEntries.size(), "Track-entry index");
                station.positionOnPreviousTrackEntryMeters = route.trackEntries.back().lengthMeters;
            } else {
                completeRoute = false;
                break;
            }
            route.stations.push_back(std::move(station));
        }
        if (!completeRoute || route.splineIds.size() < 2 || route.trackEntries.size() < 2) {
            diagnostics.warning(
                0, "passenger_timetable",
                "timetable " + timetable.timetableKey +
                    " has resolved stops but insufficient ordered station-link path entries "
                    "for a scheduled driving route");
            continue;
        }
        aiTraffic.timetableRoutes.push_back(std::move(route));
    }

    for (MapAiTimetableRoute& route : aiTraffic.timetableRoutes) {
        if (route.profiles.empty()) {
            route.profiles.emplace_back();
        }
        const std::size_t stationCount = route.stationCount;
        std::vector<double> alongRouteMeters(stationCount, 0.0);
        std::vector<double> trackEntryStarts(route.trackEntries.size(), 0.0);
        double accumulatedTrackMeters = 0.0;
        for (std::size_t trackIndex = 0; trackIndex < route.trackEntries.size(); ++trackIndex) {
            trackEntryStarts[trackIndex] = accumulatedTrackMeters;
            const double length = route.trackEntries[trackIndex].lengthMeters;
            if (std::isfinite(length) && length > 0.0) {
                accumulatedTrackMeters += length;
            }
        }
        for (std::size_t stationIndex = 0; stationIndex < stationCount; ++stationIndex) {
            bool hasRoutePosition = false;
            if (stationIndex < route.stations.size()) {
                const auto position =
                    mapAiTimetableStationPosition(route, route.stations[stationIndex]);
                if (position && position->trackEntryIndex < trackEntryStarts.size()) {
                    alongRouteMeters[stationIndex] =
                        trackEntryStarts[position->trackEntryIndex] + position->offsetMeters;
                    hasRoutePosition = true;
                }
            }
            if (stationIndex > 0 && (!hasRoutePosition || alongRouteMeters[stationIndex] <=
                                                              alongRouteMeters[stationIndex - 1])) {
                alongRouteMeters[stationIndex] = alongRouteMeters[stationIndex - 1] + 500.0;
            }
        }

        for (MapAiTimetableProfile& profile : route.profiles) {
            std::vector<std::optional<double>> arrivals(stationCount);
            std::vector<std::optional<double>> departures(stationCount);
            const auto collectTimes = [&](const std::vector<std::pair<int, double>>& source,
                                          std::vector<std::optional<double>>& destination,
                                          const char* section) {
                for (const auto& [stationIndex, minutesAfterDeparture] : source) {
                    if (stationIndex < 0 ||
                        static_cast<std::size_t>(stationIndex) >= stationCount) {
                        diagnostics.warning(0, section,
                                            "timetable profile stop index is outside " +
                                                route.timetableKey);
                        continue;
                    }
                    destination[static_cast<std::size_t>(stationIndex)] =
                        minutesAfterDeparture * 60.0;
                }
            };
            collectTimes(profile.manualArrivalMinutes, arrivals, "profile_man_arr_time");
            collectTimes(profile.manualDepartureMinutes, departures, "profile_man_dep_time");

            const double durationSeconds =
                profile.durationMinutes > 0.0 && std::isfinite(profile.durationMinutes)
                    ? profile.durationMinutes * 60.0
                    : 600.0;
            if (stationCount > 0 && !arrivals[0] && !departures[0]) {
                departures[0] = 0.0;
            }
            if (stationCount > 1 && !arrivals.back() && !departures.back()) {
                arrivals.back() = durationSeconds;
            }
            std::vector<std::size_t> timedStations;
            for (std::size_t index = 0; index < stationCount; ++index) {
                if (arrivals[index] || departures[index]) {
                    timedStations.push_back(index);
                }
            }
            profile.arrivalOffsetSeconds.resize(stationCount);
            profile.departureOffsetSeconds.resize(stationCount);
            double lastDepartureSeconds = 0.0;
            for (std::size_t index = 0; index < stationCount; ++index) {
                double arrivalSeconds = 0.0;
                double departureSeconds = 0.0;
                if (arrivals[index] && departures[index]) {
                    arrivalSeconds = *arrivals[index];
                    departureSeconds = std::max(*departures[index], arrivalSeconds);
                } else if (arrivals[index]) {
                    arrivalSeconds = departureSeconds = *arrivals[index];
                } else if (departures[index]) {
                    arrivalSeconds = departureSeconds = *departures[index];
                } else {
                    const auto previous =
                        std::find_if(timedStations.rbegin(), timedStations.rend(),
                                     [index](std::size_t station) { return station < index; });
                    const auto next =
                        std::find_if(timedStations.begin(), timedStations.end(),
                                     [index](std::size_t station) { return station > index; });
                    if (previous != timedStations.rend() && next != timedStations.end()) {
                        const std::size_t previousIndex = *previous;
                        const std::size_t nextIndex = *next;
                        const double startSeconds = departures[previousIndex]
                                                        ? *departures[previousIndex]
                                                        : arrivals[previousIndex].value_or(0.0);
                        const double endSeconds = arrivals[nextIndex]
                                                      ? *arrivals[nextIndex]
                                                      : departures[nextIndex].value_or(0.0);
                        const double distanceSpan =
                            alongRouteMeters[nextIndex] - alongRouteMeters[previousIndex];
                        const double fraction =
                            distanceSpan > 0.0
                                ? (alongRouteMeters[index] - alongRouteMeters[previousIndex]) /
                                      distanceSpan
                                : 0.0;
                        arrivalSeconds = startSeconds + (endSeconds - startSeconds) * fraction;
                        departureSeconds = arrivalSeconds;
                    } else if (previous != timedStations.rend()) {
                        const std::size_t previousIndex = *previous;
                        arrivalSeconds = departureSeconds =
                            departures[previousIndex] ? *departures[previousIndex]
                                                      : arrivals[previousIndex].value_or(0.0);
                    } else if (next != timedStations.end()) {
                        const std::size_t nextIndex = *next;
                        arrivalSeconds = departureSeconds =
                            arrivals[nextIndex] ? *arrivals[nextIndex]
                                                : departures[nextIndex].value_or(0.0);
                    }
                }
                arrivalSeconds = std::max(arrivalSeconds, lastDepartureSeconds);
                departureSeconds = std::max(departureSeconds, arrivalSeconds);
                profile.arrivalOffsetSeconds[index] = arrivalSeconds;
                profile.departureOffsetSeconds[index] = departureSeconds;
                lastDepartureSeconds = departureSeconds;
            }
        }
    }
}

void parseDriverListFile(const std::filesystem::path& path, MapAiTrafficData& aiTraffic) {
    const std::vector<std::string> lines = splitLines(decodeText(path));
    for (const std::string& sourceLine : lines) {
        const std::string line = trim(sourceLine);
        if (!isIgnoredAiListLine(line) && hasExtension(line, ".hum")) {
            aiTraffic.driverModelPaths.push_back(line);
        }
    }
}

void parseHumanListFile(const std::filesystem::path& path, MapAiTrafficData& aiTraffic) {
    const std::vector<std::string> lines = splitLines(decodeText(path));
    for (const std::string& sourceLine : lines) {
        const std::string line = trim(sourceLine);
        if (!isIgnoredAiListLine(line) && hasExtension(line, ".hum")) {
            aiTraffic.humanModelPaths.push_back(line);
        }
    }
}

std::uint32_t readU32(const std::vector<unsigned char>& bytes, std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset]) |
           (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 16U) |
           (static_cast<std::uint32_t>(bytes[offset + 3]) << 24U);
}

void parseTileFilename(const std::string& filename, int& x, int& y) {
    constexpr std::string_view prefix = "tile_";
    constexpr std::string_view suffix = ".map";
    if (!filename.starts_with(prefix) || !filename.ends_with(suffix)) {
        throw std::runtime_error("Unexpected tile filename in [map]: " + filename);
    }
    const std::string_view coordinates(filename.data() + prefix.size(),
                                       filename.size() - prefix.size() - suffix.size());
    const std::size_t separator = coordinates.find('_');
    if (separator == std::string_view::npos ||
        !parseInteger(trim(coordinates.substr(0, separator)), x) ||
        !parseInteger(trim(coordinates.substr(separator + 1)), y)) {
        throw std::runtime_error("Invalid tile coordinates in [map] filename: " + filename);
    }
}

std::unordered_map<std::string, std::vector<MapGroundTextureSidecar>>
discoverGroundTextureSidecars(const std::filesystem::path& mapDirectory) {
    std::unordered_map<std::string, std::vector<MapGroundTextureSidecar>> sidecarsByTile;
    const std::filesystem::path sidecarDirectory = mapDirectory / "texture" / "map";
    std::error_code error;
    std::filesystem::directory_iterator iterator(sidecarDirectory, error);
    if (error) {
        return sidecarsByTile;
    }
    const std::filesystem::directory_iterator end;
    for (; iterator != end; iterator.increment(error)) {
        if (error) {
            break;
        }
        std::error_code statusError;
        if (!iterator->is_regular_file(statusError) || statusError) {
            continue;
        }
        const std::string candidate = pathToUtf8(iterator->path().filename());
        constexpr std::string_view extension = ".dds";
        if (!candidate.starts_with("tile_") || !candidate.ends_with(extension)) {
            continue;
        }
        const std::string stem = candidate.substr(0, candidate.size() - extension.size());
        const std::size_t separator = stem.rfind('.');
        if (separator == std::string::npos) {
            continue;
        }
        const std::string tileFilename = stem.substr(0, separator);
        const std::string suffix = stem.substr(separator + 1);
        if (!tileFilename.ends_with(".map")) {
            continue;
        }
        if (suffix.empty() || !std::all_of(suffix.begin(), suffix.end(), [](unsigned char value) {
                return std::isdigit(value) != 0;
            })) {
            continue;
        }
        sidecarsByTile[tileFilename].push_back({suffix, iterator->path()});
    }
    for (auto& [tileFilename, sidecars] : sidecarsByTile) {
        static_cast<void>(tileFilename);
        std::sort(sidecars.begin(), sidecars.end(),
                  [](const MapGroundTextureSidecar& left, const MapGroundTextureSidecar& right) {
                      return left.ordinalSuffix < right.ordinalSuffix;
                  });
    }
    return sidecarsByTile;
}

struct MapChronoDateRange {
    std::chrono::year_month_day start;
    std::optional<std::chrono::year_month_day> end;
};

std::optional<std::chrono::year_month_day>
parseChronoDate(std::string_view value, const std::filesystem::path& configPath) {
    int date = 0;
    if (value.size() != 8 ||
        !std::all_of(value.begin(), value.end(),
                     [](unsigned char character) { return std::isdigit(character) != 0; }) ||
        !parseInteger(std::string(value), date)) {
        throw std::runtime_error("Invalid Chrono date in " + pathToUtf8(configPath) +
                                 "; expected YYYYMMDD");
    }
    const int year = date / 10000;
    const unsigned month = static_cast<unsigned>((date / 100) % 100);
    const unsigned day = static_cast<unsigned>(date % 100);
    const std::chrono::year_month_day parsed{std::chrono::year{year}, std::chrono::month{month},
                                             std::chrono::day{day}};
    if (!parsed.ok()) {
        throw std::runtime_error("Invalid Chrono calendar date in " + pathToUtf8(configPath));
    }
    return parsed;
}

std::optional<MapChronoDateRange> loadMapChronoDateRange(const std::filesystem::path& configPath,
                                                         ConfigurationDiagnostics& diagnostics) {
    const std::vector<std::string> lines = splitLines(decodeText(configPath));
    std::optional<std::chrono::year_month_day> start;
    std::optional<std::chrono::year_month_day> end;
    for (std::size_t index = 0; index < lines.size(); ++index) {
        const std::string section = lowerAscii(trim(lines[index]));
        if (section != "[startdate]" && section != "[enddate]") {
            continue;
        }
        std::size_t valueIndex = index + 1;
        while (valueIndex < lines.size() && trim(lines[valueIndex]).empty()) {
            ++valueIndex;
        }
        if (valueIndex >= lines.size() || isSectionHeaderLine(trim(lines[valueIndex]))) {
            diagnostics.warning(index + 1, section.substr(1, section.size() - 2),
                                "missing Chrono date; scenario is inactive");
            return std::nullopt;
        }
        try {
            const auto parsed = parseChronoDate(trim(lines[valueIndex]), configPath);
            if (section == "[startdate]") {
                start = parsed;
            } else {
                end = parsed;
            }
        } catch (const std::exception& error) {
            diagnostics.warning(index + 1, section.substr(1, section.size() - 2),
                                std::string(error.what()) + "; scenario is inactive");
            return std::nullopt;
        }
        index = valueIndex;
    }
    if (!start.has_value()) {
        diagnostics.warning(0, "startdate",
                            "Chrono scenario has no mandatory [startdate]; scenario is inactive");
        return std::nullopt;
    }
    if (end.has_value() && std::chrono::sys_days{*end} < std::chrono::sys_days{*start}) {
        diagnostics.warning(0, "enddate",
                            "Chrono [enddate] precedes [startdate]; scenario is inactive");
        return std::nullopt;
    }
    return MapChronoDateRange{*start, end};
}

bool mapChronoDateRangeContains(const MapChronoDateRange& range, std::chrono::year_month_day date) {
    const std::chrono::sys_days current{date};
    return current >= std::chrono::sys_days{range.start} &&
           (!range.end.has_value() || current <= std::chrono::sys_days{*range.end});
}

std::vector<std::filesystem::path> mapChronoConfigPaths(const std::filesystem::path& chronoRoot) {
    std::error_code error;
    const bool exists = std::filesystem::exists(chronoRoot, error);
    if (error)
        throw std::filesystem::filesystem_error("Could not inspect map Chrono directory",
                                                chronoRoot, error);
    if (!exists)
        return {};
    if (!std::filesystem::is_directory(chronoRoot, error) || error)
        throw std::filesystem::filesystem_error("Map Chrono path is not a readable directory",
                                                chronoRoot, error);
    std::vector<std::filesystem::path> paths;
    std::filesystem::recursive_directory_iterator iterator(chronoRoot, error);
    const std::filesystem::recursive_directory_iterator end;
    for (; iterator != end; iterator.increment(error)) {
        if (error)
            throw std::filesystem::filesystem_error("Could not enumerate map Chrono directory",
                                                    chronoRoot, error);
        std::error_code statusError;
        if (iterator->is_regular_file(statusError) && !statusError &&
            lowerAscii(pathToUtf8(iterator->path().filename())) == "chrono.cfg")
            paths.push_back(iterator->path());
    }
    if (error)
        throw std::filesystem::filesystem_error("Could not enumerate map Chrono directory",
                                                chronoRoot, error);
    std::sort(paths.begin(), paths.end(), [](const auto& left, const auto& right) {
        return lowerAscii(pathToUtf8(left)) < lowerAscii(pathToUtf8(right));
    });
    return paths;
}

std::chrono::year_month_day checkedChronoDate(int dateYyyyMmDd) {
    if (dateYyyyMmDd < 0 || dateYyyyMmDd > 99'999'999)
        throw std::invalid_argument("Map Chrono date must be a valid YYYYMMDD calendar date");
    const std::chrono::year_month_day date{
        std::chrono::year{dateYyyyMmDd / 10000},
        std::chrono::month{static_cast<unsigned>((dateYyyyMmDd / 100) % 100)},
        std::chrono::day{static_cast<unsigned>(dateYyyyMmDd % 100)}};
    if (!date.ok())
        throw std::invalid_argument("Map Chrono date must be a valid YYYYMMDD calendar date");
    return date;
}

std::vector<std::string> chronoDeactivatedLines(const std::filesystem::path& configPath) {
    const auto lines = splitLines(decodeText(configPath));
    std::vector<std::string> result;
    bool inSection = false;
    for (const auto& rawLine : lines) {
        const std::string line = trim(rawLine);
        if (isSectionHeaderLine(line)) {
            inSection = lowerAscii(line) == "[deactivate_lines]";
        } else if (inSection && !line.empty()) {
            result.push_back(lowerAscii(line));
        }
    }
    return result;
}

void applyChronoTileSidecars(
    MapTileReference& tile, const std::filesystem::path& variantPath,
    const std::unordered_map<std::string, std::vector<MapGroundTextureSidecar>>& sidecarsByTile) {
    const std::filesystem::path terrainPath = pathWithSuffix(variantPath, ".terrain");
    if (std::filesystem::is_regular_file(terrainPath)) {
        tile.terrainPath = terrainPath;
        tile.hasTerrainFile = true;
    }
    const std::filesystem::path waterPath = pathWithSuffix(variantPath, ".water");
    if (std::filesystem::is_regular_file(waterPath)) {
        tile.waterPath = waterPath;
        tile.hasWaterFile = true;
    }
    const std::filesystem::path lightmapPath = pathWithSuffix(variantPath, ".LM.bmp");
    if (std::filesystem::is_regular_file(lightmapPath)) {
        tile.lightmapPath = lightmapPath;
        tile.hasLightmapFile = true;
    }
    const auto sidecars = sidecarsByTile.find(lowerAscii(pathToUtf8(variantPath.filename())));
    if (sidecars != sidecarsByTile.end()) {
        tile.groundTextureSidecars = sidecars->second;
    }
}

struct MapChronoSelectorEdits {
    bool deleteTarget = false;
    std::optional<std::size_t> deleteLine;
    std::optional<std::string> assetPath;
    std::optional<std::vector<std::string>> labels;
    std::vector<MapTileRule> rules;
};

MapChronoSelectorEdits parseMapChronoSelectorEdits(
    const MapChronoSelector& selector, const std::filesystem::path& path,
    ConfigurationDiagnostics& diagnostics) {
    MapChronoSelectorEdits edits;
    const auto warn = [&](std::size_t line, const std::string& keyword,
                          const std::string& message) {
        openbus::config::logUnhandledConfigurationTag("Chrono map tile", path, line, keyword,
                                                       message);
        diagnostics.warning(line, keyword, message);
    };
    for (std::size_t index = 0; index < selector.followingLines.size(); ++index) {
        const std::string section = trim(selector.followingLines[index]);
        const std::size_t line = selector.idLine + index + 1;
        if (section.empty()) {
            continue;
        }
        if (isSectionHeaderLine(section)) {
            const std::string keyword = section.substr(1, section.size() - 2);
            openbus::config::ConfigurationTagStatus status =
                openbus::config::ConfigurationTagStatus::Unknown;
            std::string detail = "unknown Chrono selector operation";
            if (section == "[delete]") {
                status = openbus::config::ConfigurationTagStatus::Active;
                detail = "selector deletion is applied when target references permit it";
            } else if (section == "[typ]") {
                status = openbus::config::ConfigurationTagStatus::Active;
                detail = "selector asset-path override has runtime handling";
            } else if (section == "[relabel]" && selector.keyword == "selobject") {
                status = openbus::config::ConfigurationTagStatus::Active;
                detail = "object selector labels have runtime handling";
            } else if (section == "[relabel]") {
                status = openbus::config::ConfigurationTagStatus::Ignored;
                detail = "Chrono spline relabeling is not implemented";
            } else if ((section == "[rule]" || section == "[kill_rule]") &&
                       selector.keyword == "selspline") {
                status = openbus::config::ConfigurationTagStatus::Active;
                detail = "spline selector traffic rules have runtime handling";
            } else if (section == "[rule]" || section == "[kill_rule]") {
                status = openbus::config::ConfigurationTagStatus::Ignored;
                detail = "Chrono object selector rule edits are not implemented";
            }
            openbus::config::logConfigurationTagStatus("Chrono map tile", path, line, keyword,
                                                       status, detail);
        }
        if (section == "[delete]") {
            edits.deleteTarget = true;
            edits.deleteLine = line;
        } else if (section == "[typ]" || section == "[relabel]" ||
                   section == "[rule]" || section == "[kill_rule]") {
            std::size_t valueIndex = index + 1;
            while (valueIndex < selector.followingLines.size() &&
                   trim(selector.followingLines[valueIndex]).empty()) {
                ++valueIndex;
            }
            if (valueIndex >= selector.followingLines.size() ||
                isSectionHeaderLine(trim(selector.followingLines[valueIndex]))) {
                warn(line, section.substr(1, section.size() - 2),
                     "Chrono selector operation is missing its value");
                continue;
            }
            if (section == "[typ]") {
                const std::string assetPath = trim(selector.followingLines[valueIndex]);
                if (assetPath.empty()) {
                    warn(line, "typ", "Chrono object type path is empty");
                } else {
                    edits.assetPath = assetPath;
                }
            } else {
                if (section == "[rule]" || section == "[kill_rule]") {
                    if (selector.followingLines.size() - valueIndex < 4) {
                        warn(line, section.substr(1, section.size() - 2),
                             "Chrono rule is missing one or more of its four fields");
                        continue;
                    }
                    MapTileRule rule;
                    rule.kill = section == "[kill_rule]";
                    bool valid = true;
                    for (std::size_t fieldIndex = 0; fieldIndex < rule.fields.size();
                         ++fieldIndex) {
                        const std::string& field =
                            selector.followingLines[valueIndex + fieldIndex];
                        if (isSectionHeaderLine(trim(field))) {
                            warn(line, section.substr(1, section.size() - 2),
                                 "Chrono rule is truncated before its four fields");
                            valid = false;
                            break;
                        }
                        rule.fields[fieldIndex] = field;
                    }
                    if (valid) {
                        if (selector.keyword == "selspline") {
                            edits.rules.push_back(std::move(rule));
                        } else {
                            warn(line, section.substr(1, section.size() - 2),
                                 "Chrono object selector rule edits are not implemented");
                        }
                        index = valueIndex + 3;
                    }
                    continue;
                }
                std::size_t labelCount = 0;
                const std::string countText = trim(selector.followingLines[valueIndex]);
                if (!parseSize(countText, labelCount) || labelCount > 4096 ||
                    labelCount > selector.followingLines.size() - valueIndex - 1) {
                    warn(line, "relabel", "Chrono relabel count is invalid or truncated");
                    index = valueIndex;
                    continue;
                }
                std::vector<std::string> labels;
                labels.reserve(labelCount);
                for (std::size_t labelIndex = 0; labelIndex < labelCount; ++labelIndex) {
                    labels.push_back(selector.followingLines[valueIndex + 1 + labelIndex]);
                }
                edits.labels = std::move(labels);
                index = valueIndex + labelCount;
                continue;
            }
            index = valueIndex;
        } else if (isSectionHeaderLine(section)) {
            warn(line, section.substr(1, section.size() - 2),
                 "Chrono selector operation is not implemented");
        }
    }
    return edits;
}

void applyMapChronoSelector(MapTileData& tile, const MapChronoSelector& selector,
                            const std::filesystem::path& path) {
    if (selector.keyword != "selobject" && selector.keyword != "selspline") {
        const std::string message = "Chrono selector operations are not implemented";
        openbus::config::logUnhandledConfigurationTag("Chrono map tile", path,
                                                       selector.sectionLine, selector.keyword,
                                                       message);
        tile.diagnostics.warning(selector.sectionLine, selector.keyword, message);
        return;
    }
    if (!selector.id.has_value()) {
        return;
    }

    const MapChronoSelectorEdits edits =
        parseMapChronoSelectorEdits(selector, path, tile.diagnostics);
    const int id = *selector.id;
    if (selector.keyword == "selspline") {
        const auto matches = [id](const MapSplinePlacement& spline) {
            return spline.splineId == id;
        };
        const auto match = std::find_if(tile.splines.begin(), tile.splines.end(), matches);
        const std::size_t matchCount =
            static_cast<std::size_t>(std::count_if(tile.splines.begin(), tile.splines.end(),
                                                   matches));
        if (matchCount != 1) {
            tile.diagnostics.warning(
                selector.sectionLine, selector.keyword,
                matchCount == 0 ? "selector ID did not match a tile spline"
                                : "selector ID matched multiple tile splines");
            return;
        }
        if (edits.deleteTarget) {
            const std::size_t splineIndex =
                static_cast<std::size_t>(std::distance(tile.splines.begin(), match));
            const bool hasLinkReferences =
                match->previousSplineId != 0 || match->nextSplineId != 0 ||
                std::any_of(tile.splines.begin(), tile.splines.end(),
                            [id](const MapSplinePlacement& spline) {
                                return spline.splineId != id &&
                                       (spline.previousSplineId == id ||
                                        spline.nextSplineId == id);
                            });
            const bool hasAttachments =
                std::any_of(tile.splineAttachments.begin(), tile.splineAttachments.end(),
                            [splineIndex](const MapSplineAttachment& attachment) {
                                return attachment.splineIndex >= 0 &&
                                       static_cast<std::size_t>(attachment.splineIndex) ==
                                           splineIndex;
                            });
            if (hasLinkReferences || hasAttachments) {
                const std::size_t line = edits.deleteLine.value_or(selector.sectionLine);
                const std::string message =
                    "Chrono spline deletion is not applied because the spline has link "
                    "references or attached placements";
                openbus::config::logUnhandledConfigurationTag("Chrono map tile", path, line,
                                                               "delete", message);
                tile.diagnostics.warning(line, "delete", message);
            } else {
                tile.splines.erase(match);
                for (MapSplineAttachment& attachment : tile.splineAttachments) {
                    if (attachment.splineIndex >= 0 &&
                        static_cast<std::size_t>(attachment.splineIndex) > splineIndex) {
                        --attachment.splineIndex;
                    }
                }
                return;
            }
        }
        if (edits.labels.has_value()) {
            tile.diagnostics.warning(selector.sectionLine, "relabel",
                                     "Chrono spline relabeling is not implemented");
        }
        if (!edits.assetPath.has_value() && edits.rules.empty()) {
            if (!edits.deleteTarget && !edits.labels.has_value()) {
                tile.diagnostics.warning(selector.sectionLine, selector.keyword,
                                         "selector has no supported spline override");
            }
            return;
        }
        if (edits.assetPath.has_value()) {
            match->assetPath = *edits.assetPath;
        }
        match->rules.insert(match->rules.end(), edits.rules.begin(), edits.rules.end());
        return;
    }

    const auto matches = [id](const auto& placement) { return placement.id == id; };
    const std::size_t matchCount =
        static_cast<std::size_t>(std::count_if(tile.sceneryObjects.begin(),
                                               tile.sceneryObjects.end(), matches)) +
        static_cast<std::size_t>(std::count_if(tile.attachedObjects.begin(),
                                               tile.attachedObjects.end(), matches)) +
        static_cast<std::size_t>(std::count_if(tile.splineAttachments.begin(),
                                               tile.splineAttachments.end(), matches));
    if (matchCount != 1) {
        tile.diagnostics.warning(
            selector.sectionLine, selector.keyword,
            matchCount == 0 ? "selector ID did not match a base tile object"
                            : "selector ID matched multiple base tile objects");
        return;
    }
    if (!edits.deleteTarget && !edits.assetPath.has_value() && !edits.labels.has_value() &&
        edits.rules.empty()) {
        tile.diagnostics.warning(selector.sectionLine, selector.keyword,
                                 "selector has no supported object override");
        return;
    }
    if (edits.deleteTarget) {
        std::erase_if(tile.sceneryObjects, matches);
        std::erase_if(tile.attachedObjects, matches);
        std::erase_if(tile.splineAttachments, matches);
        return;
    }
    for (MapSceneryPlacement& object : tile.sceneryObjects) {
        if (!matches(object)) {
            continue;
        }
        if (edits.assetPath.has_value()) {
            object.assetPath = *edits.assetPath;
        }
        if (edits.labels.has_value()) {
            object.labels = *edits.labels;
            object.trailingField = std::to_string(object.labels.size());
        }
        object.rules.insert(object.rules.end(), edits.rules.begin(), edits.rules.end());
    }
    for (MapAttachedObject& object : tile.attachedObjects) {
        if (!matches(object)) {
            continue;
        }
        if (edits.assetPath.has_value()) {
            object.assetPath = *edits.assetPath;
        }
        if (edits.labels.has_value()) {
            object.labels = *edits.labels;
            object.labelCount = std::to_string(object.labels.size());
        }
        object.rules.insert(object.rules.end(), edits.rules.begin(), edits.rules.end());
    }
    for (MapSplineAttachment& attachment : tile.splineAttachments) {
        if (!matches(attachment)) {
            continue;
        }
        if (edits.assetPath.has_value()) {
            attachment.assetPath = *edits.assetPath;
        }
        if (edits.labels.has_value()) {
            attachment.labels = *edits.labels;
        }
        attachment.rules.insert(attachment.rules.end(), edits.rules.begin(), edits.rules.end());
    }
}

} // namespace

std::size_t applyMapChronoTimetables(MapDefinition& map, int dateYyyyMmDd) {
    const auto date = checkedChronoDate(dateYyyyMmDd);
    struct ActiveScenario {
        std::chrono::year_month_day start;
        std::filesystem::path config;
    };
    std::vector<ActiveScenario> scenarios;
    for (const auto& config : mapChronoConfigPaths(map.rootPath / "Chrono")) {
        const auto range = loadMapChronoDateRange(config, map.diagnostics);
        if (range && mapChronoDateRangeContains(*range, date))
            scenarios.push_back({range->start, config});
    }
    std::stable_sort(scenarios.begin(), scenarios.end(), [](const auto& left, const auto& right) {
        return std::chrono::sys_days{left.start} < std::chrono::sys_days{right.start};
    });
    std::vector<TimetableSource> sources{{map.rootPath / "TTData", {}}};
    std::size_t applied = 0;
    for (const auto& scenario : scenarios) {
        auto deactivated = chronoDeactivatedLines(scenario.config);
        const auto directory = scenario.config.parent_path() / "TTData";
        if (!deactivated.empty() || std::filesystem::is_directory(directory)) {
            sources.push_back({directory, std::move(deactivated)});
            ++applied;
        }
    }
    MapAiTrafficData timetables;
    parseAiTimetableRoutes(sources, timetables, map.diagnostics);
    map.aiTraffic.timetableRoutes = std::move(timetables.timetableRoutes);
    map.aiTraffic.timetableCourses = std::move(timetables.timetableCourses);
    map.aiTraffic.busStops = std::move(timetables.busStops);
    map.aiTraffic.stationLinks = std::move(timetables.stationLinks);
    map.aiTraffic.passengerTimetables = std::move(timetables.passengerTimetables);
    return applied;
}

std::size_t applyMapChronoTiles(MapDefinition& map, int dateYyyyMmDd) {
    const auto date = checkedChronoDate(dateYyyyMmDd);
    const std::filesystem::path chronoRoot = map.rootPath / "Chrono";
    const auto configPaths = mapChronoConfigPaths(chronoRoot);

    for (MapTileReference& tile : map.tiles) {
        if (!tile.chronoBaseAssets.has_value()) {
            continue;
        }
        const MapChronoBaseAssets& base = *tile.chronoBaseAssets;
        tile.textPath = base.textPath;
        tile.terrainPath = base.terrainPath;
        tile.hasTerrainFile = base.hasTerrainFile;
        tile.waterPath = base.waterPath;
        tile.hasWaterFile = base.hasWaterFile;
        tile.lightmapPath = base.lightmapPath;
        tile.hasLightmapFile = base.hasLightmapFile;
        tile.groundTextureSidecars = base.groundTextureSidecars;
        tile.chronoOverlayPaths.clear();
    }

    std::unordered_map<std::string, std::size_t> tileIndexByFilename;
    for (std::size_t index = 0; index < map.tiles.size(); ++index) {
        tileIndexByFilename.emplace(lowerAscii(pathToUtf8(map.tiles[index].textPath.filename())),
                                    index);
    }
    std::size_t appliedTileCount = 0;
    for (const std::filesystem::path& configPath : configPaths) {
        const std::optional<MapChronoDateRange> range =
            loadMapChronoDateRange(configPath, map.diagnostics);
        if (!range.has_value() || !mapChronoDateRangeContains(*range, date)) {
            continue;
        }
        const std::filesystem::path scenarioDirectory = configPath.parent_path();
        const auto scenarioSidecars = discoverGroundTextureSidecars(scenarioDirectory);
        std::error_code directoryError;
        for (std::filesystem::directory_iterator tileIterator(scenarioDirectory, directoryError),
             tileEnd;
             !directoryError && tileIterator != tileEnd; tileIterator.increment(directoryError)) {
            std::error_code statusError;
            if (!tileIterator->is_regular_file(statusError) || statusError ||
                lowerAscii(pathToUtf8(tileIterator->path().extension())) != ".map") {
                continue;
            }
            const std::string tileFilename =
                lowerAscii(pathToUtf8(tileIterator->path().filename()));
            const auto tileIndex = tileIndexByFilename.find(tileFilename);
            if (tileIndex == tileIndexByFilename.end()) {
                continue;
            }
            MapTileReference& tile = map.tiles[tileIndex->second];
            if (!tile.chronoBaseAssets.has_value()) {
                tile.chronoBaseAssets = MapChronoBaseAssets{
                    tile.textPath,          tile.terrainPath, tile.hasTerrainFile,
                    tile.waterPath,         tile.hasWaterFile, tile.lightmapPath,
                    tile.hasLightmapFile,   tile.groundTextureSidecars};
            }
            tile.chronoOverlayPaths.push_back(tileIterator->path());
            tile.textPath = tileIterator->path();
            applyChronoTileSidecars(tile, tile.textPath, scenarioSidecars);
            ++appliedTileCount;
        }
        if (directoryError) {
            throw std::filesystem::filesystem_error("Could not enumerate Chrono scenario tiles",
                                                    scenarioDirectory, directoryError);
        }
    }
    return appliedTileCount;
}

MapDefinition loadMapDefinition(const std::filesystem::path& mapDirectory) {
    openbus::rendering::TraceScope trace("map", "MapConfigLoader::loadMapDefinition");
    const auto loadStart = std::chrono::steady_clock::now();
    Logger mapLog("Map");
    MapDefinition definition;
    definition.rootPath = mapDirectory;
    const std::filesystem::path globalPath = mapDirectory / "global.cfg";
    mapLog.Log("Reading manifest: " + pathToUtf8(globalPath));
    std::vector<std::string> lines;
    {
        openbus::rendering::TraceScope trace("map", "MapConfigLoader.readGlobalCfg");
        lines = splitLines(decodeText(globalPath));
    }
    mapLog.Log("Discovering ground-texture sidecars in " + pathToUtf8(mapDirectory));
    std::unordered_map<std::string, std::vector<MapGroundTextureSidecar>>
        groundTextureSidecarsByTile;
    {
        openbus::rendering::TraceScope trace("map",
                                             "MapConfigLoader.discoverGroundTextureSidecars");
        groundTextureSidecarsByTile = discoverGroundTextureSidecars(mapDirectory);
    }
    std::unordered_map<std::uint64_t, std::size_t> tileIndicesByCoordinates;
    constexpr std::size_t missingTileIndex = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> tileOrdinalToUniqueIndex;
    std::size_t cursor = 0;
    const auto isSectionHeader = [](const std::string& value) {
        return value.size() > 2 && value.front() == '[' && value.back() == ']';
    };
    const auto readOptionalValue = [&]() {
        if (cursor >= lines.size() || isSectionHeader(trim(lines[cursor]))) {
            return std::string{};
        }
        return trim(lines[cursor++]);
    };
    const auto readMetadataFields = [&](auto& fields, const std::string& section) {
        for (std::string& field : fields) {
            field = readField(lines, cursor, globalPath, section);
        }
    };

    openbus::rendering::TraceScope manifestTrace("map", "MapConfigLoader.parseGlobalCfg");
    static const std::unordered_set<std::string> knownGlobalSections = {
        "[name]",          "[friendlyname]",       "[version]",
        "[NextIDCode]",    "[description]",        "[worldcoordinates]",
        "[dynhelperactive]", "[realrail]",          "[LHT]",
        "[backgroundimage]", "[mapcam]",            "[moneysystem]",
        "[ticketpack]",    "[repair_time_min]",    "[years]",
        "[realyearoffset]", "[standarddepot]",     "[addseason]",
        "[trafficdensity_road]", "[trafficdensity_passenger]", "[groundtex]",
        "[entrypoints]",   "[map]"};
    for (; cursor < lines.size();) {
        const std::size_t sectionLine = cursor;
        const std::string section = trim(lines[cursor]);
        ++cursor;
        if (isSectionHeader(section)) {
            const std::string keyword = section.substr(1, section.size() - 2);
            if (knownGlobalSections.contains(section)) {
                const char* ignoredReason = nullptr;
                if (section == "[version]") {
                    ignoredReason = "map version is retained but has no runtime behavior";
                } else if (section == "[NextIDCode]") {
                    ignoredReason = "next object ID is retained for editor use but not applied";
                } else if (section == "[description]") {
                    ignoredReason = "description is retained but not displayed at runtime";
                } else if (section == "[dynhelperactive]") {
                    ignoredReason = "dynamic-helper activation is not implemented";
                } else if (section == "[realrail]") {
                    ignoredReason = "real-rail behavior is not implemented";
                } else if (section == "[backgroundimage]") {
                    ignoredReason = "background image settings are retained but not rendered";
                } else if (section == "[mapcam]") {
                    ignoredReason = "map-camera settings are retained but not applied";
                } else if (section == "[repair_time_min]") {
                    ignoredReason = "repair-time setting is retained but not applied";
                } else if (section == "[years]") {
                    ignoredReason = "map years are retained but not applied";
                } else if (section == "[realyearoffset]") {
                    ignoredReason = "real-year offset is retained but not applied";
                } else if (section == "[standarddepot]") {
                    ignoredReason = "standard depot is retained but not selected at runtime";
                } else if (section == "[addseason]") {
                    ignoredReason = "season metadata is retained but seasonal switching is not implemented";
                }
                openbus::config::logConfigurationTagStatus(
                    "global.cfg", globalPath, sectionLine + 1, keyword,
                    ignoredReason == nullptr
                        ? openbus::config::ConfigurationTagStatus::Active
                        : openbus::config::ConfigurationTagStatus::Ignored,
                    ignoredReason == nullptr ? "global map record has runtime handling"
                                             : ignoredReason);
            } else {
                openbus::config::logConfigurationTagStatus(
                    "global.cfg", globalPath, sectionLine + 1, keyword,
                    openbus::config::ConfigurationTagStatus::Unknown,
                    "unknown global.cfg section");
            }
        }
        if (section == "[name]") {
            definition.metadata.name = readOptionalValue();
        } else if (section == "[friendlyname]") {
            definition.metadata.friendlyName = readOptionalValue();
        } else if (section == "[version]") {
            definition.metadata.version = readOptionalValue();
        } else if (section == "[NextIDCode]") {
            definition.metadata.nextIdCode = readOptionalValue();
        } else if (section == "[description]") {
            bool firstLine = true;
            bool foundEnd = false;
            while (cursor < lines.size()) {
                if (trim(lines[cursor]) == "[end]") {
                    ++cursor;
                    foundEnd = true;
                    break;
                }
                if (!firstLine) {
                    definition.metadata.description.push_back('\n');
                }
                definition.metadata.description += lines[cursor++];
                firstLine = false;
            }
            if (!foundEnd) {
                throw std::runtime_error("Unterminated [description] in " + pathToUtf8(globalPath));
            }
        } else if (section == "[worldcoordinates]") {
            definition.metadata.hasWorldCoordinates = true;
        } else if (section == "[dynhelperactive]") {
            definition.metadata.hasDynamicHelperActive = true;
        } else if (section == "[realrail]") {
            definition.metadata.hasRealRail = true;
        } else if (section == "[LHT]") {
            definition.metadata.hasLeftHandTraffic = true;
        } else if (section == "[backgroundimage]") {
            readMetadataFields(definition.metadata.backgroundImage, "backgroundimage");
            definition.metadata.hasBackgroundImage = true;
        } else if (section == "[mapcam]") {
            readMetadataFields(definition.metadata.mapCamera, "mapcam");
            definition.metadata.hasMapCamera = true;
        } else if (section == "[moneysystem]") {
            definition.metadata.moneySystem = readOptionalValue();
        } else if (section == "[ticketpack]") {
            definition.metadata.ticketPack = readOptionalValue();
        } else if (section == "[repair_time_min]") {
            definition.metadata.repairTimeMin = readOptionalValue();
        } else if (section == "[years]") {
            readMetadataFields(definition.metadata.years, "years");
        } else if (section == "[realyearoffset]") {
            definition.metadata.realYearOffset = readOptionalValue();
        } else if (section == "[standarddepot]") {
            definition.metadata.standardDepot = readOptionalValue();
        } else if (section == "[addseason]") {
            MapSeasonDefinition season;
            if (sectionLine > 0) {
                season.label = trim(lines[sectionLine - 1]);
            }
            if (!season.label.ends_with(':')) {
                definition.diagnostics.warning(sectionLine + 1, "addseason",
                                               "expected a preceding season label ending in ':'");
            }
            readMetadataFields(season.fields, "addseason");
            definition.metadata.seasons.push_back(std::move(season));
        } else if (section == "[trafficdensity_road]") {
            std::array<std::string, 2> values;
            readMetadataFields(values, "trafficdensity_road");
            MapTrafficDensityPoint point;
            if (parseReal(values[0], point.hour) && point.hour >= 0.0 && point.hour <= 24.0 &&
                parseReal(values[1], point.density) && point.density >= 0.0) {
                definition.aiTraffic.roadDensity.push_back(point);
            } else {
                definition.diagnostics.warning(sectionLine + 1, "trafficdensity_road",
                                               "invalid time or density value");
            }
            definition.metadata.roadTrafficDensity.push_back(std::move(values));
        } else if (section == "[trafficdensity_passenger]") {
            std::array<std::string, 2> values;
            readMetadataFields(values, "trafficdensity_passenger");
            MapTrafficDensityPoint point;
            if (parseReal(values[0], point.hour) && point.hour >= 0.0 && point.hour <= 24.0 &&
                parseReal(values[1], point.density) && point.density >= 0.0) {
                definition.aiTraffic.passengerDensity.push_back(point);
            } else {
                definition.diagnostics.warning(sectionLine + 1, "trafficdensity_passenger",
                                               "invalid time or density value");
            }
            definition.metadata.passengerTrafficDensity.push_back(std::move(values));
        } else if (section == "[groundtex]") {
            GroundTextureDefinition groundTexture;
            groundTexture.texturePath = readField(lines, cursor, globalPath, "groundtex");
            groundTexture.detailTexturePath = readField(lines, cursor, globalPath, "groundtex");
            for (std::string& parameter : groundTexture.parameters) {
                parameter = readField(lines, cursor, globalPath, "groundtex");
            }
            if (groundTexture.texturePath.empty()) {
                throw std::runtime_error("Empty base texture in [groundtex] in " +
                                         pathToUtf8(globalPath));
            }
            definition.groundTextures.push_back(std::move(groundTexture));
        } else if (section == "[entrypoints]") {
            const std::string countText = readField(lines, cursor, globalPath, "entrypoints");
            int count = 0;
            if (!parseInteger(countText, count) || count < 0 || count > 100000) {
                throw std::runtime_error("Invalid [entrypoints] count in " +
                                         pathToUtf8(globalPath));
            }
            definition.entryPoints.reserve(static_cast<std::size_t>(count));
            for (int index = 0; index < count; ++index) {
                MapEntryPoint entryPoint;
                for (std::string& field : entryPoint.rawFields) {
                    field = readField(lines, cursor, globalPath, "entrypoints");
                }
                if (!parseInteger(entryPoint.rawFields[0], entryPoint.objectOnTileIndex) ||
                    !parseInteger(entryPoint.rawFields[1], entryPoint.id) ||
                    !parseInteger(entryPoint.rawFields[10], entryPoint.tileIndex)) {
                    throw std::runtime_error("Invalid integer in [entrypoints] record " +
                                             std::to_string(index) + " in " +
                                             pathToUtf8(globalPath));
                }
                entryPoint.name = entryPoint.rawFields[11];
                definition.entryPoints.push_back(std::move(entryPoint));
            }
        } else if (section == "[map]") {
            const std::string xText = readField(lines, cursor, globalPath, "map");
            const std::string yText = readField(lines, cursor, globalPath, "map");
            const std::string filename = readField(lines, cursor, globalPath, "map");
            MapTileReference tile;
            if (!parseInteger(xText, tile.x) || !parseInteger(yText, tile.y)) {
                throw std::runtime_error("Invalid tile coordinates in [map] in " +
                                         pathToUtf8(globalPath));
            }
            int filenameX = 0;
            int filenameY = 0;
            parseTileFilename(filename, filenameX, filenameY);
            if (filenameX != tile.x || filenameY != tile.y) {
                definition.diagnostics.warning(
                    sectionLine + 1, "map",
                    "[map] coordinates do not match tile filename; manifest coordinates used: " +
                        filename);
            }
            const std::uint64_t key =
                (static_cast<std::uint64_t>(static_cast<std::uint32_t>(tile.x)) << 32U) |
                static_cast<std::uint32_t>(tile.y);
            const auto existingTile = tileIndicesByCoordinates.find(key);
            if (existingTile != tileIndicesByCoordinates.end()) {
                definition.diagnostics.warning(
                    sectionLine + 1, "map",
                    "duplicate tile record ignored; tile index aliases the first occurrence: " +
                        filename);
                tileOrdinalToUniqueIndex.push_back(existingTile->second);
                continue;
            }
            tile.textPath = mapDirectory / pathFromUtf8(filename);
            if (!std::filesystem::is_regular_file(tile.textPath)) {
                definition.diagnostics.warning(sectionLine + 1, "map",
                                               "listed map tile does not exist: " +
                                                   pathToUtf8(tile.textPath));
                tileOrdinalToUniqueIndex.push_back(missingTileIndex);
                continue;
            }
            tile.terrainPath = pathWithSuffix(tile.textPath, ".terrain");
            tile.hasTerrainFile = std::filesystem::is_regular_file(tile.terrainPath);
            tile.waterPath = pathWithSuffix(tile.textPath, ".water");
            tile.hasWaterFile = std::filesystem::is_regular_file(tile.waterPath);
            tile.lightmapPath = pathWithSuffix(tile.textPath, ".LM.bmp");
            tile.hasLightmapFile = std::filesystem::is_regular_file(tile.lightmapPath);
            tile.terrainReadinessPath = pathWithSuffix(tile.textPath, ".terrain_0.rdy");
            tile.hasTerrainReadinessFile =
                std::filesystem::is_regular_file(tile.terrainReadinessPath);
            const auto sidecars = groundTextureSidecarsByTile.find(filename);
            if (sidecars != groundTextureSidecarsByTile.end()) {
                tile.groundTextureSidecars = sidecars->second;
            }
            tileIndicesByCoordinates.emplace(key, definition.tiles.size());
            definition.tiles.push_back(std::move(tile));
            tileOrdinalToUniqueIndex.push_back(definition.tiles.size() - 1);
        } else if (isSectionHeader(section)) {
            openbus::config::logUnhandledConfigurationTag("global.cfg", globalPath, sectionLine + 1,
                                                          section.substr(1, section.size() - 2),
                                                          "section is not implemented");
            definition.diagnostics.warning(sectionLine + 1, section.substr(1, section.size() - 2),
                                           "global.cfg section is not implemented");
            MapOpaqueSection opaqueSection;
            opaqueSection.keyword = section.substr(1, section.size() - 2);
            opaqueSection.sectionLine = sectionLine + 1;
            while (cursor < lines.size() && !isSectionHeader(trim(lines[cursor]))) {
                opaqueSection.payloadLines.push_back(lines[cursor++]);
            }
            definition.unsupportedSections.push_back(std::move(opaqueSection));
        }
    }
    manifestTrace.finish();

    if (definition.tiles.empty()) {
        throw std::runtime_error("No [map] tile records found in " + pathToUtf8(globalPath));
    }
    std::vector<int> tileRows;
    tileRows.reserve(definition.tiles.size());
    for (const MapTileReference& tile : definition.tiles) {
        tileRows.push_back(tile.y);
    }
    definition.coordinates =
        MapCoordinates::forRows(definition.metadata.hasWorldCoordinates, std::move(tileRows));
    std::erase_if(definition.entryPoints, [&](const MapEntryPoint& entryPoint) {
        if (entryPoint.tileIndex >= 0 &&
            static_cast<std::size_t>(entryPoint.tileIndex) < tileOrdinalToUniqueIndex.size() &&
            tileOrdinalToUniqueIndex[static_cast<std::size_t>(entryPoint.tileIndex)] ==
                missingTileIndex) {
            definition.diagnostics.warning(0, "entrypoints",
                                           "entry point references a missing tile; ignored");
            return true;
        }
        return false;
    });
    openbus::rendering::TraceScope entryPointTrace("map", "MapConfigLoader.resolveEntryPoints");
    for (MapEntryPoint& entryPoint : definition.entryPoints) {
        if (entryPoint.tileIndex < 0 ||
            static_cast<std::size_t>(entryPoint.tileIndex) >= tileOrdinalToUniqueIndex.size()) {
            throw std::runtime_error("[entrypoints] tile index is outside the [map] list in " +
                                     pathToUtf8(globalPath));
        }
        entryPoint.tileIndex = static_cast<int>(
            tileOrdinalToUniqueIndex[static_cast<std::size_t>(entryPoint.tileIndex)]);
        double localX = 0.0;
        double elevation = 0.0;
        double localY = 0.0;
        double quaternionX = 0.0;
        double quaternionY = 0.0;
        double quaternionZ = 0.0;
        double quaternionW = 0.0;
        if (!parseReal(entryPoint.rawFields[3], localX) ||
            !parseReal(entryPoint.rawFields[4], elevation) ||
            !parseReal(entryPoint.rawFields[5], localY) ||
            !parseReal(entryPoint.rawFields[6], quaternionX) ||
            !parseReal(entryPoint.rawFields[7], quaternionY) ||
            !parseReal(entryPoint.rawFields[8], quaternionZ) ||
            !parseReal(entryPoint.rawFields[9], quaternionW)) {
            throw std::runtime_error("Invalid position or orientation in [entrypoints] record in " +
                                     pathToUtf8(globalPath));
        }
        const double quaternionNorm =
            std::hypot(std::hypot(quaternionX, quaternionY), std::hypot(quaternionZ, quaternionW));
        if (quaternionNorm <= std::numeric_limits<double>::epsilon()) {
            throw std::runtime_error("Zero orientation quaternion in [entrypoints] record in " +
                                     pathToUtf8(globalPath));
        }
        quaternionX /= quaternionNorm;
        quaternionY /= quaternionNorm;
        quaternionZ /= quaternionNorm;
        quaternionW /= quaternionNorm;
        // OMSI global.cfg quaternions are in the source basis (east, up, north),
        // where forward is +Z. Rotate that forward vector, then project it into
        // the simulation's (east, north) plane. Simulation yaw zero points east
        // and positive yaw turns counter-clockwise toward north.
        const double forwardEast = 2.0 * (quaternionX * quaternionZ + quaternionW * quaternionY);
        const double forwardNorth =
            1.0 - 2.0 * (quaternionX * quaternionX + quaternionY * quaternionY);
        if (std::hypot(forwardEast, forwardNorth) <= std::numeric_limits<double>::epsilon()) {
            throw std::runtime_error("Vertical orientation in [entrypoints] record in " +
                                     pathToUtf8(globalPath));
        }
        const MapTileReference& tile = definition.tiles[entryPoint.tileIndex];
        const auto transform = definition.coordinates.tile(tile.x, tile.y);
        const auto position = transform.toWorld(localX, localY);
        entryPoint.placement = {
            {position[0], position[1], elevation},
            std::atan2(forwardNorth * transform.scaleY, forwardEast * transform.scaleX) * 180.0 /
                3.14159265358979323846};
    }
    entryPointTrace.finish();
    const std::filesystem::path aiListPath = mapDirectory / "ailists.cfg";
    mapLog.Log("Manifest parsed: " + std::to_string(definition.tiles.size()) +
               " tiles; loading AI lists and population schedules.");
    openbus::rendering::TraceScope trafficTrace("map", "MapConfigLoader.loadTrafficSchedules");
    if (std::filesystem::is_regular_file(aiListPath)) {
        parseAiListFile(aiListPath, definition.aiTraffic);
    }
    const std::filesystem::path unscheduledGroupsPath = mapDirectory / "unsched_vehgroups.txt";
    if (std::filesystem::is_regular_file(unscheduledGroupsPath)) {
        parseUnscheduledVehicleGroups(unscheduledGroupsPath, definition.aiTraffic);
        for (const MapAiVehicleGroup& group : definition.aiTraffic.vehicleGroups) {
            if (!isSupportedRoadAiGroup(group.name)) {
                continue;
            }
            const bool groupIndexFound = std::any_of(
                definition.aiTraffic.unscheduledVehicleGroupNames.begin(),
                definition.aiTraffic.unscheduledVehicleGroupNames.end(),
                [&](const std::string& name) {
                    return canonicalAiRoadGroupName(name) == canonicalAiRoadGroupName(group.name);
                });
            if (!groupIndexFound) {
                definition.diagnostics.warning(
                    0, "unsched_vehgroups.txt",
                    "no population group matches AI-list group '" + group.name +
                        "'; ailists.cfg order will be used for its density schedule");
            }
        }
    } else if (!definition.aiTraffic.vehicleGroups.empty()) {
        definition.diagnostics.warning(
            0, "unsched_vehgroups.txt",
            "dynamic unscheduled density targets use ailists.cfg order because the map's "
            "population group table is unavailable");
    }
    const std::filesystem::path unscheduledTrafficDensityPath =
        mapDirectory / "unsched_trafficdens.txt";
    if (std::filesystem::is_regular_file(unscheduledTrafficDensityPath)) {
        parseUnscheduledTrafficDensities(unscheduledTrafficDensityPath, definition.aiTraffic,
                                         definition.diagnostics);
    }
    mapLog.Log("Loading AI timetables: " + pathToUtf8(mapDirectory / "TTData"));
    parseAiTimetableRoutes({{mapDirectory / "TTData", {}}}, definition.aiTraffic,
                           definition.diagnostics);
    mapLog.Log("AI timetables loaded; loading driver and pedestrian lists.");
    const std::filesystem::path driverListPath = mapDirectory / "drivers.txt";
    if (std::filesystem::is_regular_file(driverListPath)) {
        parseDriverListFile(driverListPath, definition.aiTraffic);
    }
    const std::filesystem::path humanListPath = mapDirectory / "humans.txt";
    if (std::filesystem::is_regular_file(humanListPath)) {
        parseHumanListFile(humanListPath, definition.aiTraffic);
    }
    trafficTrace.finish();
    mapLog.Log("Map manifest and traffic configuration loaded in " +
               std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - loadStart)
                                  .count()) +
               " ms.");
    return definition;
}

MapTicketPackDefinition loadMapTicketPackDefinition(const std::filesystem::path& ticketPackPath) {
    if (!std::filesystem::is_regular_file(ticketPackPath)) {
        throw std::runtime_error("Ticket pack is missing: " + pathToUtf8(ticketPackPath));
    }
    const std::vector<std::string> lines = splitLines(decodeText(ticketPackPath));
    MapTicketPackDefinition definition;
    const auto unquote = [](std::string field) {
        if (field.size() >= 2 && field.front() == '"' && field.back() == '"') {
            field = field.substr(1, field.size() - 2);
        }
        return field;
    };
    const auto readFields = [&](std::size_t lineIndex) {
        std::vector<std::string> fields;
        std::size_t next = lineIndex + 1;
        while (next < lines.size() && !isSectionHeaderLine(trim(lines[next]))) {
            const std::string field = trim(lines[next++]);
            if (!field.empty()) {
                fields.push_back(field);
            }
        }
        return std::pair{std::move(fields), next};
    };
    for (std::size_t lineIndex = 0; lineIndex < lines.size(); ++lineIndex) {
        const std::string section = lowerAscii(trim(lines[lineIndex]));
        if (section == "[ticketpack]") {
            auto [fields, next] = readFields(lineIndex);
            std::array<double, 4> values{};
            const std::size_t fieldCount = std::min(fields.size(), values.size());
            bool valid = !fields.empty();
            for (std::size_t field = 0; field < fieldCount; ++field) {
                valid = parseReal(fields[field], values[field]) && values[field] >= 0.0 &&
                        values[field] <= 1.0 && valid;
            }
            if (fields.size() == 1 && valid) {
                definition.ticketBuyProbability = values[0];
            } else if (fields.size() == values.size() && valid) {
                definition.stamperProbability = values[0];
                definition.ticketBuyProbability = values[1];
                definition.chattiness = values[2];
                definition.whingeProbability = values[3];
            } else {
                definition.diagnostics.warning(
                    lineIndex + 1, "ticketpack",
                    "invalid or incomplete ticket-pack properties in " +
                        pathToUtf8(ticketPackPath.filename()));
            }
            if (next > lineIndex + 1) {
                lineIndex = next - 1;
            }
        } else if (section == "[ticket]") {
            auto [fields, next] = readFields(lineIndex);
            MapTicketDefinition ticket;
            bool valid = false;
            if (fields.size() == 7) {
                valid = !fields[0].empty() && !fields[1].empty() &&
                        parseInteger(fields[2], ticket.maxStations) &&
                        ticket.maxStations >= 0 && parseReal(fields[3], ticket.ageMin) &&
                        ticket.ageMin >= 0.0 && parseReal(fields[4], ticket.ageMax) &&
                        ticket.ageMax >= ticket.ageMin && parseReal(fields[5], ticket.value) &&
                        ticket.value >= 0.0 &&
                        ticket.value <= static_cast<double>((std::numeric_limits<float>::max)());
                if (valid) {
                    ticket.name = unquote(std::move(fields[0]));
                    ticket.englishName = unquote(std::move(fields[1]));
                    ticket.displayString = unquote(std::move(fields[6]));
                    ticket.displayValue = ticket.displayString;
                }
            } else if (fields.size() == 4) {
                valid = !fields[0].empty() && parseReal(fields[2], ticket.value) &&
                        ticket.value >= 0.0 &&
                        ticket.value <= static_cast<double>((std::numeric_limits<float>::max)()) &&
                        parseInteger(fields[3], ticket.type) && ticket.type >= 0;
                if (valid) {
                    ticket.name = unquote(std::move(fields[0]));
                    ticket.displayValue = unquote(std::move(fields[1]));
                    ticket.displayString = ticket.displayValue;
                    ticket.englishName = ticket.name;
                }
            }
            if (valid) {
                definition.tickets.push_back(std::move(ticket));
            } else {
                definition.diagnostics.warning(lineIndex + 1, "ticket",
                                               "invalid or incomplete ticket entry in " +
                                                   pathToUtf8(ticketPackPath.filename()));
            }
            if (next > lineIndex + 1) {
                lineIndex = next - 1;
            }
        } else {
            continue;
        }
    }
    return definition;
}

MapCurrencyDefinition loadMapCurrencyDefinition(const std::filesystem::path& currencyPath) {
    if (!std::filesystem::is_regular_file(currencyPath)) {
        throw std::runtime_error("Currency configuration is missing: " + pathToUtf8(currencyPath));
    }
    const std::vector<std::string> lines = splitLines(decodeText(currencyPath));
    MapCurrencyDefinition definition;
    for (std::size_t lineIndex = 0; lineIndex < lines.size(); ++lineIndex) {
        const std::string section = lowerAscii(trim(lines[lineIndex]));
        if (section == "[currency]") {
            std::size_t next = lineIndex + 1;
            while (next < lines.size() && trim(lines[next]).empty()) {
                ++next;
            }
            if (next + 1 < lines.size() && !isSectionHeaderLine(trim(lines[next])) &&
                !isSectionHeaderLine(trim(lines[next + 1]))) {
                definition.name = trim(lines[next]);
                definition.mode = trim(lines[next + 1]);
                lineIndex = next + 1;
            } else {
                definition.diagnostics.warning(lineIndex + 1, "currency",
                                               "incomplete currency header in " +
                                                   pathToUtf8(currencyPath.filename()));
            }
        } else if (section == "[coin]") {
            std::size_t next = lineIndex + 1;
            while (next < lines.size() && trim(lines[next]).empty()) {
                ++next;
            }
            double value = 0.0;
            if (next + 1 < lines.size() && !isSectionHeaderLine(trim(lines[next])) &&
                !isSectionHeaderLine(trim(lines[next + 1])) &&
                parseReal(trim(lines[next + 1]), value) && value > 0.0 &&
                value <= static_cast<double>((std::numeric_limits<float>::max)())) {
                definition.coins.push_back({trim(lines[next]), value});
                lineIndex = next + 1;
            } else {
                definition.diagnostics.warning(lineIndex + 1, "coin",
                                               "invalid or incomplete coin entry in " +
                                                   pathToUtf8(currencyPath.filename()));
            }
        }
    }
    return definition;
}

std::optional<MapAiTimetableStationPosition>
mapAiTimetableStationPosition(const MapAiTimetableRoute& route,
                              const MapAiTimetableRoute::Station& station) {
    if (station.trackEntryIndex < 0 || route.trackEntries.empty()) {
        return std::nullopt;
    }
    const std::size_t trackEntryIndex = static_cast<std::size_t>(station.trackEntryIndex);
    if (station.positionOnPreviousTrackEntryMeters) {
        if (trackEntryIndex > route.trackEntries.size()) {
            return std::nullopt;
        }
        const std::size_t previousEntryIndex = trackEntryIndex == 0 ? 0 : trackEntryIndex - 1;
        const double offsetMeters = *station.positionOnPreviousTrackEntryMeters;
        const double previousEntryLength = route.trackEntries[previousEntryIndex].lengthMeters;
        if (!std::isfinite(offsetMeters) || offsetMeters < 0.0 ||
            !std::isfinite(previousEntryLength) || offsetMeters > previousEntryLength) {
            return std::nullopt;
        }
        return MapAiTimetableStationPosition{previousEntryIndex, offsetMeters};
    }
    if (trackEntryIndex >= route.trackEntries.size()) {
        return std::nullopt;
    }
    return MapAiTimetableStationPosition{trackEntryIndex, 0.0};
}

std::optional<std::size_t>
mapAiTimetableNextCourseTripIndex(const MapAiTimetableCourse& course, std::size_t currentTripIndex,
                                  std::span<const std::string> availableTimetableKeys) {
    if (currentTripIndex >= course.trips.size()) {
        return std::nullopt;
    }
    for (std::size_t tripIndex = currentTripIndex + 1; tripIndex < course.trips.size();
         ++tripIndex) {
        const auto& trip = course.trips[tripIndex];
        if (std::ranges::find(availableTimetableKeys, trip.timetableKey) !=
            availableTimetableKeys.end()) {
            return tripIndex;
        }
    }
    return std::nullopt;
}

std::vector<MapAiPassengerStopCall>
mapAiPassengerStopCalls(const MapAiTrafficData& traffic, const MapAiPassengerTimetable& timetable) {
    std::vector<MapAiPassengerStopCall> calls;
    calls.reserve(timetable.stationIds.size());
    for (std::size_t stationIndex = 0; stationIndex < timetable.stationIds.size(); ++stationIndex) {
        MapAiPassengerStopCall call;
        call.stationId = timetable.stationIds[stationIndex];
        std::optional<std::size_t> matchingStop;
        bool duplicateStop = false;
        for (std::size_t stopIndex = 0; stopIndex < traffic.busStops.size(); ++stopIndex) {
            if (traffic.busStops[stopIndex].id != call.stationId) {
                continue;
            }
            if (matchingStop) {
                duplicateStop = true;
                break;
            }
            matchingStop = stopIndex;
        }
        if (!duplicateStop) {
            call.busStopIndex = matchingStop;
        }
        if (stationIndex + 1 < timetable.stationIds.size()) {
            std::optional<std::size_t> matchingLink;
            bool duplicateLink = false;
            for (std::size_t linkIndex = 0; linkIndex < traffic.stationLinks.size(); ++linkIndex) {
                const MapAiStationLink& link = traffic.stationLinks[linkIndex];
                if (link.fromStopId != call.stationId ||
                    link.toStopId != timetable.stationIds[stationIndex + 1]) {
                    continue;
                }
                if (matchingLink) {
                    duplicateLink = true;
                    break;
                }
                matchingLink = linkIndex;
            }
            if (!duplicateLink) {
                call.outgoingStationLinkIndex = matchingLink;
            }
        }
        calls.push_back(call);
    }
    return calls;
}

bool mapAiPassengerTimetableHasResolvedLinks(const MapAiTrafficData& traffic,
                                             const MapAiPassengerTimetable& timetable) {
    if (timetable.stationIds.empty()) {
        return false;
    }
    const std::vector<MapAiPassengerStopCall> calls = mapAiPassengerStopCalls(traffic, timetable);
    return std::all_of(
               calls.begin(), calls.end(),
               [](const MapAiPassengerStopCall& call) { return call.busStopIndex.has_value(); }) &&
           std::all_of(calls.begin(), calls.end() - 1, [](const MapAiPassengerStopCall& call) {
               return call.outgoingStationLinkIndex.has_value();
           });
}

std::vector<MapAiRoadVehicle> mapAiRoadVehicles(const MapAiTrafficData& traffic) {
    std::vector<MapAiRoadVehicle> vehicles;
    std::unordered_set<std::string> seen;
    for (std::size_t groupIndex = 0; groupIndex < traffic.vehicleGroups.size(); ++groupIndex) {
        const MapAiVehicleGroup& group = traffic.vehicleGroups[groupIndex];
        std::size_t unscheduledGroupIndex = groupIndex;
        bool groupIsConfigured = false;
        for (std::size_t index = 0; index < traffic.unscheduledVehicleGroupNames.size(); ++index) {
            if (canonicalAiRoadGroupName(traffic.unscheduledVehicleGroupNames[index]) ==
                canonicalAiRoadGroupName(group.name)) {
                unscheduledGroupIndex = index;
                groupIsConfigured = true;
                break;
            }
        }
        if (!groupIsConfigured && (!traffic.unscheduledVehicleGroupNames.empty() ||
                                   !isLegacySupportedRoadAiGroup(group.name))) {
            continue;
        }
        for (const MapAiVehicleEntry& vehicle : group.vehicles) {
            const std::string extension =
                lowerAscii(std::filesystem::path(vehicle.assetPath).extension().string());
            if (extension != ".ovh" && extension != ".bus") {
                continue;
            }
            const std::string key =
                lowerAscii(vehicle.assetPath) + "\n" + std::to_string(unscheduledGroupIndex);
            if (seen.insert(key).second) {
                vehicles.push_back({vehicle.assetPath, unscheduledGroupIndex, unscheduledGroupIndex,
                                    lowerAscii(group.name) == "trucks" ? 3 : 0,
                                    canonicalAiRoadGroupName(group.name), 1.0});
            }
        }
    }
    return vehicles;
}

namespace {

const MapAiTrafficDensityGroup* findAiTrafficDensityGroup(const MapAiTrafficData& traffic,
                                                          std::string_view groupName) {
    const std::string normalized = canonicalAiRoadGroupName(std::string(groupName));
    const auto found = std::find_if(traffic.unscheduledTrafficDensityGroups.begin(),
                                    traffic.unscheduledTrafficDensityGroups.end(),
                                    [&](const MapAiTrafficDensityGroup& group) {
                                        return canonicalAiRoadGroupName(group.name) == normalized;
                                    });
    return found == traffic.unscheduledTrafficDensityGroups.end() ? nullptr : &*found;
}

double aiTrafficGroupDensityAtHour(const MapAiTrafficDensityGroup& group, double hour,
                                   int dayOfWeekMask) {
    const MapAiTrafficDensitySchedule* selectedSchedule = nullptr;
    int selectedSpecificity = std::numeric_limits<int>::max();
    for (const MapAiTrafficDensitySchedule& schedule : group.schedules) {
        if (!mapTrafficDensityAtHour(schedule.points, hour).has_value()) {
            continue;
        }
        if (schedule.dayOfWeekMask == 0) {
            if (selectedSchedule == nullptr) {
                selectedSchedule = &schedule;
            }
            continue;
        }
        if ((schedule.dayOfWeekMask & dayOfWeekMask) == 0) {
            continue;
        }
        const int specificity = std::popcount(static_cast<unsigned int>(schedule.dayOfWeekMask));
        if (specificity < selectedSpecificity) {
            selectedSchedule = &schedule;
            selectedSpecificity = specificity;
        }
    }
    if (selectedSchedule == nullptr) {
        return 0.0;
    }
    const std::optional<double> density = mapTrafficDensityAtHour(selectedSchedule->points, hour);
    if (!density.has_value()) {
        return 0.0;
    }
    return group.factor * std::max(0.0, *density);
}

} // namespace

std::vector<MapAiRoadVehicle> mapAiRoadVehicles(const MapAiTrafficData& traffic, double hour,
                                                int dayOfWeekMask) {
    if (!std::isfinite(hour) || hour < 0.0 || hour > 24.0 || dayOfWeekMask < 0) {
        throw std::invalid_argument("AI traffic group density lookup inputs are invalid");
    }
    std::vector<MapAiRoadVehicle> vehicles = mapAiRoadVehicles(traffic);
    if (!traffic.hasUnscheduledTrafficDensityFile) {
        return vehicles;
    }

    std::unordered_map<std::string, std::size_t> entriesPerGroup;
    std::unordered_map<std::string, double> densityPerGroup;
    for (const MapAiRoadVehicle& vehicle : vehicles) {
        ++entriesPerGroup[vehicle.groupName];
        const MapAiTrafficDensityGroup* group =
            findAiTrafficDensityGroup(traffic, vehicle.groupName);
        densityPerGroup.emplace(
            vehicle.groupName,
            group == nullptr ? 0.0 : aiTrafficGroupDensityAtHour(*group, hour, dayOfWeekMask));
    }
    for (MapAiRoadVehicle& vehicle : vehicles) {
        const double density = densityPerGroup.at(vehicle.groupName);
        const std::size_t entryCount = entriesPerGroup.at(vehicle.groupName);
        vehicle.selectionWeight = entryCount == 0 ? 0.0 : density / static_cast<double>(entryCount);
    }
    return vehicles;
}

std::vector<MapAiRoadVehicle> mapAiRoadVehiclesForPool(const MapAiTrafficData& traffic) {
    std::vector<MapAiRoadVehicle> vehicles = mapAiRoadVehicles(traffic);
    if (!traffic.hasUnscheduledTrafficDensityFile) {
        return vehicles;
    }

    std::vector<double> candidateHours{0.0, 24.0};
    for (const MapAiTrafficDensityGroup& group : traffic.unscheduledTrafficDensityGroups) {
        for (const MapAiTrafficDensitySchedule& schedule : group.schedules) {
            for (const MapTrafficDensityPoint& point : schedule.points) {
                candidateHours.push_back(point.hour);
            }
        }
    }
    std::sort(candidateHours.begin(), candidateHours.end());
    candidateHours.erase(std::unique(candidateHours.begin(), candidateHours.end()),
                         candidateHours.end());

    std::unordered_map<std::string, std::size_t> entriesPerGroup;
    std::unordered_map<std::string, double> maximumDensityPerGroup;
    for (const MapAiRoadVehicle& vehicle : vehicles) {
        ++entriesPerGroup[vehicle.groupName];
        maximumDensityPerGroup.emplace(vehicle.groupName, 0.0);
    }
    for (const int dayOfWeekMask : {1, 2, 4}) {
        for (const double hour : candidateHours) {
            const std::vector<MapAiRoadVehicle> scheduledVehicles =
                mapAiRoadVehicles(traffic, hour, dayOfWeekMask);
            for (const MapAiRoadVehicle& vehicle : scheduledVehicles) {
                maximumDensityPerGroup[vehicle.groupName] =
                    std::max(maximumDensityPerGroup[vehicle.groupName],
                             vehicle.selectionWeight *
                                 static_cast<double>(entriesPerGroup[vehicle.groupName]));
            }
        }
    }
    for (MapAiRoadVehicle& vehicle : vehicles) {
        const std::size_t entryCount = entriesPerGroup.at(vehicle.groupName);
        vehicle.selectionWeight = entryCount == 0 ? 0.0
                                                  : maximumDensityPerGroup.at(vehicle.groupName) /
                                                        static_cast<double>(entryCount);
    }
    return vehicles;
}

std::optional<std::size_t> selectMapAiRoadVehicle(const std::vector<MapAiRoadVehicle>& vehicles,
                                                  std::size_t selectionSeed) {
    long double totalWeight = 0.0L;
    for (const MapAiRoadVehicle& vehicle : vehicles) {
        if (!std::isfinite(vehicle.selectionWeight) || vehicle.selectionWeight < 0.0) {
            throw std::invalid_argument("AI road vehicle selection weight is invalid");
        }
        totalWeight += static_cast<long double>(vehicle.selectionWeight);
    }
    if (totalWeight <= 0.0L) {
        return std::nullopt;
    }

    std::uint64_t random = static_cast<std::uint64_t>(selectionSeed) + 0x9e3779b97f4a7c15ULL;
    random = (random ^ (random >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    random = (random ^ (random >> 27U)) * 0x94d049bb133111ebULL;
    random ^= random >> 31U;
    const long double unit =
        (static_cast<long double>(random) + 0.5L) /
        (static_cast<long double>(std::numeric_limits<std::uint64_t>::max()) + 1.0L);
    const long double target = unit * totalWeight;
    long double cumulativeWeight = 0.0L;
    for (std::size_t index = 0; index < vehicles.size(); ++index) {
        cumulativeWeight += static_cast<long double>(vehicles[index].selectionWeight);
        if (target < cumulativeWeight) {
            return index;
        }
    }
    return std::nullopt;
}

std::vector<MapAiRoadTrafficGroupPopulation>
mapAiRoadTrafficPopulationByGroup(const MapAiTrafficData& traffic, std::size_t populationCount,
                                  double hour, int dayOfWeekMask) {
    const std::vector<MapAiRoadVehicle> vehicles = mapAiRoadVehicles(traffic, hour, dayOfWeekMask);
    std::unordered_map<std::size_t, long double> weightByGroup;
    long double totalWeight = 0.0L;
    for (const MapAiRoadVehicle& vehicle : vehicles) {
        if (!std::isfinite(vehicle.selectionWeight) || vehicle.selectionWeight < 0.0) {
            throw std::invalid_argument("AI road vehicle selection weight is invalid");
        }
        weightByGroup[vehicle.groupIndex] += static_cast<long double>(vehicle.selectionWeight);
        totalWeight += static_cast<long double>(vehicle.selectionWeight);
    }
    if (populationCount == 0 || totalWeight <= 0.0L) {
        return {};
    }

    struct GroupRemainder {
        std::size_t groupIndex = 0;
        long double remainder = 0.0L;
    };
    std::vector<MapAiRoadTrafficGroupPopulation> populations;
    std::vector<GroupRemainder> remainders;
    populations.reserve(weightByGroup.size());
    remainders.reserve(weightByGroup.size());
    std::size_t assignedCount = 0;
    for (const auto& [groupIndex, weight] : weightByGroup) {
        if (weight <= 0.0L) {
            continue;
        }
        const long double exactCount =
            static_cast<long double>(populationCount) * weight / totalWeight;
        const std::size_t groupCount = static_cast<std::size_t>(std::floor(exactCount));
        populations.push_back({groupIndex, groupCount});
        remainders.push_back({groupIndex, exactCount - groupCount});
        assignedCount += groupCount;
    }
    std::sort(populations.begin(), populations.end(),
              [](const MapAiRoadTrafficGroupPopulation& left,
                 const MapAiRoadTrafficGroupPopulation& right) {
                  return left.groupIndex < right.groupIndex;
              });
    std::sort(remainders.begin(), remainders.end(),
              [](const GroupRemainder& left, const GroupRemainder& right) {
                  if (left.remainder != right.remainder) {
                      return left.remainder > right.remainder;
                  }
                  return left.groupIndex < right.groupIndex;
              });
    for (std::size_t index = 0; assignedCount < populationCount && index < remainders.size();
         ++index) {
        const auto population =
            std::find_if(populations.begin(), populations.end(),
                         [&](const MapAiRoadTrafficGroupPopulation& candidate) {
                             return candidate.groupIndex == remainders[index].groupIndex;
                         });
        ++population->populationCount;
        ++assignedCount;
    }
    return populations;
}

std::vector<MapAiRoadTrafficGroupPopulation>
mapAiMaximumRoadTrafficPoolByGroup(const MapAiTrafficData& traffic,
                                   const std::vector<MapAiRoadVehicle>& vehicles,
                                   std::size_t basePopulation, double densityMultiplier) {
    if (!std::isfinite(densityMultiplier) || densityMultiplier < 0.0) {
        throw std::invalid_argument("AI road traffic pool density multiplier is invalid");
    }
    if (!traffic.hasUnscheduledTrafficDensityFile || basePopulation == 0 ||
        densityMultiplier == 0.0) {
        return {};
    }

    std::vector<double> candidateHours{0.0, 24.0};
    for (const MapAiTrafficDensityGroup& group : traffic.unscheduledTrafficDensityGroups) {
        for (const MapAiTrafficDensitySchedule& schedule : group.schedules) {
            for (const MapTrafficDensityPoint& point : schedule.points) {
                candidateHours.push_back(point.hour);
            }
        }
    }
    std::sort(candidateHours.begin(), candidateHours.end());
    candidateHours.erase(std::unique(candidateHours.begin(), candidateHours.end()),
                         candidateHours.end());

    std::unordered_set<std::string> countedGroups;
    long double totalGroupFactors = 0.0L;
    for (const MapAiRoadVehicle& vehicle : vehicles) {
        if (!countedGroups.insert(vehicle.groupName).second) {
            continue;
        }
        const MapAiTrafficDensityGroup* group =
            findAiTrafficDensityGroup(traffic, vehicle.groupName);
        if (group != nullptr) {
            totalGroupFactors += static_cast<long double>(group->factor);
        }
    }
    if (totalGroupFactors <= 0.0L) {
        return {};
    }

    // Bound each group's maximum across curve knots, including rounding between knots.
    std::unordered_map<std::size_t, std::size_t> maximumByGroup;
    std::unordered_map<std::size_t, long double> maximumExpectedByGroup;
    for (const int dayOfWeekMask : {1, 2, 4}) {
        for (const double hour : candidateHours) {
            const double density =
                mapAiRoadTrafficDensityAtHour(traffic, vehicles, hour, dayOfWeekMask).value_or(0.0);
            const std::size_t targetPopulation =
                mapTrafficPopulationForDensity(basePopulation, density * densityMultiplier);
            for (const MapAiRoadTrafficGroupPopulation& population :
                 mapAiRoadTrafficPopulationByGroup(traffic, targetPopulation, hour,
                                                   dayOfWeekMask)) {
                maximumByGroup[population.groupIndex] =
                    std::max(maximumByGroup[population.groupIndex], population.populationCount);
            }

            std::unordered_map<std::size_t, long double> groupWeightByIndex;
            long double totalWeight = 0.0L;
            for (const MapAiRoadVehicle& vehicle :
                 mapAiRoadVehicles(traffic, hour, dayOfWeekMask)) {
                const long double weight = static_cast<long double>(vehicle.selectionWeight);
                groupWeightByIndex[vehicle.groupIndex] += weight;
                totalWeight += weight;
            }
            if (totalWeight <= 0.0L) {
                continue;
            }
            for (const auto& [groupIndex, groupWeight] : groupWeightByIndex) {
                if (groupWeight <= 0.0L) {
                    continue;
                }
                maximumExpectedByGroup[groupIndex] =
                    std::max(maximumExpectedByGroup[groupIndex],
                             static_cast<long double>(basePopulation) * densityMultiplier *
                                 groupWeight / totalGroupFactors);
            }
        }
    }

    for (const auto& expectedMaximum : maximumExpectedByGroup) {
        maximumByGroup.try_emplace(expectedMaximum.first, 0);
    }

    std::vector<MapAiRoadTrafficGroupPopulation> pool;
    pool.reserve(maximumByGroup.size());
    for (const auto& [groupIndex, sampledMaximum] : maximumByGroup) {
        const long double densityBound = std::ceil(maximumExpectedByGroup[groupIndex] + 0.5L);
        if (!std::isfinite(densityBound) ||
            densityBound >= std::ldexp(1.0L, std::numeric_limits<std::size_t>::digits)) {
            throw std::overflow_error("AI road traffic pool exceeds the supported integer range");
        }
        const std::size_t boundedMaximum = static_cast<std::size_t>(densityBound);
        const std::size_t populationCount = std::max(sampledMaximum, boundedMaximum);
        if (populationCount == 0) {
            continue;
        }
        pool.push_back({groupIndex, populationCount});
    }
    std::sort(pool.begin(), pool.end(),
              [](const MapAiRoadTrafficGroupPopulation& left,
                 const MapAiRoadTrafficGroupPopulation& right) {
                  return left.groupIndex < right.groupIndex;
              });
    return pool;
}

std::optional<double> mapAiRoadTrafficDensityAtHour(const MapAiTrafficData& traffic,
                                                    const std::vector<MapAiRoadVehicle>& vehicles,
                                                    double hour, int dayOfWeekMask) {
    if (!traffic.hasUnscheduledTrafficDensityFile) {
        return std::nullopt;
    }
    if (!std::isfinite(hour) || hour < 0.0 || hour > 24.0 || dayOfWeekMask < 0) {
        throw std::invalid_argument("AI traffic group density lookup inputs are invalid");
    }
    std::unordered_set<std::string> countedGroups;
    long double totalFactors = 0.0L;
    long double totalDensity = 0.0L;
    for (const MapAiRoadVehicle& vehicle : vehicles) {
        if (!countedGroups.insert(vehicle.groupName).second) {
            continue;
        }
        const MapAiTrafficDensityGroup* group =
            findAiTrafficDensityGroup(traffic, vehicle.groupName);
        if (group == nullptr) {
            continue;
        }
        totalFactors += static_cast<long double>(group->factor);
        totalDensity +=
            static_cast<long double>(aiTrafficGroupDensityAtHour(*group, hour, dayOfWeekMask));
    }
    if (totalFactors <= 0.0L) {
        return 0.0;
    }
    const long double averageDensity = totalDensity / totalFactors;
    return static_cast<double>(std::clamp(averageDensity, 0.0L, 2.0L));
}

std::optional<double>
mapAiMaximumRoadTrafficDensity(const MapAiTrafficData& traffic,
                               const std::vector<MapAiRoadVehicle>& vehicles) {
    if (!traffic.hasUnscheduledTrafficDensityFile) {
        return std::nullopt;
    }
    std::vector<double> candidateHours{0.0, 24.0};
    for (const MapAiTrafficDensityGroup& group : traffic.unscheduledTrafficDensityGroups) {
        for (const MapAiTrafficDensitySchedule& schedule : group.schedules) {
            for (const MapTrafficDensityPoint& point : schedule.points) {
                candidateHours.push_back(point.hour);
            }
        }
    }
    std::sort(candidateHours.begin(), candidateHours.end());
    candidateHours.erase(std::unique(candidateHours.begin(), candidateHours.end()),
                         candidateHours.end());

    double maximumDensity = 0.0;
    for (const int dayOfWeekMask : {1, 2, 4}) {
        for (const double hour : candidateHours) {
            maximumDensity =
                std::max(maximumDensity,
                         mapAiRoadTrafficDensityAtHour(traffic, vehicles, hour, dayOfWeekMask)
                             .value_or(0.0));
        }
    }
    return maximumDensity;
}

std::vector<std::string> mapAiRoadVehicleAssetPaths(const MapAiTrafficData& traffic) {
    std::vector<std::string> paths;
    std::unordered_set<std::string> seen;
    for (const MapAiRoadVehicle& vehicle : mapAiRoadVehicles(traffic)) {
        if (seen.insert(lowerAscii(vehicle.assetPath)).second) {
            paths.push_back(vehicle.assetPath);
        }
    }
    return paths;
}

std::vector<std::string> mapAiBusAssetPaths(const MapAiTrafficData& traffic) {
    std::vector<std::string> paths;
    std::unordered_set<std::string> seen;
    for (const MapAiBusTypeDefinition& busType : traffic.busTypes) {
        if (lowerAscii(std::filesystem::path(busType.assetPath).extension().string()) != ".bus") {
            continue;
        }
        if (seen.insert(lowerAscii(busType.assetPath)).second) {
            paths.push_back(busType.assetPath);
        }
    }
    return paths;
}

std::size_t selectMapSpawnPoint(const MapDefinition& map, std::string_view selector) {
    if (map.entryPoints.empty()) {
        throw std::runtime_error("Map has no [entrypoints]");
    }
    const std::string requested = trim(selector);
    if (requested.empty()) {
        return 0;
    }

    int index = 0;
    if (!parseInteger(requested, index) || index < 0 ||
        static_cast<std::size_t>(index) >= map.entryPoints.size()) {
        throw std::runtime_error("Map spawn index is outside the [entrypoints] list: " + requested);
    }
    return static_cast<std::size_t>(index);
}

std::size_t selectMapEntryPoint(const MapDefinition& map, std::string_view selector) {
    if (map.entryPoints.empty()) {
        throw std::runtime_error("Map has no [entrypoints]");
    }
    const std::string requested = trim(selector);
    if (requested.empty()) {
        return 0;
    }

    int index = 0;
    if (parseInteger(requested, index)) {
        if (index < 0 || static_cast<std::size_t>(index) >= map.entryPoints.size()) {
            throw std::runtime_error("Map entrypoint index is outside the [entrypoints] list: " +
                                     requested);
        }
        return static_cast<std::size_t>(index);
    }

    const auto equalsIgnoringAsciiCase = [](std::string_view left, std::string_view right) {
        if (left.size() != right.size()) {
            return false;
        }
        for (std::size_t position = 0; position < left.size(); ++position) {
            const auto leftChar = static_cast<unsigned char>(left[position]);
            const auto rightChar = static_cast<unsigned char>(right[position]);
            if (std::tolower(leftChar) != std::tolower(rightChar)) {
                return false;
            }
        }
        return true;
    };
    for (std::size_t candidate = 0; candidate < map.entryPoints.size(); ++candidate) {
        if (equalsIgnoringAsciiCase(map.entryPoints[candidate].name, requested)) {
            return candidate;
        }
    }
    throw std::runtime_error("No map entrypoint matches [" + requested + "]");
}

std::size_t selectMapGroundTextureIndex(const MapDefinition& map, std::string_view selector) {
    if (map.groundTextures.empty()) {
        throw std::runtime_error("Map has no [groundtex] records");
    }
    const std::string requested = trim(selector);
    if (requested.empty()) {
        return 0;
    }
    int index = 0;
    if (!parseInteger(requested, index) || index < 0 ||
        static_cast<std::size_t>(index) >= map.groundTextures.size()) {
        throw std::runtime_error("Map ground-texture preview index is outside the [groundtex] "
                                 "list: " +
                                 requested);
    }
    return static_cast<std::size_t>(index);
}

std::optional<std::size_t>
resolveMapGroundTextureSidecarIndex(std::string_view ordinalSuffix,
                                    std::size_t groundTextureCount) noexcept {
    if (ordinalSuffix.empty() || groundTextureCount < 2) {
        return std::nullopt;
    }
    std::size_t index = 0;
    const auto parsed =
        std::from_chars(ordinalSuffix.data(), ordinalSuffix.data() + ordinalSuffix.size(), index);
    if (parsed.ec != std::errc{} || parsed.ptr != ordinalSuffix.data() + ordinalSuffix.size() ||
        index == 0 || index > groundTextureCount) {
        return std::nullopt;
    }
    // Existing maps use the suffix directly for non-base records (1..N-1).
    // Some maps also emit the terminal suffix N for their final record N-1.
    return index == groundTextureCount ? groundTextureCount - 1 : index;
}

std::optional<double> mapTrafficDensityAtHour(const std::vector<MapTrafficDensityPoint>& schedule,
                                              double hour) {
    if (!std::isfinite(hour) || hour < 0.0 || hour > 24.0) {
        throw std::invalid_argument("Map traffic density lookup hour must be in [0, 24]");
    }

    if (schedule.empty()) {
        return std::nullopt;
    }
    for (std::size_t index = 0; index < schedule.size(); ++index) {
        const MapTrafficDensityPoint& point = schedule[index];
        if (!std::isfinite(point.hour) || point.hour < 0.0 || point.hour > 24.0 ||
            !std::isfinite(point.density) || point.density < 0.0 ||
            (index > 0 && point.hour <= schedule[index - 1].hour)) {
            return std::nullopt;
        }
    }
    if (hour <= schedule.front().hour) {
        return schedule.front().density;
    }
    if (hour >= schedule.back().hour) {
        return schedule.back().density;
    }
    const auto right = std::upper_bound(schedule.begin(), schedule.end(), hour,
                                        [](double targetHour, const MapTrafficDensityPoint& point) {
                                            return targetHour < point.hour;
                                        });
    const MapTrafficDensityPoint& leftPoint = *(right - 1);
    const MapTrafficDensityPoint& rightPoint = *right;
    const double fraction = (hour - leftPoint.hour) / (rightPoint.hour - leftPoint.hour);
    return leftPoint.density + (rightPoint.density - leftPoint.density) * fraction;
}

std::size_t mapTrafficPopulationForDensity(std::size_t basePopulation,
                                           std::optional<double> density) {
    if (basePopulation == 0) {
        return 0;
    }
    if (!density.has_value()) {
        return basePopulation;
    }
    if (!std::isfinite(*density) || *density < 0.0) {
        throw std::invalid_argument("Map traffic density must be finite and non-negative");
    }
    const long double scaled =
        static_cast<long double>(basePopulation) * static_cast<long double>(*density);
    const long double rounded = std::round(scaled);
    if (!std::isfinite(rounded) ||
        rounded >= std::ldexp(1.0L, std::numeric_limits<std::size_t>::digits)) {
        throw std::overflow_error("Map traffic population exceeds the supported integer range");
    }

    Logger mapLog("Map");
    mapLog.Log("Map traffic population for density: basePopulation = {}" +
               std::to_string(basePopulation));
    mapLog.Log("Map traffic population for density: density = {}" + std::to_string(*density));
    mapLog.Log("Map traffic population for density: scaled = {}" + std::to_string(scaled));
    mapLog.Log("Map traffic population for density: rounded = {}" + std::to_string(rounded));

    return static_cast<std::size_t>(rounded);
}

MapGroundTextureTiling
mapGroundTextureTiling(const GroundTextureDefinition& groundTexture) noexcept {
    const auto parseRepeatCount = [](const std::string& text) {
        if (text.empty()) {
            return 1.0;
        }
        const char* begin = text.data();
        const char* end = begin + text.size();
        if (*begin == '+') {
            ++begin;
        }
        double value = 0.0;
        const auto parsed = std::from_chars(begin, end, value);
        if (begin == end || parsed.ec != std::errc{} || parsed.ptr != end ||
            !std::isfinite(value) || value <= 0.0 ||
            value > static_cast<double>((std::numeric_limits<float>::max)())) {
            return 1.0;
        }
        return value;
    };
    return {parseRepeatCount(groundTexture.parameters[1]),
            parseRepeatCount(groundTexture.parameters[2])};
}

static MapTileData loadMapTileInternal(const MapTileReference& tile,
                                       MapChronoTileData* chronoData) {
    openbus::rendering::TraceScope trace("map", "MapConfigLoader::loadMapTile");
    MapTileData data;
    data.reference = tile;
    const std::vector<std::string> lines = splitLines(decodeText(tile.textPath));
    if (chronoData != nullptr) {
        chronoData->rawLines = lines;
        if (!lines.empty()) {
            chronoData->initialComment = lines.front();
        }
    }
    const auto isChronoRecordBoundary = [](const std::string& line) {
        constexpr std::array<std::string_view, 10> boundaries = {"[selspline]",
                                                                 "[selobject]",
                                                                 "[spline_terrain_align_2]",
                                                                 "[spline]",
                                                                 "[spline_h]",
                                                                 "[spline_terrain_align]",
                                                                 "[object]",
                                                                 "[attachObj]",
                                                                 "[splineAttachement]",
                                                                 "[splineAttachement_repeater]"};
        return std::find(boundaries.begin(), boundaries.end(), line) != boundaries.end();
    };
    std::size_t automaticRecordId = 0;
    const auto nextAutomaticRecordId = [&]() {
        if (automaticRecordId >= static_cast<std::size_t>((std::numeric_limits<int>::max)())) {
            throw std::overflow_error("Automatic map record IDs exceed the supported range in " +
                                      pathToUtf8(tile.textPath));
        }
        return -static_cast<int>(++automaticRecordId);
    };
    const auto currentTileVersion = [&]() {
        int tileVersion = 0;
        if (!data.version.empty() &&
            (!parseInteger(data.version, tileVersion) || tileVersion < 0)) {
            throw std::runtime_error("Invalid [version] in " + pathToUtf8(tile.textPath));
        }
        return tileVersion;
    };
    static const std::unordered_set<std::string> knownTileSections = {
        "[version]", "[terrain]", "[water]", "[variable_terrain]",
        "[variable_terrainlightmap]", "[spline]", "[spline_h]", "[object]",
        "[attachObj]", "[splineAttachement]", "[splineAttachement_repeater]",
        "[varparent]", "[spline_terrain_align]", "[spline_terrain_align_2]",
        "[rule]", "[kill_rule]", "[selobject]", "[selspline]"};
    enum class ModifierTarget { None, SceneryObject, AttachedObject, SplineAttachment, Spline };
    ModifierTarget modifierTarget = ModifierTarget::None;
    std::size_t modifierTargetIndex = 0;
    for (std::size_t cursor = 0; cursor < lines.size(); ++cursor) {
        const std::string section = trim(lines[cursor]);
        if (section.size() > 2 && section.front() == '[' && section.back() == ']') {
            const std::string keyword = section.substr(1, section.size() - 2);
            if (knownTileSections.contains(section)) {
                const char* ignoredReason = nullptr;
                if (section == "[variable_terrain]") {
                    ignoredReason = "dynamic terrain behavior is not implemented";
                } else if (section == "[variable_terrainlightmap]") {
                    ignoredReason = "dynamic terrain lightmap behavior is not implemented";
                } else if (section == "[spline_terrain_align_2]") {
                    ignoredReason = "record is retained but its runtime behavior is not implemented";
                } else if ((section == "[selobject]" || section == "[selspline]") &&
                           chronoData == nullptr) {
                    ignoredReason = "Chrono selector records are only applied to Chrono map tiles";
                }
                openbus::config::logConfigurationTagStatus(
                    "map tile", tile.textPath, cursor + 1, keyword,
                    ignoredReason == nullptr
                        ? openbus::config::ConfigurationTagStatus::Active
                        : openbus::config::ConfigurationTagStatus::Ignored,
                    ignoredReason == nullptr ? "map tile record has runtime handling"
                                             : ignoredReason);
            } else {
                openbus::config::logConfigurationTagStatus(
                    "map tile", tile.textPath, cursor + 1, keyword,
                    openbus::config::ConfigurationTagStatus::Unknown,
                    "unknown map tile section");
            }
        }
        const bool isModifier = section == "[varparent]" || section == "[spline_terrain_align]" ||
                                section == "[spline_terrain_align_2]" || section == "[rule]" ||
                                section == "[kill_rule]";
        if (!isModifier && section.size() > 2 && section.front() == '[' && section.back() == ']') {
            modifierTarget = ModifierTarget::None;
        }
        if (section == "[version]") {
            if (cursor + 1 >= lines.size()) {
                throw std::runtime_error("Truncated [version] in " + pathToUtf8(tile.textPath));
            }
            data.version = trim(lines[++cursor]);
            if (chronoData != nullptr) {
                chronoData->version = data.version;
            }
        } else if (chronoData != nullptr &&
                   (section == "[selobject]" || section == "[selspline]")) {
            MapChronoSelector selector;
            selector.keyword = section.substr(1, section.size() - 2);
            selector.sectionLine = cursor + 1;
            std::size_t field = cursor + 1;
            if (field < lines.size() && !isChronoRecordBoundary(trim(lines[field]))) {
                selector.rawId = lines[field];
                selector.idLine = field + 1;
                int selectorId = 0;
                if (parseInteger(trim(selector.rawId), selectorId)) {
                    selector.id = selectorId;
                } else {
                    data.diagnostics.warning(selector.idLine, selector.keyword,
                                             "selector ID is not an integer; raw text retained");
                }
                ++field;
            } else {
                data.diagnostics.warning(selector.sectionLine, selector.keyword,
                                         "selector ID is missing; source lines retained");
            }
            while (field < lines.size() && !isChronoRecordBoundary(trim(lines[field]))) {
                selector.followingLines.push_back(lines[field++]);
            }
            chronoData->selectors.push_back(std::move(selector));
            cursor = field - 1;
        } else if (section == "[terrain]") {
            data.hasTerrainMarker = true;
        } else if (section == "[water]") {
            data.hasWaterMarker = true;
        } else if (section == "[variable_terrain]") {
            data.hasVariableTerrainMarker = true;
        } else if (section == "[variable_terrainlightmap]") {
            data.hasVariableTerrainLightmapMarker = true;
        } else if (section == "[spline]" || section == "[spline_h]") {
            const bool elevated = section == "[spline_h]";
            const int tileVersion = currentTileVersion();
            const auto hasField = [tileVersion](int version) {
                return tileVersion == 0 || tileVersion >= version;
            };
            std::size_t field = cursor + 1;
            MapSplinePlacement spline;
            spline.elevated = elevated;
            std::size_t rawFieldCount = 0;
            const auto readSplineField = [&]() -> const std::string& {
                if (field >= lines.size()) {
                    throw std::runtime_error("Truncated " + section + " record in " +
                                             pathToUtf8(tile.textPath));
                }
                const std::string& value = lines[field++];
                if (rawFieldCount < spline.rawFields.size()) {
                    spline.rawFields[rawFieldCount++] = trim(value);
                }
                return value;
            };
            if (hasField(9)) {
                readSplineField(); // detail level
            }
            spline.assetPath = trim(readSplineField());
            if (hasField(6)) {
                if (!parseInteger(trim(readSplineField()), spline.splineId)) {
                    throw std::runtime_error("Invalid spline ID in " + pathToUtf8(tile.textPath));
                }
            } else {
                spline.splineId = nextAutomaticRecordId();
            }
            if (hasField(11)) {
                if (!parseInteger(trim(readSplineField()), spline.previousSplineId) ||
                    !parseInteger(trim(readSplineField()), spline.nextSplineId)) {
                    throw std::runtime_error("Invalid spline links in " +
                                             pathToUtf8(tile.textPath));
                }
            } else {
                int linkedToPrevious = -1;
                if (!parseInteger(trim(readSplineField()), linkedToPrevious)) {
                    throw std::runtime_error("Invalid legacy spline link in " +
                                             pathToUtf8(tile.textPath));
                }
                if (linkedToPrevious != -1 && !data.splines.empty()) {
                    spline.previousSplineId = data.splines.back().splineId;
                    data.splines.back().nextSplineId = spline.splineId;
                }
            }
            bool numericFieldsValid = true;
            const auto readSplineReal = [&](double& destination) {
                const bool valid = parseReal(trim(readSplineField()), destination);
                numericFieldsValid = valid && numericFieldsValid;
                return valid;
            };
            readSplineReal(spline.localX);
            readSplineReal(spline.elevation);
            readSplineReal(spline.localY);
            readSplineReal(spline.rotationDegrees);
            readSplineReal(spline.length);
            readSplineReal(spline.radius);
            readSplineReal(spline.gradientStart);
            readSplineReal(spline.gradientEnd);
            if (elevated) {
                readSplineReal(spline.heightDelta);
            }
            if (hasField(5)) {
                // The map editor serializes cant directly in percentage points.
                readSplineReal(spline.cantStart);
                readSplineReal(spline.cantEnd);
            }
            // Versions 14+ add skew at both ends before the v11+ chain texture offset.
            if (hasField(14)) {
                readSplineReal(spline.skewStart);
                readSplineReal(spline.skewEnd);
            }
            if (hasField(11)) {
                spline.chainOffsetValid = parseReal(trim(readSplineField()), spline.chainOffset);
            }
            spline.geometryValid = numericFieldsValid;
            // [mirror] is a bare marker after the numeric record, not a fixed-width field.
            if (hasField(7) && field < lines.size() && trim(lines[field]) == "mirror") {
                spline.mirrored = true;
                readSplineField();
            }
            if (elevated) {
                ++data.elevatedSplineCount;
            } else {
                ++data.splineCount;
            }
            data.splines.push_back(std::move(spline));
            modifierTarget = ModifierTarget::Spline;
            modifierTargetIndex = data.splines.size() - 1;
            cursor = field - 1;
        } else if (section == "[object]") {
            ++data.objectCount;
            const int tileVersion = currentTileVersion();
            const auto hasField = [tileVersion](int version) {
                return tileVersion == 0 || tileVersion >= version;
            };
            std::size_t field = cursor + 1;
            const auto requireObjectField = [&](std::size_t at) -> const std::string& {
                if (at >= lines.size()) {
                    throw std::runtime_error("Truncated [object] record in " +
                                             pathToUtf8(tile.textPath));
                }
                return lines[at];
            };
            MapSceneryPlacement object;
            object.label = cursor == 0 ? std::string{} : trim(lines[cursor - 1]);
            object.editorObjectNumber = parseEditorObjectNumber(object.label);
            object.line1 = trim(requireObjectField(field));
            if (hasField(9)) {
                if (!parseInteger(trim(requireObjectField(field++)), object.detailLevel)) {
                    throw std::runtime_error("Invalid [object] detail level in " +
                                             pathToUtf8(tile.textPath));
                }
            }
            object.assetPath = trim(requireObjectField(field++));
            if (hasField(6)) {
                if (!parseInteger(trim(requireObjectField(field++)), object.id)) {
                    throw std::runtime_error("Invalid [object] ID in " + pathToUtf8(tile.textPath));
                }
            } else {
                object.id = nextAutomaticRecordId();
            }
            object.transformValid = true;
            for (std::size_t axis = 0; axis < object.localPosition.size(); ++axis) {
                object.rawTransformFields[axis] = trim(requireObjectField(field++));
                object.transformValid =
                    parseReal(object.rawTransformFields[axis], object.localPosition[axis]) &&
                    object.transformValid;
            }
            const std::size_t rotationFields = hasField(12) ? 3 : (hasField(8) ? 1 : 0);
            for (std::size_t axis = 0; axis < rotationFields; ++axis) {
                object.rawTransformFields[axis + 3] = trim(requireObjectField(field++));
                object.transformValid =
                    parseReal(object.rawTransformFields[axis + 3], object.rotationDegrees[axis]) &&
                    object.transformValid;
            }
            for (std::size_t axis = rotationFields; axis < object.rotationDegrees.size(); ++axis) {
                object.rawTransformFields[axis + 3] = "0";
                object.rotationDegrees[axis] = 0.0;
            }
            if (hasField(4)) {
                std::size_t labelCount = 0;
                object.trailingField = trim(requireObjectField(field++));
                if (!parseSize(object.trailingField, labelCount) || labelCount > 4096 ||
                    labelCount > lines.size() - std::min(field, lines.size())) {
                    throw std::runtime_error("Invalid [object] label count in " +
                                             pathToUtf8(tile.textPath));
                }
                object.labels.reserve(labelCount);
                for (std::size_t index = 0; index < labelCount; ++index) {
                    object.labels.push_back(requireObjectField(field++));
                }
            }
            data.sceneryObjects.push_back(std::move(object));
            modifierTarget = ModifierTarget::SceneryObject;
            modifierTargetIndex = data.sceneryObjects.size() - 1;
            cursor = field - 1;
        } else if (section == "[attachObj]") {
            const int tileVersion = currentTileVersion();
            std::size_t field = cursor + 1;
            const auto requireField = [&](std::size_t at) -> const std::string& {
                if (at >= lines.size()) {
                    throw std::runtime_error("Truncated [attachObj] record in " +
                                             pathToUtf8(tile.textPath));
                }
                return lines[at];
            };
            MapAttachedObject object;
            object.label = cursor == 0 ? std::string{} : trim(lines[cursor - 1]);
            object.editorObjectNumber = parseEditorObjectNumber(object.label);
            object.line1 = trim(requireField(field++));
            object.assetPath = trim(requireField(field++));
            object.hasExplicitId = tileVersion >= 6;
            if (object.hasExplicitId) {
                if (!parseInteger(trim(requireField(field++)), object.id)) {
                    throw std::runtime_error("Invalid [attachObj] ID in " +
                                             pathToUtf8(tile.textPath));
                }
            } else {
                object.id = nextAutomaticRecordId();
            }
            if (!parseInteger(trim(requireField(field++)), object.attachedToObjectId)) {
                throw std::runtime_error("Invalid [attachObj] parent object ID in " +
                                         pathToUtf8(tile.textPath));
            }
            object.line5 = trim(requireField(field++));
            object.attachPointIndex = trim(requireField(field++));
            object.transformValid =
                parseInteger(object.attachPointIndex, object.attachPointIndexValue);
            for (std::size_t axis = 0; axis < object.rotationFields.size(); ++axis) {
                object.rotationFields[axis] = trim(requireField(field++));
                object.transformValid =
                    parseReal(object.rotationFields[axis], object.rotationDegrees[axis]) &&
                    object.transformValid;
            }
            object.labelCount = trim(requireField(field++));
            std::size_t labelCount = 0;
            if (parseSize(object.labelCount, labelCount) && labelCount <= 4096 &&
                labelCount <= lines.size() - std::min(field, lines.size())) {
                object.labels.reserve(labelCount);
                for (std::size_t index = 0; index < labelCount; ++index) {
                    object.labels.push_back(requireField(field++));
                }
            }
            // Preserve any remaining object-specific tail while leaving the next record's
            // description for its parser.
            while (field < lines.size()) {
                const std::string candidate = trim(lines[field]);
                if (candidate.size() > 2 && candidate.front() == '[' && candidate.back() == ']') {
                    break;
                }
                if (field + 1 < lines.size()) {
                    const std::string next = trim(lines[field + 1]);
                    if (next == "[object]" || next == "[attachObj]" ||
                        next == "[splineAttachement]" || next == "[splineAttachement_repeater]") {
                        break;
                    }
                }
                object.optionalLines.push_back(lines[field++]);
            }
            data.attachedObjects.push_back(std::move(object));
            modifierTarget = ModifierTarget::AttachedObject;
            modifierTargetIndex = data.attachedObjects.size() - 1;
            cursor = field - 1;
        } else if (section == "[splineAttachement]" || section == "[splineAttachement_repeater]") {
            const bool repeater = section == "[splineAttachement_repeater]";
            if (repeater) {
                ++data.splineRepeaterCount;
            } else {
                ++data.splineAttachmentCount;
            }
            const int tileVersion = currentTileVersion();
            if (tileVersion < 6) {
                static_cast<void>(nextAutomaticRecordId());
            }
            const bool hasDetail = tileVersion == 0 || tileVersion >= 9;
            const bool modernRotation = tileVersion == 0 || tileVersion >= 12;
            const bool hasStrings = tileVersion == 0 || tileVersion >= 4;
            std::size_t field = cursor + 1 + (hasDetail ? 1 : 0);
            const auto requireField = [&](std::size_t at) -> const std::string& {
                if (at >= lines.size()) {
                    throw std::runtime_error("Truncated " + section + " record in " +
                                             pathToUtf8(tile.textPath));
                }
                return lines[at];
            };
            MapSplineAttachment attachment;
            attachment.label = cursor == 0 ? std::string{} : trim(lines[cursor - 1]);
            attachment.editorObjectNumber = parseEditorObjectNumber(attachment.label);
            attachment.repeater = repeater;
            if (repeater) {
                if (!parseInteger(trim(requireField(field++)),
                                  attachment.repeaterMasterTileIndex) ||
                    !parseSize(trim(requireField(field++)), attachment.repeaterFirstObjectIndex)) {
                    throw std::runtime_error("Invalid repeater index in " +
                                             pathToUtf8(tile.textPath));
                }
            }
            attachment.assetPath = trim(requireField(field++));
            if (!parseInteger(trim(requireField(field++)), attachment.id) ||
                !parseInteger(trim(requireField(field++)), attachment.splineIndex)) {
                throw std::runtime_error("Invalid ID or spline index in " + section + " in " +
                                         pathToUtf8(tile.textPath));
            }
            attachment.transformValid = true;
            for (double& offset : attachment.offset) {
                attachment.transformValid =
                    parseReal(trim(requireField(field++)), offset) && attachment.transformValid;
            }
            const std::size_t rotationCount = modernRotation ? 3 : 1;
            for (std::size_t axis = 0; axis < rotationCount; ++axis) {
                attachment.transformValid =
                    parseReal(trim(requireField(field++)), attachment.rotationDegrees[axis]) &&
                    attachment.transformValid;
            }
            if (!parseReal(trim(requireField(field++)), attachment.interval) ||
                !parseReal(trim(requireField(field++)), attachment.range)) {
                attachment.transformValid = false;
            }
            if (modernRotation) {
                int tilt = 0;
                attachment.transformValid =
                    parseInteger(trim(requireField(field++)), tilt) && attachment.transformValid;
                attachment.tilt = tilt != 0;
            }
            if (hasStrings) {
                std::size_t stringCount = 0;
                if (!parseSize(trim(requireField(field++)), stringCount) || stringCount > 4096 ||
                    stringCount > lines.size() - std::min(field, lines.size())) {
                    throw std::runtime_error("Invalid label count in " + section + " in " +
                                             pathToUtf8(tile.textPath));
                }
                attachment.labels.reserve(stringCount);
                for (std::size_t index = 0; index < stringCount; ++index) {
                    attachment.labels.push_back(requireField(field++));
                }
            }
            data.splineAttachments.push_back(std::move(attachment));
            modifierTarget = ModifierTarget::SplineAttachment;
            modifierTargetIndex = data.splineAttachments.size() - 1;
            cursor = field - 1;
        } else if (section == "[varparent]") {
            if (cursor + 1 >= lines.size()) {
                throw std::runtime_error("Truncated [varparent] in " + pathToUtf8(tile.textPath));
            }
            int parentId = 0;
            if (!parseInteger(trim(lines[cursor + 1]), parentId)) {
                throw std::runtime_error("Invalid [varparent] ID in " + pathToUtf8(tile.textPath));
            }
            switch (modifierTarget) {
            case ModifierTarget::SceneryObject:
                data.sceneryObjects[modifierTargetIndex].variableParentId = parentId;
                break;
            case ModifierTarget::AttachedObject:
                data.attachedObjects[modifierTargetIndex].variableParentId = parentId;
                break;
            case ModifierTarget::SplineAttachment:
                data.splineAttachments[modifierTargetIndex].variableParentId = parentId;
                break;
            case ModifierTarget::None:
                data.diagnostics.warning(cursor + 1, "varparent",
                                         "modifier has no supported preceding tile object");
                break;
            case ModifierTarget::Spline:
                data.diagnostics.warning(cursor + 1, "varparent",
                                         "modifier is not valid for a spline record");
                break;
            }
            ++cursor;
        } else if (section == "[spline_terrain_align]") {
            switch (modifierTarget) {
            case ModifierTarget::SceneryObject:
                data.sceneryObjects[modifierTargetIndex].splineTerrainAlign = true;
                break;
            case ModifierTarget::AttachedObject:
                data.attachedObjects[modifierTargetIndex].splineTerrainAlign = true;
                break;
            case ModifierTarget::SplineAttachment:
                data.splineAttachments[modifierTargetIndex].splineTerrainAlign = true;
                break;
            case ModifierTarget::None:
            case ModifierTarget::Spline:
                data.diagnostics.warning(cursor + 1, "spline_terrain_align",
                                         "marker has no supported preceding object record");
                break;
            }
        } else if (section == "[spline_terrain_align_2]") {
            if (cursor + 1 >= lines.size()) {
                throw std::runtime_error("Truncated [spline_terrain_align_2] in " +
                                         pathToUtf8(tile.textPath));
            }
            if (modifierTarget == ModifierTarget::Spline) {
                data.splines[modifierTargetIndex].splineTerrainAlign2 = lines[cursor + 1];
            } else {
                data.diagnostics.warning(cursor + 1, "spline_terrain_align_2",
                                         "record has no supported preceding spline");
            }
            ++cursor;
        } else if (section == "[rule]" || section == "[kill_rule]") {
            MapTileRule rule;
            rule.kill = section == "[kill_rule]";
            for (std::size_t index = 0; index < rule.fields.size(); ++index) {
                const std::size_t field = cursor + index + 1;
                if (field >= lines.size()) {
                    throw std::runtime_error("Truncated " + section + " record in " +
                                             pathToUtf8(tile.textPath));
                }
                rule.fields[index] = lines[field];
            }
            switch (modifierTarget) {
            case ModifierTarget::SceneryObject:
                data.sceneryObjects[modifierTargetIndex].rules.push_back(std::move(rule));
                break;
            case ModifierTarget::AttachedObject:
                data.attachedObjects[modifierTargetIndex].rules.push_back(std::move(rule));
                break;
            case ModifierTarget::SplineAttachment:
                data.splineAttachments[modifierTargetIndex].rules.push_back(std::move(rule));
                break;
            case ModifierTarget::Spline:
                data.splines[modifierTargetIndex].rules.push_back(std::move(rule));
                break;
            case ModifierTarget::None:
                data.diagnostics.warning(cursor + 1, section.substr(1, section.size() - 2),
                                         "modifier has no supported preceding tile record");
                break;
            }
            cursor += rule.fields.size();
        } else if (section.size() > 2 && section.front() == '[' && section.back() == ']') {
            openbus::config::logUnhandledConfigurationTag("map tile", tile.textPath, cursor + 1,
                                                          section.substr(1, section.size() - 2),
                                                          "section is not implemented");
            data.diagnostics.warning(cursor + 1, section.substr(1, section.size() - 2),
                                     "map tile section is not implemented");
            MapOpaqueSection opaqueSection;
            opaqueSection.keyword = section.substr(1, section.size() - 2);
            opaqueSection.sectionLine = cursor + 1;
            std::size_t field = cursor + 1;
            while (field < lines.size()) {
                const std::string candidate = trim(lines[field]);
                if (candidate.size() > 2 && candidate.front() == '[' && candidate.back() == ']') {
                    break;
                }
                opaqueSection.payloadLines.push_back(lines[field++]);
            }
            data.unsupportedSections.push_back(std::move(opaqueSection));
            cursor = field - 1;
        }
    }
    if (data.version.empty()) {
        throw std::runtime_error("No [version] section found in " + pathToUtf8(tile.textPath));
    }
    if (chronoData == nullptr && !data.hasTerrainMarker) {
        throw std::runtime_error("No [terrain] section found in " + pathToUtf8(tile.textPath));
    }
    if (chronoData != nullptr) {
        chronoData->diagnostics = data.diagnostics;
    }
    return data;
}

MapTileData loadMapTile(const MapTileReference& tile) {
    if (!tile.chronoBaseAssets.has_value()) {
        return loadMapTileInternal(tile, nullptr);
    }

    std::vector<std::filesystem::path> overlayPaths = tile.chronoOverlayPaths;
    if (overlayPaths.empty() && tile.textPath != tile.chronoBaseAssets->textPath) {
        overlayPaths.push_back(tile.textPath);
    }
    if (overlayPaths.empty()) {
        return loadMapTileInternal(tile, nullptr);
    }

    MapTileReference baseReference = tile;
    baseReference.textPath = tile.chronoBaseAssets->textPath;
    baseReference.chronoBaseAssets.reset();
    baseReference.chronoOverlayPaths.clear();
    MapTileData baseData = loadMapTileInternal(baseReference, nullptr);

    for (const std::filesystem::path& overlayPath : overlayPaths) {
        MapTileReference overlayReference = tile;
        overlayReference.textPath = overlayPath;
        overlayReference.chronoBaseAssets.reset();
        overlayReference.chronoOverlayPaths.clear();
        MapChronoTileData chronoData;
        MapTileData overlayData = loadMapTileInternal(overlayReference, &chronoData);
        for (const MapSplinePlacement& spline : overlayData.splines) {
            const auto existing = std::find_if(
                baseData.splines.begin(), baseData.splines.end(),
                [&](const MapSplinePlacement& candidate) {
                    return candidate.splineId == spline.splineId;
                });
            if (existing == baseData.splines.end()) {
                baseData.splines.push_back(spline);
            } else {
                *existing = spline;
            }
        }
        for (const MapSceneryPlacement& object : overlayData.sceneryObjects) {
            const auto matches = [&](const auto& candidate) { return candidate.id == object.id; };
            std::erase_if(baseData.attachedObjects, matches);
            std::erase_if(baseData.splineAttachments, matches);
            const auto existing = std::find_if(baseData.sceneryObjects.begin(),
                                                baseData.sceneryObjects.end(), matches);
            if (existing == baseData.sceneryObjects.end()) {
                baseData.sceneryObjects.push_back(object);
            } else {
                *existing = object;
            }
        }
        for (const MapAttachedObject& object : overlayData.attachedObjects) {
            const auto matches = [&](const auto& candidate) { return candidate.id == object.id; };
            std::erase_if(baseData.sceneryObjects, matches);
            std::erase_if(baseData.splineAttachments, matches);
            const auto existing = std::find_if(baseData.attachedObjects.begin(),
                                                baseData.attachedObjects.end(), matches);
            if (existing == baseData.attachedObjects.end()) {
                baseData.attachedObjects.push_back(object);
            } else {
                *existing = object;
            }
        }
        for (const MapSplineAttachment& attachment : overlayData.splineAttachments) {
            const auto matches = [&](const auto& candidate) {
                return candidate.id == attachment.id;
            };
            std::erase_if(baseData.sceneryObjects, matches);
            std::erase_if(baseData.attachedObjects, matches);
            const auto existing = std::find_if(baseData.splineAttachments.begin(),
                                                baseData.splineAttachments.end(), matches);
            if (existing == baseData.splineAttachments.end()) {
                baseData.splineAttachments.push_back(attachment);
            } else {
                *existing = attachment;
            }
        }
        for (const MapChronoSelector& selector : chronoData.selectors) {
            applyMapChronoSelector(baseData, selector, overlayPath);
        }

        baseData.unsupportedSections.insert(baseData.unsupportedSections.end(),
                                            overlayData.unsupportedSections.begin(),
                                            overlayData.unsupportedSections.end());
        baseData.diagnostics.entries.insert(baseData.diagnostics.entries.end(),
                                            overlayData.diagnostics.entries.begin(),
                                            overlayData.diagnostics.entries.end());
    }
    baseData.splineCount = static_cast<std::size_t>(std::count_if(
        baseData.splines.begin(), baseData.splines.end(),
        [](const MapSplinePlacement& spline) { return !spline.elevated; }));
    baseData.elevatedSplineCount = baseData.splines.size() - baseData.splineCount;
    baseData.objectCount = baseData.sceneryObjects.size();
    baseData.splineAttachmentCount = static_cast<std::size_t>(std::count_if(
        baseData.splineAttachments.begin(), baseData.splineAttachments.end(),
        [](const MapSplineAttachment& attachment) { return !attachment.repeater; }));
    baseData.splineRepeaterCount =
        baseData.splineAttachments.size() - baseData.splineAttachmentCount;
    baseData.reference = tile;
    return baseData;
}

MapChronoTileData loadChronoTile(const std::filesystem::path& chronoTilePath) {
    MapTileReference tile;
    tile.textPath = chronoTilePath;
    MapChronoTileData chronoData;
    static_cast<void>(loadMapTileInternal(tile, &chronoData));
    return chronoData;
}

TerrainGrid loadTerrainGrid(const std::filesystem::path& terrainPath) {
    openbus::rendering::TraceScope trace("map", "MapConfigLoader::loadTerrainGrid");
    std::ifstream input(terrainPath, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not open terrain file: " + pathToUtf8(terrainPath));
    }
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)),
                                           std::istreambuf_iterator<char>());
    if (bytes.size() < 4) {
        throw std::runtime_error("Terrain file is shorter than its header: " +
                                 pathToUtf8(terrainPath));
    }
    const std::uint32_t intervals = readU32(bytes, 0);
    if (intervals == 0 || intervals > 1024U) {
        throw std::runtime_error("Invalid terrain grid interval count in " +
                                 pathToUtf8(terrainPath));
    }
    const std::uint64_t vertexCount = static_cast<std::uint64_t>(intervals + 1U) * (intervals + 1U);
    const std::uint64_t expectedBytes = 4U + vertexCount * 4U;
    if (expectedBytes != bytes.size()) {
        throw std::runtime_error("Terrain file size does not match its grid header: " +
                                 pathToUtf8(terrainPath));
    }

    TerrainGrid grid;
    grid.intervals = intervals;
    grid.heights.reserve(static_cast<std::size_t>(vertexCount));
    for (std::size_t offset = 4; offset < bytes.size(); offset += 4) {
        const float height = std::bit_cast<float>(readU32(bytes, offset));
        if (!std::isfinite(height)) {
            throw std::runtime_error("Non-finite terrain height in " + pathToUtf8(terrainPath));
        }
        grid.heights.push_back(height);
    }
    return grid;
}

WaterData loadWaterData(const std::filesystem::path& waterPath) {
    openbus::rendering::TraceScope trace("map", "MapConfigLoader::loadWaterData");
    std::ifstream input(waterPath, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not open water file: " + pathToUtf8(waterPath));
    }
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)),
                                           std::istreambuf_iterator<char>());
    if (bytes.size() < 4) {
        throw std::runtime_error("Water file is shorter than its header: " + pathToUtf8(waterPath));
    }
    const std::uint32_t surfaceCount = readU32(bytes, 0);
    constexpr std::uint32_t MAX_WATER_SURFACES = 1'000'000;
    if (surfaceCount > MAX_WATER_SURFACES) {
        throw std::runtime_error("Water surface count exceeds the supported limit in " +
                                 pathToUtf8(waterPath));
    }
    const std::uint64_t expectedBytes = 4U + static_cast<std::uint64_t>(surfaceCount) * 16U;
    if (bytes.size() != expectedBytes) {
        throw std::runtime_error("Water file size does not match its surface count: " +
                                 pathToUtf8(waterPath));
    }

    WaterData water;
    water.surfaces.reserve(surfaceCount);
    std::size_t offset = 4;
    for (std::uint32_t index = 0; index < surfaceCount; ++index) {
        WaterSurface surface;
        for (float& height : surface.heights) {
            height = std::bit_cast<float>(readU32(bytes, offset));
            offset += 4;
            if (!std::isfinite(height)) {
                throw std::runtime_error("Non-finite water height in " + pathToUtf8(waterPath));
            }
        }
        water.surfaces.push_back(surface);
    }
    return water;
}

} // namespace openbus::map
