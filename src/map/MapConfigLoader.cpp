#include "map/MapConfigLoader.h"

#include "config/ConfigurationParser.h"
#include "core/Logger.h"
#include "core/PerfTrace.h"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace openbus::map {

namespace {

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
        throw std::runtime_error("Could not open map text file: " + path.string());
    }
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)),
                                           std::istreambuf_iterator<char>());
    if (bytes.size() >= 2 && bytes[0] == 0xffU && bytes[1] == 0xfeU) {
        if ((bytes.size() - 2) % 2 != 0) {
            throw std::runtime_error("Odd-length UTF-16LE map text file: " + path.string());
        }
        std::string output;
        for (std::size_t offset = 2; offset < bytes.size(); offset += 2) {
            std::uint32_t codepoint = static_cast<std::uint32_t>(bytes[offset]) |
                                      (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U);
            if (codepoint >= 0xd800U && codepoint <= 0xdbffU) {
                if (offset + 3 >= bytes.size()) {
                    throw std::runtime_error("Unpaired UTF-16 surrogate in: " + path.string());
                }
                const std::uint32_t low = static_cast<std::uint32_t>(bytes[offset + 2]) |
                                          (static_cast<std::uint32_t>(bytes[offset + 3]) << 8U);
                if (low < 0xdc00U || low > 0xdfffU) {
                    throw std::runtime_error("Unpaired UTF-16 surrogate in: " + path.string());
                }
                codepoint = 0x10000U + ((codepoint - 0xd800U) << 10U) + (low - 0xdc00U);
                offset += 2;
            } else if (codepoint >= 0xdc00U && codepoint <= 0xdfffU) {
                throw std::runtime_error("Unpaired UTF-16 surrogate in: " + path.string());
            }
            appendUtf8(output, codepoint);
        }
        return output;
    }
    if (bytes.size() >= 2 && bytes[0] == 0xfeU && bytes[1] == 0xffU) {
        throw std::runtime_error("UTF-16BE map text is not supported: " + path.string());
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
        throw std::runtime_error("Truncated [" + section + "] record in " + path.string());
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
    std::optional<std::size_t> currentDepotIndex;
    for (std::size_t cursor = 0; cursor < lines.size();) {
        const std::string section = lowerAscii(trim(lines[cursor]));
        ++cursor;
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

void parseAiTimetableRoutes(const std::filesystem::path& directory, MapAiTrafficData& aiTraffic,
                            ConfigurationDiagnostics& diagnostics) {
    if (!std::filesystem::is_directory(directory)) {
        return;
    }
    std::unordered_map<std::string, std::filesystem::path> trackFiles;
    std::vector<std::filesystem::path> tripFiles;
    std::vector<std::filesystem::path> lineFiles;
    std::optional<std::filesystem::path> busStopsPath;
    std::optional<std::filesystem::path> stationLinksPath;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(directory)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        std::string extension = lowerAscii(entry.path().extension().string());
        if (extension == ".ttr") {
            trackFiles.emplace(lowerAscii(entry.path().stem().string()), entry.path());
        } else if (extension == ".ttp") {
            tripFiles.push_back(entry.path());
        } else if (extension == ".ttl") {
            lineFiles.push_back(entry.path());
        } else if (lowerAscii(entry.path().filename().string()) == "busstops.cfg") {
            busStopsPath = entry.path();
        } else if (lowerAscii(entry.path().filename().string()) == "stnlinks.cfg") {
            stationLinksPath = entry.path();
        }
    }
    std::sort(tripFiles.begin(), tripFiles.end());
    std::sort(lineFiles.begin(), lineFiles.end());

    if (busStopsPath) {
        const std::vector<std::string> lines = splitLines(decodeText(*busStopsPath));
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
                                        busStopsPath->filename().string());
            }
        }
    }

    if (stationLinksPath) {
        const std::vector<std::string> lines = splitLines(decodeText(*stationLinksPath));
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
                                            stationLinksPath->filename().string());
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
                if (complete && numericEntryValid && !aiTraffic.stationLinks.empty()) {
                    std::copy(entryValues.begin() + 4, entryValues.end(),
                              entry.remainingFields.begin());
                    aiTraffic.stationLinks.back().entries.push_back(std::move(entry));
                } else {
                    diagnostics.warning(lineIndex + 1, "StnLink_entry",
                                        "invalid or orphaned station-link path entry in " +
                                            stationLinksPath->filename().string());
                }
            }
        }
    }

    for (const std::filesystem::path& tripPath : tripFiles) {
        const std::vector<std::string> tripLines = splitLines(decodeText(tripPath));
        MapAiTimetableRoute route;
        MapAiPassengerTimetable passengerTimetable;
        route.timetableKey = lowerAscii(tripPath.stem().string());
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
                } else {
                    diagnostics.warning(lineIndex + 1, "station",
                                        "invalid or truncated timetable station record in " +
                                            tripPath.filename().string());
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
                    lineIndex = valueLine;
                } else {
                    diagnostics.warning(lineIndex + 1, "station_typ2",
                                        "invalid or truncated timetable station ID in " +
                                            tripPath.filename().string());
                }
            }
        }
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
                                        " in " + tripPath.filename().string());
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
                                        trackFile->second.filename().string());
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
                                        trackFile->second.filename().string());
                continue;
            }
            route.trackEntries.push_back({splineId, pathIndex, tileIndex, lengthMeters});
            if (route.splineIds.empty() || route.splineIds.back() != splineId) {
                route.splineIds.push_back(splineId);
            }
            lineIndex = selectorLine - 1;
        }
        if (malformedTrackEntry || route.splineIds.size() < 2) {
            continue;
        }
        aiTraffic.timetableRoutes.push_back(std::move(route));
        if (!passengerTimetable.stationIds.empty()) {
            aiTraffic.passengerTimetables.push_back(std::move(passengerTimetable));
        }
    }

    for (const std::filesystem::path& linePath : lineFiles) {
        const std::vector<std::string> lineLines = splitLines(decodeText(linePath));
        for (std::size_t lineIndex = 0; lineIndex < lineLines.size(); ++lineIndex) {
            if (lowerAscii(trim(lineLines[lineIndex])) != "[addtrip]") {
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
                                        linePath.filename().string());
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
                                        linePath.filename().string());
            } else {
                if (route != aiTraffic.timetableRoutes.end()) {
                    route->scheduledDepartureMinutes.push_back(departureMinutes);
                }
                if (passengerTimetable != aiTraffic.passengerTimetables.end()) {
                    passengerTimetable->scheduledDepartureMinutes.push_back(departureMinutes);
                }
            }
            lineIndex = next > 0 ? next - 1 : lineIndex;
        }
    }
    for (MapAiTimetableRoute& route : aiTraffic.timetableRoutes) {
        std::sort(route.scheduledDepartureMinutes.begin(), route.scheduledDepartureMinutes.end());
    }
    for (MapAiPassengerTimetable& timetable : aiTraffic.passengerTimetables) {
        std::sort(timetable.scheduledDepartureMinutes.begin(),
                  timetable.scheduledDepartureMinutes.end());
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
        if (hasDrivingRoute || timetable.scheduledDepartureMinutes.empty()) {
            continue;
        }

        const std::vector<MapAiPassengerStopCall> calls =
            mapAiPassengerStopCalls(aiTraffic, timetable);
        MapAiTimetableRoute route;
        route.timetableKey = timetable.timetableKey;
        route.tripName = timetable.timetableKey;
        route.destinationName = timetable.timetableKey;
        route.lineName = timetable.timetableKey;
        route.stationCount = calls.size();
        route.stationIds = timetable.stationIds;
        route.scheduledDepartureMinutes = timetable.scheduledDepartureMinutes;
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
                station.trackEntryIndex = static_cast<int>(route.trackEntries.size());
                for (const MapAiStationLink::Entry& entry : link.entries) {
                    route.trackEntries.push_back(
                        {entry.splineId, entry.pathIndex, entry.tileIndex, entry.lengthMeters});
                    if (route.splineIds.empty() || route.splineIds.back() != entry.splineId) {
                        route.splineIds.push_back(entry.splineId);
                    }
                }
            } else if (!route.trackEntries.empty()) {
                station.trackEntryIndex = static_cast<int>(route.trackEntries.size());
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
        const std::string candidate = iterator->path().filename().string();
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
        throw std::runtime_error("Invalid Chrono date in " + configPath.string() +
                                 "; expected YYYYMMDD");
    }
    const int year = date / 10000;
    const unsigned month = static_cast<unsigned>((date / 100) % 100);
    const unsigned day = static_cast<unsigned>(date % 100);
    const std::chrono::year_month_day parsed{std::chrono::year{year}, std::chrono::month{month},
                                             std::chrono::day{day}};
    if (!parsed.ok()) {
        throw std::runtime_error("Invalid Chrono calendar date in " + configPath.string());
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

void applyChronoTileSidecars(
    MapTileReference& tile, const std::filesystem::path& variantPath,
    const std::unordered_map<std::string, std::vector<MapGroundTextureSidecar>>& sidecarsByTile) {
    const std::filesystem::path terrainPath(variantPath.string() + ".terrain");
    if (std::filesystem::is_regular_file(terrainPath)) {
        tile.terrainPath = terrainPath;
        tile.hasTerrainFile = true;
    }
    const std::filesystem::path waterPath(variantPath.string() + ".water");
    if (std::filesystem::is_regular_file(waterPath)) {
        tile.waterPath = waterPath;
        tile.hasWaterFile = true;
    }
    const std::filesystem::path lightmapPath(variantPath.string() + ".LM.bmp");
    if (std::filesystem::is_regular_file(lightmapPath)) {
        tile.lightmapPath = lightmapPath;
        tile.hasLightmapFile = true;
    }
    const auto sidecars = sidecarsByTile.find(lowerAscii(variantPath.filename().string()));
    if (sidecars != sidecarsByTile.end()) {
        tile.groundTextureSidecars = sidecars->second;
    }
}

} // namespace

std::size_t applyMapChronoTiles(MapDefinition& map, int dateYyyyMmDd) {
    const std::chrono::year_month_day date{
        std::chrono::year{dateYyyyMmDd / 10000},
        std::chrono::month{static_cast<unsigned>((dateYyyyMmDd / 100) % 100)},
        std::chrono::day{static_cast<unsigned>(dateYyyyMmDd % 100)}};
    if (!date.ok()) {
        throw std::invalid_argument("Map Chrono date must be a valid YYYYMMDD calendar date");
    }
    const std::filesystem::path chronoRoot = map.rootPath / "Chrono";
    std::error_code filesystemError;
    const bool chronoRootExists = std::filesystem::exists(chronoRoot, filesystemError);
    if (filesystemError) {
        throw std::filesystem::filesystem_error("Could not inspect map Chrono directory",
                                                chronoRoot, filesystemError);
    }
    if (!chronoRootExists) {
        return 0;
    }
    if (!std::filesystem::is_directory(chronoRoot, filesystemError) || filesystemError) {
        throw std::filesystem::filesystem_error("Map Chrono path is not a readable directory",
                                                chronoRoot, filesystemError);
    }

    std::vector<std::filesystem::path> configPaths;
    std::filesystem::recursive_directory_iterator iterator(chronoRoot, filesystemError);
    const std::filesystem::recursive_directory_iterator end;
    for (; iterator != end; iterator.increment(filesystemError)) {
        if (filesystemError) {
            throw std::filesystem::filesystem_error("Could not enumerate map Chrono directory",
                                                    chronoRoot, filesystemError);
        }
        std::error_code statusError;
        if (iterator->is_regular_file(statusError) && !statusError &&
            lowerAscii(iterator->path().filename().string()) == "chrono.cfg") {
            configPaths.push_back(iterator->path());
        }
    }
    if (filesystemError) {
        throw std::filesystem::filesystem_error("Could not enumerate map Chrono directory",
                                                chronoRoot, filesystemError);
    }
    std::sort(configPaths.begin(), configPaths.end(),
              [](const std::filesystem::path& left, const std::filesystem::path& right) {
                  return lowerAscii(left.generic_string()) < lowerAscii(right.generic_string());
              });

    std::unordered_map<std::string, std::size_t> tileIndexByFilename;
    for (std::size_t index = 0; index < map.tiles.size(); ++index) {
        tileIndexByFilename.emplace(lowerAscii(map.tiles[index].textPath.filename().string()),
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
                lowerAscii(tileIterator->path().extension().string()) != ".map") {
                continue;
            }
            const std::string tileFilename = lowerAscii(tileIterator->path().filename().string());
            const auto tileIndex = tileIndexByFilename.find(tileFilename);
            if (tileIndex == tileIndexByFilename.end()) {
                continue;
            }
            MapTileReference& tile = map.tiles[tileIndex->second];
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
    mapLog.Log("Reading manifest: " + globalPath.string());
    const std::vector<std::string> lines = splitLines(decodeText(globalPath));
    mapLog.Log("Discovering ground-texture sidecars in " + mapDirectory.string());
    const auto groundTextureSidecarsByTile = discoverGroundTextureSidecars(mapDirectory);
    std::unordered_map<std::uint64_t, std::size_t> tileIndicesByCoordinates;
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

    for (; cursor < lines.size();) {
        const std::size_t sectionLine = cursor;
        const std::string section = trim(lines[cursor]);
        ++cursor;
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
                throw std::runtime_error("Unterminated [description] in " + globalPath.string());
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
                                         globalPath.string());
            }
            definition.groundTextures.push_back(std::move(groundTexture));
        } else if (section == "[entrypoints]") {
            const std::string countText = readField(lines, cursor, globalPath, "entrypoints");
            int count = 0;
            if (!parseInteger(countText, count) || count < 0 || count > 100000) {
                throw std::runtime_error("Invalid [entrypoints] count in " + globalPath.string());
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
                                             std::to_string(index) + " in " + globalPath.string());
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
                                         globalPath.string());
            }
            int filenameX = 0;
            int filenameY = 0;
            parseTileFilename(filename, filenameX, filenameY);
            if (filenameX != tile.x || filenameY != tile.y) {
                throw std::runtime_error("[map] coordinates do not match tile filename: " +
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
            tile.textPath = mapDirectory / filename;
            if (!std::filesystem::is_regular_file(tile.textPath)) {
                throw std::runtime_error("Listed map tile does not exist: " +
                                         tile.textPath.string());
            }
            tile.terrainPath = std::filesystem::path(tile.textPath.string() + ".terrain");
            tile.hasTerrainFile = std::filesystem::is_regular_file(tile.terrainPath);
            tile.waterPath = std::filesystem::path(tile.textPath.string() + ".water");
            tile.hasWaterFile = std::filesystem::is_regular_file(tile.waterPath);
            tile.lightmapPath = std::filesystem::path(tile.textPath.string() + ".LM.bmp");
            tile.hasLightmapFile = std::filesystem::is_regular_file(tile.lightmapPath);
            tile.terrainReadinessPath =
                std::filesystem::path(tile.textPath.string() + ".terrain_0.rdy");
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

    if (definition.tiles.empty()) {
        throw std::runtime_error("No [map] tile records found in " + globalPath.string());
    }
    std::vector<int> tileRows;
    tileRows.reserve(definition.tiles.size());
    for (const MapTileReference& tile : definition.tiles) {
        tileRows.push_back(tile.y);
    }
    definition.coordinates =
        MapCoordinates::forRows(definition.metadata.hasWorldCoordinates, std::move(tileRows));
    for (MapEntryPoint& entryPoint : definition.entryPoints) {
        if (entryPoint.tileIndex < 0 ||
            static_cast<std::size_t>(entryPoint.tileIndex) >= tileOrdinalToUniqueIndex.size()) {
            throw std::runtime_error("[entrypoints] tile index is outside the [map] list in " +
                                     globalPath.string());
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
                                     globalPath.string());
        }
        const double quaternionNorm =
            std::hypot(std::hypot(quaternionX, quaternionY), std::hypot(quaternionZ, quaternionW));
        if (quaternionNorm <= std::numeric_limits<double>::epsilon()) {
            throw std::runtime_error("Zero orientation quaternion in [entrypoints] record in " +
                                     globalPath.string());
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
                                     globalPath.string());
        }
        const MapTileReference& tile = definition.tiles[entryPoint.tileIndex];
        const auto transform = definition.coordinates.tile(tile.x, tile.y);
        const auto position = transform.toWorld(localX, localY);
        entryPoint.placement = {
            {position[0], position[1], elevation},
            std::atan2(forwardNorth * transform.scaleY, forwardEast * transform.scaleX) * 180.0 /
                3.14159265358979323846};
    }
    const std::filesystem::path aiListPath = mapDirectory / "ailists.cfg";
    mapLog.Log("Manifest parsed: " + std::to_string(definition.tiles.size()) +
               " tiles; loading AI lists and population schedules.");
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
    mapLog.Log("Loading AI timetables: " + (mapDirectory / "TTData").string());
    parseAiTimetableRoutes(mapDirectory / "TTData", definition.aiTraffic, definition.diagnostics);
    mapLog.Log("AI timetables loaded; loading driver and pedestrian lists.");
    const std::filesystem::path driverListPath = mapDirectory / "drivers.txt";
    if (std::filesystem::is_regular_file(driverListPath)) {
        parseDriverListFile(driverListPath, definition.aiTraffic);
    }
    const std::filesystem::path humanListPath = mapDirectory / "humans.txt";
    if (std::filesystem::is_regular_file(humanListPath)) {
        parseHumanListFile(humanListPath, definition.aiTraffic);
    }
    mapLog.Log("Map manifest and traffic configuration loaded in " +
               std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - loadStart)
                                  .count()) +
               " ms.");
    return definition;
}

MapTicketPackDefinition loadMapTicketPackDefinition(const std::filesystem::path& ticketPackPath) {
    if (!std::filesystem::is_regular_file(ticketPackPath)) {
        throw std::runtime_error("Ticket pack is missing: " + ticketPackPath.string());
    }
    const std::vector<std::string> lines = splitLines(decodeText(ticketPackPath));
    MapTicketPackDefinition definition;
    for (std::size_t lineIndex = 0; lineIndex < lines.size(); ++lineIndex) {
        if (lowerAscii(trim(lines[lineIndex])) != "[ticket]") {
            continue;
        }
        std::array<std::string, 4> fields;
        std::size_t next = lineIndex + 1;
        bool complete = true;
        for (std::string& field : fields) {
            while (next < lines.size() && trim(lines[next]).empty()) {
                ++next;
            }
            if (next >= lines.size() || isSectionHeaderLine(trim(lines[next]))) {
                complete = false;
                break;
            }
            field = trim(lines[next++]);
        }
        MapTicketDefinition ticket;
        double value = 0.0;
        int type = 0;
        if (complete && !fields[0].empty() && parseReal(fields[2], value) && value >= 0.0 &&
            value <= static_cast<double>((std::numeric_limits<float>::max)()) &&
            parseInteger(fields[3], type) && type >= 0) {
            const auto unquote = [](std::string field) {
                if (field.size() >= 2 && field.front() == '"' && field.back() == '"') {
                    field = field.substr(1, field.size() - 2);
                }
                return field;
            };
            ticket.name = unquote(std::move(fields[0]));
            ticket.displayValue = unquote(std::move(fields[1]));
            ticket.value = value;
            ticket.type = type;
            definition.tickets.push_back(std::move(ticket));
            lineIndex = next - 1;
        } else {
            definition.diagnostics.warning(lineIndex + 1, "ticket",
                                           "invalid or incomplete ticket entry in " +
                                               ticketPackPath.filename().string());
        }
    }
    return definition;
}

MapCurrencyDefinition loadMapCurrencyDefinition(const std::filesystem::path& currencyPath) {
    if (!std::filesystem::is_regular_file(currencyPath)) {
        throw std::runtime_error("Currency configuration is missing: " + currencyPath.string());
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
                                                   currencyPath.filename().string());
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
                                                   currencyPath.filename().string());
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
        if (!groupIsConfigured &&
            (!traffic.unscheduledVehicleGroupNames.empty() ||
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
                vehicles.push_back({vehicle.assetPath, unscheduledGroupIndex,
                                    unscheduledGroupIndex,
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

std::vector<MapAiRoadTrafficGroupPopulation> mapAiMaximumRoadTrafficPoolByGroup(
    const MapAiTrafficData& traffic, const std::vector<MapAiRoadVehicle>& vehicles,
    std::size_t basePopulation, double densityMultiplier, std::size_t activePopulationCap) {
    if (!std::isfinite(densityMultiplier) || densityMultiplier < 0.0) {
        throw std::invalid_argument("AI road traffic pool density multiplier is invalid");
    }
    if (!traffic.hasUnscheduledTrafficDensityFile || basePopulation == 0 ||
        densityMultiplier == 0.0 || activePopulationCap == 0) {
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
    std::unordered_map<std::size_t, long double> maximumShareByGroup;
    for (const int dayOfWeekMask : {1, 2, 4}) {
        for (const double hour : candidateHours) {
            const double density =
                mapAiRoadTrafficDensityAtHour(traffic, vehicles, hour, dayOfWeekMask).value_or(0.0);
            const std::size_t targetPopulation = mapTrafficPopulationForDensity(
                basePopulation, density * densityMultiplier, activePopulationCap);
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
                maximumExpectedByGroup[groupIndex] =
                    std::max(maximumExpectedByGroup[groupIndex],
                             static_cast<long double>(basePopulation) * densityMultiplier *
                                 groupWeight / totalGroupFactors);
                maximumShareByGroup[groupIndex] =
                    std::max(maximumShareByGroup[groupIndex], groupWeight / totalWeight);
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
        const long double capBound = std::ceil(static_cast<long double>(activePopulationCap) *
                                               maximumShareByGroup[groupIndex]);
        const std::size_t boundedMaximum = static_cast<std::size_t>(
            std::min({densityBound, capBound,
                      static_cast<long double>(std::numeric_limits<std::size_t>::max())}));
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
                                           std::optional<double> density,
                                           std::size_t populationCap) {
    if (populationCap == 0 || basePopulation == 0) {
        return 0;
    }
    if (!density.has_value()) {
        return std::min(basePopulation, populationCap);
    }
    if (!std::isfinite(*density) || *density < 0.0) {
        throw std::invalid_argument("Map traffic density must be finite and non-negative");
    }
    const long double scaled =
        static_cast<long double>(basePopulation) * static_cast<long double>(*density);
    if (scaled >= static_cast<long double>(populationCap)) {
        return populationCap;
    }
    return static_cast<std::size_t>(std::floor(scaled + 0.5L));
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
            !std::isfinite(value) || value <= 0.0) {
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
    enum class ModifierTarget { None, SceneryObject, AttachedObject, SplineAttachment, Spline };
    ModifierTarget modifierTarget = ModifierTarget::None;
    std::size_t modifierTargetIndex = 0;
    for (std::size_t cursor = 0; cursor < lines.size(); ++cursor) {
        const std::string section = trim(lines[cursor]);
        if (chronoData != nullptr && section == "[object]") {
            // Chrono object payloads can contain selector-looking text; preserve the complete
            // source in rawLines, but do not reinterpret that tail as the selector inventory.
            openbus::config::logUnhandledConfigurationTag(
                "Chrono map tile", tile.textPath, cursor + 1, "object",
                "object inventory is retained as raw data and not interpreted");
            break;
        }
        if (chronoData != nullptr && section != "[version]" && section != "[selobject]" &&
            section != "[selspline]") {
            // Chrono tile data is retained verbatim. Only its selector records are typed here;
            // normal tile records and selector override keywords have separate semantics.
            if (section.size() > 2 && section.front() == '[' && section.back() == ']') {
                openbus::config::logUnhandledConfigurationTag(
                    "Chrono map tile", tile.textPath, cursor + 1,
                    section.substr(1, section.size() - 2),
                    "section retained as raw Chrono data; runtime behavior is not implemented");
            }
            continue;
        }
        const bool isModifier = section == "[varparent]" || section == "[spline_terrain_align]" ||
                                section == "[spline_terrain_align_2]" || section == "[rule]" ||
                                section == "[kill_rule]";
        if (!isModifier && section.size() > 2 && section.front() == '[' && section.back() == ']') {
            modifierTarget = ModifierTarget::None;
        }
        if (section == "[version]") {
            if (cursor + 1 >= lines.size()) {
                throw std::runtime_error("Truncated [version] in " + tile.textPath.string());
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
            openbus::config::logUnhandledConfigurationTag(
                "map tile", tile.textPath, cursor + 1, "variable_terrain",
                "dynamic terrain behavior is not implemented");
        } else if (section == "[variable_terrainlightmap]") {
            data.hasVariableTerrainLightmapMarker = true;
            openbus::config::logUnhandledConfigurationTag(
                "map tile", tile.textPath, cursor + 1, "variable_terrainlightmap",
                "dynamic terrain lightmap behavior is not implemented");
        } else if (section == "[spline]" || section == "[spline_h]") {
            const bool elevated = section == "[spline_h]";
            int tileVersion = 0;
            if (!data.version.empty() && !parseInteger(data.version, tileVersion)) {
                throw std::runtime_error("Invalid [version] in " + tile.textPath.string());
            }
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
                                             tile.textPath.string());
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
                    throw std::runtime_error("Invalid spline ID in " + tile.textPath.string());
                }
            } else {
                spline.splineId = -static_cast<int>(++automaticRecordId);
            }
            if (hasField(11)) {
                if (!parseInteger(trim(readSplineField()), spline.previousSplineId) ||
                    !parseInteger(trim(readSplineField()), spline.nextSplineId)) {
                    throw std::runtime_error("Invalid spline links in " + tile.textPath.string());
                }
            } else {
                int linkedToPrevious = -1;
                if (!parseInteger(trim(readSplineField()), linkedToPrevious)) {
                    throw std::runtime_error("Invalid legacy spline link in " +
                                             tile.textPath.string());
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
            int tileVersion = 0;
            if (!data.version.empty() && !parseInteger(data.version, tileVersion)) {
                throw std::runtime_error("Invalid [version] in " + tile.textPath.string());
            }
            const auto hasField = [tileVersion](int version) {
                return tileVersion == 0 || tileVersion >= version;
            };
            std::size_t field = cursor + 1;
            const auto requireObjectField = [&](std::size_t at) -> const std::string& {
                if (at >= lines.size()) {
                    throw std::runtime_error("Truncated [object] record in " +
                                             tile.textPath.string());
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
                                             tile.textPath.string());
                }
            }
            object.assetPath = trim(requireObjectField(field++));
            if (hasField(6)) {
                if (!parseInteger(trim(requireObjectField(field++)), object.id)) {
                    throw std::runtime_error("Invalid [object] ID in " + tile.textPath.string());
                }
            } else {
                object.id = -static_cast<int>(++automaticRecordId);
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
                                             tile.textPath.string());
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
            int tileVersion = 0;
            if (!data.version.empty() && !parseInteger(data.version, tileVersion)) {
                throw std::runtime_error("Invalid [version] in " + tile.textPath.string());
            }
            std::size_t field = cursor + 1;
            const auto requireField = [&](std::size_t at) -> const std::string& {
                if (at >= lines.size()) {
                    throw std::runtime_error("Truncated [attachObj] record in " +
                                             tile.textPath.string());
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
                    throw std::runtime_error("Invalid [attachObj] ID in " + tile.textPath.string());
                }
            } else {
                object.id = -static_cast<int>(++automaticRecordId);
            }
            if (!parseInteger(trim(requireField(field++)), object.attachedToObjectId)) {
                throw std::runtime_error("Invalid [attachObj] parent object ID in " +
                                         tile.textPath.string());
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
            const int tileVersion = data.version.empty() ? 0 : std::stoi(data.version);
            if (tileVersion < 6) {
                ++automaticRecordId;
            }
            const bool hasDetail = tileVersion == 0 || tileVersion >= 9;
            const bool modernRotation = tileVersion == 0 || tileVersion >= 12;
            const bool hasStrings = tileVersion == 0 || tileVersion >= 4;
            std::size_t field = cursor + 1 + (hasDetail ? 1 : 0);
            const auto requireField = [&](std::size_t at) -> const std::string& {
                if (at >= lines.size()) {
                    throw std::runtime_error("Truncated " + section + " record in " +
                                             tile.textPath.string());
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
                    throw std::runtime_error("Invalid repeater index in " + tile.textPath.string());
                }
            }
            attachment.assetPath = trim(requireField(field++));
            if (!parseInteger(trim(requireField(field++)), attachment.id) ||
                !parseInteger(trim(requireField(field++)), attachment.splineIndex)) {
                throw std::runtime_error("Invalid ID or spline index in " + section + " in " +
                                         tile.textPath.string());
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
                                             tile.textPath.string());
                }
                field += stringCount;
            }
            data.splineAttachments.push_back(std::move(attachment));
            modifierTarget = ModifierTarget::SplineAttachment;
            modifierTargetIndex = data.splineAttachments.size() - 1;
            cursor = field - 1;
        } else if (section == "[varparent]") {
            if (cursor + 1 >= lines.size()) {
                throw std::runtime_error("Truncated [varparent] in " + tile.textPath.string());
            }
            int parentId = 0;
            if (!parseInteger(trim(lines[cursor + 1]), parentId)) {
                throw std::runtime_error("Invalid [varparent] ID in " + tile.textPath.string());
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
            openbus::config::logUnhandledConfigurationTag(
                "map tile", tile.textPath, cursor + 1, "spline_terrain_align_2",
                "record is retained but its runtime behavior is not implemented");
            if (cursor + 1 >= lines.size()) {
                throw std::runtime_error("Truncated [spline_terrain_align_2] in " +
                                         tile.textPath.string());
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
                                             tile.textPath.string());
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
        throw std::runtime_error("No [version] section found in " + tile.textPath.string());
    }
    if (chronoData == nullptr && !data.hasTerrainMarker) {
        throw std::runtime_error("No [terrain] section found in " + tile.textPath.string());
    }
    if (chronoData != nullptr) {
        chronoData->diagnostics = data.diagnostics;
    }
    return data;
}

MapTileData loadMapTile(const MapTileReference& tile) {
    return loadMapTileInternal(tile, nullptr);
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
        throw std::runtime_error("Could not open terrain file: " + terrainPath.string());
    }
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)),
                                           std::istreambuf_iterator<char>());
    if (bytes.size() < 4) {
        throw std::runtime_error("Terrain file is shorter than its header: " +
                                 terrainPath.string());
    }
    const std::uint32_t intervals = readU32(bytes, 0);
    if (intervals == 0 || intervals > 1024U) {
        throw std::runtime_error("Invalid terrain grid interval count in " + terrainPath.string());
    }
    const std::uint64_t vertexCount = static_cast<std::uint64_t>(intervals + 1U) * (intervals + 1U);
    const std::uint64_t expectedBytes = 4U + vertexCount * 4U;
    if (expectedBytes != bytes.size()) {
        throw std::runtime_error("Terrain file size does not match its grid header: " +
                                 terrainPath.string());
    }

    TerrainGrid grid;
    grid.intervals = intervals;
    grid.heights.reserve(static_cast<std::size_t>(vertexCount));
    for (std::size_t offset = 4; offset < bytes.size(); offset += 4) {
        const float height = std::bit_cast<float>(readU32(bytes, offset));
        if (!std::isfinite(height)) {
            throw std::runtime_error("Non-finite terrain height in " + terrainPath.string());
        }
        grid.heights.push_back(height);
    }
    return grid;
}

WaterData loadWaterData(const std::filesystem::path& waterPath) {
    openbus::rendering::TraceScope trace("map", "MapConfigLoader::loadWaterData");
    std::ifstream input(waterPath, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not open water file: " + waterPath.string());
    }
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)),
                                           std::istreambuf_iterator<char>());
    if (bytes.size() < 4) {
        throw std::runtime_error("Water file is shorter than its header: " + waterPath.string());
    }
    const std::uint32_t surfaceCount = readU32(bytes, 0);
    constexpr std::uint32_t MAX_WATER_SURFACES = 1'000'000;
    if (surfaceCount > MAX_WATER_SURFACES) {
        throw std::runtime_error("Water surface count exceeds the supported limit in " +
                                 waterPath.string());
    }
    const std::uint64_t expectedBytes = 4U + static_cast<std::uint64_t>(surfaceCount) * 16U;
    if (bytes.size() != expectedBytes) {
        throw std::runtime_error("Water file size does not match its surface count: " +
                                 waterPath.string());
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
                throw std::runtime_error("Non-finite water height in " + waterPath.string());
            }
        }
        water.surfaces.push_back(surface);
    }
    return water;
}

} // namespace openbus::map
