#include "config/HofData.h"

#include "config/ConfigurationParser.h"

#include <algorithm>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace openbus::config {

HofData loadHofData(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        throw std::runtime_error("unable to open selected HOF file: " + path.string());
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        if (lines.empty() && line.starts_with("\xEF\xBB\xBF")) {
            line.erase(0, 3);
        }
        while (!line.empty() &&
               (line.back() == '\r' || line.back() == '\t' || line.back() == ' ')) {
            line.pop_back();
        }
        lines.push_back(std::move(line));
    }
    if (input.bad()) {
        throw std::runtime_error("failed while reading selected HOF file: " + path.string());
    }

    const auto sectionName = [](const std::string& value) {
        const std::string normalized = trim(value);
        if (normalized.size() < 3 || normalized.front() != '[' || normalized.back() != ']') {
            return std::string{};
        }
        return lower(normalized.substr(1, normalized.size() - 2));
    };
    const auto isSectionEnd = [&sectionName](const std::string& value) {
        return sectionName(value) == "end";
    };
    const auto nextPayload = [&lines](std::size_t& cursor) -> std::optional<std::string> {
        while (cursor < lines.size()) {
            std::string value = lines[cursor++];
            if (!trim(value).empty()) {
                return value;
            }
        }
        return std::nullopt;
    };
    const auto nextCountedPayload = [&lines](std::size_t& cursor) -> std::optional<std::string> {
        if (cursor >= lines.size()) {
            return std::nullopt;
        }
        return lines[cursor++];
    };
    const auto readCount = [&nextPayload](std::size_t& cursor, std::size_t& count) {
        const std::optional<std::string> value = nextPayload(cursor);
        int parsed = 0;
        if (!value || !parseInt(*value, parsed) || parsed < 0) {
            return false;
        }
        count = static_cast<std::size_t>(parsed);
        return true;
    };
    const auto splitTabs = [](const std::string& value) {
        std::vector<std::string> fields;
        std::size_t start = 0;
        while (true) {
            const std::size_t separator = value.find('\t', start);
            fields.push_back(value.substr(
                start, separator == std::string::npos ? std::string::npos : separator - start));
            if (separator == std::string::npos) {
                return fields;
            }
            start = separator + 1;
        }
    };

    std::size_t terminusStringCount = 0;
    std::size_t busstopStringCount = 0;
    for (std::size_t index = 0; index + 1 < lines.size(); ++index) {
        const std::string directive = lower(trim(lines[index]));
        if (directive != "stringcount_terminus" && directive != "stringcount_busstop") {
            continue;
        }
        std::size_t valueIndex = index + 1;
        while (valueIndex < lines.size() && trim(lines[valueIndex]).empty()) {
            ++valueIndex;
        }
        int count = 0;
        if (valueIndex < lines.size() && parseInt(lines[valueIndex], count) && count >= 0) {
            if (directive == "stringcount_terminus") {
                terminusStringCount = static_cast<std::size_t>(count);
            } else {
                busstopStringCount = static_cast<std::size_t>(count);
            }
        }
    }

    HofData parsed;
    for (std::size_t index = 0; index < lines.size();) {
        const std::string section = sectionName(lines[index]);
        if (section.empty()) {
            ++index;
            continue;
        }
        std::size_t cursor = index + 1;
        if (section == "global_strings") {
            std::size_t count = 0;
            if (readCount(cursor, count)) {
                for (std::size_t stringIndex = 0; stringIndex < count; ++stringIndex) {
                    const std::optional<std::string> value = nextCountedPayload(cursor);
                    if (!value) {
                        break;
                    }
                    parsed.globalStrings.push_back(*value);
                }
            }
        } else if (section == "addterminus" || section == "addterminus_allexit") {
            const std::optional<std::string> codeValue = nextPayload(cursor);
            const std::optional<std::string> ident = nextPayload(cursor);
            int code = 0;
            if (codeValue && ident && parseInt(*codeValue, code)) {
                HofTerminus terminus;
                terminus.code = code;
                for (std::size_t stringIndex = 0; stringIndex < terminusStringCount;
                     ++stringIndex) {
                    const std::optional<std::string> value = nextCountedPayload(cursor);
                    if (!value) {
                        break;
                    }
                    terminus.strings.push_back(*value);
                }
                parsed.termini.push_back(std::move(terminus));
            }
        } else if (section == "addterminus_list") {
            while (cursor < lines.size() && !isSectionEnd(lines[cursor])) {
                const std::string row = lines[cursor++];
                if (trim(row).empty()) {
                    continue;
                }
                const std::vector<std::string> fields = splitTabs(row);
                if (fields.size() < 3) {
                    continue;
                }
                int code = 0;
                if (!parseInt(fields[1], code)) {
                    continue;
                }
                HofTerminus terminus;
                terminus.code = code;
                for (std::size_t stringIndex = 3; stringIndex < fields.size(); ++stringIndex) {
                    terminus.strings.push_back(fields[stringIndex]);
                }
                if (terminusStringCount > 0) {
                    terminus.strings.resize(terminusStringCount);
                }
                parsed.termini.push_back(std::move(terminus));
            }
            if (cursor < lines.size() && isSectionEnd(lines[cursor])) {
                ++cursor;
            }
        } else if (section == "addbusstop") {
            const std::optional<std::string> ident = nextPayload(cursor);
            if (ident) {
                HofBusstop busstop;
                busstop.ident = trim(*ident);
                for (std::size_t stringIndex = 0; stringIndex < busstopStringCount; ++stringIndex) {
                    const std::optional<std::string> value = nextCountedPayload(cursor);
                    if (!value) {
                        break;
                    }
                    busstop.strings.push_back(*value);
                }
                parsed.busstops.push_back(std::move(busstop));
            }
        } else if (section == "addbusstop_list") {
            while (cursor < lines.size() && !isSectionEnd(lines[cursor])) {
                const std::string row = lines[cursor++];
                if (trim(row).empty()) {
                    continue;
                }
                const std::vector<std::string> fields = splitTabs(row);
                HofBusstop busstop;
                busstop.ident = trim(fields.front());
                for (std::size_t stringIndex = 1; stringIndex < fields.size(); ++stringIndex) {
                    busstop.strings.push_back(fields[stringIndex]);
                }
                parsed.busstops.push_back(std::move(busstop));
            }
            if (cursor < lines.size() && isSectionEnd(lines[cursor])) {
                ++cursor;
            }
        } else if (section == "infosystem_trip") {
            const std::optional<std::string> code = nextPayload(cursor);
            const std::optional<std::string> name = nextPayload(cursor);
            const std::optional<std::string> terminusValue = nextPayload(cursor);
            const std::optional<std::string> lineValue = nextPayload(cursor);
            if (code && name && terminusValue && lineValue) {
                HofRoute route;
                route.code = trim(*code);
                route.name = trim(*name);
                route.line = trim(*lineValue);
                route.hasTerminusIndex = parseInt(*terminusValue, route.terminusIndex);
                parsed.routes.push_back(std::move(route));
            }
        } else if (section == "infosystem_busstop_list") {
            std::size_t count = 0;
            if (readCount(cursor, count) && !parsed.routes.empty()) {
                HofRoute& route = parsed.routes.back();
                for (std::size_t stopIndex = 0; stopIndex < count; ++stopIndex) {
                    const std::optional<std::string> value = nextPayload(cursor);
                    if (!value) {
                        break;
                    }
                    route.busstopIdents.push_back(trim(*value));
                }
            }
        }
        index = (std::max)(cursor, index + 1);
    }
    return parsed;
}

} // namespace openbus::config
