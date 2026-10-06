#include "MapSplineProfile.h"

#include "PerfTrace.h"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <vector>

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

std::vector<std::string> readLines(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not open spline profile: " + path.string());
    }
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)),
                                           std::istreambuf_iterator<char>());
    std::string text;
    if (bytes.size() >= 2 && bytes[0] == 0xffU && bytes[1] == 0xfeU) {
        if ((bytes.size() - 2) % 2 != 0) {
            throw std::runtime_error("Odd-length UTF-16LE spline profile: " + path.string());
        }
        text.reserve((bytes.size() - 2) / 2);
        for (std::size_t offset = 2; offset < bytes.size(); offset += 2) {
            const unsigned int character = static_cast<unsigned int>(bytes[offset]) |
                                           (static_cast<unsigned int>(bytes[offset + 1]) << 8U);
            text.push_back(character <= 0x7fU ? static_cast<char>(character) : '?');
        }
    } else {
        std::size_t offset =
            bytes.size() >= 3 && bytes[0] == 0xefU && bytes[1] == 0xbbU && bytes[2] == 0xbfU ? 3
                                                                                             : 0;
        text.assign(bytes.begin() + static_cast<std::ptrdiff_t>(offset), bytes.end());
    }

    std::vector<std::string> lines;
    std::size_t begin = 0;
    while (begin < text.size()) {
        const std::size_t end = text.find('\n', begin);
        const std::size_t lineEnd = end == std::string::npos ? text.size() : end;
        lines.push_back(trim(std::string_view(text).substr(begin, lineEnd - begin)));
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return lines;
}

bool parseReal(const std::string& text, double& result) {
    if (text.empty()) {
        return false;
    }
    const char* begin = text.data();
    const char* end = begin + text.size();
    const auto parsed = std::from_chars(begin, end, result);
    return parsed.ec == std::errc{} && parsed.ptr == end && std::isfinite(result);
}

bool readPayload(const std::vector<std::string>& lines, std::size_t& cursor, std::string& payload) {
    while (cursor < lines.size()) {
        payload = lines[cursor++];
        if (payload.empty()) {
            continue;
        }
        if (payload.size() >= 2 && payload.front() == '[' && payload.back() == ']') {
            --cursor;
            return false;
        }
        return true;
    }
    return false;
}

bool readRealPayloads(const std::vector<std::string>& lines, std::size_t& cursor,
                      std::array<double, 4>& values) {
    std::array<std::string, 4> fields;
    for (std::string& field : fields) {
        if (!readPayload(lines, cursor, field)) {
            return false;
        }
    }
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (!parseReal(fields[index], values[index])) {
            return false;
        }
    }
    return true;
}

} // namespace

MapSplineProfile loadMapSplineProfile(const std::filesystem::path& path) {
    openbus::rendering::TraceScope trace("map", "MapSplineProfile::load");
    const std::vector<std::string> lines = readLines(path);
    MapSplineProfile profile;
    MapSplineProfileSection* currentSection = nullptr;

    for (std::size_t cursor = 0; cursor < lines.size();) {
        const std::string& section = lines[cursor++];
        if (section == "[onlyeditor]") {
            profile.editorOnly = true;
        } else if (section == "[texture]") {
            std::string texture;
            if (readPayload(lines, cursor, texture)) {
                profile.textures.push_back(std::move(texture));
            }
        } else if (section == "[profile]") {
            std::string textureIndexText;
            int textureIndex = -1;
            if (readPayload(lines, cursor, textureIndexText)) {
                const char* begin = textureIndexText.data();
                const char* end = begin + textureIndexText.size();
                const auto parsed = std::from_chars(begin, end, textureIndex);
                if (parsed.ec == std::errc{} && parsed.ptr == end) {
                    profile.sections.push_back({textureIndex, {}});
                    currentSection = &profile.sections.back();
                } else {
                    currentSection = nullptr;
                }
            } else {
                currentSection = nullptr;
            }
        } else if (section == "[profilepnt]" && currentSection != nullptr) {
            std::array<double, 4> values = {};
            if (readRealPayloads(lines, cursor, values)) {
                currentSection->points.push_back({values[0], values[1], values[2], values[3]});
            }
        }
    }

    return profile;
}

} // namespace openbus::map
