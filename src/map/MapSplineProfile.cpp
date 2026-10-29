#include "map/MapSplineProfile.h"

#include "config/ConfigurationParser.h"
#include "core/PerfTrace.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <utility>
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
    bool disabled = false;
    while (begin < text.size()) {
        const std::size_t end = text.find('\n', begin);
        const std::size_t lineEnd = end == std::string::npos ? text.size() : end;
        std::string line = trim(std::string_view(text).substr(begin, lineEnd - begin));
        if (line == "-<DISABLED>-") {
            disabled = true;
            line.clear();
        } else if (line == "-<ENABLED>-") {
            disabled = false;
            line.clear();
        } else if (disabled) {
            line.clear();
        }
        lines.push_back(std::move(line));
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
    if (*begin == '+') {
        ++begin;
        if (begin == end) {
            return false;
        }
    }
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
    std::size_t currentTextureIndex = 0;
    bool hasCurrentTexture = false;
    std::vector<bool> scaleTextureByLength;

    for (std::size_t cursor = 0; cursor < lines.size();) {
        const std::size_t sectionLine = cursor + 1;
        const std::string& sourceLine = lines[cursor++];
        const std::string section =
            sourceLine.size() > 2 && sourceLine.front() == '[' && sourceLine.back() == ']'
                ? openbus::config::lower(sourceLine)
                : sourceLine;
        if (section == "[onlyeditor]") {
            profile.editorOnly = true;
        } else if (section == "[nocollision]") {
            profile.noCollision = true;
        } else if (section == "[halfcantwidth]") {
            std::string widthText;
            double width = 0.0;
            if (!readPayload(lines, cursor, widthText) || !parseReal(widthText, width) ||
                width < 0.0) {
                openbus::config::logUnhandledConfigurationTag(
                    "spline profile (.sli)", path, sectionLine, "halfcantwidth",
                    "invalid or missing non-negative width; height-profile fallback retained");
            } else if (profile.explicitCantHalfWidth.has_value()) {
                openbus::config::logUnhandledConfigurationTag(
                    "spline profile (.sli)", path, sectionLine, "halfcantwidth",
                    "duplicate width ignored; first valid directive retained");
            } else {
                profile.explicitCantHalfWidth = width;
            }
        } else if (section == "[heightprofile]") {
            MapSplineHeightProfile heightProfile;
            bool complete = true;
            bool numeric = true;
            for (std::size_t index = 0; index < heightProfile.rawFields.size(); ++index) {
                if (!readPayload(lines, cursor, heightProfile.rawFields[index])) {
                    complete = false;
                    break;
                }
                numeric = parseReal(heightProfile.rawFields[index], heightProfile.values[index]) &&
                          numeric;
            }
            heightProfile.valid = complete && numeric;
            profile.heightProfiles.push_back(std::move(heightProfile));
        } else if (section == "[path]") {
            MapSplinePathDefinition pathDefinition;
            bool numeric = true;
            while (cursor < lines.size()) {
                const std::string& field = lines[cursor];
                if (field.size() > 2 && field.front() == '[' && field.back() == ']') {
                    break;
                }
                ++cursor;
                if (field.empty()) {
                    continue;
                }
                pathDefinition.rawFields.push_back(field);
                double value = 0.0;
                if (parseReal(field, value)) {
                    pathDefinition.numericFields.push_back(value);
                } else {
                    numeric = false;
                }
            }
            pathDefinition.numericFieldsValid =
                numeric && !pathDefinition.rawFields.empty() &&
                pathDefinition.numericFields.size() == pathDefinition.rawFields.size();
            if (!pathDefinition.rawFields.empty()) {
                const std::string& firstField = pathDefinition.rawFields.front();
                int pathTypeCode = 0;
                const auto parsed = std::from_chars(
                    firstField.data(), firstField.data() + firstField.size(), pathTypeCode);
                if (parsed.ec == std::errc{} &&
                    parsed.ptr == firstField.data() + firstField.size()) {
                    pathDefinition.pathTypeCode = pathTypeCode;
                }
            }
            profile.paths.push_back(std::move(pathDefinition));
        } else if (section == "[texture]") {
            std::string texture;
            if (readPayload(lines, cursor, texture)) {
                profile.textures.push_back(std::move(texture));
                profile.textureAlphaModes.push_back(0);
                scaleTextureByLength.push_back(false);
                currentTextureIndex = profile.textures.size() - 1;
                hasCurrentTexture = true;
            } else {
                hasCurrentTexture = false;
            }
        } else if (section == "[matl_alpha]" && hasCurrentTexture) {
            std::string alphaModeText;
            int alphaMode = 0;
            if (readPayload(lines, cursor, alphaModeText)) {
                const char* begin = alphaModeText.data();
                const char* end = begin + alphaModeText.size();
                const auto parsed = std::from_chars(begin, end, alphaMode);
                if (parsed.ec == std::errc{} && parsed.ptr == end && alphaMode >= 0 &&
                    alphaMode <= 2) {
                    profile.textureAlphaModes[currentTextureIndex] = alphaMode;
                }
            }
        } else if (section == "[scaletexbylength]" && hasCurrentTexture) {
            scaleTextureByLength[currentTextureIndex] = true;
        } else if (section == "[profile]") {
            std::string textureIndexText;
            int textureIndex = -1;
            if (readPayload(lines, cursor, textureIndexText)) {
                const char* begin = textureIndexText.data();
                const char* end = begin + textureIndexText.size();
                const auto parsed = std::from_chars(begin, end, textureIndex);
                if (parsed.ec == std::errc{} && parsed.ptr == end) {
                    const bool scalesByLength =
                        textureIndex >= 0 &&
                        static_cast<std::size_t>(textureIndex) < scaleTextureByLength.size() &&
                        scaleTextureByLength[static_cast<std::size_t>(textureIndex)];
                    profile.sections.push_back({textureIndex, scalesByLength, {}});
                    currentSection = &profile.sections.back();
                } else {
                    currentSection = nullptr;
                    openbus::config::logUnhandledConfigurationTag(
                        "spline profile (.sli)", path, sectionLine, "profile",
                        "invalid texture index; profile strip ignored");
                }
            } else {
                currentSection = nullptr;
                openbus::config::logUnhandledConfigurationTag(
                    "spline profile (.sli)", path, sectionLine, "profile",
                    "missing texture index; profile strip ignored");
            }
        } else if (section == "[profilepnt]" && currentSection != nullptr) {
            std::array<double, 4> values = {};
            if (readRealPayloads(lines, cursor, values)) {
                currentSection->points.push_back({values[0], values[1], values[2], values[3]});
            } else {
                openbus::config::logUnhandledConfigurationTag(
                    "spline profile (.sli)", path, sectionLine, "profilepnt",
                    "expected four finite numeric fields; profile point ignored");
            }
        } else if (section == "[matl_alpha]" || section == "[profilepnt]" ||
                   section == "[scaletexbylength]") {
            openbus::config::logUnhandledConfigurationTag(
                "spline profile (.sli)", path, sectionLine,
                sourceLine.substr(1, sourceLine.size() - 2),
                "directive has no applicable preceding texture mapping");
        } else if (section.size() > 2 && section.front() == '[' && section.back() == ']') {
            openbus::config::logUnhandledConfigurationTag(
                "spline profile (.sli)", path, sectionLine,
                sourceLine.substr(1, sourceLine.size() - 2), "directive is not implemented");
        }
    }

    return profile;
}

std::optional<double> mapSplineProfileCantHalfWidth(const MapSplineProfile& profile) noexcept {
    if (profile.explicitCantHalfWidth.has_value() &&
        std::isfinite(*profile.explicitCantHalfWidth) && *profile.explicitCantHalfWidth >= 0.0) {
        return profile.explicitCantHalfWidth;
    }
    std::optional<double> halfWidth;
    for (const MapSplineHeightProfile& heightProfile : profile.heightProfiles) {
        if (!heightProfile.valid || !std::isfinite(heightProfile.values[0]) ||
            !std::isfinite(heightProfile.values[1])) {
            continue;
        }
        const double extent =
            std::max(std::abs(heightProfile.values[0]), std::abs(heightProfile.values[1]));
        if (!halfWidth.has_value() || extent > *halfWidth) {
            halfWidth = extent;
        }
    }
    return halfWidth;
}

bool mapSplineProfileHasPathType(const MapSplineProfile& profile, int pathTypeCode) noexcept {
    return std::any_of(profile.paths.begin(), profile.paths.end(),
                       [pathTypeCode](const MapSplinePathDefinition& path) {
                           return path.pathTypeCode == pathTypeCode;
                       });
}

std::optional<MirroredRoadDirectionPolicy>
parseMirroredRoadDirectionPolicy(std::string_view value) noexcept {
    if (value.empty() || value == "correct") {
        return MirroredRoadDirectionPolicy::Correct;
    }
    if (value == "authored") {
        return MirroredRoadDirectionPolicy::Authored;
    }
    return std::nullopt;
}

std::vector<MapSplinePathOffset> mapSplineProfilePathOffsets(const MapSplineProfile& profile,
                                                             int pathTypeCode, bool mirrored,
                                                             MirroredRoadDirectionPolicy policy) {
    std::vector<MapSplinePathOffset> offsets;
    const bool correctDirections = mirrored && pathTypeCode == MAP_SPLINE_PATH_TYPE_ROAD &&
                                   policy == MirroredRoadDirectionPolicy::Correct;
    for (std::size_t pathIndex = 0; pathIndex < profile.paths.size(); ++pathIndex) {
        const MapSplinePathDefinition& path = profile.paths[pathIndex];
        if (path.pathTypeCode == pathTypeCode && path.numericFieldsValid &&
            path.numericFields.size() >= 3) {
            const double lateral = mirrored ? -path.numericFields[1] : path.numericFields[1];
            const bool hasDirection = path.numericFields.size() >= 5;
            const bool forward = hasDirection && path.numericFields[4] == 0.0;
            const bool reverse = hasDirection && path.numericFields[4] == 1.0;
            offsets.push_back({pathIndex, lateral, path.numericFields[2],
                               correctDirections && (forward || reverse) ? !reverse : reverse,
                               hasDirection && path.numericFields[4] == 2.0});
        }
    }
    return offsets;
}

} // namespace openbus::map
