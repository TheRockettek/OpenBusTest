#include "MapConfigLoader.h"

#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string_view>
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
    std::size_t offset = bytes.size() >= 3 && bytes[0] == 0xefU && bytes[1] == 0xbbU &&
                                 bytes[2] == 0xbfU
                             ? 3
                             : 0;
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

bool parseReal(const std::string& value, double& result) {
    if (value.empty()) {
        return false;
    }
    const char* begin = value.data();
    const char* end = begin + value.size();
    const auto parsed = std::from_chars(begin, end, result);
    return parsed.ec == std::errc{} && parsed.ptr == end && std::isfinite(result);
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

} // namespace

MapDefinition loadMapDefinition(const std::filesystem::path& mapDirectory) {
    MapDefinition definition;
    definition.rootPath = mapDirectory;
    const std::filesystem::path globalPath = mapDirectory / "global.cfg";
    const std::vector<std::string> lines = splitLines(decodeText(globalPath));
    std::unordered_set<std::string> tileNames;
    std::unordered_set<std::uint64_t> tileCoordinates;

    for (std::size_t cursor = 0; cursor < lines.size();) {
        const std::string section = trim(lines[cursor]);
        ++cursor;
        if (section == "[groundtex]") {
            GroundTextureDefinition groundTexture;
            groundTexture.texturePath = readField(lines, cursor, globalPath, "groundtex");
            groundTexture.detailTexturePath = readField(lines, cursor, globalPath, "groundtex");
            for (std::string& parameter : groundTexture.parameters) {
                parameter = readField(lines, cursor, globalPath, "groundtex");
            }
            if (groundTexture.texturePath.empty()) {
                throw std::runtime_error("Empty base texture in [groundtex] in " + globalPath.string());
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
                throw std::runtime_error("Invalid tile coordinates in [map] in " + globalPath.string());
            }
            int filenameX = 0;
            int filenameY = 0;
            parseTileFilename(filename, filenameX, filenameY);
            if (filenameX != tile.x || filenameY != tile.y) {
                throw std::runtime_error("[map] coordinates do not match tile filename: " + filename);
            }
            if (!tileNames.insert(filename).second) {
                throw std::runtime_error("Duplicate tile filename in [map]: " + filename);
            }
            const std::uint64_t key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(tile.x))
                                       << 32U) |
                                      static_cast<std::uint32_t>(tile.y);
            if (!tileCoordinates.insert(key).second) {
                throw std::runtime_error("Duplicate tile coordinates in [map]: " + filename);
            }
            tile.textPath = mapDirectory / filename;
            if (!std::filesystem::is_regular_file(tile.textPath)) {
                throw std::runtime_error("Listed map tile does not exist: " + tile.textPath.string());
            }
            tile.terrainPath = std::filesystem::path(tile.textPath.string() + ".terrain");
            tile.hasTerrainFile = std::filesystem::is_regular_file(tile.terrainPath);
            definition.tiles.push_back(std::move(tile));
        }
    }

    if (definition.tiles.empty()) {
        throw std::runtime_error("No [map] tile records found in " + globalPath.string());
    }
    for (MapEntryPoint& entryPoint : definition.entryPoints) {
        if (entryPoint.tileIndex < 0 ||
            static_cast<std::size_t>(entryPoint.tileIndex) >= definition.tiles.size()) {
            throw std::runtime_error("[entrypoints] tile index is outside the [map] list in " +
                                     globalPath.string());
        }
        double localX = 0.0;
        double elevation = 0.0;
        double localY = 0.0;
        double quaternionY = 0.0;
        double quaternionW = 0.0;
        if (!parseReal(entryPoint.rawFields[3], localX) ||
            !parseReal(entryPoint.rawFields[4], elevation) ||
            !parseReal(entryPoint.rawFields[5], localY) ||
            !parseReal(entryPoint.rawFields[7], quaternionY) ||
            !parseReal(entryPoint.rawFields[9], quaternionW)) {
            throw std::runtime_error("Invalid position or orientation in [entrypoints] record in " +
                                     globalPath.string());
        }
        const MapTileReference& tile = definition.tiles[entryPoint.tileIndex];
        entryPoint.placement = {
            {static_cast<double>(tile.x) * OMSI_TILE_SIZE_METERS + localX,
             static_cast<double>(tile.y) * OMSI_TILE_SIZE_METERS + localY, elevation},
            2.0 * std::atan2(quaternionY, quaternionW) * 180.0 / 3.14159265358979323846};
    }
    return definition;
}

MapTileData loadMapTile(const MapTileReference& tile) {
    MapTileData data;
    data.reference = tile;
    const std::vector<std::string> lines = splitLines(decodeText(tile.textPath));
    for (std::size_t cursor = 0; cursor < lines.size(); ++cursor) {
        const std::string section = trim(lines[cursor]);
        if (section == "[version]") {
            if (cursor + 1 >= lines.size()) {
                throw std::runtime_error("Truncated [version] in " + tile.textPath.string());
            }
            data.version = trim(lines[++cursor]);
        } else if (section == "[terrain]") {
            data.hasTerrainMarker = true;
        } else if (section == "[spline]") {
            ++data.splineCount;
            if (cursor + 19 >= lines.size()) {
                throw std::runtime_error("Truncated [spline] record in " + tile.textPath.string());
            }
            MapSplinePlacement spline;
            for (std::size_t field = 0; field < spline.rawFields.size(); ++field) {
                spline.rawFields[field] = trim(lines[cursor + 1 + field]);
            }
            spline.assetPath = spline.rawFields[1];
            spline.geometryValid = parseReal(spline.rawFields[5], spline.localX) &&
                                   parseReal(spline.rawFields[6], spline.elevation) &&
                                   parseReal(spline.rawFields[7], spline.localY) &&
                                   parseReal(spline.rawFields[8], spline.rotationDegrees) &&
                                   parseReal(spline.rawFields[9], spline.length) &&
                                   parseReal(spline.rawFields[10], spline.radius) &&
                                   parseReal(spline.rawFields[11], spline.gradientStart) &&
                                   parseReal(spline.rawFields[12], spline.gradientEnd);
            data.splines.push_back(std::move(spline));
            cursor += 19;
        } else if (section == "[spline_h]") {
            ++data.elevatedSplineCount;
            if (cursor + 20 >= lines.size()) {
                throw std::runtime_error("Truncated [spline_h] record in " +
                                         tile.textPath.string());
            }
            cursor += 20;
        } else if (section == "[object]") {
            ++data.objectCount;
            if (cursor + 10 >= lines.size()) {
                throw std::runtime_error("Truncated [object] record in " + tile.textPath.string());
            }
            MapSceneryPlacement object;
            object.label = cursor == 0 ? std::string{} : trim(lines[cursor - 1]);
            object.line1 = trim(lines[cursor + 1]);
            object.assetPath = trim(lines[cursor + 2]);
            if (!parseInteger(trim(lines[cursor + 3]), object.id)) {
                throw std::runtime_error("Invalid [object] ID in " + tile.textPath.string());
            }
            for (std::size_t field = 0; field < object.rawTransformFields.size(); ++field) {
                object.rawTransformFields[field] = trim(lines[cursor + 4 + field]);
            }
            object.transformValid = true;
            for (std::size_t axis = 0; axis < object.localPosition.size(); ++axis) {
                object.transformValid =
                    parseReal(object.rawTransformFields[axis], object.localPosition[axis]) &&
                    parseReal(object.rawTransformFields[axis + 3],
                              object.rotationDegrees[axis]) &&
                    object.transformValid;
            }
            object.trailingField = trim(lines[cursor + 10]);
            data.sceneryObjects.push_back(std::move(object));
            cursor += 10;
        } else if (section == "[attachObj]") {
            ++data.attachedObjectCount;
        } else if (section == "[splineAttachement]") {
            ++data.splineAttachmentCount;
        } else if (section == "[splineAttachement_repeater]") {
            ++data.splineRepeaterCount;
        }
    }
    if (data.version.empty()) {
        throw std::runtime_error("No [version] section found in " + tile.textPath.string());
    }
    if (!data.hasTerrainMarker) {
        throw std::runtime_error("No [terrain] section found in " + tile.textPath.string());
    }
    return data;
}

TerrainGrid loadTerrainGrid(const std::filesystem::path& terrainPath) {
    std::ifstream input(terrainPath, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not open terrain file: " + terrainPath.string());
    }
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(input)),
                                           std::istreambuf_iterator<char>());
    if (bytes.size() < 4) {
        throw std::runtime_error("Terrain file is shorter than its header: " + terrainPath.string());
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

} // namespace openbus::map
