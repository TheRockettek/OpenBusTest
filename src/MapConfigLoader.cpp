#include "MapConfigLoader.h"

#include "PerfTrace.h"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cctype>
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

} // namespace

MapDefinition loadMapDefinition(const std::filesystem::path& mapDirectory) {
    openbus::rendering::TraceScope trace("map", "MapConfigLoader::loadMapDefinition");
    MapDefinition definition;
    definition.rootPath = mapDirectory;
    const std::filesystem::path globalPath = mapDirectory / "global.cfg";
    const std::vector<std::string> lines = splitLines(decodeText(globalPath));
    const auto groundTextureSidecarsByTile = discoverGroundTextureSidecars(mapDirectory);
    std::unordered_set<std::string> tileNames;
    std::unordered_set<std::uint64_t> tileCoordinates;
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
            definition.metadata.roadTrafficDensity.push_back(std::move(values));
        } else if (section == "[trafficdensity_passenger]") {
            std::array<std::string, 2> values;
            readMetadataFields(values, "trafficdensity_passenger");
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
            if (!tileNames.insert(filename).second) {
                throw std::runtime_error("Duplicate tile filename in [map]: " + filename);
            }
            const std::uint64_t key =
                (static_cast<std::uint64_t>(static_cast<std::uint32_t>(tile.x)) << 32U) |
                static_cast<std::uint32_t>(tile.y);
            if (!tileCoordinates.insert(key).second) {
                throw std::runtime_error("Duplicate tile coordinates in [map]: " + filename);
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
            definition.tiles.push_back(std::move(tile));
        } else if (isSectionHeader(section)) {
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
    for (MapEntryPoint& entryPoint : definition.entryPoints) {
        if (entryPoint.tileIndex < 0 ||
            static_cast<std::size_t>(entryPoint.tileIndex) >= definition.tiles.size()) {
            throw std::runtime_error("[entrypoints] tile index is outside the [map] list in " +
                                     globalPath.string());
        }
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
        const double yawNumerator = 2.0 * (quaternionW * quaternionY + quaternionX * quaternionZ);
        const double yawDenominator =
            1.0 - 2.0 * (quaternionY * quaternionY + quaternionZ * quaternionZ);
        const MapTileReference& tile = definition.tiles[entryPoint.tileIndex];
        entryPoint.placement = {
            {static_cast<double>(tile.x) * OMSI_TILE_SIZE_METERS + localX,
             static_cast<double>(tile.y) * OMSI_TILE_SIZE_METERS + localY, elevation},
            std::atan2(yawNumerator, yawDenominator) * 180.0 / 3.14159265358979323846};
    }
    return definition;
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
            break;
        }
        if (chronoData != nullptr && section != "[version]" && section != "[selobject]" &&
            section != "[selspline]") {
            // Chrono tile data is retained verbatim. Only its selector records are typed here;
            // normal tile records and selector override keywords have separate semantics.
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
        } else if (section == "[variable_terrainlightmap]") {
            data.hasVariableTerrainLightmapMarker = true;
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

            // labelCount remains opaque and does not define the number of tail lines. Preserve
            // object-specific text while leaving the next record's description for its parser.
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
