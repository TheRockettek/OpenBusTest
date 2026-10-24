#include "map/MapConfigLoader.h"
#include "ProbeTempDirectory.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* description) {
    if (!condition) {
        throw std::runtime_error(description);
    }
}

void writeUtf16Le(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("could not create Chrono tile fixture");
    }
    output.put(static_cast<char>(0xff));
    output.put(static_cast<char>(0xfe));
    for (const unsigned char character : text) {
        output.put(static_cast<char>(character));
        output.put('\0');
    }
}

void runProbe() {
    const std::filesystem::path root =
        openbus::probe::makeProbeDirectory("openbus-map-chrono-tile-probe");
    std::filesystem::create_directories(root);
    try {
        const std::string chronoText =
            "Chrono fixture comment\n[version]\n14\n"
            "[selobject]\n30273\n[typ]\n1\nScenery\\selected.sco\n"
            "[selspline]\n-44\n[typ]\nprofile override\n"
            "[object]\n[selobject]\n77\nScenery\\dummy.sco\n5\n"
            "1\n2\n3\n0\n0\n0\n0\n";
        const std::filesystem::path chronoPath = root / "chrono_tile.map";
        writeUtf16Le(chronoPath, chronoText);

        const openbus::map::MapChronoTileData chrono =
            openbus::map::loadChronoTile(chronoPath);
        require(chrono.initialComment == "Chrono fixture comment" && chrono.version == "14",
                "Chrono comment and version are captured");
        require(chrono.selectors.size() == 2 &&
                    chrono.selectors[0].keyword == "selobject" &&
                    chrono.selectors[0].id == 30273 && chrono.selectors[0].rawId == "30273" &&
                    chrono.selectors[0].sectionLine == 4 && chrono.selectors[0].idLine == 5 &&
                    chrono.selectors[0].followingLines ==
                        std::vector<std::string>{"[typ]", "1", "Scenery\\selected.sco"},
                "object selector ID is typed and its following override lines remain opaque");
        require(chrono.selectors[1].keyword == "selspline" &&
                    chrono.selectors[1].id == -44 && chrono.selectors[1].rawId == "-44" &&
                    chrono.selectors[1].sectionLine == 9 && chrono.selectors[1].idLine == 10 &&
                    chrono.selectors[1].followingLines ==
                        std::vector<std::string>{"[typ]", "profile override"},
                "spline selector ID and raw override lines are retained");
        require(chrono.rawLines.size() == 25 && chrono.rawLines[12] == "[object]" &&
                    chrono.rawLines[13] == "[selobject]" && chrono.rawLines[14] == "77" &&
                    chrono.rawLines.back().empty(),
                "all Chrono tile lines, including trailing blank and object payload, are preserved");
        require(chrono.diagnostics.entries.empty(),
                "valid Chrono selector inventory produces no parser warnings");

        const std::filesystem::path malformedPath = root / "malformed_chrono_tile.map";
        writeUtf16Le(malformedPath,
                     "Malformed selector\n[version]\n14\n[selobject]\nbad-id\n"
                     "[typ]\nopaque payload\n");
        const openbus::map::MapChronoTileData malformed =
            openbus::map::loadChronoTile(malformedPath);
        require(malformed.selectors.size() == 1 && !malformed.selectors[0].id.has_value() &&
                    malformed.selectors[0].rawId == "bad-id" &&
                    malformed.selectors[0].followingLines ==
                        std::vector<std::string>{"[typ]", "opaque payload", ""} &&
                    malformed.diagnostics.entries.size() == 1 &&
                    malformed.diagnostics.entries[0].line == 5 &&
                    malformed.diagnostics.entries[0].keyword == "selobject",
                "malformed selector ID is warned about without losing raw lines");

        const std::filesystem::path truncatedPath = root / "truncated_chrono_tile.map";
        writeUtf16Le(truncatedPath, "Truncated selector\n[version]\n14\n[selspline]");
        const openbus::map::MapChronoTileData truncated =
            openbus::map::loadChronoTile(truncatedPath);
        require(truncated.selectors.size() == 1 && !truncated.selectors[0].id.has_value() &&
                    truncated.selectors[0].rawId.empty() &&
                    truncated.diagnostics.entries.size() == 1 &&
                    truncated.diagnostics.entries[0].line == 4 &&
                    truncated.diagnostics.entries[0].message ==
                        "selector ID is missing; source lines retained",
                "truncated selector ID is reported without rejecting the raw tile");

        openbus::map::MapDefinition map;
        map.rootPath = root;
        openbus::map::MapTileReference baseTile;
        baseTile.x = 1;
        baseTile.y = 2;
        baseTile.textPath = root / "tile_1_2.map";
        baseTile.terrainPath = root / "tile_1_2.map.terrain";
        baseTile.hasTerrainFile = true;
        map.tiles.push_back(baseTile);
        const std::filesystem::path firstScenario = root / "Chrono" / "01_Construction";
        const std::filesystem::path laterScenario = root / "Chrono" / "02_Detour";
        const std::filesystem::path indefiniteScenario = root / "Chrono" / "03_Indefinite";
        std::filesystem::create_directories(firstScenario);
        std::filesystem::create_directories(laterScenario);
        std::filesystem::create_directories(indefiniteScenario);
        writeUtf16Le(firstScenario / "Chrono.cfg",
                     "[startdate]\n20200101\n[enddate]\n20201231\n");
        writeUtf16Le(firstScenario / "tile_1_2.map", "[version]\n14\n[object]\n");
        writeUtf16Le(laterScenario / "Chrono.cfg",
                     "[startdate]\n20200601\n[enddate]\n20200630\n");
        writeUtf16Le(laterScenario / "tile_1_2.map", "[version]\n14\n[object]\n");
        std::ofstream(laterScenario / "tile_1_2.map.terrain").put('\0');
        writeUtf16Le(indefiniteScenario / "Chrono.cfg", "[startdate]\n20220101\n");
        writeUtf16Le(indefiniteScenario / "tile_1_2.map", "[version]\n14\n[object]\n");

        openbus::map::MapDefinition beforeScenario = map;
        require(openbus::map::applyMapChronoTiles(beforeScenario, 20191231) == 0 &&
                    beforeScenario.tiles[0].textPath == baseTile.textPath,
                "Chrono tile variants are inactive before their start date");
        openbus::map::MapDefinition firstScenarioActive = map;
        require(openbus::map::applyMapChronoTiles(firstScenarioActive, 20200101) == 1 &&
                    firstScenarioActive.tiles[0].textPath == firstScenario / "tile_1_2.map" &&
                    firstScenarioActive.tiles[0].hasTerrainFile &&
                    firstScenarioActive.tiles[0].terrainPath == baseTile.terrainPath,
                "Chrono start dates are inclusive and variant files retain absent base sidecars");
        openbus::map::MapDefinition overlappingScenarios = map;
        require(openbus::map::applyMapChronoTiles(overlappingScenarios, 20200601) == 2 &&
                    overlappingScenarios.tiles[0].textPath ==
                        laterScenario / "tile_1_2.map" &&
                    overlappingScenarios.tiles[0].terrainPath ==
                        laterScenario / "tile_1_2.map.terrain",
                "Chrono start dates are inclusive and later active scenarios override available tiles");
        openbus::map::MapDefinition overlappingEndDate = map;
        require(openbus::map::applyMapChronoTiles(overlappingEndDate, 20200630) == 2 &&
                    overlappingEndDate.tiles[0].textPath == laterScenario / "tile_1_2.map",
                "Chrono end dates are inclusive");
        openbus::map::MapDefinition firstScenarioOnly = map;
        require(openbus::map::applyMapChronoTiles(firstScenarioOnly, 20200701) == 1 &&
                    firstScenarioOnly.tiles[0].textPath == firstScenario / "tile_1_2.map",
                "Chrono end dates are inclusive and expired scenarios no longer override tiles");
        openbus::map::MapDefinition afterScenarios = map;
        require(openbus::map::applyMapChronoTiles(afterScenarios, 20210101) == 0 &&
                    afterScenarios.tiles[0].textPath == baseTile.textPath,
                "base tile version is restored when no scenario is active");
        openbus::map::MapDefinition openEndedScenario = map;
        require(openbus::map::applyMapChronoTiles(openEndedScenario, 20220101) == 1 &&
                    openEndedScenario.tiles[0].textPath ==
                        indefiniteScenario / "tile_1_2.map",
                "Chrono scenarios without an end date remain active after their start date");
        bool invalidDateRejected = false;
        try {
            static_cast<void>(openbus::map::applyMapChronoTiles(map, 20200230));
        } catch (const std::invalid_argument&) {
            invalidDateRejected = true;
        }
        require(invalidDateRejected, "invalid calendar dates are rejected for Chrono selection");
    } catch (...) {
        openbus::probe::removeProbeDirectory(root);
        throw;
    }
    openbus::probe::removeProbeDirectory(root);
}

} // namespace

int main() {
    try {
        runProbe();
    } catch (const std::exception& error) {
        std::cerr << "map Chrono tile probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
