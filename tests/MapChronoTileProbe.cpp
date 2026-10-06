#include "MapConfigLoader.h"

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
        std::filesystem::temp_directory_path() / "openbus-map-chrono-tile-probe";
    std::filesystem::remove_all(root);
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
    } catch (...) {
        std::filesystem::remove_all(root);
        throw;
    }
    std::filesystem::remove_all(root);
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
