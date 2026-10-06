#include "MapSplineProfile.h"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, const char* message) {
    if (std::abs(actual - expected) > 1.0e-9) {
        throw std::runtime_error(std::string(message) + ": got " + std::to_string(actual));
    }
}

void runProbe(const std::filesystem::path& fixture) {
    const openbus::map::MapSplineProfile profile = openbus::map::loadMapSplineProfile(fixture);
    require(!profile.editorOnly, "runtime profile is not editor-only");
    require(profile.textures.size() == 2, "both profile textures are loaded");
    require(profile.textures[0] == "sidewalk.bmp", "first texture order is preserved");
    require(profile.textures[1] == "road.bmp", "second texture order is preserved");
        require(profile.heightProfiles.size() == 4,
            "complete, malformed, and truncated height-profile records are retained");
    require(profile.heightProfiles[0].valid &&
                profile.heightProfiles[0].values ==
                    std::array<double, 4>{-4.0, -3.0, 0.25, 0.1} &&
                profile.heightProfiles[0].rawFields ==
                    std::array<std::string, 4>{"-4.0", "-3.0", "0.25", "0.10"},
            "height profile preserves four source fields and their numeric values");
    require(profile.heightProfiles[1].valid &&
                profile.heightProfiles[1].values ==
                    std::array<double, 4>{-3.0, 3.0, 0.1, 0.1},
            "second height-profile range is parsed in order");
    require(!profile.heightProfiles[2].valid &&
                profile.heightProfiles[2].rawFields[2] == "not-a-number",
            "malformed height values are retained but marked invalid");
    require(!profile.heightProfiles[3].valid &&
                profile.heightProfiles[3].rawFields[0] == "4.0" &&
                profile.heightProfiles[3].rawFields[1] == "5.0",
            "truncated height-profile records remain visible as invalid records");
    require(profile.sections.size() == 2, "both cross-section lanes are loaded");
    require(profile.sections[0].textureIndex == 0, "first lane uses texture zero");
    require(profile.sections[1].textureIndex == 1, "second lane uses texture one");
    require(profile.sections[0].points.size() == 2, "first lane has both profile points");
    require(profile.sections[1].points.size() == 2, "second lane has both profile points");
    requireNear(profile.sections[1].points[0].lateral, -3.0, "profile lateral offset");
    requireNear(profile.sections[1].points[0].height, 0.1, "profile height");
    requireNear(profile.sections[1].points[0].textureU, 0.995, "profile U coordinate");
    requireNear(profile.sections[1].points[0].textureVPerMeter, 0.167,
                "profile V repeat per meter");
}

void runInstalledProfileSmoke(const std::filesystem::path& path) {
    const openbus::map::MapSplineProfile profile = openbus::map::loadMapSplineProfile(path);
    require(!profile.editorOnly, "selected installed road profile is runtime-visible");
    require(!profile.textures.empty(), "installed profile provides textures");
    for (const openbus::map::MapSplineHeightProfile& heightProfile : profile.heightProfiles) {
        require(heightProfile.valid, "installed profile height records are numeric and complete");
    }
    bool hasRenderableSection = false;
    for (const openbus::map::MapSplineProfileSection& section : profile.sections) {
        if (section.points.size() >= 2 && section.textureIndex >= 0 &&
            static_cast<std::size_t>(section.textureIndex) < profile.textures.size()) {
            hasRenderableSection = true;
            break;
        }
    }
    require(hasRenderableSection, "installed profile has a textured cross-section strip");
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2) {
            runProbe(argv[1]);
        } else if (argc == 3 && std::string(argv[1]) == "--inspect") {
            runInstalledProfileSmoke(argv[2]);
        } else {
            throw std::runtime_error("expected fixture path or --inspect <installed-profile>");
        }
    } catch (const std::exception& error) {
        std::cerr << "map spline profile probe failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
