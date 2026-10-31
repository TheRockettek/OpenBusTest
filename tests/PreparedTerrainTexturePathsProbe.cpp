#include "ProbeTempDirectory.h"
#include "render/map/PreparedTerrainTexturePaths.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void writeFile(const std::filesystem::path& path) {
    std::ofstream output(path);
    output << "texture";
    require(static_cast<bool>(output), "Could not write texture fixture");
}

void runProbe(const std::filesystem::path& root) {
    openbus::rendering::PreparedTerrainTexturePaths paths;
    const auto first = root / "first.tga";
    const auto fallback = root / "fallback.tga";
    int calls = 0;
    const auto resolve = [&]() -> std::filesystem::path {
        ++calls;
        if (std::filesystem::is_regular_file(first)) {
            return first;
        }
        if (std::filesystem::is_regular_file(fallback)) {
            return fallback;
        }
        return {};
    };

    require(paths.resolve("ground", resolve).empty(), "Missing texture was not reported");
    writeFile(first);
    writeFile(fallback);
    require(paths.resolve("ground", resolve) == first, "A previously missing texture was cached");
    require(paths.resolve("ground", resolve) == first && calls == 2,
            "Successful resolution was not reused");
    require(std::filesystem::remove(first), "Could not remove the primary texture");
    require(paths.resolve("ground", resolve) == fallback,
            "A removed cached texture did not resolve its fallback");
    require(paths.resolve("ground", resolve) == fallback && calls == 3,
            "Fallback resolution was not reused");

    try {
        paths.resolve("broken", []() -> std::filesystem::path {
            throw std::runtime_error("resolution failed");
        });
        throw std::runtime_error("Resolution failure was not propagated");
    } catch (const std::runtime_error& error) {
        require(std::string(error.what()) == "resolution failed", "Wrong resolution failure");
    }
    require(paths.resolve("broken", resolve) == fallback, "Failed resolution could not retry");
    require(paths.resolutions() == 5 && paths.reuses() == 2,
            "Resolution and reuse counts were incorrect");
    std::cout << "Seven lookups: five resolution attempts, two reused paths\n";
}

} // namespace

int main() {
    const auto root = openbus::probe::makeProbeDirectory("openbus-terrain-texture-paths");
    int result = 0;
    try {
        runProbe(root);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    openbus::probe::removeProbeDirectory(root);
    return result;
}
