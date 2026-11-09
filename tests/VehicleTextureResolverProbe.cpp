#include "assets/VehicleTextureResolver.h"
#include "ProbeTempDirectory.h"

#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void write(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!(output << "fixture")) {
        throw std::runtime_error("Could not write texture fixture");
    }
}

bool sameFile(const std::filesystem::path& actual, const std::filesystem::path& expected) {
    return !actual.empty() && std::filesystem::equivalent(actual, expected);
}

void run(const std::filesystem::path& root) {
    const auto model = root / "vehicle" / "model";
    const auto texture = root / "vehicle" / "texture";
    std::filesystem::create_directories(model);
    write(model / "direct.bmp");
    write(model / "direct.dds");
    write(texture / "direct.bmp");
    write(texture / "parent.bmp");
    write(texture / "nested" / "Plate.png");
    write(texture / "nested" / "alternate.tga");
    write(texture / "nested" / "alternate.dds");
    write(model / "fallback.tga");
    write(model / "fallback.dds");
    write(model / "Texture" / "nearest.bmp");
    write(texture / "nearest.bmp");

    openbus::assets::VehicleTextureResolver resolver(false);
    require(sameFile(resolver.find(model, " direct.bmp "), model / "direct.dds"),
            "direct texture or compressed sibling priority changed");
    require(sameFile(resolver.find(model, "parent.bmp"), texture / "parent.bmp"),
            "ancestor texture directory was not searched");
    require(sameFile(resolver.find(model, "plate.PNG"), texture / "nested" / "Plate.png"),
            "recursive case-insensitive lookup failed");
    require(sameFile(resolver.find(model, "alternate.bmp"), texture / "nested" / "alternate.dds"),
            "recursive stem fallback did not prefer DDS");
    require(sameFile(resolver.find(model, "fallback.bmp"), model / "fallback.dds"),
            "direct stem fallback did not prefer DDS");
    require(sameFile(resolver.find(model, "nearest.bmp"), model / "Texture" / "nearest.bmp"),
            "nearest texture directory did not take priority");
    require(resolver.find(model, "").empty(), "empty texture name did not remain empty");

    require(resolver.find(model, "later.png").empty(), "missing texture unexpectedly resolved");
    write(texture / "later.png");
    require(resolver.find(model, "later.png").empty(),
            "runtime-style resolver did not retain its cached result");
    openbus::assets::VehicleTextureResolver freshResolver(false);
    require(sameFile(freshResolver.find(model, "later.png"), texture / "later.png"),
            "new resolver did not observe a newly installed texture");

    const auto notDirectory = root / "not-a-directory";
    write(notDirectory);
    openbus::assets::VehicleTextureResolver strictResolver(
        false, openbus::assets::VehicleTextureAccessPolicy::Strict);
    bool accessFailureReported = false;
    try {
        (void)strictResolver.find(notDirectory, "missing.png");
    } catch (const std::filesystem::filesystem_error&) {
        accessFailureReported = true;
    }
    require(accessFailureReported, "strict resolver suppressed a directory access error");
    require(std::filesystem::remove(notDirectory), "could not replace invalid fixture root");
    write(notDirectory / "missing.dds");
    require(sameFile(strictResolver.find(notDirectory, "missing.png"),
                     notDirectory / "missing.dds"),
            "strict resolver cached an access failure");

    openbus::assets::VehicleTextureResolver concurrentResolver(false);
    std::vector<std::future<std::filesystem::path>> lookups;
    for (int index = 0; index < 12; ++index) {
        lookups.push_back(std::async(std::launch::async,
                                     [&] { return concurrentResolver.find(model, "plate.PNG"); }));
    }
    for (auto& result : lookups) {
        require(sameFile(result.get(), texture / "nested" / "Plate.png"),
                "concurrent texture lookup returned a different path");
    }
}

} // namespace

int main() {
    const auto root = openbus::probe::makeProbeDirectory("openbus-vehicle-texture-resolver");
    try {
        run(root);
        openbus::probe::removeProbeDirectory(root);
        return 0;
    } catch (const std::exception& error) {
        openbus::probe::removeProbeDirectory(root);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
