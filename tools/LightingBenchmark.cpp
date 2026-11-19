#include "render/CoreRenderer.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"
#include "render/map/StaticLightTileIndex.h"
#include "render/map/StaticLightViewCache.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace {

struct Options {
    int lights = 500;
    int width = 320;
    int height = 180;
    int warmup = 5;
    int frames = 20;
    int vehicleLights = 0;
    bool moveVehicles = true;
    bool moveMapLights = false;
    bool persistent = true;
    bool sparse = false;
    bool sparsePrefix = false;
    int spacingMultiplier = 1;
};

int parseInteger(std::string_view text, int minimum, int maximum) {
    int value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || value < minimum ||
        value > maximum) {
        throw std::invalid_argument("Invalid benchmark argument: " + std::string(text));
    }
    return value;
}

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view name = argv[i];
        if (i + 1 >= argc) {
            throw std::invalid_argument("Missing value for " + std::string(name));
        }
        const std::string_view value = argv[++i];
        if (name == "--lights") {
            options.lights = parseInteger(value, 0, 100000);
        } else if (name == "--width") {
            options.width = parseInteger(value, 1, 4096);
        } else if (name == "--height") {
            options.height = parseInteger(value, 1, 4096);
        } else if (name == "--warmup") {
            options.warmup = parseInteger(value, 0, 10000);
        } else if (name == "--frames") {
            options.frames = parseInteger(value, 1, 10000);
        } else if (name == "--vehicle-lights") {
            options.vehicleLights = parseInteger(value, 0, 10000);
        } else if (name == "--move-vehicles") {
            options.moveVehicles = parseInteger(value, 0, 1) != 0;
        } else if (name == "--move-map-lights") {
            options.moveMapLights = parseInteger(value, 0, 1) != 0;
        } else if (name == "--spacing-multiplier") {
            options.spacingMultiplier = parseInteger(value, 1, 100);
        } else if (name == "--layout" && value == "sparse") {
            options.sparse = true;
            options.sparsePrefix = false;
        } else if (name == "--layout" && value == "sparse-prefix") {
            options.sparse = true;
            options.sparsePrefix = true;
        } else if (name == "--layout" && value == "dense") {
            options.sparse = false;
            options.sparsePrefix = false;
        } else if (name == "--storage" && value == "persistent") {
            options.persistent = true;
        } else if (name == "--storage" && value == "direct") {
            options.persistent = false;
        } else {
            throw std::invalid_argument("Invalid benchmark option or value: " + std::string(name) +
                                        " " + std::string(value));
        }
    }
    return options;
}

double percentile95(std::vector<double> samples) {
    std::sort(samples.begin(), samples.end());
    const std::size_t index = (samples.size() * 95 + 99) / 100 - 1;
    return samples[index];
}

int run(const Options& options) {
    using namespace openbus::rendering;
    if (!glfwInit()) {
        std::cerr << "GLFW initialization failed; no benchmark context is available\n";
        return 77;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window =
        glfwCreateWindow(options.width, options.height, "Lighting benchmark", nullptr, nullptr);
    if (window == nullptr) {
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        window =
            glfwCreateWindow(options.width, options.height, "Lighting benchmark", nullptr, nullptr);
    }
    if (window == nullptr) {
        std::cerr << "OpenGL context creation failed\n";
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    if (!loadOpenGLFunctions() || !initializeCoreRenderer()) {
        std::cerr << "Renderer initialization failed\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    if (pglGenQueries == nullptr || pglDeleteQueries == nullptr || pglBeginQuery == nullptr ||
        pglEndQuery == nullptr || pglGetQueryObjectui64v == nullptr) {
        std::cerr << "OpenGL elapsed-time queries are unavailable\n";
        shutdownCoreRenderer();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    std::cout << "vendor=" << glGetString(GL_VENDOR) << "\nrenderer=" << glGetString(GL_RENDERER)
              << "\nversion=" << glGetString(GL_VERSION) << '\n';
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
    if (framebufferWidth <= 0 || framebufferHeight <= 0) {
        std::cerr << "The benchmark framebuffer has an invalid size\n";
        shutdownCoreRenderer();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    glViewport(0, 0, framebufferWidth, framebufferHeight);
    setPerspective(framebufferWidth, framebufferHeight, 90);
    setModelViewMatrix(identityMatrix());
    setLightingViewMatrix(identityMatrix());
    setWeatherLighting(0.0);
    setWeatherFog(std::nullopt);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_DITHER);

    const float aspect =
        static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);
    const std::array<float, 39> vertices = {-3 * aspect, -3, -1, 0, 0, 0, 0, 0, -1, 1, 1, 1, 1,
                                            9 * aspect,  -3, -1, 1, 0, 0, 0, 0, -1, 1, 1, 1, 1,
                                            -3 * aspect, 9,  -1, 0, 1, 0, 0, 0, -1, 1, 1, 1, 1};
    GLuint buffer = 0;
    pglGenBuffers(1, &buffer);
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);

    std::vector<MapPointLight> lights;
    lights.reserve(static_cast<std::size_t>(options.lights));
    const double positionScale =
        (options.sparse ? 4.0 : 0.1) * static_cast<double>(options.spacingMultiplier);
    for (int i = 0; i < options.lights; ++i) {
        if (options.sparsePrefix) {
            const bool nearPrefix = i < 100;
            lights.push_back({nearPrefix ? std::array<double, 3>{0, 0, -90}
                                         : std::array<double, 3>{1000.0 * options.spacingMultiplier,
                                                                 0, -2},
                              {0.001, 0.0005, 0.00025}, nearPrefix ? 1.0 : 0.001});
            continue;
        }
        const double x = static_cast<double>((i * 37) % 101 - 50) * positionScale;
        const double y = static_cast<double>((i * 53) % 101 - 50) * positionScale;
        const double z = -2.0 - static_cast<double>(i % 19) / 10.0;
        lights.push_back({{x, y, z}, {0.002, 0.0015, 0.001}, 0.5});
    }
    std::vector<VehicleSpotlight> vehicleLights;
    vehicleLights.reserve(static_cast<std::size_t>(options.vehicleLights));
    for (int i = 0; i < options.vehicleLights; ++i) {
        const double x = static_cast<double>((i * 37) % 101 - 50) * positionScale;
        const double y = static_cast<double>((i * 53) % 101 - 50) * positionScale;
        vehicleLights.push_back({{x, y, -2.0}, {0, 0, 1}, {0.01, 0.008, 0.006}, 1.0, 30, 70});
    }
    resetLightingBufferUploadStats();
    setVehicleSpotlights(vehicleLights);
    StaticMapLightBuffer staticBuffer;
    std::vector<std::size_t> sourceIndices(lights.size());
    std::iota(sourceIndices.begin(), sourceIndices.end(), std::size_t{0});
    map_detail::StaticLightTileIndex tileIndex;
    constexpr double tileSize = 25.0;
    for (std::size_t i = 0; i < lights.size(); ++i) {
        const auto& position = lights[i].position;
        const int x = static_cast<int>(std::floor(position[0] / tileSize));
        const int y = static_cast<int>(std::floor(position[1] / tileSize));
        tileIndex.add(map_detail::tileKey(x, y), i);
        tileIndex.addInfluence(i, position, 20.0, tileSize);
    }
    std::array<std::unordered_set<std::uint64_t>, 2> tileSets;
    for (int x = -2; x <= 2; ++x) {
        for (int y = -2; y <= 2; ++y) {
            tileSets[0].insert(map_detail::tileKey(x, y));
            tileSets[1].insert(map_detail::tileKey(x + 1, y));
        }
    }
    std::vector<double> tileSelectionMs;
    tileSelectionMs.reserve(static_cast<std::size_t>(options.frames));
    std::size_t selectedTilesLights = 0;
    for (int frame = 0; frame < options.warmup + options.frames; ++frame) {
        const auto start = std::chrono::steady_clock::now();
        const auto& selected = tileIndex.activeIndices(tileSets[frame % 2]);
        const double elapsed =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
        selectedTilesLights = selected.size();
        if (selectedTilesLights > lights.size()) {
            throw std::runtime_error("Tile index returned duplicate or invalid lights");
        }
        if (frame >= options.warmup) {
            tileSelectionMs.push_back(elapsed);
        }
    }
    map_detail::StaticLightViewCache viewCache;
    std::vector<double> viewPrepareMs;
    viewPrepareMs.reserve(static_cast<std::size_t>(options.frames));
    for (int frame = 0; frame < options.warmup + options.frames; ++frame) {
        const double offset = static_cast<double>(frame) * 0.001;
        const auto view = translationMatrix({offset, 0, 0});
        const auto start = std::chrono::steady_clock::now();
        const bool prepared =
            viewCache.prepare(lights, sourceIndices, view, {-offset, 0, 0}, lights.size(), 1);
        const double elapsed =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
        if (!prepared || viewCache.lights().size() != lights.size()) {
            throw std::runtime_error("Static-light view preparation did not produce all lights");
        }
        if (frame >= options.warmup) {
            viewPrepareMs.push_back(elapsed);
        }
    }
    viewCache.prepare(lights, sourceIndices, identityMatrix(), {0, 0, 0}, lights.size(), 1);
    const auto& viewLights = viewCache.lights();
    auto movingViews = viewLights;
    auto movingLights = lights;
    const auto selectionStart = std::chrono::steady_clock::now();
    if (options.persistent) {
        setStaticMapLights(staticBuffer, lights, sourceIndices, viewLights, 1.0, 1);
    } else {
        setMapLights(lights, 1.0);
    }
    const double selectionMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - selectionStart)
            .count();
    const auto stats = lightingBufferUploadStats();
    if (stats.mapLightsMax != lights.size()) {
        std::cerr << "Requested " << lights.size() << " lights but renderer selected "
                  << stats.mapLightsMax << "; check OPENBUS_MAP_LIGHT_LIMIT\n";
        releaseStaticMapLights(staticBuffer);
        deleteRendererBuffers(1, &buffer);
        shutdownCoreRenderer();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    std::array<GLuint, 3> queries = {};
    pglGenQueries(static_cast<GLsizei>(queries.size()), queries.data());
    if (queries[0] == 0 || queries[1] == 0 || queries[2] == 0) {
        std::cerr << "GPU timer query allocation failed\n";
        pglDeleteQueries(static_cast<GLsizei>(queries.size()), queries.data());
        releaseStaticMapLights(staticBuffer);
        deleteRendererBuffers(1, &buffer);
        shutdownCoreRenderer();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    std::vector<double> gpuMs;
    std::vector<double> gpuVehicleUpdateMs;
    std::vector<double> vehicleSetMs;
    std::vector<double> mapSetMs;
    std::vector<double> gpuMapUpdateMs;
    LightingBufferUploadStats accumulatedVehicleStats;
    const auto accumulateVehicleStats = [&](const LightingBufferUploadStats& snapshot) {
        accumulatedVehicleStats.vehicleBytes += snapshot.vehicleBytes;
        accumulatedVehicleStats.vehicleSets += snapshot.vehicleSets;
        accumulatedVehicleStats.vehicleLightsMax =
            std::max(accumulatedVehicleStats.vehicleLightsMax, snapshot.vehicleLightsMax);
        accumulatedVehicleStats.vehicleSpotlightTileSets += snapshot.vehicleSpotlightTileSets;
        accumulatedVehicleStats.vehicleSpotlightGpuTileSets += snapshot.vehicleSpotlightGpuTileSets;
        accumulatedVehicleStats.vehicleSpotlightTileFallbackSets +=
            snapshot.vehicleSpotlightTileFallbackSets;
        accumulatedVehicleStats.mapGpuTileSets += snapshot.mapGpuTileSets;
        accumulatedVehicleStats.mapGpuTileCountTotal += snapshot.mapGpuTileCountTotal;
        accumulatedVehicleStats.mapGpuTileAssignmentsTotal += snapshot.mapGpuTileAssignmentsTotal;
        accumulatedVehicleStats.mapGpuTileFallbackSets += snapshot.mapGpuTileFallbackSets;
    };
    gpuMs.reserve(static_cast<std::size_t>(options.frames));
    vehicleSetMs.reserve(static_cast<std::size_t>(options.frames));
    gpuVehicleUpdateMs.reserve(static_cast<std::size_t>(options.frames));
    const ModelMaterial material;
    glClearColor(0, 0, 0, 0);
    for (int frame = 0; frame < options.warmup + options.frames; ++frame) {
        const auto beforePoll = lightingBufferUploadStats();
        beginGpuTimingFrame();
        if (lightingBufferUploadStats().vehicleSets < beforePoll.vehicleSets ||
            lightingBufferUploadStats().mapSets < beforePoll.mapSets)
            accumulateVehicleStats(beforePoll);
        if (options.moveMapLights && !lights.empty()) {
            const double offset = 0.1 * std::sin(static_cast<double>(frame) * 0.1);
            for (std::size_t i = 0; i < lights.size(); ++i) {
                movingViews[i].position[0] = viewLights[i].position[0] + offset;
                movingLights[i].position[0] = lights[i].position[0] + offset;
            }
            pglBeginQuery(GL_TIME_ELAPSED, queries[2]);
            const auto updateStart = std::chrono::steady_clock::now();
            if (options.persistent)
                setStaticMapLights(staticBuffer, lights, sourceIndices, movingViews, 1.0, 1);
            else
                setMapLights(movingLights, 1.0);
            const double updateMs = std::chrono::duration<double, std::milli>(
                                        std::chrono::steady_clock::now() - updateStart)
                                        .count();
            pglEndQuery(GL_TIME_ELAPSED);
            if (frame >= options.warmup)
                mapSetMs.push_back(updateMs);
        }
        if (options.moveVehicles && !vehicleLights.empty()) {
            for (std::size_t i = 0; i < vehicleLights.size(); ++i) {
                vehicleLights[i].position[2] =
                    -2.0 - 0.1 * std::sin(static_cast<double>(frame) * 0.1 + i * 0.01);
            }
            pglBeginQuery(GL_TIME_ELAPSED, queries[1]);
            const auto updateStart = std::chrono::steady_clock::now();
            setVehicleSpotlights(vehicleLights);
            const double updateMs = std::chrono::duration<double, std::milli>(
                                        std::chrono::steady_clock::now() - updateStart)
                                        .count();
            pglEndQuery(GL_TIME_ELAPSED);
            if (frame >= options.warmup) {
                vehicleSetMs.push_back(updateMs);
            }
        }
        glClear(GL_COLOR_BUFFER_BIT);
        pglBeginQuery(GL_TIME_ELAPSED, queries[0]);
        drawModelBatch(buffer, 3, material, {1, 1, 1}, 1.0, 0);
        pglEndQuery(GL_TIME_ELAPSED);
        std::uint64_t nanoseconds = 0;
        pglGetQueryObjectui64v(queries[0], GL_QUERY_RESULT, &nanoseconds);
        if (frame >= options.warmup) {
            gpuMs.push_back(static_cast<double>(nanoseconds) / 1'000'000.0);
        }
        if (options.moveMapLights && !lights.empty()) {
            pglGetQueryObjectui64v(queries[2], GL_QUERY_RESULT, &nanoseconds);
            if (frame >= options.warmup)
                gpuMapUpdateMs.push_back(static_cast<double>(nanoseconds) / 1'000'000.0);
        }
        if (options.moveVehicles && !vehicleLights.empty()) {
            pglGetQueryObjectui64v(queries[1], GL_QUERY_RESULT, &nanoseconds);
            if (frame >= options.warmup)
                gpuVehicleUpdateMs.push_back(static_cast<double>(nanoseconds) / 1'000'000.0);
        }
    }
    accumulateVehicleStats(lightingBufferUploadStats());
    const auto& finalStats = accumulatedVehicleStats;
    if (finalStats.vehicleLightsMax != vehicleLights.size() && !vehicleLights.empty()) {
        std::cerr << "Requested " << vehicleLights.size()
                  << " vehicle lights but renderer selected " << finalStats.vehicleLightsMax
                  << "; check OPENBUS_VEHICLE_SPOTLIGHT_LIMIT\n";
        pglDeleteQueries(static_cast<GLsizei>(queries.size()), queries.data());
        releaseStaticMapLights(staticBuffer);
        deleteRendererBuffers(1, &buffer);
        shutdownCoreRenderer();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    for (const auto& position :
         {std::array<int, 2>{0, 0}, std::array<int, 2>{framebufferWidth - 1, 0},
          std::array<int, 2>{0, framebufferHeight - 1},
          std::array<int, 2>{framebufferWidth - 1, framebufferHeight - 1}}) {
        std::array<std::uint8_t, 4> pixel = {};
        glReadPixels(position[0], position[1], 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
        if (pixel[3] != 255) {
            std::cerr << "Benchmark geometry did not cover the full framebuffer\n";
            pglDeleteQueries(static_cast<GLsizei>(queries.size()), queries.data());
            releaseStaticMapLights(staticBuffer);
            deleteRendererBuffers(1, &buffer);
            shutdownCoreRenderer();
            glfwDestroyWindow(window);
            glfwTerminate();
            return 1;
        }
    }
    const double mean = std::accumulate(gpuMs.begin(), gpuMs.end(), 0.0) / gpuMs.size();
    std::cout << std::fixed << std::setprecision(4) << "submitted_map_lights=" << lights.size()
              << " selected_map_lights=" << stats.mapLightsMax
              << " map_gpu_tile_sets=" << finalStats.mapGpuTileSets
              << " map_gpu_tile_count=" << finalStats.mapGpuTileCountTotal
              << " map_gpu_tile_assignments=" << finalStats.mapGpuTileAssignmentsTotal
              << " map_gpu_tile_fallback_sets=" << finalStats.mapGpuTileFallbackSets
              << " tile_selected_map_lights=" << selectedTilesLights
              << " submitted_vehicle_lights=" << vehicleLights.size()
              << " selected_vehicle_lights=" << finalStats.vehicleLightsMax
              << " moving_vehicles=" << (options.moveVehicles ? 1 : 0)
              << " moving_map_lights=" << (options.moveMapLights ? 1 : 0)
              << " layout=" << (options.sparsePrefix ? "sparse-prefix"
                                : options.sparse    ? "sparse"
                                                    : "dense")
              << " spacing_multiplier=" << options.spacingMultiplier
              << " storage=" << (options.persistent ? "persistent" : "direct")
              << " requested_window=" << options.width << 'x' << options.height
              << " framebuffer=" << framebufferWidth << 'x' << framebufferHeight
              << " warmup=" << options.warmup << " samples=" << gpuMs.size()
              << " cpu_initial_set_ms=" << selectionMs << " cpu_tile_selection_mean_ms="
              << std::accumulate(tileSelectionMs.begin(), tileSelectionMs.end(), 0.0) /
                     tileSelectionMs.size()
              << " cpu_tile_selection_p95_ms=" << percentile95(tileSelectionMs)
              << " cpu_view_prepare_mean_ms="
              << std::accumulate(viewPrepareMs.begin(), viewPrepareMs.end(), 0.0) /
                     viewPrepareMs.size()
              << " cpu_view_prepare_p95_ms=" << percentile95(viewPrepareMs)
              << " view_upload_bytes=" << stats.mapBytes
              << " source_upload_bytes=" << stats.staticMapBytes
              << " vehicle_upload_bytes=" << finalStats.vehicleBytes
              << " cpu_spotlight_tile_sets=" << finalStats.vehicleSpotlightTileSets
              << " gpu_spotlight_tile_sets=" << finalStats.vehicleSpotlightGpuTileSets
              << " spotlight_tile_fallback_sets=" << finalStats.vehicleSpotlightTileFallbackSets
              << " gpu_draw_mean_ms=" << mean << " gpu_draw_p95_ms=" << percentile95(gpuMs);
    if (!vehicleSetMs.empty()) {
        std::cout << " cpu_vehicle_set_mean_ms="
                  << std::accumulate(vehicleSetMs.begin(), vehicleSetMs.end(), 0.0) /
                         vehicleSetMs.size()
                  << " cpu_vehicle_set_p95_ms=" << percentile95(vehicleSetMs);
    }
    if (!mapSetMs.empty()) {
        std::cout << " cpu_map_set_mean_ms="
                  << std::accumulate(mapSetMs.begin(), mapSetMs.end(), 0.0) / mapSetMs.size()
                  << " cpu_map_set_p95_ms=" << percentile95(mapSetMs)
                  << " gpu_map_update_mean_ms="
                  << std::accumulate(gpuMapUpdateMs.begin(), gpuMapUpdateMs.end(), 0.0) /
                         gpuMapUpdateMs.size()
                  << " gpu_map_update_p95_ms=" << percentile95(gpuMapUpdateMs);
    }
    if (!gpuVehicleUpdateMs.empty()) {
        std::cout << " gpu_vehicle_update_mean_ms="
                  << std::accumulate(gpuVehicleUpdateMs.begin(), gpuVehicleUpdateMs.end(), 0.0) /
                         gpuVehicleUpdateMs.size()
                  << " gpu_vehicle_update_p95_ms=" << percentile95(gpuVehicleUpdateMs);
    }
    std::cout << '\n';
    pglDeleteQueries(static_cast<GLsizei>(queries.size()), queries.data());
    releaseStaticMapLights(staticBuffer);
    deleteRendererBuffers(1, &buffer);
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    try {
        return run(parseOptions(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "Lighting benchmark failed: " << error.what() << '\n';
        return 1;
    }
}
