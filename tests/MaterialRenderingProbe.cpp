#include "render/CoreRenderer.h"
#include "core/Environment.h"
#include "core/Variables.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"
#include "render/RendererReflection.h"
#include "render/StaticMapLightFallback.h"
#include "render/map/MapRendererInternal.h"
#include "app/traffic/ReflectionTrafficViews.h"
#include "app/vehicle/VehicleDrawSorting.h"
#include "app/vehicle/VehicleSpotlight.h"
#include "physics/BusSimulation.h"
#include "physics/World.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string_view>

using namespace openbus::rendering;

namespace {

bool setReflectionInterval(const char* value) {
#ifdef _WIN32
    return _putenv_s("OPENBUS_REFLECTION_INTERVAL", value) == 0;
#else
    return setenv("OPENBUS_REFLECTION_INTERVAL", value, 1) == 0;
#endif
}

void failBufferUpload(GLenum, std::ptrdiff_t, const void*, GLenum) {
    glEnable(static_cast<GLenum>(0xffffffffU));
    static_cast<void>(glGetError());
}

decltype(pglBufferData) successfulBufferData = nullptr;

void failAfterBufferUpload(GLenum target, std::ptrdiff_t size, const void* data, GLenum usage) {
    successfulBufferData(target, size, data, usage);
    glEnable(static_cast<GLenum>(0xffffffffU));
    static_cast<void>(glGetError());
}

void APIENTRY failTextureBufferBinding(GLenum, GLenum, GLuint) {
    glEnable(static_cast<GLenum>(0xffffffffU));
    static_cast<void>(glGetError());
}

} // namespace

int main(int argc, char** argv) {
    const bool spotlightTileProbe =
        argc == 2 && std::string_view(argv[1]) == "--spotlight-tile-image";
    using openbus::app::vehicle::makeSpotlight;
    using openbus::app::vehicle::spotlightSelectionChanged;
    using openbus::app::vehicle::spotlightSelectionStatus;
    using openbus::app::vehicle::SpotlightStatus;
    ModelSpotlight high;
    high.parameters = {1, 2, 3, 0, 1, -0.3, 255, 128, 0, 500, 30, 70};
    ModelSpotlight low = high;
    low.parameters[9] = 100;
    const std::vector<ModelSpotlight> definitions = {high, low};
    if (spotlightSelectionStatus(definitions.size(), -0.6) != SpotlightStatus::Off ||
        spotlightSelectionStatus(definitions.size(), 0.6) != SpotlightStatus::Active ||
        spotlightSelectionStatus(definitions.size(), 2) != SpotlightStatus::Invalid ||
        spotlightSelectionStatus(definitions.size(), std::numeric_limits<double>::quiet_NaN()) !=
            SpotlightStatus::Invalid) {
        std::cerr << "vehicle spotlight early selection failed\n";
        return 1;
    }
    const Matrix4 vehicleView = multiplyMatrix4(translationMatrix({10, 20, 30}),
                                                 rotationMatrix(90, 0, 0, 1));
    VehicleSpotlight transformed;
    if (makeSpotlight(definitions, -1, vehicleView, transformed) != SpotlightStatus::Off ||
        makeSpotlight(definitions, 2, vehicleView, transformed) != SpotlightStatus::Invalid ||
        makeSpotlight(definitions, 1, vehicleView, transformed) != SpotlightStatus::Active ||
        transformed.range != 100 || std::abs(transformed.position[0] - 11) > 1e-8 ||
        std::abs(transformed.position[1] - 22) > 1e-8 ||
        std::abs(transformed.position[2] - 33) > 1e-8 ||
        std::abs(transformed.direction[0]) > 1e-8 ||
        std::abs(transformed.direction[1] - 1) > 1e-8 ||
        std::abs(transformed.direction[2] + 0.3) > 1e-8 ||
        makeSpotlight(definitions, 0, multiplyMatrix4(translationMatrix({-5, 0, 0}), vehicleView),
                      transformed) != SpotlightStatus::Active ||
        transformed.range != 500 || std::abs(transformed.position[0] - 6) > 1e-8 ||
        std::abs(transformed.direction[1] - 1) > 1e-8) {
        std::cerr << "vehicle spotlight selection or per-view transform failed\n";
        return 1;
    }
    if (makeSpotlight(definitions, 0.6, vehicleView, transformed) != SpotlightStatus::Active ||
        transformed.range != 100 ||
        makeSpotlight(definitions, -0.6, vehicleView, transformed) != SpotlightStatus::Off ||
        !spotlightSelectionChanged(0.49999, 0.50001) ||
        spotlightSelectionChanged(0.1, 0.2) ||
        !spotlightSelectionChanged(std::numeric_limits<double>::quiet_NaN(), 0.0)) {
        std::cerr << "vehicle spotlight selector was not rounded\n";
        return 1;
    }
    ModelSpotlight bounded = high;
    bounded.parameters = {0, 0, 0, 0, 1, 0, 255, 128, 0, 0.1, 30, 70};
    const auto unitFrustum = buildViewFrustum(identityMatrix());
    const auto shiftedFrustum = buildViewFrustum(translationMatrix({-5, 0, 0}));
    if (openbus::app::vehicle::spotlightOutsideView(
            {bounded}, 0, translationMatrix({1.3, 0, 0}), unitFrustum) ||
        !openbus::app::vehicle::spotlightOutsideView(
            {bounded}, 0, translationMatrix({1.5, 0, 0}), unitFrustum) ||
        openbus::app::vehicle::spotlightOutsideView(
            {bounded}, 0, translationMatrix({5, 0, 0}), shiftedFrustum) ||
        !openbus::app::vehicle::spotlightOutsideView(
            {bounded}, 0, translationMatrix({0, 0, 0}), shiftedFrustum) ||
        openbus::app::vehicle::spotlightOutsideView(
            {bounded}, 1, translationMatrix({10, 0, 0}), unitFrustum)) {
        std::cerr << "AI spotlight conservative view suspension failed\n";
        return 1;
    }
    ModelSpotlight invalidSpotlight = high;
    invalidSpotlight.parameters[3] = 0;
    invalidSpotlight.parameters[4] = 0;
    invalidSpotlight.parameters[5] = 0;
    if (makeSpotlight({invalidSpotlight}, 0, vehicleView, transformed) != SpotlightStatus::Invalid ||
        makeSpotlight(definitions, std::numeric_limits<double>::quiet_NaN(), vehicleView,
                      transformed) != SpotlightStatus::Invalid) {
        std::cerr << "invalid vehicle spotlight was accepted\n";
        return 1;
    }
    VehicleSpotlight playerLight;
    playerLight.position = {100, 0, 0};
    VehicleSpotlight nearLight;
    nearLight.position = {1, 0, 0};
    VehicleSpotlight otherNearLight;
    otherNearLight.position = {-1, 0, 0};
    std::vector<openbus::app::vehicle::SpotlightCandidate> candidates = {
        {nearLight, false}, {playerLight, true}, {otherNearLight, false}};
    const auto selected = openbus::app::vehicle::selectSpotlights(candidates, 2);
    if (selected.size() != 2 || selected[0].position != playerLight.position ||
        selected[1].position != nearLight.position ||
        !openbus::app::vehicle::selectSpotlights(candidates, 0).empty()) {
        std::cerr << "vehicle spotlight priority or stable distance ordering failed\n";
        return 1;
    }
    candidates = {{otherNearLight, false}, {nearLight, false}};
    const auto aiOnly = openbus::app::vehicle::selectSpotlights(candidates, 2);
    if (aiOnly.size() != 2 || aiOnly[0].position != otherNearLight.position ||
        aiOnly[1].position != nearLight.position) {
        std::cerr << "AI-only spotlight ordering changed\n";
        return 1;
    }
    map_detail::StaticLightTileIndex lightIndex;
    for (std::size_t i = 0; i < 250; ++i) {
        lightIndex.add(i % 4, i);
    }
    const std::unordered_set<std::uint64_t> tiles = {1, 3};
    const auto& activeLights = lightIndex.activeIndices(tiles);
    if (activeLights.size() != 125 || lightIndex.allIndices().size() != 250 ||
        lightIndex.allIndices().front() != 0 || lightIndex.allIndices().back() != 249) {
        std::cerr << "static light tile index omitted active lights\n";
        return 1;
    }
    for (std::size_t i = 0; i < activeLights.size(); ++i) {
        if (activeLights[i] != i * 2 + 1) {
            std::cerr << "static light tile index changed source order\n";
            return 1;
        }
    }
    lightIndex.add(3, 250);
    if (lightIndex.activeIndices(tiles).size() != 126 ||
        lightIndex.activeIndices(tiles).back() != 250 ||
        lightIndex.allIndices().size() != 251 || lightIndex.allIndices().back() != 250 ||
        lightIndex.activeIndices({}).size() != 0 ||
        lightIndex.activeIndices({0}).size() != 63 ||
        lightIndex.activeIndices(tiles).size() != 126) {
        std::cerr << "static light tile index did not refresh on streaming changes\n";
        return 1;
    }
    map_detail::StaticLightTileIndex influenceIndex;
    influenceIndex.add(map_detail::tileKey(0, 0), 0);
    influenceIndex.addInfluence(0, {299, 150, 0}, 5, 300);
    influenceIndex.add(map_detail::tileKey(-1, -1), 1);
    influenceIndex.addInfluence(1, {-1, -1, 0}, 2, 300);
    const std::unordered_set<std::uint64_t> adjacent = {map_detail::tileKey(1, 0)};
    if (influenceIndex.activeIndices(adjacent) != std::vector<std::size_t>{0} ||
        influenceIndex.activeIndices({map_detail::tileKey(0, 0)}) !=
            std::vector<std::size_t>({0, 1}) ||
        influenceIndex.activeIndices({map_detail::tileKey(2, 0)}).size() != 0 ||
        influenceIndex.allIndices() != std::vector<std::size_t>({0, 1}) ||
        influenceIndex.activeIndices({}).size() != 0) {
        std::cerr << "static light influence tile selection or deduplication failed\n";
        return 1;
    }
    influenceIndex.add(map_detail::tileKey(0, 0), 2);
    influenceIndex.addInfluence(2, {0, 0, 0}, 1.0e20, 300);
    if (influenceIndex.activeIndices({map_detail::tileKey(2, 0)}) !=
            std::vector<std::size_t>{2} ||
        !influenceIndex.activeIndices({}).empty()) {
        std::cerr << "large static light influence fallback failed\n";
        return 1;
    }
    bool rejectedInvalidInfluence = false;
    try {
        influenceIndex.addInfluence(3, {0, 0, 0}, 0, 300);
    } catch (const std::invalid_argument&) {
        rejectedInvalidInfluence = true;
    }
    if (!rejectedInvalidInfluence) {
        std::cerr << "invalid static light influence was not rejected\n";
        return 1;
    }
    map_detail::StaticLightViewCache viewCache;
    std::vector<MapPointLight> viewSources = {
        {{0, 0, 0}, {1, 0, 0}, 1}, {{10, 0, 0}, {0, 1, 0}, 1},
        {{20, 0, 0}, {0, 0, 1}, 1}};
    const std::vector<std::size_t> viewIndices = {2, 0, 1};
    const Matrix4 viewIdentity = identityMatrix();
    if (!viewCache.prepare(viewSources, viewIndices, viewIdentity, {0, 0, 0}, 3) ||
        viewCache.lights().size() != 3 || viewCache.lights()[0].position[0] != 20 ||
        viewCache.lights()[1].position[0] != 0 ||
        viewCache.prepare(viewSources, viewIndices, viewIdentity, {0, 0, 0}, 3) ||
        !viewCache.prepare(viewSources, viewIndices, translationMatrix({1, 0, 0}), {0, 0, 0}, 3) ||
        viewCache.lights()[0].position[0] != 21 ||
        !viewCache.prepare(viewSources, viewIndices, viewIdentity, {0, 0, 0}, 1) ||
        viewCache.lights().size() != 1 || viewCache.lights()[0].sourceIndex != 1 ||
        !viewCache.prepare(viewSources, viewIndices, viewIdentity, {20, 0, 0}, 1) ||
        viewCache.lights()[0].sourceIndex != 0) {
        std::cerr << "static light view cache reuse, transforms, or selection failed\n";
        return 1;
    }
    viewSources[2].position[0] = 21;
    if (!viewCache.prepare(viewSources, viewIndices, viewIdentity, {20, 0, 0}, 1, 1) ||
        viewCache.lights()[0].position[0] != 21 ||
        !viewCache.prepare(viewSources, {0}, viewIdentity, {0, 0, 0}, 1, 1) ||
        viewCache.lights()[0].position[0] != 0 ||
        !viewCache.prepare(viewSources, viewIndices, viewIdentity, {0, 0, 0}, 0, 1) ||
        !viewCache.lights().empty()) {
        std::cerr << "static light view cache invalidation failed\n";
        return 1;
    }
    const std::vector<MapPointLight> fallbackSources = {
        {{0, 0, 0}, {1, 0, 0}, 1}, {{1, 0, 0}, {0, 1, 0}, 2},
        {{2, 0, 0}, {0, 0, 1}, 3}, {{3, 0, 0}, {1, 1, 0}, 4}};
    const std::vector<std::size_t> fallbackIndices = {2, 0, 3, 1};
    const std::vector<StaticMapLightView> fallbackViews = {
        {{10, 0, -1}, 3}, {{20, 0, -2}, 0}, {{30, 0, -3}, 99}};
    const auto fallback = detail::compactStaticMapLightSelection(
        fallbackSources, fallbackIndices, fallbackViews, 2, 3);
    if (!fallback || fallback->sourceIndices != std::vector<std::size_t>{1, 2} ||
        fallback->viewLights.size() != 2 ||
        fallback->viewLights[0].position != fallbackViews[0].position ||
        fallback->viewLights[0].sourceIndex != 0 ||
        fallback->viewLights[1].position != fallbackViews[1].position ||
        fallback->viewLights[1].sourceIndex != 1 ||
        detail::compactStaticMapLightSelection(fallbackSources, fallbackIndices, fallbackViews, 2, 4)) {
        std::cerr << "oversized static sources did not preserve the selected view lights\n";
        return 1;
    }
    bool rejectedFallbackIndex = false;
    try {
        static_cast<void>(detail::compactStaticMapLightSelection(
            fallbackSources, fallbackIndices, fallbackViews, 3, 3));
    } catch (const std::invalid_argument&) {
        rejectedFallbackIndex = true;
    }
    if (!rejectedFallbackIndex) {
        std::cerr << "oversized static sources accepted an invalid selected view\n";
        return 1;
    }
    const std::vector<StaticMapLightView> duplicateViews = {fallbackViews[0], fallbackViews[0]};
    const auto duplicateFallback = detail::compactStaticMapLightSelection(
        fallbackSources, fallbackIndices, duplicateViews, 2, 3);
    if (!duplicateFallback ||
        duplicateFallback->sourceIndices != std::vector<std::size_t>{1, 1} ||
        duplicateFallback->viewLights.size() != 2 ||
        duplicateFallback->viewLights[0].sourceIndex != 0 ||
        duplicateFallback->viewLights[1].sourceIndex != 1) {
        std::cerr << "oversized static sources did not preserve duplicate selections\n";
        return 1;
    }
    bool rejectedFallbackSource = false;
    try {
        static_cast<void>(detail::compactStaticMapLightSelection(
            fallbackSources, {fallbackSources.size(), 0}, {{{0, 0, 0}, 0}}, 1, 1));
    } catch (const std::invalid_argument&) {
        rejectedFallbackSource = true;
    }
    if (!rejectedFallbackSource) {
        std::cerr << "oversized static sources accepted an invalid source mapping\n";
        return 1;
    }
    MapPoleState pole;
    pole.basePosition = {51.0, 50.3, 5.5};
    pole.rotationAxis = {0.0, 1.0, 0.0};
    pole.angleRadians = std::numbers::pi / 2.0;
    const Matrix4 bent = map_detail::bentPoleTransform(translationMatrix({50.0, 50.0, 5.0}), pole);
    const auto base = transformPoint(bent, {1.0, 0.3, 0.5, 1.0});
    const auto top = transformPoint(bent, {1.0, 0.3, 4.5, 1.0});
    if (std::abs(base[0] - 51.0) > 1.0e-8 ||
        std::abs(base[1] - 50.3) > 1.0e-8 ||
        std::abs(base[2] - 5.5) > 1.0e-8 ||
        std::abs(top[0] - 55.0) > 1.0e-8 ||
        std::abs(top[1] - 50.3) > 1.0e-8 ||
        std::abs(top[2] - 5.5) > 1.0e-8) {
        std::cerr << "bent model does not rotate about the collision base\n";
        return 1;
    }
    if (!glfwInit()) {
        return 77;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    const int windowSize = spotlightTileProbe ? 160 : 32;
    GLFWwindow* window = glfwCreateWindow(windowSize, windowSize, "Material probe", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    if (!loadOpenGLFunctions() || !initializeCoreRenderer()) {
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    if (argc == 2 && (std::string_view(argv[1]) == "--spotlight-limit-zero" ||
                      std::string_view(argv[1]) == "--spotlight-limit-one")) {
        const std::size_t expected = std::string_view(argv[1]) == "--spotlight-limit-one" ? 1 : 0;
        VehicleSpotlight light{{0, 0, -2}, {0, 0, 1}, {1, 0.5, 0.25}, 10, 30, 70};
        resetLightingBufferUploadStats();
        setVehicleSpotlights({light, light});
        const auto stats = lightingBufferUploadStats();
        const bool ok = stats.vehicleLightsMax == expected &&
                        stats.vehicleLightsTotal == expected &&
                        stats.vehicleUploads == expected &&
                        stats.vehicleBytes == expected * 48 && glGetError() == GL_NO_ERROR;
        if (!ok) {
            std::cerr << "explicit vehicle spotlight limit was not applied\n";
        }
        shutdownCoreRenderer();
        glfwDestroyWindow(window);
        glfwTerminate();
        return ok ? 0 : 1;
    }
    static_cast<void>(hasOpenGLExtension("GL_EXT_texture_compression_s3tc"));
    if (glGetError() != GL_NO_ERROR) {
        std::cerr << "OpenGL core-profile extension enumeration produced an error\n";
        shutdownCoreRenderer();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    setPerspective(32, 32, 90);
    setModelViewMatrix(identityMatrix());
    glViewport(0, 0, 32, 32);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_DITHER);

    openbus::rendering::beginGpuTimingFrame();
    {
        openbus::rendering::GpuTimerScope gpuTimer(openbus::rendering::GpuTimingPass::MainView);
        glClear(GL_COLOR_BUFFER_BIT);
    }
    {
        openbus::rendering::GpuTimerScope gpuTimer(openbus::rendering::GpuTimingPass::Reflection);
        glClear(GL_COLOR_BUFFER_BIT);
    }
    if (glGetError() != GL_NO_ERROR) {
        std::cerr << "GPU timer query scope produced an OpenGL error\n";
        shutdownCoreRenderer();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    constexpr std::array<float, 39> vertices = {
        -2, -2, -1, 0, 0, 0, 0, 0, -1, 1, 1, 1, 1,
         2, -2, -1, 1, 0, 0, 0, 0, -1, 1, 1, 1, 1,
         0,  2, -1, 0, 1, 0, 0, 0, -1, 1, 1, 1, 1};
    GLuint buffer = 0;
    pglGenBuffers(1, &buffer);
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);
    const auto texture = [](std::array<std::uint8_t, 4> pixel) {
        GLuint id = 0;
        glGenTextures(1, &id);
        glBindTexture(GL_TEXTURE_2D, id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
        invalidateTextureBindings();
        return id;
    };
    const GLuint blue = texture({65, 85, 112, 255});
    const GLuint white = texture({255, 255, 255, 255});
    const GLuint mask = texture({0, 0, 0, 128});
    if (spotlightTileProbe) {
        constexpr int viewportX = 11;
        constexpr int viewportY = 13;
        constexpr int viewportSize = 128;
        glViewport(viewportX, viewportY, viewportSize, viewportSize);
        setPerspective(viewportSize, viewportSize, 90);
        setWeatherLighting(0.0);
        setWeatherFog(std::nullopt);
        setMapLights({}, 0.0);
        const VehicleSpotlight behind{{0, 0, 5}, {0, 0, 1}, {0, 0, 0}, 0.1, 10, 20};
        const VehicleSpotlight visible{{-0.25, 0, -1.08}, {0, 0, 1},
                                       {1, 0.75, 0.5}, 0.1, 30, 70};
        const VehicleSpotlight nearCrossing{{-0.25, 0, 0.1}, {0, 0, -1},
                                            {1, 0.75, 0.5}, 2, 20, 30};
        const VehicleSpotlight broad{{0, 0, -2}, {0, 0, 1}, {0, 0, 0}, 10, 30, 70};
        ModelMaterial tileMaterial;
        tileMaterial.textured = true;
        tileMaterial.texture = white;
        const std::vector<std::array<float, 4>> colors = {{1, 1, 1, 1}};
        const auto image = [&](bool multiMaterial) {
            glClearColor(0, 0, 0, 0);
            glClear(GL_COLOR_BUFFER_BIT);
            if (multiMaterial) {
                drawMaterialBatch(buffer, 3, colors, {white}, {false}, 0, 1.0);
            } else {
                drawModelBatch(buffer, 3, tileMaterial, {1, 1, 1}, 1.0, 2);
            }
            std::vector<std::uint8_t> pixels(viewportSize * viewportSize * 4);
            glReadPixels(viewportX, viewportY, viewportSize, viewportSize, GL_RGBA,
                         GL_UNSIGNED_BYTE, pixels.data());
            return pixels;
        };
        bool ok = true;
        std::vector<VehicleSpotlight> failureLights(31, behind);
        failureLights.push_back(visible);
        setVehicleSpotlights(failureLights);
        const auto initialMaskImage = image(false);
        resetLightingBufferUploadStats();
        glViewport(viewportX, viewportY, viewportSize / 2, viewportSize / 2);
        const auto normalTileBufferData = pglBufferData;
        successfulBufferData = normalTileBufferData;
        pglBufferData = failAfterBufferUpload;
        bool rejectedFailedMask = false;
        try {
            setVehicleSpotlights(failureLights);
        } catch (const std::runtime_error&) {
            rejectedFailedMask = true;
        }
        pglBufferData = normalTileBufferData;
        glViewport(viewportX, viewportY, viewportSize, viewportSize);
        setVehicleSpotlights(failureLights);
        if (!rejectedFailedMask ||
            lightingBufferUploadStats().vehicleSpotlightTileUploads != 1 ||
            image(false) != initialMaskImage) {
            std::cerr << "spotlight tile-mask upload did not recover after an error\n";
            ok = false;
        }
        for (const std::size_t lightIndex : {std::size_t{31}, std::size_t{32}}) {
            for (const auto& testLight : {visible, nearCrossing}) {
                for (const bool multipleLights : {false, true}) {
                    std::vector<VehicleSpotlight> sparse(lightIndex, behind);
                    if (multipleLights)
                        sparse[0] = testLight;
                    sparse.push_back(testLight);
                    std::vector<VehicleSpotlight> dense = sparse;
                    dense.insert(dense.end(), 128, broad);
                    for (const bool multiMaterial : {false, true}) {
                        setVehicleSpotlights({});
                        const auto unlit = image(multiMaterial);
                        resetLightingBufferUploadStats();
                        setVehicleSpotlights(sparse);
                        const auto tiled = image(multiMaterial);
                        const auto sparseStats = lightingBufferUploadStats();
                        resetLightingBufferUploadStats();
                        setVehicleSpotlights(dense);
                        const auto global = image(multiMaterial);
                        const auto denseStats = lightingBufferUploadStats();
                        bool litBelowBoundary = false;
                        bool litAboveBoundary = false;
                        for (int y = 0; y < viewportSize; ++y) {
                            for (int x = 0; x < viewportSize; ++x) {
                                const std::size_t offset = (y * viewportSize + x) * 4;
                                const bool lit = tiled[offset] != unlit[offset] ||
                                                 tiled[offset + 1] != unlit[offset + 1] ||
                                                 tiled[offset + 2] != unlit[offset + 2];
                                if (lit && y < viewportSize / 2)
                                    litBelowBoundary = true;
                                if (lit && y >= viewportSize / 2)
                                    litAboveBoundary = true;
                            }
                        }
                        if (sparseStats.vehicleSpotlightTileSets != 1 ||
                            sparseStats.vehicleSpotlightTileAssignmentsTotal == 0 ||
                            sparseStats.vehicleSpotlightTileAssignmentsTotal >=
                                (multipleLights ? 32 : 16) ||
                            denseStats.vehicleSpotlightTileFallbackSets != 1 || !litBelowBoundary ||
                            !litAboveBoundary ||
                            !std::equal(tiled.begin(), tiled.end(), global.begin(),
                                        [](std::uint8_t a, std::uint8_t b) {
                                            return std::abs(static_cast<int>(a) -
                                                            static_cast<int>(b)) <= 1;
                                        })) {
                            std::cerr
                                << "spotlight tile image differs from global path for "
                                << (multiMaterial ? "material" : "model") << " shader: tiled sets "
                                << sparseStats.vehicleSpotlightTileSets << ", assignments "
                                << sparseStats.vehicleSpotlightTileAssignmentsTotal
                                << ", fallback sets " << denseStats.vehicleSpotlightTileFallbackSets
                                << ", lit on both sides " << (litBelowBoundary && litAboveBoundary)
                                << ", images match " << (tiled == global) << '\n';
                            ok = false;
                        }
                    }
                }
            }
        }
        if (glGetError() != GL_NO_ERROR) {
            std::cerr << "spotlight tile image probe produced an OpenGL error\n";
            ok = false;
        }
        pglDeleteBuffers(1, &buffer);
        glDeleteTextures(1, &blue);
        glDeleteTextures(1, &white);
        glDeleteTextures(1, &mask);
        shutdownCoreRenderer();
        glfwDestroyWindow(window);
        glfwTerminate();
        return ok ? 0 : 1;
    }
    const auto alphaTestPixel = [&](const ModelMaterial& material, int alphaMode, double alpha,
                                    double visibilityFade) {
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT);
        configureModelAlphaState(alphaMode, false);
        drawModelBatch(buffer, 3, material, {1, 1, 1}, alpha, alphaMode, false, std::nullopt,
                       visibilityFade);
        std::array<std::uint8_t, 4> result = {};
        glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result.data());
        glDisable(GL_POLYGON_OFFSET_FILL);
        glDepthMask(GL_TRUE);
        return result;
    };
    const auto draw = [&](const ModelMaterial& material, std::array<double, 3> color,
                          double alpha) {
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT);
        drawModelBatch(buffer, 3, material, color, alpha, 2);
        std::array<std::uint8_t, 4> pixel = {};
        glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
        return pixel;
    };
    const auto close = [](int actual, int expected) { return std::abs(actual - expected) <= 2; };
    bool passed = true;
    ModelMaterial alphaTestMaterial;
    alphaTestMaterial.emissionColor = {1, 1, 1};
    ModelMaterial transmappedAlphaTest = alphaTestMaterial;
    transmappedAlphaTest.transmap = mask;
    transmappedAlphaTest.useTransmap = true;
    ModelMaterial bumpmappedAlphaTest = alphaTestMaterial;
    bumpmappedAlphaTest.bumpmap = white;
    bumpmappedAlphaTest.useBumpmap = true;
    const auto belowHalfAlpha = alphaTestPixel(alphaTestMaterial, 1, 0.49, 1.0);
    const auto aboveHalfAlpha = alphaTestPixel(alphaTestMaterial, 1, 0.51, 1.0);
    const auto belowOpaqueAlpha = alphaTestPixel(alphaTestMaterial, 3, 0.98, 1.0);
    const auto opaqueAlpha = alphaTestPixel(alphaTestMaterial, 3, 1.0, 1.0);
    const auto transmapCutout = alphaTestPixel(transmappedAlphaTest, 1, 1.0, 1.0);
    const auto transmapOpaqueCutout = alphaTestPixel(transmappedAlphaTest, 3, 1.0, 1.0);
    const auto fadedAtThreshold = alphaTestPixel(alphaTestMaterial, 1, 1.0, 0.5);
    const auto fadedBelowThreshold = alphaTestPixel(alphaTestMaterial, 1, 1.0, 0.49);
    const auto bumpmappedCutout = alphaTestPixel(bumpmappedAlphaTest, 1, 0.49, 1.0);
    if (belowHalfAlpha[3] != 0 || aboveHalfAlpha[3] == 0 || belowOpaqueAlpha[3] != 0 ||
        opaqueAlpha[3] == 0 || transmapCutout[3] == 0 || transmapOpaqueCutout[3] != 0 ||
        fadedAtThreshold[3] == 0 || fadedBelowThreshold[3] != 0 ||
        bumpmappedCutout[3] != 0) {
        std::cerr << "early alpha rejection changed cutout, transmap, fade, or bumpmap behavior\n";
        passed = false;
    }
    StaticPrimitiveBuffer staticOverlay;
    constexpr std::array<float, 39> skinVertices = {
        -0.5F, -0.5F, -2.0F, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
         0.5F, -0.5F, -2.0F, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
         0.0F,  0.5F, -2.0F, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1};
    GLuint skinBuffer = 0;
    pglGenBuffers(1, &skinBuffer);
    std::array<std::array<float, HUMAN_WEIGHT_COUNT>, 3> weights = {};
    for (auto& vertex : weights)
        vertex[0] = 1.0F;
    pglBindBuffer(GL_ARRAY_BUFFER, skinBuffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(weights), weights.data(), GL_STATIC_DRAW);
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(skinVertices), skinVertices.data(), GL_STATIC_DRAW);
    setModelViewMatrix(identityMatrix());
    setLightingViewMatrix(identityMatrix());
    std::array<Matrix4, HUMAN_BONE_COUNT> palette;
    palette.fill(identityMatrix());
    const auto skinFrame = [&](const HumanSkinDraw* skin) {
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT);
        drawModelBatch(buffer, 3, ModelMaterial{}, {1, 1, 1}, 1.0, 0, false, std::nullopt,
                       1.0, skin);
        std::array<std::uint8_t, 32 * 32 * 4> image = {};
        glReadPixels(0, 0, 32, 32, GL_RGBA, GL_UNSIGNED_BYTE, image.data());
        return image;
    };
    const HumanSkinDraw skin{skinBuffer, &palette};
    const auto unskinned = skinFrame(nullptr);
    const auto bindPose = skinFrame(&skin);
    HumanJoints sittingJoints;
    sittingJoints.hip = {0.0, 0.0, -1.5};
    sittingJoints.knee = {0.0, 0.0, -2.0};
    HumanGaitParameters sittingGait;
    sittingGait.upperArmBetaDegrees = 0.0;
    palette = makeHumanBonePalette(sittingJoints, sittingGait, 0.5);
    const auto sitting = skinFrame(&skin);
    palette.fill(identityMatrix());
    palette[0] = translationMatrix({0.7, 0.0, 0.0});
    const auto moved = skinFrame(&skin);
    for (auto& vertex : weights) {
        vertex[0] = 0.5F;
        vertex[1] = 0.5F;
    }
    pglBindBuffer(GL_ARRAY_BUFFER, skinBuffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(weights), weights.data(), GL_STATIC_DRAW);
    const auto blended = skinFrame(&skin);
    for (auto& vertex : weights)
        vertex.fill(0.0F);
    pglBindBuffer(GL_ARRAY_BUFFER, skinBuffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(weights), weights.data(), GL_STATIC_DRAW);
    const auto zeroWeights = skinFrame(&skin);
    const auto restored = skinFrame(nullptr);
    for (auto& vertex : weights)
        vertex[0] = 1.0F;
    pglBindBuffer(GL_ARRAY_BUFFER, skinBuffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(weights), weights.data(), GL_STATIC_DRAW);
    palette[0] = multiplyMatrix4(
        multiplyMatrix4(translationMatrix({0, 0, -2}), rotationMatrix(180, 1, 0, 0)),
        translationMatrix({0, 0, 2}));
    const auto rotated = skinFrame(&skin);
    GLuint skinTestIndexBuffer = 0;
    constexpr std::array<std::uint32_t, 3> skinTestIndices = {0, 1, 2};
    pglGenBuffers(1, &skinTestIndexBuffer);
    pglBindBuffer(GL_ELEMENT_ARRAY_BUFFER, skinTestIndexBuffer);
    pglBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(skinTestIndices), skinTestIndices.data(),
                  GL_STATIC_DRAW);
    glClear(GL_COLOR_BUFFER_BIT);
    drawIndexedModelBatch(buffer, skinTestIndexBuffer, 3, ModelMaterial{}, {1, 1, 1}, 1, 0);
    std::array<std::uint8_t, 32 * 32 * 4> indexed = {};
    glReadPixels(0, 0, 32, 32, GL_RGBA, GL_UNSIGNED_BYTE, indexed.data());
    const auto horizontalCentroid = [](const auto& image) {
        double sum = 0.0;
        int count = 0;
        for (int y = 0; y < 32; ++y) {
            for (int x = 0; x < 32; ++x) {
                if (image[(y * 32 + x) * 4] != 0) {
                    sum += x;
                    ++count;
                }
            }
        }
        return count == 0 ? -1.0 : sum / count;
    };
    const std::size_t centerPixel = (16 * 32 + 16) * 4;
    if (bindPose != unskinned || moved == bindPose || sitting == bindPose ||
        std::none_of(sitting.begin(), sitting.end(), [](std::uint8_t value) { return value != 0; }) ||
        restored != unskinned ||
        indexed != unskinned ||
        zeroWeights != bindPose ||
        horizontalCentroid(blended) <= horizontalCentroid(bindPose) ||
        horizontalCentroid(blended) >= horizontalCentroid(moved) ||
        unskinned[centerPixel] < 215 || rotated[centerPixel] < 145 ||
        rotated[centerPixel] > 150) {
        std::cerr << "Bone matrices did not preserve bind pose, deform vertices and normals, "
                     "or restore unskinned drawing\n";
        passed = false;
    }
    deleteRendererBuffers(1, &skinTestIndexBuffer);
    deleteRendererBuffers(1, &skinBuffer);
    constexpr std::array<float, 39> humanVertices = {
        0.08F, -0.03F, 0.20F, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.14F, -0.03F, 0.20F, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.11F,  0.04F, 0.20F, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(humanVertices), humanVertices.data(), GL_STATIC_DRAW);
    const auto drawHumanGaitFrame = [&](double phaseRadians, double gaitWeight,
                                        const std::array<float, 39>& frameVertices,
                                        double modelHeight, bool gaitEnabled = true,
                                        double viewTranslation = -0.4,
                                        double upperArmBetaDegrees = 66.0,
                                        double armSwing = 1.0,
                                        double hipSway = 1.0,
                                        double waistAnimation = 0.0,
                                        double shoulderXHeightFraction = 0.105,
                                        double shoulderZHeightFraction = 0.76,
                                        double hipXHeightFraction = 0.06,
                                        double hipZHeightFraction = 0.5,
                                        double kneeXHeightFraction = 0.08,
                                        double kneeYHeightFraction = 0.0,
                                        double kneeZHeightFraction = 0.25,
                                        double elbowXHeightFraction = 0.25,
                                        double elbowZHeightFraction = 0.76,
                                        double handXHeightFraction = 0.40,
                                        double handZHeightFraction = 0.76,
                                        double fingerXHeightFraction = 0.5,
                                        double fingerZHeightFraction = 0.76,
                                        double neckYHeightFraction = 0.0,
                                        double neckZHeightFraction = 0.85,
                                        double waistYHeightFraction = 0.0,
                                        double waistZHeightFraction = 0.6) {
        pglBindBuffer(GL_ARRAY_BUFFER, buffer);
        pglBufferData(GL_ARRAY_BUFFER, sizeof(frameVertices), frameVertices.data(), GL_STATIC_DRAW);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT);
        setModelViewMatrix(translationMatrix({0, 0, viewTranslation}));
        ModelMaterial humanMaterial;
        humanMaterial.emissionColor = {1, 1, 1};
        if (gaitEnabled) {
            drawModelBatch(buffer, 3, humanMaterial, {1, 1, 1}, 1.0, 0, false,
                           HumanGaitParameters{phaseRadians, gaitWeight, modelHeight,
                                               upperArmBetaDegrees, armSwing, hipSway,
                                               waistAnimation, shoulderXHeightFraction,
                                               shoulderZHeightFraction, hipXHeightFraction,
                                               hipZHeightFraction, kneeXHeightFraction,
                                               kneeYHeightFraction, kneeZHeightFraction,
                                               elbowXHeightFraction, elbowZHeightFraction,
                                               handXHeightFraction, handZHeightFraction,
                                               fingerXHeightFraction, fingerZHeightFraction,
                                               neckYHeightFraction, neckZHeightFraction,
                                               waistYHeightFraction, waistZHeightFraction});
        } else {
            drawModelBatch(buffer, 3, humanMaterial, {1, 1, 1}, 1.0, 0, false);
        }
        std::array<std::uint8_t, 32 * 32 * 4> image = {};
        glReadPixels(0, 0, 32, 32, GL_RGBA, GL_UNSIGNED_BYTE, image.data());
        return image;
    };
    const auto gaitFrameA = drawHumanGaitFrame(0.0, 1.0, humanVertices, 1.7);
    const auto gaitFrameB = drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, humanVertices, 1.7);
    const bool gaitFrameVisible =
        std::any_of(gaitFrameA.begin(), gaitFrameA.end(),
                    [](std::uint8_t channel) { return channel != 0; }) &&
        std::any_of(gaitFrameB.begin(), gaitFrameB.end(),
                    [](std::uint8_t channel) { return channel != 0; });
    if (!gaitFrameVisible || gaitFrameA == gaitFrameB) {
        std::cerr << "human gait shader did not visibly deform the pedestrian mesh\n";
        passed = false;
    }
    constexpr std::array<float, 39> tPoseArmVertices = {
        0.10F, -0.1F, 1.30F, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.40F, -0.1F, 1.30F, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.25F, 0.1F, 1.30F, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    const auto undeformedArm =
        drawHumanGaitFrame(0.0, 0.0, tPoseArmVertices, 1.7, false, -2.0);
    const auto loweredArm = drawHumanGaitFrame(0.0, 0.0, tPoseArmVertices, 1.7, true, -2.0);
    if (undeformedArm == loweredArm) {
        std::cerr << "stationary pedestrian arms were not lowered from the T-pose\n";
        passed = false;
    }
    const auto noArmSwing =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, tPoseArmVertices, 1.7, true, -2.0,
                           66.0, 0.0);
    const auto configuredArmSwing =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, tPoseArmVertices, 1.7, true, -2.0,
                           66.0, 1.0);
    if (noArmSwing == configuredArmSwing) {
        std::cerr << "authored human arm swing did not deform the T-pose\n";
        passed = false;
    }
    const auto armBetaLow =
        drawHumanGaitFrame(0.0, 0.0, tPoseArmVertices, 1.7, true, -2.0, 30.0, 1.0);
    const auto armBetaHigh =
        drawHumanGaitFrame(0.0, 0.0, tPoseArmVertices, 1.7, true, -2.0, 75.0, 1.0);
    if (armBetaLow == armBetaHigh) {
        std::cerr << "authored upper-arm beta did not change the resting arm pose\n";
        passed = false;
    }
    const auto estimatedShoulder =
        drawHumanGaitFrame(0.0, 0.0, tPoseArmVertices, 1.7, true, -2.0, 66.0, 1.0, 1.0,
                           0.0, 0.105, 0.76);
    const auto authoredShoulder =
        drawHumanGaitFrame(0.0, 0.0, tPoseArmVertices, 1.7, true, -2.0, 66.0, 1.0, 1.0,
                           0.0, 0.10, 0.82);
    if (estimatedShoulder == authoredShoulder) {
        std::cerr << "authored human shoulder link did not change the arm pivot\n";
        passed = false;
    }
    constexpr std::array<float, 39> torsoVertices = {
        0.10F, -0.1F, 1.00F, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.40F, -0.1F, 1.00F, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.25F, 0.1F, 1.00F, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    const auto noHipSway =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, torsoVertices, 1.7, true, -2.0, 66.0,
                           1.0, 0.0);
    const auto configuredHipSway =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, torsoVertices, 1.7, true, -2.0, 66.0,
                           1.0, 10.0);
    if (noHipSway == configuredHipSway) {
        std::cerr << "authored human hip sway did not move the torso\n";
        passed = false;
    }
    const auto estimatedHip =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, torsoVertices, 1.7, true, -2.0, 66.0,
                           1.0, 10.0, 0.0, 0.105, 0.76, 0.06, 0.5);
    const auto authoredHip =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, torsoVertices, 1.7, true, -2.0, 66.0,
                           1.0, 10.0, 0.0, 0.105, 0.76, 0.10, 0.75);
    if (estimatedHip == authoredHip) {
        std::cerr << "authored human hip link did not change the hip sway pivot\n";
        passed = false;
    }
    const auto noWaistAnimation =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, torsoVertices, 1.7, true, -2.0,
                           66.0, 1.0, 1.0, 0.0);
    const auto configuredWaistAnimation =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, torsoVertices, 1.7, true, -2.0,
                           66.0, 1.0, 1.0, 10.0);
    if (noWaistAnimation == configuredWaistAnimation) {
        std::cerr << "authored human waist animation did not affect torso gait\n";
        passed = false;
    }
    constexpr std::array<float, 39> lowerLegVertices = {
        0.10F, 0.02F, 0.18F, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.19F, 0.02F, 0.18F, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.14F, 0.08F, 0.38F, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    const auto estimatedKnee =
        drawHumanGaitFrame(0.0, 1.0, lowerLegVertices, 1.7, true, -2.0, 66.0, 1.0, 1.0,
                           0.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0, 0.25);
    const auto authoredKnee =
        drawHumanGaitFrame(0.0, 1.0, lowerLegVertices, 1.7, true, -2.0, 66.0, 1.0, 1.0,
                           0.0, 0.105, 0.76, 0.06, 0.5, 0.12, -0.02, 0.30);
    if (estimatedKnee == authoredKnee) {
        std::cerr << "authored human knee links did not change the lower-leg gait pivot\n";
        passed = false;
    }
    constexpr std::array<float, 39> forearmVertices = {
        0.70F, -0.08F, 1.10F, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        1.10F, -0.08F, 1.10F, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.90F, 0.02F, 1.30F, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    const auto estimatedElbow =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, forearmVertices, 1.7, true, -2.0,
                           66.0, 1.0, 1.0, 0.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0,
                           0.25, 0.08, 0.45);
    const auto authoredElbow =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, forearmVertices, 1.7, true, -2.0,
                           66.0, 1.0, 1.0, 0.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0,
                           0.25, 0.40, 0.75);
    if (estimatedElbow == authoredElbow) {
        std::cerr << "authored human elbow links did not change the forearm gait pivot\n";
        passed = false;
    }
    constexpr std::array<float, 39> handVertices = {
        0.70F, -0.08F, 1.10F, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        1.10F, -0.08F, 1.10F, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.90F, 0.02F, 1.30F, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    const auto estimatedHand =
        drawHumanGaitFrame(std::numbers::pi / 4.0, 1.0, handVertices, 1.7, true, -2.0,
                           0.0, 1.0, 1.0, 0.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0,
                           0.25, 0.25, 0.76, 0.20, 0.45);
    const auto authoredHand =
        drawHumanGaitFrame(std::numbers::pi / 4.0, 1.0, handVertices, 1.7, true, -2.0,
                           0.0, 1.0, 1.0, 0.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0,
                           0.25, 0.25, 0.76, 0.50, 0.75);
    if (estimatedHand == authoredHand) {
        std::cerr << "authored human hand links did not change the wrist pivot\n";
        passed = false;
    }
    const auto estimatedFinger =
        drawHumanGaitFrame(std::numbers::pi / 4.0, 1.0, handVertices, 1.7, true, -2.0,
                           0.0, 1.0, 1.0, 0.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0,
                           0.25, 0.25, 0.76, 0.40, 0.76, 0.45, 0.76);
    const auto authoredFinger =
        drawHumanGaitFrame(std::numbers::pi / 4.0, 1.0, handVertices, 1.7, true, -2.0,
                           0.0, 1.0, 1.0, 0.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0,
                           0.25, 0.25, 0.76, 0.40, 0.76, 0.65, 0.82);
    if (estimatedFinger == authoredFinger) {
        std::cerr << "authored human finger links did not change the fingertip gait pivot\n";
        passed = false;
    }
    constexpr std::array<float, 39> headVertices = {
        0.03F, -0.10F, 1.60F, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.16F, -0.10F, 1.60F, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        0.095F, 0.03F, 1.70F, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    const auto estimatedNeck =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, headVertices, 1.75, true, -2.0,
                           66.0, 1.0, 1.0, 0.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0,
                           0.25, 0.25, 0.76, 0.40, 0.76, 0.5, 0.76, 0.0, 0.85);
    const auto authoredNeck =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, headVertices, 1.75, true, -2.0,
                           66.0, 1.0, 1.0, 0.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0,
                           0.25, 0.25, 0.76, 0.40, 0.76, 0.5, 0.76, -0.08, 0.91);
    if (estimatedNeck == authoredNeck) {
        std::cerr << "authored human neck links did not change the head gait pivot\n";
        passed = false;
    }
    const auto estimatedWaist =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, torsoVertices, 1.7, true, -2.0,
                           66.0, 1.0, 1.0, 10.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0,
                           0.25, 0.25, 0.76, 0.40, 0.76, 0.5, 0.76, 0.0, 0.85, 0.0,
                           0.55);
    const auto authoredWaist =
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, torsoVertices, 1.7, true, -2.0,
                           66.0, 1.0, 1.0, 10.0, 0.105, 0.76, 0.06, 0.5, 0.08, 0.0,
                           0.25, 0.25, 0.76, 0.40, 0.76, 0.5, 0.76, 0.0, 0.85, -0.02,
                           0.72);
    if (estimatedWaist == authoredWaist) {
        std::cerr << "authored human waist links did not change the torso pivot\n";
        passed = false;
    }
    const auto nonHumanFrameAfterGait =
        drawHumanGaitFrame(0.0, 0.0, tPoseArmVertices, 1.7, false, -2.0);
    if (undeformedArm != nonHumanFrameAfterGait) {
        std::cerr << "pedestrian gait state leaked into the following non-human draw\n";
        passed = false;
    }
    setModelViewMatrix(identityMatrix());
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);
    ModelMaterial material;
    {
        ModelMaterial scenery;
        scenery.textured = true;
        scenery.texture = white;
        scenery.whiteAmbient = true;
        setLightingViewMatrix(identityMatrix(), std::array<double, 3>{0.0, 0.0, 1.0});
        setWeatherLighting(1.0);
        const auto shaded = draw(scenery, {0.04, 0.25, 0.0}, 1.0);
        scenery.texture = blue;
        const auto textured = draw(scenery, {0.04, 0.25, 0.0}, 1.0);
        scenery.texture = white;
        scenery.whiteAmbient = false;
        const auto legacy = draw(scenery, {0.04, 0.25, 0.0}, 1.0);
        scenery.whiteAmbient = true;
        setLightingViewMatrix(identityMatrix(), std::array<double, 3>{0.0, 0.0, -1.0});
        const auto lit = draw(scenery, {0.04, 0.25, 0.0}, 1.0);
        setWeatherLighting(0.0);
        const auto night = draw(scenery, {0.04, 0.25, 0.0}, 1.0);
        if (!close(shaded[0], 148) || !close(shaded[1], 148) ||
            !close(shaded[2], 148) || !close(textured[0], 38) ||
            !close(textured[1], 49) || !close(textured[2], 65) || !close(legacy[0], 6) ||
            !close(legacy[1], 37) || legacy[2] != 0 ||
            !close(lit[0], 152) || !close(lit[1], 175) || !close(lit[2], 148) ||
            night[0] != 0 || night[1] != 0 || night[2] != 0) {
            std::cerr << "scenery ambient and diffuse material lighting mismatch\n";
            passed = false;
        }
        setWeatherLighting(std::nullopt);
        setLightingViewMatrix(identityMatrix());
    }
    const auto setTriangleDepth = [&](float depth) {
        auto depthVertices = vertices;
        for (std::size_t index = 2; index < depthVertices.size(); index += 13) {
            depthVertices[index] = depth;
        }
        pglBindBuffer(GL_ARRAY_BUFFER, buffer);
        pglBufferData(GL_ARRAY_BUFFER, sizeof(depthVertices), depthVertices.data(), GL_STATIC_DRAW);
    };
    const auto drawOverlappingAlphaSurfaces = [&](bool noZwrite) {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ModelMaterial overlapMaterial;
        overlapMaterial.emissionColor = {1, 1, 1};
        setTriangleDepth(-1.0F);
        configureModelAlphaState(2, noZwrite);
        drawModelBatch(buffer, 3, overlapMaterial, {1, 0, 0}, 0.5, 2);
        setTriangleDepth(-2.0F);
        configureModelAlphaState(2, noZwrite);
        drawModelBatch(buffer, 3, overlapMaterial, {0, 0, 1}, 0.5, 2);
        std::array<std::uint8_t, 4> overlapPixel = {};
        glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, overlapPixel.data());
        return overlapPixel;
    };
    const auto alphaDepthPixel = drawOverlappingAlphaSurfaces(false);
    if (alphaDepthPixel[0] < 100 || alphaDepthPixel[2] > 4) {
        std::cerr << "alpha mode 2 did not depth-occlude a later overlapping surface\n";
        passed = false;
    }
    const auto noDepthWritePixel = drawOverlappingAlphaSurfaces(true);
    if (noDepthWritePixel[2] <= noDepthWritePixel[0]) {
        std::cerr << "matl_noZwrite did not preserve blended overlap behavior\n";
        passed = false;
    }
    const auto drawOverlappingTransmappedSurfaces = [&](bool noZwrite) {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ModelMaterial transmappedMaterial;
        transmappedMaterial.emissionColor = {1, 1, 1};
        transmappedMaterial.transmap = mask;
        transmappedMaterial.useTransmap = true;
        setTriangleDepth(-1.0F);
        configureModelAlphaState(2, noZwrite);
        drawModelBatch(buffer, 3, transmappedMaterial, {1, 0, 0}, 0.5, 2);
        setTriangleDepth(-2.0F);
        configureModelAlphaState(2, noZwrite);
        drawModelBatch(buffer, 3, transmappedMaterial, {0, 0, 1}, 0.5, 2);
        std::array<std::uint8_t, 4> overlapPixel = {};
        glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, overlapPixel.data());
        return overlapPixel;
    };
    const auto transmapDepthPixel = drawOverlappingTransmappedSurfaces(false);
    if (transmapDepthPixel[0] <= 20 || transmapDepthPixel[2] > 4) {
        std::cerr << "transmapped alpha mode 2 did not depth-occlude a later surface\n";
        passed = false;
    }
    const auto transmapNoDepthWritePixel = drawOverlappingTransmappedSurfaces(true);
    if (transmapNoDepthWritePixel[2] <= transmapNoDepthWritePixel[0]) {
        std::cerr << "transmapped matl_noZwrite did not preserve blended overlap behavior\n";
        passed = false;
    }
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    const auto sampleAddressMode = [&](float u, float v, GLenum wrapS, GLenum wrapT) {
        auto addressVertices = vertices;
        for (std::size_t index = 0; index < addressVertices.size(); index += 13) {
            addressVertices[index + 3] = u;
            addressVertices[index + 4] = v;
        }
        pglBindBuffer(GL_ARRAY_BUFFER, buffer);
        pglBufferData(GL_ARRAY_BUFFER, sizeof(addressVertices), addressVertices.data(),
                      GL_STATIC_DRAW);
        ModelMaterial addressMaterial;
        addressMaterial.texture = white;
        addressMaterial.textured = true;
        addressMaterial.textureWrapS = wrapS;
        addressMaterial.textureWrapT = wrapT;
        addressMaterial.emissionColor = {1, 1, 1};
        return draw(addressMaterial, {1, 1, 1}, 1.0);
    };
    const auto borderInside =
        sampleAddressMode(0.5F, 0.5F, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER);
    const auto borderOutsideS =
        sampleAddressMode(1.5F, 0.5F, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER);
    const auto borderOutsideT =
        sampleAddressMode(0.5F, -0.5F, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER);
    const auto repeatAfterBorder = sampleAddressMode(1.5F, -0.5F, GL_REPEAT, GL_REPEAT);
    const auto clampAfterRepeat =
        sampleAddressMode(1.5F, -0.5F, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE);
    if (borderInside[0] < 250 || borderOutsideS[0] > 2 || borderOutsideT[0] > 2 ||
        repeatAfterBorder[0] < 250 || clampAfterRepeat[0] < 250) {
        std::cerr << "texture border addressing or shared-texture wrap transition failed\n";
        passed = false;
    }
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);

    resetFrameTriangleCount();
    resetFrameReflectionTriangleCount();
    const ModelMaterial counterMaterial;
    drawModelBatch(buffer, 3, counterMaterial, {1, 1, 1}, 1.0, 0);
    setFrameTriangleCountReflectionPass(true);
    drawModelBatch(buffer, 3, counterMaterial, {1, 1, 1}, 1.0, 0);
    setFrameTriangleCountReflectionPass(false);
    if (frameTriangleCount() != 1 || frameReflectionTriangleCount() != 1) {
        std::cerr << "reflection-pass triangles were not counted separately\n";
        passed = false;
    }

    material = {};
    setTriangleDepth(-1.0F);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawModelBatch(buffer, 3, material, {1, 1, 1}, 1.0, 2, false, std::nullopt, 0.5);
    std::array<std::uint8_t, 4> fadePixel = {};
    glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, fadePixel.data());
    if (!close(fadePixel[3], 128)) {
        std::cerr << "model visibility fade did not scale fragment opacity\n";
        passed = false;
    }
    const std::vector<std::array<float, 4>> materialColors = {{1, 1, 1, 1}};
    drawMaterialBatch(buffer, 3, materialColors, {white}, {false}, 0, 0.5);
    fadePixel = {};
    glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, fadePixel.data());
    if (!close(fadePixel[3], 128)) {
        std::cerr << "multi-material visibility fade did not scale fragment opacity\n";
        passed = false;
    }
    setWeatherLighting(0.0);
    setWeatherFog(std::nullopt);
    const VehicleSpotlight testSpotlight{{0, 0, -2}, {0, 0, 1}, {1, 0.5, 0.25},
                                         10, 30, 70};
    VehicleSpotlight underflowSpotlight = testSpotlight;
    underflowSpotlight.range = std::numeric_limits<float>::min() / 2.0;
    bool rejectedUnderflowSpotlight = false;
    try {
        setVehicleSpotlights({underflowSpotlight});
    } catch (const std::invalid_argument&) {
        rejectedUnderflowSpotlight = true;
    }
    if (!rejectedUnderflowSpotlight) {
        std::cerr << "vehicle spotlight accepted a range that can underflow on the GPU\n";
        passed = false;
    }
    VehicleSpotlight minimumSpotlight = testSpotlight;
    minimumSpotlight.range = std::numeric_limits<float>::min();
    setVehicleSpotlights({minimumSpotlight});
    const auto spotlightPixel = [&](bool multiMaterial) {
        glClear(GL_COLOR_BUFFER_BIT);
        if (multiMaterial) {
            drawMaterialBatch(buffer, 3, materialColors, {white}, {false}, 0, 0.5);
        } else {
            drawModelBatch(buffer, 3, material, {1, 1, 1}, 0.5, 2);
        }
        std::array<std::uint8_t, 4> result = {};
        glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result.data());
        return result;
    };
    setVehicleSpotlights({testSpotlight});
    const auto cachedVehicleLit = spotlightPixel(false);
    resetLightingBufferUploadStats();
    setVehicleSpotlights({testSpotlight});
    setVehicleSpotlights({});
    const auto cachedVehicleOff = spotlightPixel(false);
    setVehicleSpotlights({testSpotlight});
    const auto cachedVehicleAgain = spotlightPixel(false);
    auto uploadStats = lightingBufferUploadStats();
    if (uploadStats.vehicleUploads != 0 || uploadStats.vehicleBytes != 0 ||
        cachedVehicleAgain != cachedVehicleLit || cachedVehicleOff == cachedVehicleLit) {
        std::cerr << "unchanged vehicle light buffer was not reused correctly\n";
        passed = false;
    }
    VehicleSpotlight changedVehicleLight = testSpotlight;
    changedVehicleLight.direction = {0, 0, -1};
    setVehicleSpotlights({changedVehicleLight});
    const auto changedVehiclePixel = spotlightPixel(false);
    setVehicleSpotlights({testSpotlight});
    uploadStats = lightingBufferUploadStats();
    if (uploadStats.vehicleUploads != 2 ||
        uploadStats.vehicleBytes != 2 * 48 ||
        changedVehiclePixel != cachedVehicleOff || spotlightPixel(false) != cachedVehicleLit) {
        std::cerr << "changed vehicle light buffer was not uploaded correctly\n";
        passed = false;
    }
    resetLightingBufferUploadStats();
    const auto normalVehicleBufferData = pglBufferData;
    successfulBufferData = normalVehicleBufferData;
    pglBufferData = failAfterBufferUpload;
    bool rejectedFailedVehicleUpload = false;
    try {
        setVehicleSpotlights({changedVehicleLight});
    } catch (const std::runtime_error&) {
        rejectedFailedVehicleUpload = true;
    }
    pglBufferData = normalVehicleBufferData;
    setVehicleSpotlights({testSpotlight});
    if (!rejectedFailedVehicleUpload || lightingBufferUploadStats().vehicleUploads != 1 ||
        spotlightPixel(false) != cachedVehicleLit) {
        std::cerr << "vehicle light upload did not recover after an error\n";
        passed = false;
    }
    for (const bool multiMaterial : {false, true}) {
        setVehicleSpotlights({});
        const auto unlit = spotlightPixel(multiMaterial);
        setVehicleSpotlights({testSpotlight});
        const auto illuminated = spotlightPixel(multiMaterial);
        std::vector<VehicleSpotlight> fullSpotlightBudget(250, testSpotlight);
        for (auto& light : fullSpotlightBudget) {
            light.color = {0, 0, 0};
        }
        fullSpotlightBudget.back() = testSpotlight;
        setVehicleSpotlights(fullSpotlightBudget);
        const auto lastSpotlightSlot = spotlightPixel(multiMaterial);
        if (lightingBufferUploadStats().vehicleLightsMax < 250) {
            std::cerr << "vehicle spotlight count did not include the 250th light\n";
            passed = false;
        }
        VehicleSpotlight coneEdge = testSpotlight;
        coneEdge.direction = {0.5, 0, 0.866025403784};
        setVehicleSpotlights({coneEdge});
        const auto softenedCone = spotlightPixel(multiMaterial);
        VehicleSpotlight nearerRange = testSpotlight;
        nearerRange.range = 2.0;
        setVehicleSpotlights({nearerRange});
        const auto attenuated = spotlightPixel(multiMaterial);
        VehicleSpotlight away = testSpotlight;
        away.direction = {0, 0, -1};
        setVehicleSpotlights({away});
        const auto outsideCone = spotlightPixel(multiMaterial);
        VehicleSpotlight hardCone = testSpotlight;
        hardCone.innerAngleDegrees = hardCone.outerAngleDegrees;
        setVehicleSpotlights({hardCone});
        const auto insideHardCone = spotlightPixel(multiMaterial);
        hardCone.direction = {0, 0, -1};
        setVehicleSpotlights({hardCone});
        const auto outsideHardCone = spotlightPixel(multiMaterial);
        VehicleSpotlight shortRange = testSpotlight;
        shortRange.range = 0.1;
        setVehicleSpotlights({shortRange});
        const auto outsideRange = spotlightPixel(multiMaterial);
        VehicleSpotlight hugeRange = testSpotlight;
        hugeRange.range = 1.0e30;
        setVehicleSpotlights({hugeRange});
        const auto hugeRangePixel = spotlightPixel(multiMaterial);
        const double distance = std::sqrt(1.0 + 2.0 / (32.0 * 32.0));
        const double relativeDistance = distance / testSpotlight.range;
        const int expectedRedIncrease = static_cast<int>(std::lround(
            255.0 * (0.1 + 0.3 / distance) /
            (0.2 + relativeDistance + 40.0 * relativeDistance * relativeDistance)));
        if (illuminated[0] <= unlit[0] + 30 || illuminated[1] <= unlit[1] + 10 ||
            illuminated[0] <= illuminated[1] || illuminated[1] <= illuminated[2] ||
            std::abs((illuminated[0] - unlit[0]) - expectedRedIncrease) > 3 ||
            std::abs(static_cast<int>(lastSpotlightSlot[0]) - illuminated[0]) > 1 ||
            std::abs(static_cast<int>(lastSpotlightSlot[1]) - illuminated[1]) > 1 ||
            std::abs(static_cast<int>(lastSpotlightSlot[2]) - illuminated[2]) > 1 ||
            softenedCone[0] <= unlit[0] + 2 || softenedCone[0] >= illuminated[0] - 5 ||
            attenuated[0] <= unlit[0] + 2 || attenuated[0] >= illuminated[0] - 5 ||
            insideHardCone != illuminated || outsideHardCone != unlit ||
            outsideCone != unlit || outsideRange != unlit ||
            hugeRangePixel[0] <= unlit[0] + 30 || hugeRangePixel[3] != unlit[3] ||
            illuminated[3] != unlit[3]) {
            std::cerr << "vehicle spotlight illumination, cone, range, or alpha failed for "
                      << (multiMaterial ? "material" : "model") << " shader\n";
            passed = false;
        }
    }
    setVehicleSpotlights({});
    const MapPointLight testMapLight{{0, 0, -2}, {1, 0.5, 0.25}, 2.0};
    const MapPointLight attenuatedMapLight{{0, 0, -2}, {1, 0.5, 0.25}, 0.25};
    MapPointLight underflowMapLight = testMapLight;
    underflowMapLight.radius = std::numeric_limits<float>::min() / 2.0;
    bool rejectedUnderflowMapLight = false;
    try {
        setMapLights({underflowMapLight}, 1.0);
    } catch (const std::invalid_argument&) {
        rejectedUnderflowMapLight = true;
    }
    StaticMapLightBuffer underflowCache;
    bool rejectedUnderflowStaticLight = false;
    try {
        setStaticMapLights(underflowCache, {underflowMapLight}, {0},
                           {{{0, 0, -2}, 0}}, 1.0);
    } catch (const std::invalid_argument&) {
        rejectedUnderflowStaticLight = true;
    }
    releaseStaticMapLights(underflowCache);
    if (!rejectedUnderflowMapLight || !rejectedUnderflowStaticLight) {
        std::cerr << "map light accepted a radius that can underflow on the GPU\n";
        passed = false;
    }
    MapPointLight minimumMapLight = testMapLight;
    minimumMapLight.radius = std::numeric_limits<float>::min();
    setMapLights({minimumMapLight}, 1.0);
    setStaticMapLights(underflowCache, {minimumMapLight}, {0},
                       {{{0, 0, -2}, 0}}, 1.0);
    releaseStaticMapLights(underflowCache);
    const auto textMaterialPixel = [&] {
        glClear(GL_COLOR_BUFFER_BIT);
        drawMaterialBatch(buffer, 3, materialColors, {white}, {false}, 0, 0.5, white, true);
        std::array<std::uint8_t, 4> pixel = {};
        glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
        return pixel;
    };
    setMapLights({}, 0.0);
    const auto unlitTextMaterial = textMaterialPixel();
    setMapLights({testMapLight}, 1.0);
    const auto litTextMaterial = textMaterialPixel();
    if (litTextMaterial[0] <= unlitTextMaterial[0] + 10) {
        std::cerr << "scenery light did not illuminate material text\n";
        passed = false;
    }
    setMapLights({testMapLight}, 1.0);
    const auto cachedMapLit = spotlightPixel(false);
    resetLightingBufferUploadStats();
    setMapLights({testMapLight}, 1.0);
    setMapLights({testMapLight}, 0.0);
    const auto cachedMapOff = spotlightPixel(false);
    setMapLights({testMapLight}, 1.0);
    const auto cachedMapAgain = spotlightPixel(false);
    uploadStats = lightingBufferUploadStats();
    if (uploadStats.mapUploads != 0 || uploadStats.mapBytes != 0 ||
        cachedMapAgain != cachedMapLit || cachedMapOff == cachedMapLit) {
        std::cerr << "unchanged map light buffer was not reused correctly\n";
        passed = false;
    }
    MapPointLight changedMapView = testMapLight;
    changedMapView.position = {2, 0, -2};
    setMapLights({changedMapView}, 1.0);
    const auto changedMapPixel = spotlightPixel(false);
    setMapLights({testMapLight}, 1.0);
    uploadStats = lightingBufferUploadStats();
    if (uploadStats.mapUploads != 2 || uploadStats.mapBytes != 2 * 32 ||
        changedMapPixel == cachedMapLit || spotlightPixel(false) != cachedMapLit) {
        std::cerr << "changed map light buffer was not uploaded correctly\n";
        passed = false;
    }
    resetLightingBufferUploadStats();
    const auto normalMapBufferData = pglBufferData;
    successfulBufferData = normalMapBufferData;
    pglBufferData = failAfterBufferUpload;
    bool rejectedFailedMapUpload = false;
    try {
        setMapLights({changedMapView}, 1.0);
    } catch (const std::runtime_error&) {
        rejectedFailedMapUpload = true;
    }
    pglBufferData = normalMapBufferData;
    setMapLights({testMapLight}, 1.0);
    if (!rejectedFailedMapUpload || lightingBufferUploadStats().mapUploads != 1 ||
        spotlightPixel(false) != cachedMapLit) {
        std::cerr << "map light upload did not recover after an error\n";
        passed = false;
    }
    StaticMapLightBuffer staticCache;
    std::vector<MapPointLight> staticSources = {testMapLight, attenuatedMapLight};
    std::vector<std::size_t> staticIndices = {0, 1};
    std::vector<StaticMapLightView> staticViews = {{{0, 0, -2}, 0}};
    resetLightingBufferUploadStats();
    StaticMapLightBuffer failedUploadCache;
    const auto normalBufferData = pglBufferData;
    pglBufferData = failBufferUpload;
    bool rejectedFailedUpload = false;
    try {
        setStaticMapLights(failedUploadCache, staticSources, staticIndices, staticViews, 1.0);
    } catch (const std::runtime_error&) {
        rejectedFailedUpload = true;
    }
    pglBufferData = normalBufferData;
    if (!rejectedFailedUpload || failedUploadCache.sourceCount != 0 ||
        lightingBufferUploadStats().staticMapUploads != 0) {
        std::cerr << "failed static light upload was treated as successful\n";
        passed = false;
    }
    const auto normalTexBuffer = pglTexBuffer;
    pglTexBuffer = failTextureBufferBinding;
    bool rejectedFailedBinding = false;
    try {
        setStaticMapLights(failedUploadCache, staticSources, staticIndices, staticViews, 1.0);
    } catch (const std::runtime_error&) {
        rejectedFailedBinding = true;
    }
    pglTexBuffer = normalTexBuffer;
    if (!rejectedFailedBinding || failedUploadCache.sourceCount != 0 ||
        lightingBufferUploadStats().staticMapUploads != 0) {
        std::cerr << "failed static light texture binding was treated as successful\n";
        passed = false;
    }
    setStaticMapLights(failedUploadCache, staticSources, staticIndices, staticViews, 1.0);
    if (failedUploadCache.sourceCount != 2 || spotlightPixel(false) != cachedMapLit ||
        lightingBufferUploadStats().staticMapUploads != 1) {
        std::cerr << "static light upload did not recover after an error\n";
        passed = false;
    }
    pglBufferData = failBufferUpload;
    bool rejectedFailedReplacement = false;
    try {
        setStaticMapLights(failedUploadCache, staticSources, staticIndices, staticViews, 1.0, 1);
    } catch (const std::runtime_error&) {
        rejectedFailedReplacement = true;
    }
    pglBufferData = normalBufferData;
    setStaticMapLights(failedUploadCache, staticSources, staticIndices, staticViews, 1.0);
    if (!rejectedFailedReplacement || lightingBufferUploadStats().staticMapUploads != 2 ||
        spotlightPixel(false) != cachedMapLit) {
        std::cerr << "static source cache was not invalidated after a failed replacement\n";
        passed = false;
    }
    pglTexBuffer = failTextureBufferBinding;
    bool rejectedFailedReplacementBinding = false;
    try {
        setStaticMapLights(failedUploadCache, staticSources, staticIndices, staticViews, 1.0, 1);
    } catch (const std::runtime_error&) {
        rejectedFailedReplacementBinding = true;
    }
    pglTexBuffer = normalTexBuffer;
    setStaticMapLights(failedUploadCache, staticSources, staticIndices, staticViews, 1.0);
    if (!rejectedFailedReplacementBinding || lightingBufferUploadStats().staticMapUploads != 3 ||
        spotlightPixel(false) != cachedMapLit) {
        std::cerr << "static source cache was not invalidated after a failed binding\n";
        passed = false;
    }
    staticViews[0].position = changedMapView.position;
    successfulBufferData = normalBufferData;
    pglBufferData = failAfterBufferUpload;
    bool rejectedFailedReplacementView = false;
    try {
        setStaticMapLights(failedUploadCache, staticSources, staticIndices, staticViews, 1.0);
    } catch (const std::runtime_error&) {
        rejectedFailedReplacementView = true;
    }
    pglBufferData = normalBufferData;
    staticViews[0].position = testMapLight.position;
    const auto uploadsBeforeViewRetry = lightingBufferUploadStats().mapUploads;
    setStaticMapLights(failedUploadCache, staticSources, staticIndices, staticViews, 1.0);
    if (!rejectedFailedReplacementView ||
        lightingBufferUploadStats().mapUploads != uploadsBeforeViewRetry + 1 ||
        spotlightPixel(false) != cachedMapLit) {
        std::cerr << "static view cache was not invalidated after a failed replacement\n";
        passed = false;
    }
    releaseStaticMapLights(failedUploadCache);
    resetLightingBufferUploadStats();
    setStaticMapLights(staticCache, staticSources, staticIndices, staticViews, 1.0);
    if (spotlightPixel(false) != cachedMapLit ||
        lightingBufferUploadStats().staticMapUploads != 1 ||
        lightingBufferUploadStats().staticMapBytes != 32 ||
        lightingBufferUploadStats().mapBytes != 0) {
        std::cerr << "static map-light registration or shading failed\n";
        passed = false;
    }
    staticViews[0].position = changedMapView.position;
    pglBufferData = failBufferUpload;
    bool rejectedFailedView = false;
    try {
        setStaticMapLights(staticCache, staticSources, staticIndices, staticViews, 1.0);
    } catch (const std::runtime_error&) {
        rejectedFailedView = true;
    }
    pglBufferData = normalBufferData;
    if (!rejectedFailedView || lightingBufferUploadStats().mapBytes != 0) {
        std::cerr << "failed static light view upload was treated as successful\n";
        passed = false;
    }
    setStaticMapLights(staticCache, staticSources, staticIndices, staticViews, 1.0);
    if (spotlightPixel(false) != changedMapPixel ||
        lightingBufferUploadStats().staticMapUploads != 1 ||
        lightingBufferUploadStats().mapBytes != 16) {
        std::cerr << "static map-light source was reuploaded or view position was stale\n";
        passed = false;
    }
    staticIndices = {1};
    staticViews[0] = {{0, 0, -2}, 0};
    setMapLights({attenuatedMapLight}, 1.0);
    const auto expectedStaticTile = spotlightPixel(false);
    setStaticMapLights(staticCache, staticSources, staticIndices, staticViews, 1.0);
    if (spotlightPixel(false) != expectedStaticTile ||
        lightingBufferUploadStats().staticMapUploads != 1) {
        std::cerr << "static map-light tile change failed\n";
        passed = false;
    }
    setMapLights({attenuatedMapLight}, 1.0);
    const auto expectedMaterialTile = spotlightPixel(true);
    setStaticMapLights(staticCache, staticSources, staticIndices, staticViews, 1.0);
    if (spotlightPixel(true) != expectedMaterialTile ||
        lightingBufferUploadStats().staticMapUploads != 1) {
        std::cerr << "static map-light tile change failed for material shader\n";
        passed = false;
    }
    staticIndices = {1, 0};
    staticViews[0].sourceIndex = 1;
    setStaticMapLights(staticCache, staticSources, staticIndices, staticViews, 1.0);
    if (spotlightPixel(false) != cachedMapLit ||
        lightingBufferUploadStats().staticMapUploads != 1) {
        std::cerr << "static map-light reordered source selection failed\n";
        passed = false;
    }
    staticSources.push_back(testMapLight);
    staticIndices = {2};
    staticViews[0].sourceIndex = 0;
    setStaticMapLights(staticCache, staticSources, staticIndices, staticViews, 1.0);
    if (spotlightPixel(false) != cachedMapLit ||
        lightingBufferUploadStats().staticMapUploads != 2) {
        std::cerr << "static map-light source growth failed\n";
        passed = false;
    }
    staticIndices = {1};
    staticSources[1].color = {0, 0.5, 0.25};
    setMapLights({staticSources[1]}, 1.0);
    const auto expectedStaticRevision = spotlightPixel(false);
    setStaticMapLights(staticCache, staticSources, staticIndices, staticViews, 1.0, 1);
    if (spotlightPixel(false) != expectedStaticRevision ||
        lightingBufferUploadStats().staticMapUploads != 3) {
        std::cerr << "static map-light revision change failed\n";
        passed = false;
    }
    bool rejectedInvalidSource = false;
    try {
        setStaticMapLights(staticCache, staticSources, {staticSources.size()}, staticViews, 1.0, 2);
    } catch (const std::invalid_argument&) {
        rejectedInvalidSource = true;
    }
    if (!rejectedInvalidSource || spotlightPixel(false) != expectedStaticRevision) {
        std::cerr << "invalid static map-light source changed renderer state\n";
        passed = false;
    }
    StaticMapLightBuffer invalidInactiveCache;
    std::vector<MapPointLight> partlyInvalidSources = {testMapLight, testMapLight};
    partlyInvalidSources[1].radius = 0;
    setStaticMapLights(invalidInactiveCache, partlyInvalidSources, {0},
                       {{{0, 0, -2}, 0}}, 1.0);
    if (spotlightPixel(false) != cachedMapLit || invalidInactiveCache.residentSources) {
        std::cerr << "invalid inactive static source changed active lighting\n";
        passed = false;
    }
    rejectedInvalidSource = false;
    try {
        setStaticMapLights(invalidInactiveCache, partlyInvalidSources, {1},
                           {{{0, 0, -2}, 0}}, 1.0);
    } catch (const std::invalid_argument&) {
        rejectedInvalidSource = true;
    }
    if (!rejectedInvalidSource || spotlightPixel(false) != cachedMapLit) {
        std::cerr << "invalid active static source was not rejected safely\n";
        passed = false;
    }
    releaseStaticMapLights(invalidInactiveCache);
    if (detail::mapLightTextureCapacity(5, 1) != 5 ||
        detail::mapLightTextureCapacity(5, 2) != 2 ||
        detail::mapLightTextureCapacity(0, 1) != 0) {
        std::cerr << "map-light capacities must account for each buffer's texels per light\n";
        passed = false;
    }
    const auto compactFallback = detail::compactStaticMapLightSelection(
        staticSources, {1, 0}, staticViews, 1, 1);
    if (!compactFallback || compactFallback->sourceIndices != std::vector<std::size_t>{1} ||
        compactFallback->viewLights.size() != 1 || compactFallback->viewLights[0].sourceIndex != 0) {
        std::cerr << "static map-light capacity fallback was not selected\n";
        passed = false;
    } else {
        setStaticMapLights(staticCache, staticSources, compactFallback->sourceIndices,
                           compactFallback->viewLights, 1.0);
        if (spotlightPixel(false) != expectedStaticRevision) {
            std::cerr << "static map-light capacity fallback changed the rendered light\n";
            passed = false;
        }
    }
    releaseStaticMapLights(staticCache);
    {
        map_detail::StaticLightTileIndex crossingIndex;
        std::vector<MapPointLight> crossingSources = {{{299, 150, 0}, {1, 0.5, 0.25}, 5}};
        crossingIndex.add(map_detail::tileKey(0, 0), 0);
        crossingIndex.addInfluence(0, crossingSources[0].position, crossingSources[0].radius, 300);
        StaticMapLightBuffer crossingCache;
        for (const bool multiMaterial : {false, true}) {
            setMapLights({}, 0.0);
            const auto unlit = spotlightPixel(multiMaterial);
            const auto& crossingSourcesSelected =
                crossingIndex.activeIndices({map_detail::tileKey(1, 0)});
            setStaticMapLights(crossingCache, crossingSources, crossingSourcesSelected,
                               {{{-2, 0, -2}, 0}}, 1.0);
            const auto litAcrossTile = spotlightPixel(multiMaterial);
            const auto& outsideSourcesSelected =
                crossingIndex.activeIndices({map_detail::tileKey(2, 0)});
            setStaticMapLights(crossingCache, crossingSources, outsideSourcesSelected, {}, 1.0);
            if (litAcrossTile == unlit || spotlightPixel(multiMaterial) != unlit) {
                std::cerr << "static map-light tile crossing failed for "
                          << (multiMaterial ? "material" : "model") << " shader\n";
                passed = false;
            }
        }
        crossingSources[0].radius = 0.5;
        const auto& allCrossingSources = crossingIndex.allIndices();
        const std::vector<StaticMapLightView> tailView = {{{0, 0, -2}, 0}};
        for (const bool multiMaterial : {false, true}) {
            setMapLights({}, 0.0);
            const auto unlit = spotlightPixel(multiMaterial);
            MapPointLight directTail = crossingSources[0];
            directTail.position = {0, 0, -2};
            setMapLights({directTail}, 1.0);
            const auto expectedTail = spotlightPixel(multiMaterial);
            setStaticMapLights(crossingCache, crossingSources, allCrossingSources, tailView, 1.0,
                               1);
            if (expectedTail == unlit || spotlightPixel(multiMaterial) != expectedTail) {
                std::cerr << "loaded static light tail was omitted for "
                          << (multiMaterial ? "material" : "model") << " shader\n";
                passed = false;
            }
        }
        releaseStaticMapLights(crossingCache);
    }
    for (const bool multiMaterial : {false, true}) {
        setMapLights({}, 0.0);
        const auto unlit = spotlightPixel(multiMaterial);
        setMapLights({testMapLight}, 1.0);
        const auto illuminated = spotlightPixel(multiMaterial);
        std::vector<MapPointLight> fullMapLightBudget(250, testMapLight);
        for (auto& light : fullMapLightBudget) {
            light.color = {0, 0, 0};
        }
        fullMapLightBudget.back() = testMapLight;
        setMapLights(fullMapLightBudget, 1.0);
        const auto lastMapLightSlot = spotlightPixel(multiMaterial);
        staticIndices.resize(fullMapLightBudget.size());
        staticViews.resize(fullMapLightBudget.size());
        for (std::size_t index = 0; index < fullMapLightBudget.size(); ++index) {
            staticIndices[index] = index;
            staticViews[index] = {fullMapLightBudget[index].position, index};
        }
        setStaticMapLights(staticCache, fullMapLightBudget, staticIndices, staticViews, 1.0, 2);
        if (spotlightPixel(multiMaterial) != lastMapLightSlot) {
            std::cerr << "static map-light buffer lost a light beyond slot 128\n";
            passed = false;
        }
        if (lightingBufferUploadStats().mapLightsMax < 250) {
            std::cerr << "map light count did not include the 250th light\n";
            passed = false;
        }
        setMapLights({attenuatedMapLight}, 1.0);
        const auto attenuated = spotlightPixel(multiMaterial);
        const MapPointLight hugeRadiusMapLight{{0, 0, -2.0e20}, {1, 0.5, 0.25}, 1.0e20};
        setMapLights({hugeRadiusMapLight}, 1.0);
        const auto hugeRadius = spotlightPixel(multiMaterial);
        setMapLights({testMapLight}, 0.0);
        const auto daylight = spotlightPixel(multiMaterial);
        if (illuminated[0] <= unlit[0] + 30 || illuminated[1] <= unlit[1] + 10 ||
            illuminated[0] <= illuminated[1] || illuminated[1] <= illuminated[2] ||
            std::abs(static_cast<int>(lastMapLightSlot[0]) - illuminated[0]) > 1 ||
            std::abs(static_cast<int>(lastMapLightSlot[1]) - illuminated[1]) > 1 ||
            std::abs(static_cast<int>(lastMapLightSlot[2]) - illuminated[2]) > 1 ||
            attenuated[0] <= unlit[0] || attenuated[0] >= illuminated[0] ||
            hugeRadius[0] <= unlit[0] || hugeRadius[0] >= illuminated[0] ||
            daylight != unlit || illuminated[3] != unlit[3]) {
            std::cerr << "map point-light color, radius attenuation, or strength failed for "
                      << (multiMaterial ? "material" : "model") << " shader\n";
            passed = false;
        }
    }
    releaseStaticMapLights(staticCache);
    setMapLights({}, 0.0);
    StaticMapLightBuffer equivalenceCache;
    std::vector<MapPointLight> equivalenceSources;
    std::vector<std::size_t> equivalenceIndices;
    for (std::size_t index = 0; index < 250; ++index) {
        equivalenceSources.push_back(
            {{static_cast<double>(index % 17) * 0.5 - 4.0,
              static_cast<double>((index / 17) % 15) * 0.5 - 3.5,
              -1.5 - static_cast<double>(index % 5) * 0.5},
             {0.001 * (1 + index % 3), 0.001 * (1 + index % 5),
              0.001 * (1 + index % 7)},
             0.5 + 0.25 * (index % 4)});
        equivalenceIndices.push_back((index * 37) % 250);
    }
    for (const Matrix4& view : {identityMatrix(), translationMatrix({0.1, 0.2, -0.5})}) {
        setModelViewMatrix(view);
        std::vector<MapPointLight> directLights;
        std::vector<StaticMapLightView> persistentViews;
        for (std::size_t index = 0; index < equivalenceIndices.size(); ++index) {
            MapPointLight light = equivalenceSources[equivalenceIndices[index]];
            const auto position =
                transformPoint(view, {light.position[0], light.position[1], light.position[2], 1});
            light.position = {position[0], position[1], position[2]};
            directLights.push_back(light);
            persistentViews.push_back({light.position, index});
        }
        for (const bool multiMaterial : {false, true}) {
            std::array<std::uint8_t, 32 * 32 * 4> baseline = {};
            std::array<std::uint8_t, 32 * 32 * 4> direct = {};
            std::array<std::uint8_t, 32 * 32 * 4> persistent = {};
            setMapLights({}, 0.0);
            spotlightPixel(multiMaterial);
            glReadPixels(0, 0, 32, 32, GL_RGBA, GL_UNSIGNED_BYTE, baseline.data());
            setMapLights(directLights, 1.0);
            spotlightPixel(multiMaterial);
            glReadPixels(0, 0, 32, 32, GL_RGBA, GL_UNSIGNED_BYTE, direct.data());
            setStaticMapLights(equivalenceCache, equivalenceSources, equivalenceIndices,
                               persistentViews, 1.0);
            spotlightPixel(multiMaterial);
            glReadPixels(0, 0, 32, 32, GL_RGBA, GL_UNSIGNED_BYTE, persistent.data());
            if (direct != persistent || direct == baseline) {
                std::cerr << "persistent and direct 250-light images differ or are unlit\n";
                passed = false;
            }
        }
    }
    releaseStaticMapLights(equivalenceCache);
    setMapLights({}, 0.0);
    StaticMapLightBuffer mixedStaticCache;
    MapPointLight mixedSource = testMapLight;
    mixedSource.color = {0, 0, 0.4};
    for (const Matrix4& view : {identityMatrix(), translationMatrix({0, 0, -0.5})}) {
        setModelViewMatrix(view);
        VehicleSpotlight player = testSpotlight;
        VehicleSpotlight ai = testSpotlight;
        player.position[2] += view[14];
        ai.position[2] += view[14];
        player.color = {0.4, 0, 0};
        ai.color = {0, 0.4, 0};
        MapPointLight scenery = testMapLight;
        scenery.position[2] += view[14];
        scenery.color = {0, 0, 0.4};
        std::vector<openbus::app::vehicle::SpotlightCandidate> mixedCandidates = {
            {ai, false}, {player, true}};
        const auto mixedSpotlights =
            openbus::app::vehicle::selectSpotlights(mixedCandidates, mixedCandidates.size());
        for (const bool multiMaterial : {false, true}) {
            setVehicleSpotlights({});
            setMapLights({}, 0.0);
            const auto baseline = spotlightPixel(multiMaterial);
            setVehicleSpotlights(mixedSpotlights);
            setStaticMapLights(mixedStaticCache, {mixedSource}, {0},
                               {{scenery.position, 0}}, 1.0);
            const auto combined = spotlightPixel(multiMaterial);
            setVehicleSpotlights({player});
            const auto withoutAi = spotlightPixel(multiMaterial);
            setVehicleSpotlights({ai});
            const auto withoutPlayer = spotlightPixel(multiMaterial);
            setMapLights({}, 0.0);
            const auto withoutScenery = spotlightPixel(multiMaterial);
            if (combined[0] <= baseline[0] || combined[1] <= baseline[1] ||
                combined[2] <= baseline[2] || withoutAi[1] >= combined[1] ||
                withoutPlayer[0] >= combined[0] || withoutScenery[2] >= combined[2]) {
                std::cerr << "combined player, AI and scenery lighting failed across views\n";
                passed = false;
            }
            releaseStaticMapLights(mixedStaticCache);
        }
    }
    setVehicleSpotlights({});
    setMapLights({}, 0.0);
    setModelViewMatrix(identityMatrix());
    setWeatherLighting(std::nullopt);
    const auto fogSample = [&](bool multiMaterial, double distance) {
        setModelViewMatrix(translationMatrix({0, 0, 1.0 - distance}));
        glClear(GL_COLOR_BUFFER_BIT);
        if (multiMaterial) {
            drawMaterialBatch(buffer, 3, materialColors, {white}, {false}, 0, 0.5);
        } else {
            drawModelBatch(buffer, 3, material, {1, 1, 1}, 0.5, 2);
        }
        std::array<std::uint8_t, 4> result = {};
        glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result.data());
        return result;
    };
    material.textured = true;
    material.texture = blue;
    setWeatherFog(std::nullopt);
    const auto bareRoad = fogSample(false, 1.0);
    material.snowCoverage = 0.85F;
    const auto snowyRoad = fogSample(false, 1.0);
    setWeatherFog(5.0);
    const auto foggySnow = fogSample(false, 1.0);
    const double snowTransmission = std::exp(-std::log(50.0) *
                                             std::sqrt(1.0 + 2.0 / (32.0 * 32.0)) / 5.0);
    constexpr std::array<double, 3> weatherFogColor = {0.45, 0.65, 0.88};
    for (std::size_t channel = 0; channel < 3; ++channel) {
        const int expected = static_cast<int>(std::lround(
            255.0 * weatherFogColor[channel] * (1.0 - snowTransmission) +
            snowyRoad[channel] * snowTransmission));
        if (snowyRoad[channel] <= bareRoad[channel] || !close(foggySnow[channel], expected)) {
            std::cerr << "snow albedo did not respond correctly to lighting and fog\n";
            passed = false;
        }
    }
    if (snowyRoad[3] != bareRoad[3] || foggySnow[3] != bareRoad[3]) {
        std::cerr << "snow or fog changed road opacity\n";
        passed = false;
    }
    material = {};
    for (bool multiMaterial : {false, true}) {
        setWeatherFog(std::nullopt);
        const auto clearNear = fogSample(multiMaterial, 1.0);
        const auto clearFar = fogSample(multiMaterial, 10.0);
        setWeatherFog(5.0);
        const auto fogNear = fogSample(multiMaterial, 1.0);
        const auto fogFar = fogSample(multiMaterial, 10.0);
        const auto expected = [](int clear, double distance, double fogColor) {
            const double pixelDistance = distance * std::sqrt(1.0 + 2.0 / (32.0 * 32.0));
            const double transmission = std::exp(-std::log(50.0) * pixelDistance / 5.0);
            return static_cast<int>(std::lround(255.0 * fogColor * (1.0 - transmission) +
                                                clear * transmission));
        };
        for (std::size_t channel = 0; channel < 3; ++channel) {
            constexpr std::array<double, 3> fogColor = {0.45, 0.65, 0.88};
            if (!close(fogNear[channel], expected(clearNear[channel], 1.0, fogColor[channel])) ||
                !close(fogFar[channel], expected(clearFar[channel], 10.0, fogColor[channel]))) {
                std::cerr << "visibility fog did not match near/far GPU transmission\n";
                passed = false;
                break;
            }
        }
        if (!close(fogNear[3], 128) || !close(fogFar[3], 128)) {
            std::cerr << "visibility fog changed fragment opacity\n";
            passed = false;
        }
        setWeatherFog(std::nullopt);
        if (fogSample(multiMaterial, 1.0) != clearNear) {
            std::cerr << "disabling fog did not restore clear rendering\n";
            passed = false;
        }
    }
    for (bool multiMaterial : {false, true}) {
        setWeatherFog(std::nullopt);
        const auto clear = fogSample(multiMaterial, 10.0);
        setWeatherFog(5.0);
        const auto full = fogSample(multiMaterial, 10.0);
        const auto half = fogSample(multiMaterial, 5.0);
        setWeatherFog(5.0, 5.0);
        setWeatherFogView(identityMatrix(), -20.0);
        const auto above = fogSample(multiMaterial, 10.0);
        setWeatherFogView(identityMatrix(), -10.0);
        const auto crossing = fogSample(multiMaterial, 10.0);
        setWeatherFogView(identityMatrix(), 0.0);
        const auto below = fogSample(multiMaterial, 10.0);
        for (std::size_t channel = 0; channel < 4; ++channel) {
            if (!close(above[channel], clear[channel]) ||
                !close(crossing[channel], half[channel]) ||
                !close(below[channel], full[channel])) {
                std::cerr << "height fog did not integrate the camera-to-fragment layer path\n";
                passed = false;
                break;
            }
        }
    }
    setWeatherFog(5.0, 5.0);
    setWeatherFogView(identityMatrix(), 0.0);
    const auto validHeightFog = fogSample(false, 10.0);
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            setWeatherFog(5.0, invalid);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected || fogSample(false, 10.0) != validHeightFog) {
            std::cerr << "invalid fog height changed renderer state\n";
            passed = false;
        }
    }
    setWeatherFog(0.0);
    const auto zeroVisibility = fogSample(false, 1.0);
    setWeatherFog(1.0);
    if (fogSample(false, 1.0) != zeroVisibility) {
        std::cerr << "zero visibility was not clamped to one meter\n";
        passed = false;
    }
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            setWeatherFog(invalid);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected || fogSample(false, 1.0) != zeroVisibility) {
            std::cerr << "invalid visibility was not rejected without changing fog\n";
            passed = false;
        }
    }
    setWeatherFog(std::nullopt);
    setModelViewMatrix(identityMatrix());
    {
        SystemVariables clock;
        clock.configureSimulationClock(2024, 3, 20, 43200);
        clock.configureDayNight(0);
        setWeatherLighting(clock.environmentBrightness());
        const auto day = fogSample(false, 1.0);
        const auto daySky = weatherBackgroundColor(clock.environmentBrightness());
        clock.updateSimulationClock(43200);
        setWeatherLighting(clock.environmentBrightness());
        const auto night = fogSample(false, 1.0);
        const auto nightSky = weatherBackgroundColor(clock.environmentBrightness());
        if (night[0] >= day[0] / 2 || nightSky[0] >= daySky[0] / 2) {
            std::cerr << "simulation clock did not darken surface and sky\n";
            passed = false;
        }
    }
    for (bool multiMaterial : {false, true}) {
        setWeatherLighting(std::nullopt);
        const auto day = fogSample(multiMaterial, 1.0);
        setWeatherLighting(0.5);
        const auto twilight = fogSample(multiMaterial, 1.0);
        setWeatherLighting(0.0);
        const auto night = fogSample(multiMaterial, 1.0);
        setWeatherLighting(0.0, true);
        const auto reflectionNight = fogSample(multiMaterial, 1.0);
        setWeatherLighting(0.0, false);
        const auto restoredMainNight = fogSample(multiMaterial, 1.0);
        const char* reflectionFloorSetting =
            openbus::getEnvironment("OPENBUS_REFLECTION_AMBIENT_FLOOR");
        float reflectionFloor = 0.25F;
        if (reflectionFloorSetting != nullptr) {
            const std::string_view value(reflectionFloorSetting);
            float parsedFloor = 0.0F;
            const auto [end, error] =
                std::from_chars(value.data(), value.data() + value.size(), parsedFloor);
            if (error == std::errc{} && end == value.data() + value.size() &&
                std::isfinite(parsedFloor) && parsedFloor >= 0.0F && parsedFloor <= 1.0F) {
                reflectionFloor = parsedFloor;
            }
        }
        bool reflectionAmbientRaised = false;
        for (std::size_t channel = 0; channel < 3; ++channel) {
            if (!close(twilight[channel], static_cast<int>(std::lround(day[channel] * 0.5))) ||
                night[channel] > 2 ||
                (reflectionFloor == 0.0F && reflectionNight[channel] != night[channel]) ||
                (reflectionFloor > 0.0F && reflectionNight[channel] < night[channel]) ||
                restoredMainNight[channel] != night[channel]) {
                std::cerr << "weather brightness did not scale GPU surface lighting\n";
                passed = false;
                break;
            }
            reflectionAmbientRaised |= reflectionNight[channel] > night[channel];
        }
        if (reflectionFloor > 0.0F && !reflectionAmbientRaised) {
            std::cerr << "reflection ambient floor did not brighten the night surface\n";
            passed = false;
        }
        if (night[3] != day[3] || twilight[3] != day[3] || reflectionNight[3] != day[3] ||
            restoredMainNight[3] != day[3]) {
            std::cerr << "weather brightness changed surface opacity\n";
            passed = false;
        }
        setWeatherLighting(std::nullopt);
        if (fogSample(multiMaterial, 1.0) != day) {
            std::cerr << "removing brightness did not restore daylight lighting\n";
            passed = false;
        }
    }
    setWeatherLighting(0.5);
    const auto halfBackground = weatherBackgroundColor(0.5);
    if (std::abs(halfBackground[0] - 0.225F) > 0.0001F ||
        std::abs(halfBackground[1] - 0.325F) > 0.0001F ||
        std::abs(halfBackground[2] - 0.44F) > 0.0001F) {
        std::cerr << "weather background did not scale with lighting\n";
        passed = false;
    }
    const auto halfLighting = fogSample(false, 1.0);
    for (double invalid : {-0.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            setWeatherLighting(invalid);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected || fogSample(false, 1.0) != halfLighting) {
            std::cerr << "invalid brightness was not rejected without changing lighting\n";
            passed = false;
        }
    }
    setWeatherFog(1.0);
    const auto fogAtHalfBrightness = fogSample(false, 1.0);
    if (!close(fogAtHalfBrightness[0], static_cast<int>(std::lround(255.0 * halfBackground[0]))) ||
        !close(fogAtHalfBrightness[1], static_cast<int>(std::lround(255.0 * halfBackground[1]))) ||
        !close(fogAtHalfBrightness[2], static_cast<int>(std::lround(255.0 * halfBackground[2])))) {
        std::cerr << "dense fog did not approach the weather-adjusted background\n";
        passed = false;
    }
    setWeatherFog(std::nullopt);
    setWeatherLighting(0.0);
    material.emissionColor = {1, 1, 1};
    const auto emissionAtNight = draw(material, {1, 1, 1}, 1.0);
    material = {};
    material.nightmap = white;
    material.useNightmap = true;
    material.nightmapStrength = 1.0F;
    const auto nightmapAtNight = draw(material, {1, 1, 1}, 1.0);
    if (emissionAtNight[0] < 250 || nightmapAtNight[0] < 250) {
        std::cerr << "weather brightness suppressed self-lit materials or nightmaps\n";
        passed = false;
    }
    material = {};
    setWeatherLighting(std::nullopt);
    setModelViewMatrix(identityMatrix());
    material.useTextTexture = true;
    material.textTexture = blue;
    auto pixel = draw(material, {1, 1, 1}, 1.0);
    if (!close(pixel[0], 65) || !close(pixel[1], 85) || !close(pixel[2], 112)) {
        std::cerr << "text texture did not preserve its authored colour\n";
        passed = false;
    }
    setWeatherLighting(0.25);
    const auto unlitDisplay = draw(material, {1, 1, 1}, 1.0);
    material.weatherLitDynamicTexture = true;
    const auto litSign = draw(material, {1, 1, 1}, 1.0);
    if (!close(unlitDisplay[2], 112) || !close(litSign[0], 16) ||
        !close(litSign[1], 21) || !close(litSign[2], 28) ||
        litSign[3] != unlitDisplay[3]) {
        std::cerr << "scenery text did not follow weather brightness or preserve alpha\n";
        passed = false;
    }
    material.emissionColor = {1, 1, 1};
    if (draw(material, {1, 1, 1}, 1.0) != unlitDisplay) {
        std::cerr << "emissive scenery text did not remain illuminated\n";
        passed = false;
    }
    material.emissionColor = {};
    material.weatherLitDynamicTexture = false;
    setWeatherLighting(std::nullopt);
    material.textTexture = white;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (!close(pixel[0], 255) || !close(pixel[1], 255) || !close(pixel[2], 255)) {
        std::cerr << "white text texture was dimmed by surface lighting\n";
        passed = false;
    }
    material = {};
    material.transmap = mask;
    material.useTransmap = true;
    pixel = draw(material, {1, 1, 1}, 0.25);
    if (!close(pixel[3], 32)) {
        std::cerr << "transmap replaced rather than scaled material opacity\n";
        passed = false;
    }
    material = {};
    pixel = draw(material, {1, 1, 1}, 1.0);
    pglActiveTexture(GL_TEXTURE3);
    GLint transmapBinding = -1;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &transmapBinding);
    pglActiveTexture(GL_TEXTURE0);
    if (transmapBinding != 0) {
        std::cerr << "missing auxiliary transmap retained a stale texture binding\n";
        passed = false;
    }
    material = {};
    material.texture = blue;
    material.textured = true;
    material.textured = false;
    material.freeTexture = blue;
    material.useFreeTexture = true;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (!close(pixel[0], 65) || !close(pixel[1], 85) || !close(pixel[2], 112)) {
        std::cerr << "free texture did not preserve its authored colour\n";
        passed = false;
    }
    setWeatherLighting(0.25);
    material.weatherLitDynamicTexture = true;
    const auto litFreeTexture = draw(material, {1, 1, 1}, 1.0);
    if (!close(litFreeTexture[0], 16) || !close(litFreeTexture[1], 21) ||
        !close(litFreeTexture[2], 28) || litFreeTexture[3] != pixel[3]) {
        std::cerr << "scenery free texture did not follow weather brightness or preserve alpha\n";
        passed = false;
    }
    material.emissionColor = {1, 1, 1};
    if (draw(material, {1, 1, 1}, 1.0) != pixel) {
        std::cerr << "emissive scenery free texture did not remain illuminated\n";
        passed = false;
    }
    material.emissionColor = {};
    material.weatherLitDynamicTexture = false;
    setWeatherLighting(std::nullopt);
    material.freeTexture = white;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (!close(pixel[0], 255) || !close(pixel[1], 255) || !close(pixel[2], 255)) {
        std::cerr << "white free texture was dimmed by surface lighting\n";
        passed = false;
    }
    material = {};
    material.texture = blue;
    material.textured = true;
    material.lightmap = white;
    material.useLightmap = true;
    material.lightmapStrength = 1.0F;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (!close(pixel[0], 65) || !close(pixel[1], 85) || !close(pixel[2], 112)) {
        std::cerr << "dashboard lightmap did not restore the lit surface colour\n";
        passed = false;
    }
    material = {};
    material.texture = white;
    material.textured = true;
    material.transmap = mask;
    material.useTransmap = true;
    glClearColor(64.0F / 255.0F, 64.0F / 255.0F, 64.0F / 255.0F, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    drawModelBatch(buffer, 3, material, {1, 1, 1}, 1.0, 2);
    glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
    glDisable(GL_BLEND);
    if (!close(pixel[0], 106) || !close(pixel[1], 106) || !close(pixel[2], 106)) {
        std::cerr << "white road marking ignored its alpha or surface lighting\n";
        passed = false;
    }

    auto environmentVertices = vertices;
    constexpr float normalX = 0.258819F;
    constexpr float normalZ = 0.965926F;
    for (std::size_t index = 0; index < environmentVertices.size(); index += 13) {
        environmentVertices[index + 6] = normalX;
        environmentVertices[index + 7] = 0.0F;
        environmentVertices[index + 8] = normalZ;
    }
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(environmentVertices), environmentVertices.data(),
                  GL_STATIC_DRAW);
    GLuint environment = 0;
    glGenTextures(1, &environment);
    glBindTexture(GL_TEXTURE_2D, environment);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    constexpr std::array<std::uint8_t, 8> environmentPixels = {
        0, 255, 0, 255, 255, 0, 0, 255};
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 environmentPixels.data());
    invalidateTextureBindings();
    setModelViewMatrix(identityMatrix());
    const auto sampleEnvironment = [&](double alpha) {
        glClearColor(0, 0, 64.0F / 255.0F, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        drawEnvironmentBatch(buffer, 3, environment, alpha);
        glDisable(GL_BLEND);
        std::array<std::uint8_t, 4> result = {};
        glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result.data());
        return result;
    };
    pixel = sampleEnvironment(0.5);
    if (!close(pixel[0], 128) || !close(pixel[1], 0) || !close(pixel[2], 32) ||
        !close(pixel[3], 191)) {
        std::cerr << "environment reflection did not sample the expected direction and opacity: "
                  << static_cast<int>(pixel[0]) << ',' << static_cast<int>(pixel[1]) << ','
                  << static_cast<int>(pixel[2]) << ',' << static_cast<int>(pixel[3]) << '\n';
        passed = false;
    }
    if (sampleEnvironment(0.0) != std::array<std::uint8_t, 4>{0, 0, 64, 255} ||
        sampleEnvironment(1.0) != std::array<std::uint8_t, 4>{255, 0, 0, 255}) {
        std::cerr << "environment reflection opacity endpoints were not respected\n";
        passed = false;
    }
    setWeatherLighting(0.5);
    const auto dimEnvironment = sampleEnvironment(1.0);
    if (!close(dimEnvironment[0], 128) || dimEnvironment[1] != 0 ||
        dimEnvironment[2] != 0 || dimEnvironment[3] != 255) {
        std::cerr << "environment reflection did not follow weather brightness\n";
        passed = false;
    }
    setWeatherLighting(std::nullopt);
    setWeatherFog(5.0);
    const auto foggedEnvironment = sampleEnvironment(1.0);
    const double transmission = std::exp(-std::log(50.0) *
                                         std::sqrt(1.0 + 2.0 / (32.0 * 32.0)) / 5.0);
    if (!close(foggedEnvironment[0], static_cast<int>(std::lround(
                                             255.0 * (0.45 * (1.0 - transmission) + transmission)))) ||
        !close(foggedEnvironment[1], static_cast<int>(std::lround(
                                             255.0 * 0.65 * (1.0 - transmission)))) ||
        !close(foggedEnvironment[2], static_cast<int>(std::lround(
                                             255.0 * 0.88 * (1.0 - transmission)))) ||
        foggedEnvironment[3] != 255) {
        std::cerr << "environment reflection did not retain opacity under fog\n";
        passed = false;
    }
    setWeatherFog(5.0, 0.5);
    setWeatherFogView(identityMatrix(), -10.0);
    const auto aboveEnvironmentFog = sampleEnvironment(1.0);
    setWeatherFogView(identityMatrix(), 0.0);
    const auto insideEnvironmentFog = sampleEnvironment(1.0);
    if (aboveEnvironmentFog != std::array<std::uint8_t, 4>{255, 0, 0, 255} ||
        insideEnvironmentFog != foggedEnvironment) {
        std::cerr << "environment reflection did not respect the fog layer\n";
        passed = false;
    }
    setWeatherFog(std::nullopt);
    for (std::size_t index = 0; index < environmentVertices.size(); index += 13) {
        environmentVertices[index + 6] = -normalX;
    }
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(environmentVertices), environmentVertices.data(),
                  GL_STATIC_DRAW);
    if (sampleEnvironment(1.0) != std::array<std::uint8_t, 4>{0, 255, 0, 255}) {
        std::cerr << "opposite surface normal did not sample the opposite environment direction\n";
        passed = false;
    }
    ModelMaterial reflected;
    reflected.texture = blue;
    reflected.textured = true;
    reflected.emissionColor = {1, 1, 1};
    reflected.environmentMap = environment;
    reflected.useEnvironmentMap = true;
    reflected.environmentStrength = 1.0F;
    if (draw(reflected, {1, 1, 1}, 1) != std::array<std::uint8_t, 4>{0, 255, 0, 255}) {
        std::cerr << "model environment map sampled the wrong reflection direction\n";
        passed = false;
    }
    for (std::size_t index = 0; index < environmentVertices.size(); index += 13) {
        environmentVertices[index + 6] = normalX;
    }
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(environmentVertices), environmentVertices.data(),
                  GL_STATIC_DRAW);
    reflected.environmentStrength = 0.0F;
    const auto unreflected = draw(reflected, {1, 1, 1}, 1);
    reflected.environmentStrength = 0.5F;
    const auto partialReflection = draw(reflected, {1, 1, 1}, 1);
    reflected.environmentStrength = 2.0F;
    const auto fullReflection = draw(reflected, {1, 1, 1}, 1);
    if (unreflected != std::array<std::uint8_t, 4>{65, 85, 112, 255} ||
        !close(partialReflection[0], 160) || !close(partialReflection[1], 43) ||
        !close(partialReflection[2], 56) ||
        fullReflection != std::array<std::uint8_t, 4>{255, 0, 0, 255}) {
        std::cerr << "model environment strength or clamping was incorrect\n";
        passed = false;
    }
    reflected.useEnvironmentMap = false;
    if (draw(reflected, {1, 1, 1}, 1) != unreflected) {
        std::cerr << "environment state leaked into an unreflected model\n";
        passed = false;
    }
    reflected.useEnvironmentMap = true;
    const GLuint transparent = texture({255, 255, 255, 0});
    reflected.texture = transparent;
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    drawModelBatch(buffer, 3, reflected, {1, 1, 1}, 1, 1);
    glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
    if (pixel != std::array<std::uint8_t, 4>{0, 0, 0, 0}) {
        std::cerr << "environment map escaped an alpha-cutout surface\n";
        passed = false;
    }
    reflected.texture = mask;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pixel = draw(reflected, {1, 1, 1}, 1);
    glDisable(GL_BLEND);
    if (!close(pixel[0], 128) || pixel[1] != 0 || pixel[2] != 0 || !close(pixel[3], 64)) {
        std::cerr << "environment map ignored surface transparency\n";
        passed = false;
    }
    reflected.texture = blue;
    GLuint reflectionIndexBuffer = 0;
    constexpr std::array<std::uint32_t, 3> reflectionIndices = {0, 1, 2};
    pglGenBuffers(1, &reflectionIndexBuffer);
    pglBindBuffer(GL_ARRAY_BUFFER, reflectionIndexBuffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(reflectionIndices), reflectionIndices.data(),
                  GL_STATIC_DRAW);
    glClear(GL_COLOR_BUFFER_BIT);
    drawIndexedModelBatch(buffer, reflectionIndexBuffer, 3, reflected, {1, 1, 1}, 1, 2);
    glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
    if (pixel != fullReflection) {
        std::cerr << "indexed model environment binding differed from array drawing\n";
        passed = false;
    }
    pglDeleteBuffers(1, &reflectionIndexBuffer);
    glDeleteTextures(1, &transparent);
    invalidateTextureBindings();
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);

    material = {};
    material.texture = white;
    material.textured = true;
    const auto whitePixel = draw(material, {1, 1, 1}, 1.0);
    material.emissionColor = {0.2F, 0.1F, 0.0F};
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (!close(pixel[0], static_cast<int>(whitePixel[0] + (255 - whitePixel[0]) * 0.2)) ||
        !close(pixel[1], static_cast<int>(whitePixel[1] + (255 - whitePixel[1]) * 0.1)) ||
        pixel[2] != whitePixel[2]) {
        std::cerr << "material emission did not illuminate the authored surface\n";
        passed = false;
    }
    const GLuint red = texture({220, 0, 0, 255});
    const GLuint black = texture({0, 0, 0, 255});
    material = {};
    material.texture = blue;
    material.textured = true;
    const auto nightmapBasePixel = draw(material, {1, 1, 1}, 1.0);
    material.nightmap = black;
    material.useNightmap = true;
    material.nightmapStrength = 1.0F;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (pixel != nightmapBasePixel) {
        std::cerr << "black nightmap texels changed the base material\n";
        passed = false;
    }
    material.nightmap = red;
    material.nightmapStrength = 0.0F;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (pixel != nightmapBasePixel) {
        std::cerr << "zero nightmap strength changed the base material\n";
        passed = false;
    }
    material.nightmapStrength = 1.0F;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (pixel[0] <= nightmapBasePixel[0] + 60 || pixel[1] >= nightmapBasePixel[1] ||
        pixel[2] >= nightmapBasePixel[2]) {
        std::cerr << "nightmap emission did not brighten the authored red overlay\n";
        passed = false;
    }
    for (const GLuint albedo : {red, black, blue}) {
        for (int source = 0; source < 3; ++source) {
            material = {};
            material.emissionColor = {1, 1, 1};
            material.texture = albedo;
            material.textured = true;
            if (source == 1) {
                material.textTexture = albedo;
                material.useTextTexture = true;
            } else if (source == 2) {
                material.freeTexture = albedo;
                material.useFreeTexture = true;
            }
            pixel = draw(material, {1, 1, 1}, 1.0);
            const std::array<int, 3> expected =
                albedo == red ? std::array<int, 3>{220, 0, 0}
                : albedo == black ? std::array<int, 3>{0, 0, 0}
                                  : std::array<int, 3>{65, 85, 112};
            if (!close(pixel[0], expected[0]) || !close(pixel[1], expected[1]) ||
                !close(pixel[2], expected[2]) || pixel[3] != 255) {
                std::cerr << "white emission washed out route/display texture RGB; source="
                          << source << '\n';
                passed = false;
            }
        }
    }
    constexpr std::array<float, 39> specularVertices = {
        -2, -2, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
         2, -2, -1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1,
         0,  2, -1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(specularVertices), specularVertices.data(),
                  GL_STATIC_DRAW);
    material = {};
    material.texture = white;
    material.textured = true;
    const auto diffuseOnlyPixel = draw(material, {0.15, 0.15, 0.15}, 1.0);
    material.specularColor = {0.8F, 0.8F, 0.8F};
    material.specularPower = 8.0F;
    pixel = draw(material, {0.15, 0.15, 0.15}, 1.0);
    if (pixel[0] < diffuseOnlyPixel[0] + 40 || pixel[1] < diffuseOnlyPixel[1] + 40 ||
        pixel[2] < diffuseOnlyPixel[2] + 40) {
        std::cerr << "material specular highlight was not rendered\n";
        passed = false;
    }
    setWeatherLighting(0.0);
    const auto specularAtNight = draw(material, {0.15, 0.15, 0.15}, 1.0);
    if (specularAtNight[0] > 2 || specularAtNight[1] > 2 || specularAtNight[2] > 2) {
        std::cerr << "weather brightness did not attenuate specular highlights\n";
        passed = false;
    }
    setWeatherLighting(std::nullopt);
    constexpr std::array<float, 39> coloredVertices = {
        -2, -2, -1, 0, 0, 0, 0, 0, -1, 0.5F, 0.25F, 1.0F, 0.5F,
         2, -2, -1, 1, 0, 0, 0, 0, -1, 0.5F, 0.25F, 1.0F, 0.5F,
         0,  2, -1, 0, 1, 0, 0, 0, -1, 0.5F, 0.25F, 1.0F, 0.5F};
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(coloredVertices), coloredVertices.data(),
                  GL_STATIC_DRAW);
    material = {};
    material.texture = white;
    material.textured = true;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (pixel != whitePixel) {
        std::cerr << "legacy textured materials were unexpectedly tinted by vertex colors\n";
        passed = false;
    }
    material = {};
    material.textTexture = blue;
    material.useTextTexture = true;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (!close(pixel[0], 65) || !close(pixel[1], 85) || !close(pixel[2], 112)) {
        std::cerr << "text texture RGB was not preserved\n";
        passed = false;
    }
    material = {};
    material.freeTexture = blue;
    material.useFreeTexture = true;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (!close(pixel[0], 65) || !close(pixel[1], 85) || !close(pixel[2], 112)) {
        std::cerr << "free texture RGB was not preserved\n";
        passed = false;
    }
    constexpr std::array<float, 39> grayscaleVertices = {
        -2, -2, -1, 0, 0, 0, 0, 0, -1, 0.25F, 0.25F, 0.25F, 0.5F,
         2, -2, -1, 1, 0, 0, 0, 0, -1, 0.25F, 0.25F, 0.25F, 0.5F,
         0,  2, -1, 0, 1, 0, 0, 0, -1, 0.25F, 0.25F, 0.25F, 0.5F};
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(grayscaleVertices), grayscaleVertices.data(),
                  GL_STATIC_DRAW);
    material = {};
    material.texture = white;
    material.textured = true;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (pixel != whitePixel) {
        std::cerr << "neutral vertex colors changed a textured material\n";
        passed = false;
    }
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(coloredVertices), coloredVertices.data(),
                  GL_STATIC_DRAW);
    material = {};
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (pixel != whitePixel) {
        std::cerr << "legacy untextured materials were unexpectedly tinted by vertex colors\n";
        passed = false;
    }
    auto lightingVertices = vertices;
    for (std::size_t index = 0; index < lightingVertices.size(); index += 13) {
        lightingVertices[index + 6] = 0.6F;
        lightingVertices[index + 8] = 0.8F;
    }
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(lightingVertices), lightingVertices.data(),
                  GL_STATIC_DRAW);
    const GLuint normalMap = texture({128, 220, 220, 255});
    const GLuint inverseNormalMap = texture({128, 35, 220, 255});
    const std::vector<std::array<float, 4>> batchColors = {{{1, 1, 1, 1}}};
    const std::vector<GLuint> batchTextures = {white};
    GLuint indexBuffer = 0;
    constexpr std::array<std::uint32_t, 3> indices = {0, 1, 2};
    pglGenBuffers(1, &indexBuffer);
    pglBindBuffer(GL_ARRAY_BUFFER, indexBuffer);
    pglBufferData(GL_ARRAY_BUFFER, sizeof(indices), indices.data(), GL_STATIC_DRAW);
    pglBindBuffer(GL_ARRAY_BUFFER, buffer);
    std::array<std::uint8_t, 4> arrayMaterialBatchBaseline = {};
    for (int source = 0; source < 5; ++source) {
        material = {};
        material.texture = white;
        material.textured = true;
        material.useBumpmap = source == 1;
        material.bumpmap = source == 1 ? normalMap : 0;
        material.bumpmapStrength = source == 1 ? 1.0F : 0.0F;
        const auto drawLighting = [&]() {
            if (source < 2) {
                return draw(material, {1, 1, 1}, 1.0);
            }
            glClear(GL_COLOR_BUFFER_BIT);
            if (source == 2) {
                drawMaterialBatch(buffer, 3, batchColors, batchTextures, {});
            } else if (source == 3) {
                drawIndexedModelBatch(buffer, indexBuffer, 3, material, {1, 1, 1}, 1.0, 0);
            } else {
                drawMaterialBatch(buffer, 3, batchColors, batchTextures, {}, indexBuffer);
            }
            std::array<std::uint8_t, 4> result = {};
            glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, result.data());
            return result;
        };
        setModelViewMatrix(identityMatrix());
        setLightingViewMatrix(identityMatrix());
        const auto baseline = drawLighting();
        if (source == 2) {
            arrayMaterialBatchBaseline = baseline;
            const GLuint textTexture = texture({255, 0, 0, 255});
            glClear(GL_COLOR_BUFFER_BIT);
            drawMaterialBatch(buffer, 3, batchColors, batchTextures, {}, 0, 1.0, textTexture,
                              true);
            std::array<std::uint8_t, 4> textPixel = {};
            glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, textPixel.data());
            if (textPixel[0] <= textPixel[1] + 30 || textPixel[0] <= textPixel[2] + 30) {
                std::cerr << "material batches did not sample their dynamic text texture\n";
                passed = false;
            }
        } else if (source == 4 && baseline != arrayMaterialBatchBaseline) {
            std::cerr << "indexed material batch changed directional shading\n";
            passed = false;
        }
        if (baseline[0] <= 148) {
            std::cerr << "camera lighting probe did not exercise directional lighting\n";
            passed = false;
        }
        setLightingViewMatrix(identityMatrix(), std::array<double, 3>{0.6, 0.0, 0.8});
        const auto sunlit = drawLighting();
        setLightingViewMatrix(identityMatrix(), std::array<double, 3>{-0.6, 0.0, -0.8});
        const auto shaded = drawLighting();
        if (sunlit[0] <= shaded[0] + 30) {
            std::cerr << "solar direction did not affect directional shading; source=" << source
                      << '\n';
            passed = false;
        }
        setLightingViewMatrix(identityMatrix());
        if (source == 1) {
            const ModelMaterial bumpedMaterial = material;
            material.useBumpmap = false;
            material.bumpmap = 0;
            const auto unperturbed = drawLighting();
            material = bumpedMaterial;
            material.bumpmapStrength = 0.0F;
            const auto zeroStrength = drawLighting();
            if (zeroStrength != unperturbed) {
                std::cerr << "zero bump-map strength changed directional shading\n";
                passed = false;
            }
            material.bumpmapStrength = 1.0F;
            const auto fullStrength = drawLighting();
            if (fullStrength == unperturbed) {
                std::cerr << "bump-map normal did not affect directional shading\n";
                passed = false;
            }
            material.bumpmap = inverseNormalMap;
            const auto inverseStrength = drawLighting();
            if (fullStrength[0] <= inverseStrength[0]) {
                std::cerr << "opposite bump-map directions did not shade oppositely\n";
                passed = false;
            }
            material.bumpmap = normalMap;
            material.bumpmapStrength = 1.5F;
            if (drawLighting() != fullStrength) {
                std::cerr << "bump-map strength above one was not clamped\n";
                passed = false;
            }
            material.bumpmapStrength = -0.5F;
            if (drawLighting() != zeroStrength) {
                std::cerr << "negative bump-map strength was not clamped to zero\n";
                passed = false;
            }
            material = bumpedMaterial;
        }
        for (int frame = 0; frame < 3; ++frame) {
            drawTextureQuad(blue, -1, -1, 2, 2);
            const auto afterHud = drawLighting();
            if (afterHud != baseline) {
                std::cerr << "HUD texture draw changed subsequent model rendering; source="
                          << source << " frame=" << frame << '\n';
                passed = false;
            }
        }
        setLightingViewMatrix(translationMatrix({100, -200, 300}));
        if (drawLighting() != baseline) {
            std::cerr << "camera translation changed the directional light; source="
                      << source << '\n';
            passed = false;
        }
        setModelViewMatrix(rotationMatrix(180, 0, 0, 1));
        const auto objectRotatedPixel = drawLighting();
        if (close(objectRotatedPixel[0], baseline[0])) {
            std::cerr << "lighting probe did not distinguish object and camera rotation; source="
                      << source << '\n';
            passed = false;
        }
        for (const double angle : {90.0, 180.0, 270.0, 0.0}) {
            const Matrix4 view = rotationMatrix(angle, 0, 0, 1);
            setModelViewMatrix(view);
            setLightingViewMatrix(view);
            const auto rotatedPixel = drawLighting();
            for (std::size_t channel = 0; channel < 4; ++channel) {
                if (!close(rotatedPixel[channel], baseline[channel])) {
                    std::cerr << "camera rotation changed world-space lighting; source="
                              << source << " angle=" << angle << " channel=" << channel
                              << " baseline=" << static_cast<int>(baseline[channel])
                              << " actual=" << static_cast<int>(rotatedPixel[channel]) << '\n';
                    passed = false;
                }
            }
        }
    }
    setModelViewMatrix(identityMatrix());
    setLightingViewMatrix(identityMatrix());
    ModelMaterial interiorLightMaterial;
    interiorLightMaterial.texture = white;
    interiorLightMaterial.textured = true;
    const std::array<double, 3> interiorSurfaceColor = {0.3, 0.3, 0.3};
    const auto unlitInteriorPixel = draw(interiorLightMaterial, interiorSurfaceColor, 1.0);
    interiorLightMaterial.useInteriorLight = true;
    interiorLightMaterial.interiorLightCount = 1;
    interiorLightMaterial.interiorLightViewPositions[0] = {0.0F, 0.0F, 0.0F};
    interiorLightMaterial.interiorLightColors[0] = {1.0F, 0.0F, 0.0F};
    interiorLightMaterial.interiorLightStrengths[0] = 8.0F;
    const auto nearInteriorPixel = draw(interiorLightMaterial, interiorSurfaceColor, 1.0);
    interiorLightMaterial.interiorLightViewPositions[0] = {0.0F, 0.0F, 9.0F};
    const auto farInteriorPixel = draw(interiorLightMaterial, interiorSurfaceColor, 1.0);
    if (nearInteriorPixel[0] <= unlitInteriorPixel[0] + 10 ||
        nearInteriorPixel[1] != unlitInteriorPixel[1] ||
        nearInteriorPixel[2] != unlitInteriorPixel[2] ||
        farInteriorPixel[0] <= unlitInteriorPixel[0] ||
        farInteriorPixel[0] >= nearInteriorPixel[0]) {
        std::cerr << "interior point-light color or distance attenuation was incorrect\n";
        passed = false;
    }
    interiorLightMaterial.interiorLightStrengths[0] = 0.0F;
    if (draw(interiorLightMaterial, interiorSurfaceColor, 1.0) != unlitInteriorPixel) {
        std::cerr << "zero interior-light strength changed the surface\n";
        passed = false;
    }
    const auto unanimatedIndexedBaseline =
        drawHumanGaitFrame(0.0, 0.0, humanVertices, 1.7, false);
    static_cast<void>(
        drawHumanGaitFrame(std::numbers::pi / 2.0, 1.0, humanVertices, 1.7, true));
    ModelMaterial indexedSceneMaterial;
    indexedSceneMaterial.emissionColor = {1, 1, 1};
    glClear(GL_COLOR_BUFFER_BIT);
    setModelViewMatrix(translationMatrix({0, 0, -0.4}));
    drawIndexedModelBatch(buffer, indexBuffer, 3, indexedSceneMaterial, {1, 1, 1}, 1.0, 0);
    std::array<std::uint8_t, 32 * 32 * 4> indexedSceneAfterPedestrian = {};
    glReadPixels(0, 0, 32, 32, GL_RGBA, GL_UNSIGNED_BYTE, indexedSceneAfterPedestrian.data());
    if (indexedSceneAfterPedestrian != unanimatedIndexedBaseline) {
        std::cerr << "indexed scene geometry inherited the previous pedestrian gait animation\n";
        passed = false;
    }
    {
        const auto drawHud = [&]() {
            glClear(GL_COLOR_BUFFER_BIT);
            drawTextureQuad(blue, -1, -1, 2, 2);
            std::array<std::uint8_t, 4> pixel = {};
            glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
            return pixel;
        };
        const auto hudBeforeReflections = drawHud();
        VehicleCamera camera;
        camera.kind = VehicleCameraKind::Reflexion;
        const std::vector<VehicleCamera> cameras = {camera};
        ReflectionRenderer reflections;
        reflections.initialize(cameras);
        if (drawHud() != hudBeforeReflections) {
            std::cerr << "reflection initialization invalidated the HUD texture binding\n";
            passed = false;
        }
        BusConfiguration configuration{};
        configuration.mass = 12000;
        configuration.length = configuration.collisionLength = 10;
        configuration.width = configuration.collisionWidth = 2.5;
        configuration.bodyHalfLength = 5;
        configuration.bodyHalfWidth = 1.25;
        configuration.bodyHalfHeight = 1.5;
        configuration.collisionHeight = 3;
        configuration.wheelRadius = 0.5;
        configuration.wheelHalfWidth = 0.15;
        configuration.centerOfGravityHeight = 1;
        configuration.axles = {{3, 2, 2, 1.5, 1, 240000, 90000, 20000, true, false},
                                {-3, 2, 2, 1.5, 1, 240000, 90000, 20000, false, true}};
        World world;
        BusSimulation simulation(world, configuration, VehiclePlacement{{0, 0, 0}, 0});
        ModelMaterial sceneMaterial;
        sceneMaterial.texture = blue;
        sceneMaterial.textured = true;
        const std::array<GLuint, 2> reflectionLayerTextures = {
            texture({255, 0, 0, 255}), texture({0, 255, 0, 255})};
        struct ReflectionLayer {
            double depth;
            int renderType;
            std::size_t textureIndex;
        };
        std::vector<ReflectionLayer> reflectionLayers = {{1.0, 0, 1}, {3.0, 0, 0}};
        openbus::app::vehicle::sortTransparentBatches(reflectionLayers);
        std::array<std::uint8_t, 4> reflectedPixel = {};
        glClearColor(0, 0, 64.0F / 255.0F, 1);
        bool renderedTransparency = false;
        bool observedReflectionPass = false;
        reflections.render(
            simulation, cameras, {0, 0, 32, 32},
            [](std::size_t) { return ReflectionRequirement{true, 64}; },
            [&](const BusSimulation&, std::size_t, std::size_t, int width, int height) {
                renderedTransparency = true;
                observedReflectionPass = reflectionPassActive();
                pglBindBuffer(GL_ARRAY_BUFFER, buffer);
                pglBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);
                setPerspective(width, height, 90);
                setModelViewMatrix(identityMatrix());
                glDisable(GL_DEPTH_TEST);
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                for (const ReflectionLayer& layer : reflectionLayers) {
                    drawEnvironmentBatch(buffer, 3,
                                         reflectionLayerTextures[layer.textureIndex], 0.5);
                }
                glReadPixels(width / 2, height / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE,
                             reflectedPixel.data());
            },
            [](int width, int height) { setPerspective(width, height, 90); });
        glDisable(GL_BLEND);
        if (!renderedTransparency || !observedReflectionPass || reflectionPassActive() ||
            !close(reflectedPixel[0], 64) || !close(reflectedPixel[1], 128) ||
            !close(reflectedPixel[2], 16) || !close(reflectedPixel[3], 159)) {
            std::cerr << "back-to-front transparency did not blend correctly in the reflection "
                         "framebuffer or reflection-pass state was incorrect: "
                      << static_cast<int>(reflectedPixel[0]) << ','
                      << static_cast<int>(reflectedPixel[1]) << ','
                      << static_cast<int>(reflectedPixel[2]) << ','
                      << static_cast<int>(reflectedPixel[3]) << '\n';
            passed = false;
        }
        const std::vector<PrimitiveVertex> overlay = {
            {-2, -2, -1, 1, 0, 0}, {2, -2, -1, 1, 0, 0}, {0, 2, -1, 1, 0, 0}};
        GLuint streamedBuffer = 0;
        pglGenBuffers(1, &streamedBuffer);
        for (const int size : {64, 128, 64}) {
            for (const bool hud : {true, false}) {
                const auto drawSurface = [&]() {
                    return hud ? drawHud() : draw(sceneMaterial, {1, 1, 1}, 1.0);
                };
                const int targetSize = hud ? size : size * 2;
                invalidateTextureBindings();
                const auto beforeResize = drawSurface();
                bool rendered = false;
                reflections.render(
                    simulation, cameras, {0, 0, 32, 32},
                    [targetSize](std::size_t) {
                        return ReflectionRequirement{true, targetSize};
                    },
                    [&](const BusSimulation&, std::size_t, std::size_t, int, int) {
                        rendered = true;
                    },
                    [](int, int) {});
                if (!rendered || drawSurface() != beforeResize) {
                    std::cerr << "reflection resize changed " << (hud ? "HUD" : "scene")
                              << " rendering; size=" << targetSize << '\n';
                    passed = false;
                }
            }
        }
        if (pglGetBufferSubData != nullptr) {
            drawPrimitives(overlay, GL_TRIANGLES);
            pglBindBuffer(GL_ARRAY_BUFFER, streamedBuffer);
            pglBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);
            drawPrimitives(overlay, GL_TRIANGLES);
            std::array<float, 39> afterOverlay = {};
            pglBindBuffer(GL_ARRAY_BUFFER, streamedBuffer);
            pglGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(afterOverlay), afterOverlay.data());
            if (afterOverlay != vertices) {
                std::cerr << "overlay upload overwrote streamed scene geometry\n";
                passed = false;
            }
            drawStaticPrimitives(staticOverlay, overlay, GL_TRIANGLES);
            for (const std::uint64_t revision : {0U, 1U, 1U}) {
                pglBindBuffer(GL_ARRAY_BUFFER, streamedBuffer);
                pglBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);
                glClear(GL_COLOR_BUFFER_BIT);
                drawStaticPrimitives(staticOverlay, overlay, GL_TRIANGLES, 1.0F, revision);
                std::array<std::uint8_t, 4> pixel = {};
                glReadPixels(16, 16, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
                pglBindBuffer(GL_ARRAY_BUFFER, streamedBuffer);
                pglGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(afterOverlay), afterOverlay.data());
                if (afterOverlay != vertices || pixel[0] < 250 || pixel[1] != 0 ||
                    pixel[2] != 0) {
                    std::cerr << "cached overlay used or overwrote streamed scene geometry; "
                              << "revision=" << revision << '\n';
                    passed = false;
                }
            }
        } else {
            std::cerr << "OpenGL buffer readback is unavailable for the state regression\n";
            passed = false;
        }
        pglDeleteBuffers(1, &streamedBuffer);
    }
    if (!setReflectionInterval("2")) {
        std::cerr << "could not configure the reflection interval regression\n";
        passed = false;
    } else {
        VehicleCamera camera;
        camera.kind = VehicleCameraKind::Reflexion;
        const std::vector<VehicleCamera> cameras = {camera};
        ReflectionRenderer intervalReflections;
        intervalReflections.initialize(cameras);
        BusConfiguration configuration{};
        configuration.mass = 12000;
        configuration.length = configuration.collisionLength = 10;
        configuration.width = configuration.collisionWidth = 2.5;
        configuration.bodyHalfLength = 5;
        configuration.bodyHalfWidth = 1.25;
        configuration.bodyHalfHeight = 1.5;
        configuration.collisionHeight = 3;
        configuration.wheelRadius = 0.5;
        configuration.wheelHalfWidth = 0.15;
        configuration.centerOfGravityHeight = 1;
        configuration.axles = {{3, 2, 2, 1.5, 1, 240000, 90000, 20000, true, false},
                               {-3, 2, 2, 1.5, 1, 240000, 90000, 20000, false, true}};
        World world;
        BusSimulation simulation(world, configuration, VehiclePlacement{{0, 0, 0}, 0});
        std::array<bool, 3> renderedFrames = {};
        for (std::size_t frame = 0; frame < renderedFrames.size(); ++frame) {
            intervalReflections.render(
                simulation, cameras, {0, 0, 32, 32},
                [](std::size_t) { return ReflectionRequirement{true, 64}; },
                [&renderedFrames, frame](const BusSimulation&, std::size_t, std::size_t, int,
                                         int) {
                    renderedFrames[frame] = true;
                },
                [](int, int) {});
        }
        if (!renderedFrames[0] || renderedFrames[1] || !renderedFrames[2]) {
            std::cerr << "reflection interval did not render immediately and then every two frames\n";
            passed = false;
        }
        intervalReflections.destroy();
        if (!setReflectionInterval("4")) {
            std::cerr << "could not configure the staggered reflection regression\n";
            passed = false;
        }
        VehicleCamera driver;
        driver.kind = VehicleCameraKind::Driver;
        const std::vector<VehicleCamera> staggeredCameras = {driver, camera, camera};
        ReflectionRenderer staggeredReflections;
        staggeredReflections.initialize(staggeredCameras);
        openbus::app::ReflectionTrafficViews trafficViews;
        for (std::size_t frame = 0; frame < 5; ++frame) {
            std::size_t draws = 0;
            staggeredReflections.render(
                simulation, staggeredCameras, {0, 0, 32, 32},
                [&](std::size_t reflectionIndex) {
                    const bool needed = reflectionIndex == 0 ? frame != 2 : frame > 0;
                    trafficViews.setNeeded(reflectionIndex, needed);
                    return ReflectionRequirement{needed, 64};
                },
                [&](const BusSimulation&, std::size_t cameraIndex, std::size_t reflectionIndex,
                    int, int) {
                    if (cameraIndex != reflectionIndex + 1) {
                        std::cerr << "reflection camera and target indices were confused\n";
                        passed = false;
                    }
                    ++draws;
                    trafficViews.setView(reflectionIndex,
                                         {{10.0 * (reflectionIndex + 1), 0, 0}, {}});
                },
                [](int, int) {});
            const auto snapshot = trafficViews.snapshot();
            if ((frame == 0 && (snapshot.size() != 1 || snapshot[0].position[0] != 10)) ||
                (frame == 1 && (snapshot.size() != 2 || snapshot[0].position[0] != 10 ||
                                snapshot[1].position[0] != 20)) ||
                (frame == 2 && (snapshot.size() != 1 || snapshot[0].position[0] != 20)) ||
                (frame == 3 && (draws != 0 || snapshot.size() != 2 ||
                                snapshot[0].position[0] != 10 || snapshot[1].position[0] != 20)) ||
                (frame == 4 && (draws != 1 || snapshot.size() != 2))) {
                std::cerr << "staggered reflection views were lost, retained while hidden, or "
                             "not restored on reappearance\n";
                passed = false;
            }
        }
        trafficViews.clear();
        if (!trafficViews.snapshot().empty()) {
            std::cerr << "reflection traffic views survived a camera reset\n";
            passed = false;
        }
        staggeredReflections.destroy();
    }
    if (!setReflectionInterval("1")) {
        std::cerr << "could not restore the reflection interval after its regression\n";
        passed = false;
    }
    if (glGetError() != GL_NO_ERROR) {
        std::cerr << "OpenGL material probe failed\n";
        passed = false;
    }
    glFinish();
    openbus::rendering::beginGpuTimingFrame();
    if (glGetError() != GL_NO_ERROR) {
        std::cerr << "GPU timer query polling produced an OpenGL error\n";
        passed = false;
    }
    const GLuint textures[] = {blue, white, mask, red, black, normalMap, inverseNormalMap,
                               environment};
    glDeleteTextures(7, textures);
    pglDeleteBuffers(1, &indexBuffer);
    pglDeleteBuffers(1, &buffer);
    setVehicleSpotlights({testSpotlight});
    setMapLights({testMapLight}, 1.0);
    setStaticMapLights(staticCache, {testMapLight}, {0}, {{{0, 0, -2}, 0}}, 1.0);
    shutdownCoreRenderer();
    if (!initializeCoreRenderer()) {
        std::cerr << "renderer reinitialization failed\n";
        passed = false;
    } else {
        setVehicleSpotlights({testSpotlight});
        setMapLights({testMapLight}, 1.0);
        setStaticMapLights(staticCache, {testMapLight}, {0}, {{{0, 0, -2}, 0}}, 1.0);
        const auto recreatedStats = lightingBufferUploadStats();
        if (recreatedStats.vehicleUploads != 1 || recreatedStats.mapUploads != 2 ||
            recreatedStats.staticMapUploads != 1 || glGetError() != GL_NO_ERROR) {
            std::cerr << "light buffers were not restored after renderer reinitialization\n";
            passed = false;
        }
        releaseStaticMapLights(staticCache);
        shutdownCoreRenderer();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return passed ? 0 : 1;
}
