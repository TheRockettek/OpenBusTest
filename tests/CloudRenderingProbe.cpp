#include "render/CoreRenderer.h"
#include "render/OpenGLFunctions.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>

using namespace openbus::rendering;
using openbus::WeatherCloudType;

int main() {
    if (!glfwInit())
        return 77;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "Cloud probe", nullptr, nullptr);
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
    glViewport(0, 0, 64, 64);
    glDisable(GL_DITHER);
    setPerspective(64, 64, 90);
    lookAt(0, 0, 0, 0, 1, 1);
    using Image = std::array<std::uint8_t, 64 * 64 * 4>;
    const auto render = [](std::optional<WeatherCloudType> type, double time,
                           std::optional<double> brightness = std::nullopt,
                           std::optional<double> visibility = std::nullopt,
                           std::optional<double> windDirection = std::nullopt,
                           std::optional<double> windSpeed = std::nullopt,
                           double cloudBase = 1200.0) {
        glClearColor(0.45F, 0.65F, 0.88F, 1.0F);
        glDepthMask(GL_TRUE);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        drawWeatherClouds(type, cloudBase, windDirection, windSpeed, visibility, brightness, 0.0,
                          time);
        Image image = {};
        glReadPixels(0, 0, 64, 64, GL_RGBA, GL_UNSIGNED_BYTE, image.data());
        return image;
    };
    const auto difference = [](const Image& a, const Image& b) {
        std::uint64_t total = 0;
        for (std::size_t i = 0; i < a.size(); ++i) {
            total += static_cast<std::uint64_t>(
                std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i])));
        }
        return total;
    };
    bool passed = true;
    const auto check = [&](bool condition, const char* message) {
        if (!condition) {
            std::cerr << message << '\n';
            passed = false;
        }
    };
    const Image clear = render(std::nullopt, 0);
    const Image cumulus = render(WeatherCloudType::Cumulus1, 0);
    const Image medium = render(WeatherCloudType::Cumulus2, 0);
    const Image dense = render(WeatherCloudType::Cumulus3, 0);
    const Image overcast = render(WeatherCloudType::Overcast1, 0);
    check(difference(clear, cumulus) > 500, "cumulus did not render");
    check(difference(clear, medium) > difference(clear, cumulus) &&
              difference(clear, dense) > difference(clear, medium),
          "cumulus cloud styles do not vary");
    check(difference(clear, overcast) > difference(clear, dense), "overcast is not denser");
    check(difference(dense, render(WeatherCloudType::Cumulus3, 100, std::nullopt, std::nullopt, 90,
                                   20)) > 500,
          "wind did not move clouds");
    check(difference(dense, render(WeatherCloudType::Cumulus3, 100, {}, {}, {}, 20)) == 0 &&
              difference(dense, render(WeatherCloudType::Cumulus3, 100, {}, {}, 90, {})) == 0,
          "incomplete wind observations moved clouds");
    check(difference(dense, render(WeatherCloudType::Cumulus3, 0, {}, {}, {}, {}, 3000)) > 500,
          "cloud base did not affect the cloud layer");
    const Image maximumCloud =
        render(WeatherCloudType::Cumulus3, 0, {}, {}, {}, {},
               openbus::MAX_CLOUD_BASE_ABOVE_GROUND_METERS);
    lookAt(0, 0, -500, 0, 1, -499);
    check(difference(maximumCloud,
                     render(WeatherCloudType::Cumulus3, 0, {}, {}, {}, {},
                            openbus::MAX_CLOUD_BASE_ABOVE_GROUND_METERS)) == 0,
          "camera-relative extreme cloud height was not safely bounded");
    lookAt(0, 0, 0, 0, 1, 1);
    lookAt(500, 0, 0, 500, 1, 1);
    check(difference(dense, render(WeatherCloudType::Cumulus3, 25, std::nullopt, std::nullopt, 270,
                                   20)) < 100,
          "wind direction and camera translation do not use world coordinates");
    lookAt(0, 0, 0, 1, 0, 1);
    check(difference(dense, render(WeatherCloudType::Cumulus3, 0)) > 500,
          "clouds did not follow camera orientation");
    lookAt(0, 0, 0, 0, 1, 1);
    check(difference(clear, render(WeatherCloudType::Cumulus3, 0, std::nullopt, 1.0)) <
              difference(clear, dense) / 10,
          "visibility did not obscure distant clouds");
    setWeatherFog(1.0, 0.0);
    setWeatherFogView(modelViewMatrix(), 0.0);
    check(difference(dense, render(WeatherCloudType::Cumulus3, 0, {}, 1.0)) < 100,
          "clouds above an empty fog layer remained obscured");
    setWeatherFog(1.0, 1000.0);
    check(difference(clear, render(WeatherCloudType::Cumulus3, 0, {}, 1.0)) <
              difference(clear, dense) / 10,
          "clouds did not integrate the fog-layer path");
    setWeatherFog(std::nullopt);
    const Image dark = render(WeatherCloudType::Cumulus3, 0, 0.25);
    check(dark[(32 * 64 + 32) * 4] < dense[(32 * 64 + 32) * 4],
          "brightness did not affect cloud background");
    lookAt(0, 0, 0, 0, 1, -1);
    const Image below = render(WeatherCloudType::Overcast1, 0);
    check(difference(clear, below) < 200, "clouds appeared below the horizon");
    lookAt(0, 0, 0, 0, 1, 1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    glDepthMask(GL_TRUE);
    glClear(GL_DEPTH_BUFFER_BIT);
    glDepthMask(GL_FALSE);
    lookAt(0, 0, -500, 0, 1, -499);
    drawWeatherClouds(WeatherCloudType::Overcast1, openbus::MAX_CLOUD_BASE_ABOVE_GROUND_METERS,
                      {}, {}, {}, {}, 0, 0);
    GLboolean depthMask = GL_TRUE;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
    float depth = 0;
    glReadPixels(32, 32, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    check(glIsEnabled(GL_DEPTH_TEST) && glIsEnabled(GL_BLEND) && glIsEnabled(GL_CULL_FACE) &&
              depthMask == GL_FALSE && depth > 0.99F,
          "cloud draw changed depth or GL state");
    check(glGetError() == GL_NO_ERROR, "cloud draw generated an OpenGL error");
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    return passed ? 0 : 1;
}
