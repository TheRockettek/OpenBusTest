#include "render/CoreRenderer.h"
#include "render/OpenGLFunctions.h"

#include <GLFW/glfw3.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

using namespace openbus::rendering;

int main() {
    if (!glfwInit())
        return 77;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(128, 128, "Precipitation probe", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    if (!loadOpenGLFunctions() || !initializeCoreRenderer()) {
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    glViewport(0, 0, 128, 128);
    glDisable(GL_DITHER);
    setPerspective(128, 128, 90);
    lookAt(0, 0, 0, 0, 1, 0);
    using Image = std::array<std::uint8_t, 128 * 128 * 4>;
    const auto render = [](int type, double rate, double time, double depth = 1.0,
                           double wind = 0.0, double brightness = 1.0,
                           double visibility = 100000.0) {
        glClearColor(0.1F, 0.15F, 0.2F, 1.0F);
        glClearDepth(depth);
        glDepthMask(GL_TRUE);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        drawWeatherPrecipitation(type, rate, time, 90.0, wind, brightness, visibility);
        Image image = {};
        glReadPixels(0, 0, 128, 128, GL_RGBA, GL_UNSIGNED_BYTE, image.data());
        return image;
    };
    const auto difference = [](const Image& a, const Image& b) {
        std::uint64_t total = 0;
        for (std::size_t i = 0; i < a.size(); ++i)
            total += std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i]));
        return total;
    };
    bool passed = true;
    const auto check = [&](bool condition, const char* message) {
        if (!condition) {
            std::cerr << message << '\n';
            passed = false;
        }
    };
    const auto clear = render(0, 1, 0);
    const auto rain = render(1, 1, 0);
    const auto snow = render(2, 1, 0);
    check(difference(clear, rain) > 10000, "rain did not render");
    check(difference(clear, snow) > 10000, "snow did not render");
    check(difference(clear, render(1, 0, 0)) == 0, "zero precipitation rendered");
    check(difference(clear, render(1, 1, 0, 0)) == 0, "rain ignored scene depth");
    check(difference(clear, render(2, 1, 0, 0)) == 0, "snow ignored scene depth");
    check(difference(clear, render(1, 0.25, 0)) < difference(clear, rain),
          "rain intensity did not affect particle density");
    glClearColor(0.1F, 0.15F, 0.2F, 1.0F);
    glClearDepth(1.0);
    glDepthMask(GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_SCISSOR_TEST);
    glScissor(0, 0, 64, 128);
    glClearDepth(0.0);
    glClear(GL_DEPTH_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);
    drawWeatherPrecipitation(1, 1, 0, 90, 0, 1, 100000);
    Image partial = {};
    glReadPixels(0, 0, 128, 128, GL_RGBA, GL_UNSIGNED_BYTE, partial.data());
    std::uint64_t leftDifference = 0, rightDifference = 0;
    for (std::size_t y = 0; y < 128; ++y) {
        for (std::size_t x = 0; x < 128; ++x) {
            for (std::size_t c = 0; c < 4; ++c) {
                const std::size_t i = (y * 128 + x) * 4 + c;
                const auto delta =
                    std::abs(static_cast<int>(partial[i]) - static_cast<int>(clear[i]));
                if (x < 64)
                    leftDifference += delta;
                else
                    rightDifference += delta;
            }
        }
    }
    check(leftDifference == 0 && rightDifference > 5000,
          "precipitation was not selectively occluded");
    check(difference(rain, render(1, 1, 0)) == 0, "paused precipitation changed");
    check(difference(rain, render(1, 1, 0.1)) > 10000, "rain did not fall");
    check(difference(snow, render(2, 1, 0.1)) > 1000, "snow did not fall");
    check(difference(render(1, 1, 1), render(1, 1, 1, 1, 10)) > 10000,
          "wind did not move precipitation");
    check(difference(clear, render(1, 1, 0, 1, 0, 0.1)) < difference(clear, rain),
          "brightness did not affect precipitation");
    check(difference(clear, render(1, 1, 0, 1, 0, 1, 1)) < difference(clear, rain),
          "fog did not obscure precipitation");
    setWeatherFog(1.0, 5.0);
    setWeatherFogView(modelViewMatrix(), -100.0);
    check(difference(rain, render(1, 1, 0, 1, 0, 1, 1)) < 100,
          "precipitation above the fog layer remained obscured");
    setWeatherFogView(modelViewMatrix(), 0.0);
    check(difference(clear, render(1, 1, 0, 1, 0, 1, 1)) < difference(clear, rain),
          "precipitation inside the fog layer was not obscured");
    setWeatherFog(std::nullopt);
    lookAt(0, 0, 0, 1, 0, 0);
    check(difference(rain, render(1, 1, 0)) > 10000,
          "camera rotation did not affect precipitation");
    lookAt(2, 0, 0, 2, 1, 0);
    check(difference(rain, render(1, 1, 0)) > 10000,
          "camera position did not affect precipitation");
    lookAt(24, 0, 0, 24, 1, 0);
    check(difference(rain, render(1, 1, 0)) < 100,
          "precipitation is not anchored to the periodic world field");
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_GREATER);
    pglBlendFuncSeparate(GL_ONE, GL_ZERO, GL_ZERO, GL_ONE);
    constexpr GLenum reverseSubtract = 0x800B;
    pglBlendEquationSeparate(reverseSubtract, GL_FUNC_ADD);
    glPointSize(3.0F);
    GLint previousProgram = 0, previousVertexArray = 0;
    constexpr GLenum currentProgramEnum = 0x8B8D;
    constexpr GLenum vertexArrayBindingEnum = 0x85B5;
    glGetIntegerv(currentProgramEnum, &previousProgram);
    glGetIntegerv(vertexArrayBindingEnum, &previousVertexArray);
    drawWeatherPrecipitation(2, 1, 0, {}, {}, {}, {});
    GLboolean depthMask = GL_FALSE;
    GLint depthFunction = 0, srcRgb = 0, dstRgb = 0, srcAlpha = 0, dstAlpha = 0;
    GLint equationRgb = 0, equationAlpha = 0, program = 0, vertexArray = 0;
    GLfloat pointSize = 0;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
    glGetIntegerv(GL_DEPTH_FUNC, &depthFunction);
    glGetIntegerv(GL_BLEND_SRC_RGB, &srcRgb);
    glGetIntegerv(GL_BLEND_DST_RGB, &dstRgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &srcAlpha);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &dstAlpha);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &equationRgb);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &equationAlpha);
    glGetIntegerv(currentProgramEnum, &program);
    glGetIntegerv(vertexArrayBindingEnum, &vertexArray);
    glGetFloatv(GL_POINT_SIZE, &pointSize);
    check(!glIsEnabled(GL_DEPTH_TEST) && glIsEnabled(GL_BLEND) &&
              glIsEnabled(GL_PROGRAM_POINT_SIZE) && depthMask && depthFunction == GL_GREATER &&
              srcRgb == GL_ONE && dstRgb == GL_ZERO && srcAlpha == GL_ZERO && dstAlpha == GL_ONE &&
              equationRgb == reverseSubtract && equationAlpha == GL_FUNC_ADD &&
              program == previousProgram && vertexArray == previousVertexArray && pointSize == 3.0F,
          "precipitation changed OpenGL state");
    bool invalid = false;
    try {
        drawWeatherPrecipitation(1, 1.1, 0, {}, {}, {}, {});
    } catch (const std::invalid_argument&) {
        invalid = true;
    }
    check(invalid, "invalid precipitation rate was accepted");
    check(glGetError() == GL_NO_ERROR, "precipitation generated an OpenGL error");
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    return passed ? 0 : 1;
}
