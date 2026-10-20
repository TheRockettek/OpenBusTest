#include "render/CoreRenderer.h"
#include "render/MatrixMath.h"
#include "render/OpenGLFunctions.h"

#include <GLFW/glfw3.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

using namespace openbus::rendering;

int main() {
    if (!glfwInit()) {
        return 77;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(32, 32, "Material probe", nullptr, nullptr);
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
    setPerspective(32, 32, 90);
    setModelViewMatrix(identityMatrix());
    glViewport(0, 0, 32, 32);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_DITHER);

    constexpr std::array<float, 27> vertices = {
        -2, -2, -1, 0, 0, 0, 0, 0, -1,
         2, -2, -1, 1, 0, 0, 0, 0, -1,
         0,  2, -1, 0, 1, 0, 0, 0, -1};
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
    ModelMaterial material;
    material.texture = blue;
    material.textured = true;
    const auto texturedPixel = draw(material, {1, 1, 1}, 1.0);
    material = {};
    material.useTextTexture = true;
    material.textTexture = blue;
    auto pixel = draw(material, {1, 1, 1}, 1.0);
    if (pixel != texturedPixel || pixel[2] >= 112) {
        std::cerr << "text texture was not shaded like the material texture it replaces\n";
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
    material.texture = blue;
    material.textured = true;
    const auto basePixel = draw(material, {1, 1, 1}, 1.0);
    material.textured = false;
    material.freeTexture = blue;
    material.useFreeTexture = true;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (pixel != basePixel || pixel[2] >= 112) {
        std::cerr << "file-selected free texture bypassed normal surface lighting\n";
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
    if (glGetError() != GL_NO_ERROR) {
        std::cerr << "OpenGL material probe failed\n";
        passed = false;
    }
    const GLuint textures[] = {blue, white, mask};
    glDeleteTextures(3, textures);
    pglDeleteBuffers(1, &buffer);
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    return passed ? 0 : 1;
}
