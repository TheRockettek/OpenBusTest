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
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

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
    if (pixel != texturedPixel) {
        std::cerr << "text texture RGB was not preserved\n";
        passed = false;
    }
    material = {};
    material.freeTexture = blue;
    material.useFreeTexture = true;
    pixel = draw(material, {1, 1, 1}, 1.0);
    if (pixel != texturedPixel) {
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
    if (glGetError() != GL_NO_ERROR) {
        std::cerr << "OpenGL material probe failed\n";
        passed = false;
    }
    const GLuint textures[] = {blue, white, mask, red, black};
    glDeleteTextures(5, textures);
    pglDeleteBuffers(1, &buffer);
    shutdownCoreRenderer();
    glfwDestroyWindow(window);
    glfwTerminate();
    return passed ? 0 : 1;
}
