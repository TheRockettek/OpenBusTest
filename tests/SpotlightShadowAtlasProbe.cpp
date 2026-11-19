#include "render/SpotlightShadowAtlas.h"
#include "render/OpenGLFunctions.h"

#include <GLFW/glfw3.h>

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace {
void failFramebufferAllocation(GLsizei, GLuint* framebuffer) {
    *framebuffer = 0;
}

GLuint compileShader(GLenum type, const char* source) {
    const GLuint shader = pglCreateShader(type);
    pglShaderSource(shader, 1, &source, nullptr);
    pglCompileShader(shader);
    GLint success = 0;
    pglGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (success == GL_FALSE) {
        std::array<char, 2048> log = {};
        pglGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), nullptr, log.data());
        std::cerr << "Shadow probe shader compilation failed: " << log.data() << '\n';
        pglDeleteShader(shader);
        throw std::runtime_error("Shadow probe shader compilation failed");
    }
    return shader;
}
} // namespace

int main() {
    using namespace openbus::rendering;
    if (!glfwInit())
        return 77;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "Shadow atlas probe", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);
    if (!loadOpenGLFunctions()) {
        std::cerr << "Could not load OpenGL functions\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    bool ok = true;
    {
        SpotlightShadowAtlas atlas;
        GLint initialDrawBuffer = 0;
        glGetIntegerv(GL_DRAW_BUFFER, &initialDrawBuffer);
        const auto framebufferGenerator = pglGenFramebuffers;
        pglGenFramebuffers = failFramebufferAllocation;
        try {
            atlas.initialize(32, 3);
            ok = false;
        } catch (const std::runtime_error&) {
        }
        pglGenFramebuffers = framebufferGenerator;
        GLint currentDrawBuffer = 0;
        glGetIntegerv(GL_DRAW_BUFFER, &currentDrawBuffer);
        if (atlas.texture() != 0 || atlas.framebuffer() != 0 ||
            currentDrawBuffer != initialDrawBuffer || glGetError() != GL_NO_ERROR)
            ok = false;
        try {
            atlas.initialize(32, 65);
            ok = false;
        } catch (const std::invalid_argument&) {
        }
        constexpr GLenum pixelUnpackBuffer = 0x88EC;
        constexpr GLenum pixelUnpackBufferBinding = 0x88EF;
        GLuint unpackBuffer = 0;
        pglGenBuffers(1, &unpackBuffer);
        pglBindBuffer(pixelUnpackBuffer, unpackBuffer);
        pglBufferData(pixelUnpackBuffer, 4, nullptr, GL_STATIC_DRAW);
        try {
            atlas.initialize(32, 3);
            atlas.shutdown();
        } catch (const std::runtime_error& error) {
            std::cerr << "Atlas allocation with unpack buffer bound failed: " << error.what()
                      << '\n';
            ok = false;
        }
        GLint currentUnpackBuffer = 0;
        glGetIntegerv(pixelUnpackBufferBinding, &currentUnpackBuffer);
        if (currentUnpackBuffer != static_cast<GLint>(unpackBuffer))
            ok = false;
        pglGenFramebuffers = failFramebufferAllocation;
        try {
            atlas.initialize(32, 3);
            ok = false;
        } catch (const std::runtime_error&) {
        }
        pglGenFramebuffers = framebufferGenerator;
        glGetIntegerv(pixelUnpackBufferBinding, &currentUnpackBuffer);
        if (currentUnpackBuffer != static_cast<GLint>(unpackBuffer) ||
            glGetError() != GL_NO_ERROR)
            ok = false;
        pglBindBuffer(pixelUnpackBuffer, 0);
        pglDeleteBuffers(1, &unpackBuffer);
        atlas.initialize(32, 3);
        if (atlas.texture() == 0 || atlas.framebuffer() == 0 || atlas.columns() != 2 ||
            atlas.rows() != 2 || atlas.capacity() != 3)
            ok = false;
        try {
            SpotlightShadowAtlas::TileScope invalid(atlas, 3);
            ok = false;
        } catch (const std::invalid_argument&) {
        }
        pglBindFramebuffer(GL_FRAMEBUFFER, atlas.framebuffer());
        glDisable(GL_SCISSOR_TEST);
        glDepthMask(GL_TRUE);
        glClearDepth(0.4);
        glClear(GL_DEPTH_BUFFER_BIT);
        pglBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glViewport(3, 5, 23, 29);
        glEnable(GL_SCISSOR_TEST);
        glScissor(7, 9, 11, 13);
        glColorMask(GL_TRUE, GL_FALSE, GL_TRUE, GL_FALSE);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glDepthFunc(GL_GEQUAL);
        glClearDepth(0.375);
        bool enteredTile = false;
        try {
            SpotlightShadowAtlas::TileScope tile(atlas, 1);
            enteredTile = true;
            throw std::runtime_error("injected shadow draw failure");
        } catch (const std::runtime_error&) {
        }
        if (!enteredTile)
            ok = false;
        bool reportedDrawError = false;
        {
            SpotlightShadowAtlas::TileScope tile(atlas, 1);
            glEnable(0xDEAD);
            try {
                tile.finish();
            } catch (const std::runtime_error&) {
                reportedDrawError = true;
            }
        }
        if (!reportedDrawError)
            ok = false;
        for (const auto& [index, depth] :
             std::array<std::pair<int, double>, 2>{{{0, 0.2}, {2, 0.8}}}) {
            SpotlightShadowAtlas::TileScope tile(atlas, index);
            glClearDepth(depth);
            glClear(GL_DEPTH_BUFFER_BIT);
            tile.finish();
        }
        GLint drawFramebuffer = -1;
        GLint readFramebuffer = -1;
        GLint depthFunction = 0;
        std::array<GLint, 4> viewport = {};
        std::array<GLint, 4> scissor = {};
        std::array<GLboolean, 4> colorMask = {};
        GLboolean depthMask = GL_TRUE;
        GLdouble clearDepth = 0.0;
        glGetIntegerv(0x8CA6 /* GL_DRAW_FRAMEBUFFER_BINDING */, &drawFramebuffer);
        glGetIntegerv(0x8CAA /* GL_READ_FRAMEBUFFER_BINDING */, &readFramebuffer);
        glGetIntegerv(GL_VIEWPORT, viewport.data());
        glGetIntegerv(GL_SCISSOR_BOX, scissor.data());
        glGetIntegerv(GL_DEPTH_FUNC, &depthFunction);
        glGetBooleanv(GL_COLOR_WRITEMASK, colorMask.data());
        glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
        glGetDoublev(GL_DEPTH_CLEAR_VALUE, &clearDepth);
        if (drawFramebuffer != 0 || readFramebuffer != static_cast<GLint>(atlas.framebuffer()) ||
            viewport != std::array<GLint, 4>{3, 5, 23, 29} ||
            scissor != std::array<GLint, 4>{7, 9, 11, 13} ||
            colorMask != std::array<GLboolean, 4>{GL_TRUE, GL_FALSE, GL_TRUE, GL_FALSE} ||
            depthMask != GL_FALSE || depthFunction != GL_GEQUAL || clearDepth != 0.375 ||
            glIsEnabled(GL_DEPTH_TEST) != GL_FALSE || glIsEnabled(GL_SCISSOR_TEST) != GL_TRUE)
            ok = false;
        for (const auto& [x, y, expected] :
             std::array<std::array<float, 3>, 3>{{{16, 16, 0.2F}, {16, 48, 0.8F},
                                                  {48, 48, 0.4F}}}) {
            float value = 0.0F;
            glReadPixels(static_cast<GLint>(x), static_cast<GLint>(y), 1, 1, GL_DEPTH_COMPONENT,
                         GL_FLOAT, &value);
            if (std::abs(value - expected) > 0.00001F)
                ok = false;
        }
        constexpr const char* vertexSource = R"GLSL(#version 330 core
layout(location = 0) in vec2 position;
out vec2 uv;
void main() {
    uv = position * 0.5 + 0.5;
    gl_Position = vec4(position, 0.0, 1.0);
}
)GLSL";
        constexpr const char* fragmentSource = R"GLSL(#version 330 core
in vec2 uv;
uniform sampler2D maskTexture;
void main() {
    if (texture(maskTexture, uv).a < 0.5) discard;
}
)GLSL";
        const GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
        const GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
        const GLuint program = pglCreateProgram();
        pglAttachShader(program, vertexShader);
        pglAttachShader(program, fragmentShader);
        pglLinkProgram(program);
        GLint linked = 0;
        pglGetProgramiv(program, GL_LINK_STATUS, &linked);
        if (linked == GL_FALSE) {
            std::array<char, 2048> log = {};
            pglGetProgramInfoLog(program, static_cast<GLsizei>(log.size()), nullptr, log.data());
            std::cerr << "Shadow probe shader link failed: " << log.data() << '\n';
            ok = false;
        }
        pglDeleteShader(vertexShader);
        pglDeleteShader(fragmentShader);
        GLuint vertexArray = 0;
        GLuint vertexBuffer = 0;
        GLuint maskTexture = 0;
        pglGenVertexArrays(1, &vertexArray);
        pglBindVertexArray(vertexArray);
        pglGenBuffers(1, &vertexBuffer);
        pglBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        constexpr std::array<float, 6> vertices = {-1, -1, 3, -1, -1, 3};
        pglBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);
        pglEnableVertexAttribArray(0);
        pglVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        pglActiveTexture(GL_TEXTURE0);
        glGenTextures(1, &maskTexture);
        glBindTexture(GL_TEXTURE_2D, maskTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        const std::array<GLubyte, 8> mask = {255, 255, 255, 255, 255, 255, 255, 0};
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, mask.data());
        {
            SpotlightShadowAtlas::TileScope tile(atlas, 1);
            pglUseProgram(program);
            pglUniform1i(pglGetUniformLocation(program, "maskTexture"), 0);
            glDrawArrays(GL_TRIANGLES, 0, 3);
            tile.finish();
        }
        pglUseProgram(0);
        pglBindVertexArray(0);
        float opaqueDepth = 0.0F;
        float cutoutDepth = 0.0F;
        glReadPixels(40, 16, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &opaqueDepth);
        glReadPixels(56, 16, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &cutoutDepth);
        if (std::abs(opaqueDepth - 0.5F) > 0.00001F || std::abs(cutoutDepth - 1.0F) > 0.00001F)
            ok = false;
        pglDeleteBuffers(1, &vertexBuffer);
        pglDeleteVertexArrays(1, &vertexArray);
        pglDeleteProgram(program);
        glDeleteTextures(1, &maskTexture);
        if (glGetError() != GL_NO_ERROR)
            ok = false;
        pglBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        atlas.shutdown();
        atlas.initialize(4, SpotlightShadowAtlas::MAX_LIGHTS);
        if (atlas.columns() != 8 || atlas.rows() != 8 ||
            atlas.capacity() != SpotlightShadowAtlas::MAX_LIGHTS)
            ok = false;
        {
            SpotlightShadowAtlas::TileScope last(atlas, SpotlightShadowAtlas::MAX_LIGHTS - 1);
            glClearDepth(0.6);
            glClear(GL_DEPTH_BUFFER_BIT);
            last.finish();
        }
        pglBindFramebuffer(GL_READ_FRAMEBUFFER, atlas.framebuffer());
        float lastDepth = 0.0F;
        glReadPixels(30, 30, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &lastDepth);
        if (std::abs(lastDepth - 0.6F) > 0.00001F || glGetError() != GL_NO_ERROR)
            ok = false;
        pglBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        atlas.shutdown();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    if (!ok)
        std::cerr << "Shadow atlas isolation or OpenGL state restoration failed\n";
    return ok ? 0 : 1;
}
