#ifndef NOMINMAX
#define NOMINMAX
#endif
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

struct Vertex {
    float position[3];
    float normal[3];
};

void appendFace(std::vector<Vertex>& vertices, const std::array<float, 3>& normal,
                const std::array<std::array<float, 3>, 4>& corners) {
    constexpr std::array<std::size_t, 6> order = {0, 1, 2, 0, 2, 3};
    for (const std::size_t index : order) {
        const auto& position = corners[index];
        vertices.push_back(
            {{position[0], position[1], position[2]}, {normal[0], normal[1], normal[2]}});
    }
}

std::vector<Vertex> makeCube() {
    constexpr float n = -1.0f;
    constexpr float p = 1.0f;
    std::vector<Vertex> vertices;
    vertices.reserve(36);
    appendFace(vertices, {0.0f, 0.0f, 1.0f}, {{{n, n, p}, {p, n, p}, {p, p, p}, {n, p, p}}});
    appendFace(vertices, {0.0f, 0.0f, -1.0f}, {{{p, n, n}, {n, n, n}, {n, p, n}, {p, p, n}}});
    appendFace(vertices, {1.0f, 0.0f, 0.0f}, {{{p, n, p}, {p, n, n}, {p, p, n}, {p, p, p}}});
    appendFace(vertices, {-1.0f, 0.0f, 0.0f}, {{{n, n, n}, {n, n, p}, {n, p, p}, {n, p, n}}});
    appendFace(vertices, {0.0f, 1.0f, 0.0f}, {{{n, p, p}, {p, p, p}, {p, p, n}, {n, p, n}}});
    appendFace(vertices, {0.0f, -1.0f, 0.0f}, {{{n, n, n}, {p, n, n}, {p, n, p}, {n, n, p}}});
    return vertices;
}

bgfx::ShaderHandle loadShader(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input) {
        std::cerr << "Unable to open bgfx shader: " << path.string() << '\n';
        return BGFX_INVALID_HANDLE;
    }
    const std::streamsize length = input.tellg();
    if (length <= 0 ||
        static_cast<std::uint64_t>(length) > std::numeric_limits<std::uint32_t>::max()) {
        std::cerr << "Invalid bgfx shader size: " << path.string() << '\n';
        return BGFX_INVALID_HANDLE;
    }
    input.seekg(0, std::ios::beg);
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(length));
    if (!input.read(reinterpret_cast<char*>(bytes.data()), length)) {
        std::cerr << "Unable to read bgfx shader: " << path.string() << '\n';
        return BGFX_INVALID_HANDLE;
    }
    return bgfx::createShader(bgfx::copy(bytes.data(), static_cast<std::uint32_t>(bytes.size())));
}

class BgfxLightingProbe {
  public:
    ~BgfxLightingProbe() {
        shutdown();
    }

    bool initialize(const std::filesystem::path& shaderDirectory) {
        if (!glfwInit()) {
            std::cerr << "GLFW initialization failed\n";
            return false;
        }
        glfwInitialized_ = true;
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        window_ = glfwCreateWindow(1100, 760, "OpenBus bgfx GLSL lighting probe", nullptr, nullptr);
        if (window_ == nullptr) {
            std::cerr << "Unable to create the GLFW native window\n";
            return false;
        }

        bgfx::Init init;
        init.type = bgfx::RendererType::OpenGL;
        init.platformData.nwh = glfwGetWin32Window(window_);
        init.resolution.width = 1100;
        init.resolution.height = 760;
        init.resolution.reset = BGFX_RESET_VSYNC;
        if (!bgfx::init(init)) {
            std::cerr << "bgfx OpenGL renderer initialization failed\n";
            return false;
        }
        bgfxInitialized_ = true;
        bgfx::setDebug(BGFX_DEBUG_TEXT);
        bgfx::setViewName(0, "Lit geometry");
        bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x526f91ff, 1.0f, 0);

        const bgfx::ShaderHandle vertexShader = loadShader(shaderDirectory / "vs_lit_cube.bin");
        const bgfx::ShaderHandle fragmentShader = loadShader(shaderDirectory / "fs_lit_cube.bin");
        if (!bgfx::isValid(vertexShader) || !bgfx::isValid(fragmentShader)) {
            if (bgfx::isValid(vertexShader)) {
                bgfx::destroy(vertexShader);
            }
            if (bgfx::isValid(fragmentShader)) {
                bgfx::destroy(fragmentShader);
            }
            return false;
        }
        program_ = bgfx::createProgram(vertexShader, fragmentShader, true);
        if (!bgfx::isValid(program_)) {
            std::cerr << "Unable to create bgfx lighting program\n";
            return false;
        }

        bgfx::VertexLayout layout;
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Normal, 3, bgfx::AttribType::Float)
            .end();
        const std::vector<Vertex> vertices = makeCube();
        vertexBuffer_ = bgfx::createVertexBuffer(
            bgfx::copy(vertices.data(),
                       static_cast<std::uint32_t>(vertices.size() * sizeof(Vertex))),
            layout);
        lightDirection_ = bgfx::createUniform("u_lightDirection", bgfx::UniformType::Vec4);
        baseColor_ = bgfx::createUniform("u_baseColor", bgfx::UniformType::Vec4);
        lightParameters_ = bgfx::createUniform("u_lightParameters", bgfx::UniformType::Vec4);
        if (!bgfx::isValid(vertexBuffer_) || !bgfx::isValid(lightDirection_) ||
            !bgfx::isValid(baseColor_) || !bgfx::isValid(lightParameters_)) {
            std::cerr << "Unable to create bgfx lighting resources\n";
            return false;
        }
        return true;
    }

    int run() {
        int previousWidth = 0;
        int previousHeight = 0;
        while (!glfwWindowShouldClose(window_)) {
            glfwPollEvents();
            int width = 0;
            int height = 0;
            glfwGetFramebufferSize(window_, &width, &height);
            if (width <= 0 || height <= 0) {
                bgfx::frame();
                continue;
            }
            if (width != previousWidth || height != previousHeight) {
                previousWidth = width;
                previousHeight = height;
                bgfx::reset(static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height),
                            BGFX_RESET_VSYNC);
                bgfx::setViewRect(0, 0, 0, static_cast<std::uint16_t>(width),
                                  static_cast<std::uint16_t>(height));
            }

            const bx::Vec3 eye = {3.4f, -5.0f, -4.0f};
            const bx::Vec3 target = {0.0f, 0.0f, 0.0f};
            const bx::Vec3 up = {0.0f, 0.0f, 1.0f};
            float view[16];
            float projection[16];
            bx::mtxLookAt(view, eye, target, up);
            bx::mtxProj(projection, 55.0f, static_cast<float>(width) / height, 0.1f, 100.0f,
                        bgfx::getCaps()->homogeneousDepth);
            bgfx::setViewTransform(0, view, projection);
            bgfx::touch(0);

            constexpr float model[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                         0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
            constexpr float lightDirection[4] = {-0.35f, -0.55f, -0.76f, 0.0f};
            constexpr float baseColor[4] = {0.92f, 0.32f, 0.12f, 1.0f};
            constexpr float lightParameters[4] = {0.22f, 0.78f, 0.0f, 0.0f};
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, vertexBuffer_);
            bgfx::setUniform(lightDirection_, lightDirection);
            bgfx::setUniform(baseColor_, baseColor);
            bgfx::setUniform(lightParameters_, lightParameters);
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z |
                           BGFX_STATE_DEPTH_TEST_LESS);
            bgfx::submit(0, program_);
            bgfx::dbgTextClear();
            bgfx::dbgTextPrintf(1, 1, 0x0f, "OpenBus bgfx / GLSL 330 lighting");
            bgfx::dbgTextPrintf(1, 2, 0x0f, "Ambient + directional diffuse; GLFW owns input only.");
            bgfx::frame();
        }
        return 0;
    }

  private:
    void shutdown() {
        if (bgfxInitialized_) {
            if (bgfx::isValid(lightParameters_)) {
                bgfx::destroy(lightParameters_);
                lightParameters_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(baseColor_)) {
                bgfx::destroy(baseColor_);
                baseColor_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(lightDirection_)) {
                bgfx::destroy(lightDirection_);
                lightDirection_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(vertexBuffer_)) {
                bgfx::destroy(vertexBuffer_);
                vertexBuffer_ = BGFX_INVALID_HANDLE;
            }
            if (bgfx::isValid(program_)) {
                bgfx::destroy(program_);
                program_ = BGFX_INVALID_HANDLE;
            }
            bgfx::shutdown();
            bgfxInitialized_ = false;
        }
        if (window_ != nullptr) {
            glfwDestroyWindow(window_);
            window_ = nullptr;
        }
        if (glfwInitialized_) {
            glfwTerminate();
            glfwInitialized_ = false;
        }
    }

    GLFWwindow* window_ = nullptr;
    bool glfwInitialized_ = false;
    bool bgfxInitialized_ = false;
    bgfx::ProgramHandle program_ = BGFX_INVALID_HANDLE;
    bgfx::VertexBufferHandle vertexBuffer_ = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle lightDirection_ = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle baseColor_ = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle lightParameters_ = BGFX_INVALID_HANDLE;
};

} // namespace

int main(int argc, char** argv) {
    if (argc < 1 || argv[0] == nullptr) {
        return 1;
    }
    std::error_code error;
    const std::filesystem::path executable = std::filesystem::absolute(argv[0], error);
    if (error) {
        std::cerr << "Unable to resolve probe executable path\n";
        return 1;
    }
    BgfxLightingProbe probe;
    if (!probe.initialize(executable.parent_path() / "bgfx-shaders")) {
        return 1;
    }
    return probe.run();
}
