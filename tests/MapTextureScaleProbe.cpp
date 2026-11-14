#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "render/CoreRenderer.h"
#include "render/OpenGLFunctions.h"
#include "render/map/MapRendererInternal.h"

#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireTextureSize(GLuint texture, GLint expectedWidth, GLint expectedHeight) {
    glBindTexture(GL_TEXTURE_2D, texture);
    GLint width = 0;
    GLint height = 0;
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
    require(width == expectedWidth && height == expectedHeight,
            "Map texture upload used the wrong scaled dimensions");
    require(glGetError() == GL_NO_ERROR, "Map texture size query produced an OpenGL error");
}

} // namespace

int main() {
    if (glfwInit() != GLFW_TRUE) {
        return 77;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "Map texture scale probe", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 77;
    }
    glfwMakeContextCurrent(window);

    GLuint halfScaleTexture = 0;
    GLuint nativeScaleTexture = 0;
    bool rendererInitialized = false;
    int result = 0;
    try {
        require(loadOpenGLFunctions(),
                "Could not load OpenGL functions for map texture probe");
        require(openbus::rendering::initializeCoreRenderer(),
                "Could not initialize renderer for map texture probe");
        rendererInitialized = true;

        openbus::rendering::Image image;
        image.width = 2;
        image.height = 2;
        image.rgba = {0, 0, 0, 255, 64, 64, 64, 255,
                      128, 128, 128, 255, 255, 255, 255, 255};

        configureMapTextureScale(0.5);
        halfScaleTexture = openbus::rendering::map_detail::uploadTextureImage(image);
        require(halfScaleTexture != 0, "Could not upload half-scale map texture");
        requireTextureSize(halfScaleTexture, 1, 1);

        configureMapTextureScale(1.0);
        nativeScaleTexture = openbus::rendering::map_detail::uploadTextureImage(image);
        require(nativeScaleTexture != 0, "Could not upload native-scale map texture");
        requireTextureSize(nativeScaleTexture, 2, 2);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }

    if (halfScaleTexture != 0) {
        glDeleteTextures(1, &halfScaleTexture);
    }
    if (nativeScaleTexture != 0) {
        glDeleteTextures(1, &nativeScaleTexture);
    }
    if (rendererInitialized) {
        openbus::rendering::shutdownCoreRenderer();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return result;
}
