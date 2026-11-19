#include "render/OpenGLFramePresenter.h"

#include "render/OpenGLFunctions.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

#include <stdexcept>

namespace openbus::rendering {

OpenGLFramePresenter::OpenGLFramePresenter(GLFWwindow* window) : window_(window) {
    if (window_ == nullptr) {
        throw std::invalid_argument("OpenGL frame presenter requires a GLFW window");
    }
}

void OpenGLFramePresenter::resize(std::uint32_t width, std::uint32_t height) {
    suspended_ = width == 0 || height == 0;
}

bool OpenGLFramePresenter::isSuspended() const noexcept {
    return suspended_;
}

void OpenGLFramePresenter::setVerticalSync(bool enabled) noexcept {
    glfwSwapInterval(enabled ? 1 : 0);
}

void OpenGLFramePresenter::present() {
    if (!suspended_) {
        glfwSwapBuffers(window_);
    }
}

void OpenGLFramePresenter::waitForIdle() {
    glFinish();
}

} // namespace openbus::rendering
