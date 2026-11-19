#pragma once

#include "render/RendererFramePresenter.h"

struct GLFWwindow;

namespace openbus::rendering {

// Non-owning presenter for the current GLFW OpenGL window.
class OpenGLFramePresenter final : public RendererFramePresenter {
  public:
    explicit OpenGLFramePresenter(GLFWwindow* window);

    [[nodiscard]] RendererBackend backend() const noexcept override {
        return RendererBackend::OpenGL;
    }
    void resize(std::uint32_t width, std::uint32_t height) override;
    [[nodiscard]] bool isSuspended() const noexcept override;
    void setVerticalSync(bool enabled) noexcept override;
    void present() override;
    void waitForIdle() override;

  private:
    GLFWwindow* window_;
    bool suspended_ = false;
};

} // namespace openbus::rendering
