#pragma once

#include <cstdint>

namespace openbus::rendering {

enum class RendererBackend {
    OpenGL,
    Diligent,
};

// Backend-neutral presentation lifecycle. Scene rendering and resource ownership
// remain with the selected renderer backend.
class RendererFramePresenter {
  public:
    virtual ~RendererFramePresenter() = default;

    [[nodiscard]] virtual RendererBackend backend() const noexcept = 0;
    virtual void resize(std::uint32_t width, std::uint32_t height) = 0;
    [[nodiscard]] virtual bool isSuspended() const noexcept = 0;
    virtual void setVerticalSync(bool enabled) noexcept = 0;
    virtual void present() = 0;
    virtual void waitForIdle() = 0;
};

} // namespace openbus::rendering
