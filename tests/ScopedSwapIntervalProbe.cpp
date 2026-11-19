#include "app/ScopedSwapInterval.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace {

class ProbePresenter final : public openbus::rendering::RendererFramePresenter {
  public:
    [[nodiscard]] openbus::rendering::RendererBackend backend() const noexcept override {
        return openbus::rendering::RendererBackend::OpenGL;
    }

    void resize(std::uint32_t, std::uint32_t) override {}

    [[nodiscard]] bool isSuspended() const noexcept override {
        return false;
    }

    void setVerticalSync(bool enabled) noexcept override {
        if (requestCount_ < requests_.size()) {
            requests_[requestCount_++] = enabled;
        }
    }

    void present() override {}
    void waitForIdle() override {}

    [[nodiscard]] std::size_t requestCount() const noexcept {
        return requestCount_;
    }

    [[nodiscard]] bool request(std::size_t index) const {
        return requests_.at(index);
    }

  private:
    std::array<bool, 2> requests_ = {};
    std::size_t requestCount_ = 0;
};

void checkRestoration(bool gameplayVsync, bool throwDuringMenu) {
    ProbePresenter presenter;
    try {
        const openbus::app::ScopedSwapInterval menu(presenter, true, gameplayVsync);
        if (presenter.requestCount() != 1 || !presenter.request(0)) {
            throw std::runtime_error("menu did not request VSync");
        }
        if (throwDuringMenu) {
            throw std::runtime_error("simulated menu failure");
        }
    } catch (const std::runtime_error&) {
        if (!throwDuringMenu) {
            throw;
        }
    }
    if (presenter.requestCount() != 2 || !presenter.request(0) ||
        presenter.request(1) != gameplayVsync) {
        throw std::runtime_error("menu did not restore the gameplay VSync setting");
    }
}

} // namespace

int main() {
    checkRestoration(false, false);
    checkRestoration(true, false);
    checkRestoration(false, true);
    checkRestoration(true, true);
}
