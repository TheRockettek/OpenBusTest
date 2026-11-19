#pragma once

#include "render/RendererFramePresenter.h"

namespace openbus::app {

class ScopedSwapInterval {
  public:
    ScopedSwapInterval(openbus::rendering::RendererFramePresenter& presenter, bool enabled,
                       const bool& restoreVsync)
        : presenter_(&presenter), restoreVsync_(&restoreVsync) {
        presenter_->setVerticalSync(enabled);
    }

    ~ScopedSwapInterval() {
        presenter_->setVerticalSync(*restoreVsync_);
    }

    ScopedSwapInterval(const ScopedSwapInterval&) = delete;
    ScopedSwapInterval& operator=(const ScopedSwapInterval&) = delete;

  private:
    openbus::rendering::RendererFramePresenter* presenter_;
    const bool* restoreVsync_;
};

} // namespace openbus::app
