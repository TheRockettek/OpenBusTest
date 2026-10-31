#include "app/ScopedSwapInterval.h"

#include <stdexcept>
#include <vector>

namespace {

std::vector<int> intervals;

void setInterval(int interval) {
    intervals.push_back(interval);
}

void checkRestoration(int gameplayInterval, bool throwDuringMenu) {
    intervals.clear();
    try {
        const openbus::app::ScopedSwapInterval menu(setInterval, 1, gameplayInterval);
        if (intervals != std::vector<int>{1}) {
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
    if (intervals != std::vector<int>{1, gameplayInterval}) {
        throw std::runtime_error("menu did not restore the gameplay swap interval");
    }
}

} // namespace

int main() {
    checkRestoration(0, false);
    checkRestoration(1, false);
    checkRestoration(0, true);
    checkRestoration(1, true);
}
