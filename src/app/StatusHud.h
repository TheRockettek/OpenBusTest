#pragma once

// Tiny bitmap-font renderer for the on-screen status/FPS overlay.

#include <cstdint>
#include <string_view>
#include <vector>

namespace openbus::app {

inline constexpr int STATUS_HUD_TEXTURE_WIDTH = 400;
inline constexpr int STATUS_HUD_TEXTURE_HEIGHT = 42;

// Returns STATUS_HUD_TEXTURE_WIDTH x STATUS_HUD_TEXTURE_HEIGHT RGBA pixels.
[[nodiscard]] std::vector<std::uint8_t> makeStatusHudPixels(std::string_view text);

} // namespace openbus::app
