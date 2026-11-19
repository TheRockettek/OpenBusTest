#pragma once

namespace gli {
class texture;
class texture2d;
} // namespace gli

namespace openbus::rendering::diligent_detail {

gli::texture2d decodeDxt1AlphaMipChain(const gli::texture& image, bool srgb);

} // namespace openbus::rendering::diligent_detail
