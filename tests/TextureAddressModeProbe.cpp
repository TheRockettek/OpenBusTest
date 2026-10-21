#include "render/GlCompat.h"

#include <array>
#include <iostream>

int main() {
    struct AddressModeCase {
        TextureAddressMode mode;
        GLenum expected;
        const char* name;
    };
    constexpr std::array<AddressModeCase, 5> cases = {{
        {TextureAddressMode::Repeat, GL_REPEAT, "repeat"},
        {TextureAddressMode::Clamp, GL_CLAMP_TO_EDGE, "clamp"},
        {TextureAddressMode::Border, GL_CLAMP_TO_BORDER, "border"},
        {TextureAddressMode::Mirror, GL_MIRRORED_REPEAT, "mirror"},
        {TextureAddressMode::MirrorOnce, GL_MIRROR_CLAMP_TO_EDGE, "mirror once"},
    }};
    for (const AddressModeCase& testCase : cases) {
        if (openbus::rendering::textureAddressModeToGl(testCase.mode) != testCase.expected) {
            std::cerr << "texture address mode mapped incorrectly: " << testCase.name << '\n';
            return 1;
        }
    }
    if (openbus::rendering::textureAddressModeToGl(static_cast<TextureAddressMode>(-1)) !=
        GL_REPEAT) {
        std::cerr << "invalid texture address mode did not fall back to repeat\n";
        return 1;
    }
    return 0;
}
