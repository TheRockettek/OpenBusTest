#include "app/StatusHud.h"

#include <array>
#include <cctype>

namespace openbus::app {

namespace {

std::array<std::uint8_t, 7> statusHudGlyph(char character) {
    switch (character) {
    case 'A':
        return {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001};
    case 'B':
        return {0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110};
    case 'C':
        return {0b01111, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b01111};
    case 'D':
        return {0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110};
    case 'E':
        return {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111};
    case 'H':
        return {0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001};
    case 'I':
        return {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b11111};
    case 'J':
        return {0b00111, 0b00010, 0b00010, 0b00010, 0b00010, 0b10010, 0b01100};
    case 'L':
        return {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111};
    case 'N':
        return {0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001};
    case 'O':
        return {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110};
    case 'R':
        return {0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001};
    case 'S':
        return {0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110};
    case 'T':
        return {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100};
    case 'U':
        return {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110};
    case 'V':
        return {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100};
    case 'K':
        return {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001};
    case '0':
        return {0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110};
    case '1':
        return {0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110};
    case '2':
        return {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111};
    case '3':
        return {0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001, 0b11110};
    case '4':
        return {0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010};
    case '5':
        return {0b11111, 0b10000, 0b10000, 0b11110, 0b00001, 0b00001, 0b11110};
    case '6':
        return {0b01110, 0b10000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110};
    case '7':
        return {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000};
    case '8':
        return {0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110};
    case '9':
        return {0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00001, 0b01110};
    case 'X':
        return {0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b01010, 0b10001};
    case 'Y':
        return {0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100};
    case 'Z':
        return {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111};
    case '-':
        return {0, 0, 0, 0b01110, 0, 0, 0};
    case '.':
        return {0, 0, 0, 0, 0, 0b00100, 0b00100};
    case '/':
        return {0b00001, 0b00010, 0b00010, 0b00100, 0b01000, 0b01000, 0b10000};
    default:
        return {};
    }
}

} // namespace

std::vector<std::uint8_t> makeStatusHudPixels(std::string_view text) {
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(STATUS_HUD_TEXTURE_WIDTH) *
                                     STATUS_HUD_TEXTURE_HEIGHT * 4);
    for (std::size_t index = 0; index < pixels.size(); index += 4) {
        pixels[index] = 10;
        pixels[index + 1] = 16;
        pixels[index + 2] = 24;
        pixels[index + 3] = 196;
    }
    constexpr int glyphAdvance = 6;
    constexpr int lineAdvance = 9;
    constexpr int firstLineTop = (STATUS_HUD_TEXTURE_HEIGHT - (7 + 2 * lineAdvance)) / 2;
    constexpr int horizontalPadding = 6;
    int line = 0;
    int glyphColumn = 0;
    for (const char character : text) {
        if (character == '\n') {
            ++line;
            glyphColumn = 0;
            continue;
        }
        const int glyphLeft = horizontalPadding + glyphColumn * glyphAdvance;
        const int glyphTop = firstLineTop + line * lineAdvance;
        if (glyphLeft + 5 > STATUS_HUD_TEXTURE_WIDTH || glyphTop + 7 > STATUS_HUD_TEXTURE_HEIGHT) {
            break;
        }
        ++glyphColumn;
        const auto glyph = statusHudGlyph(character);
        for (int row = 0; row < static_cast<int>(glyph.size()); ++row) {
            const int y = STATUS_HUD_TEXTURE_HEIGHT - 1 - (glyphTop + row);
            for (int column = 0; column < 5; ++column) {
                if ((glyph[static_cast<std::size_t>(row)] & (1U << (4 - column))) == 0) {
                    continue;
                }
                const int x = glyphLeft + column;
                const std::size_t pixelIndex =
                    (static_cast<std::size_t>(y) * STATUS_HUD_TEXTURE_WIDTH + x) * 4;
                pixels[pixelIndex] = 245;
                pixels[pixelIndex + 1] = 249;
                pixels[pixelIndex + 2] = 255;
                pixels[pixelIndex + 3] = 255;
            }
        }
    }
    return pixels;
}

} // namespace openbus::app
