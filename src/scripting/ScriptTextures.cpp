// Script textures, bitmap fonts, text textures and the texture Lua API.

#include "scripting/ScriptRuntimeImpl.h"

bool ScriptRuntime::Impl::floatToInt(float value, int& result) {
    const double numericValue = static_cast<double>(value);
    if (!std::isfinite(numericValue) ||
        numericValue < static_cast<double>(std::numeric_limits<int>::min()) ||
        numericValue > static_cast<double>(std::numeric_limits<int>::max())) {
        return false;
    }
    result = static_cast<int>(value);
    return true;
}

bool ScriptRuntime::Impl::validDrawingOrigin(int value) {
    return value >= -openbus::scripting::maxScriptTextureDimension &&
           value <= openbus::scripting::maxScriptTextureDimension;
}

std::uint8_t ScriptRuntime::Impl::colorChannel(float value) {
    if (!std::isfinite(value)) {
        return 0;
    }
    return static_cast<std::uint8_t>(std::clamp(value, 0.0F, 255.0F));
}

std::uint32_t ScriptRuntime::Impl::packedColor(float value) {
    if (!std::isfinite(value) || value <= 0.0F) {
        return 0;
    }
    return static_cast<std::uint32_t>(
        std::min(static_cast<double>(value),
                 static_cast<double>(std::numeric_limits<std::uint32_t>::max())));
}

bool ScriptRuntime::Impl::validTextureIndex(int index) {
    return openbus::scripting::validScriptTextureIndex(index);
}

std::size_t ScriptRuntime::Impl::currentTextureBytes() const {
    std::size_t bytes = 0;
    for (const auto& [index, texture] : scriptTextures) {
        (void)index;
        bytes += texture.pixels.size();
    }
    for (const auto& [index, texture] : textTextures) {
        (void)index;
        bytes += texture.pixels.size();
    }
    return bytes;
}

bool ScriptRuntime::Impl::canReplaceTextureBytes(std::size_t oldBytes, std::size_t newBytes) const {
    const std::size_t currentBytes = currentTextureBytes();
    return oldBytes <= currentBytes &&
           newBytes <= openbus::scripting::maxScriptTextureBytesPerSurface &&
           newBytes <= openbus::scripting::maxScriptTextureBytesPerRuntime &&
           currentBytes - oldBytes <=
               openbus::scripting::maxScriptTextureBytesPerRuntime - newBytes;
}

ScriptRuntime::Impl::ScriptTexture ScriptRuntime::Impl::configuredScriptTexture(int index) {
    ScriptTexture texture;
    texture.revision = ++revisionCounter;
    if (!validTextureIndex(index)) {
        return texture;
    }
    const auto found = scriptTextureDimensions.find(index);
    if (found == scriptTextureDimensions.end()) {
        return texture;
    }
    texture.width = found->second.first;
    texture.height = found->second.second;
    const std::size_t byteSize =
        openbus::scripting::scriptTextureByteSize(texture.width, texture.height);
    const auto existing = scriptTextures.find(index);
    const std::size_t oldBytes =
        existing == scriptTextures.end() ? 0U : existing->second.pixels.size();
    if (byteSize == 0 || !canReplaceTextureBytes(oldBytes, byteSize)) {
        texture.width = 1;
        texture.height = 1;
        texture.pixels.assign(4, 0);
        return texture;
    }
    texture.pixels.assign(byteSize, 0);
    return texture;
}

std::array<std::uint8_t, 7> ScriptRuntime::Impl::glyph(char character) {
    switch (static_cast<char>(std::toupper(static_cast<unsigned char>(character)))) {
    case 'A':
        return {14, 17, 17, 31, 17, 17, 17};
    case 'B':
        return {30, 17, 17, 30, 17, 17, 30};
    case 'C':
        return {14, 17, 16, 16, 16, 17, 14};
    case 'D':
        return {30, 17, 17, 17, 17, 17, 30};
    case 'E':
        return {31, 16, 16, 30, 16, 16, 31};
    case 'F':
        return {31, 16, 16, 30, 16, 16, 16};
    case 'G':
        return {14, 17, 16, 23, 17, 17, 14};
    case 'H':
        return {17, 17, 17, 31, 17, 17, 17};
    case 'I':
        return {14, 4, 4, 4, 4, 4, 14};
    case 'J':
        return {7, 2, 2, 2, 2, 18, 12};
    case 'K':
        return {17, 18, 20, 24, 20, 18, 17};
    case 'L':
        return {16, 16, 16, 16, 16, 16, 31};
    case 'M':
        return {17, 27, 21, 21, 17, 17, 17};
    case 'N':
        return {17, 25, 25, 21, 19, 19, 17};
    case 'O':
        return {14, 17, 17, 17, 17, 17, 14};
    case 'P':
        return {30, 17, 17, 30, 16, 16, 16};
    case 'Q':
        return {14, 17, 17, 17, 21, 18, 13};
    case 'R':
        return {30, 17, 17, 30, 20, 18, 17};
    case 'S':
        return {15, 16, 16, 14, 1, 1, 30};
    case 'T':
        return {31, 4, 4, 4, 4, 4, 4};
    case 'U':
        return {17, 17, 17, 17, 17, 17, 14};
    case 'V':
        return {17, 17, 17, 17, 17, 10, 4};
    case 'W':
        return {17, 17, 17, 21, 21, 27, 17};
    case 'X':
        return {17, 17, 10, 4, 10, 17, 17};
    case 'Y':
        return {17, 17, 10, 4, 4, 4, 4};
    case 'Z':
        return {31, 1, 2, 4, 8, 16, 31};
    case '0':
        return {14, 17, 19, 21, 25, 17, 14};
    case '1':
        return {4, 12, 4, 4, 4, 4, 14};
    case '2':
        return {14, 17, 1, 2, 4, 8, 31};
    case '3':
        return {30, 1, 1, 14, 1, 1, 30};
    case '4':
        return {2, 6, 10, 18, 31, 2, 2};
    case '5':
        return {31, 16, 16, 30, 1, 1, 30};
    case '6':
        return {14, 16, 16, 30, 17, 17, 14};
    case '7':
        return {31, 1, 2, 4, 8, 8, 8};
    case '8':
        return {14, 17, 17, 14, 17, 17, 14};
    case '9':
        return {14, 17, 17, 15, 1, 1, 14};
    case '-':
        return {0, 0, 0, 31, 0, 0, 0};
    case '_':
        return {0, 0, 0, 0, 0, 0, 31};
    case '.':
        return {0, 0, 0, 0, 0, 0, 4};
    case ':':
        return {0, 4, 0, 0, 0, 4, 0};
    case '/':
        return {1, 2, 2, 4, 8, 8, 16};
    case '+':
        return {0, 4, 4, 31, 4, 4, 0};
    case '?':
        return {14, 17, 1, 2, 4, 0, 4};
    case ' ':
        return {0, 0, 0, 0, 0, 0, 0};
    default:
        return {14, 17, 1, 2, 4, 0, 4};
    }
}

std::shared_ptr<ScriptRuntime::Impl::FontAsset>
ScriptRuntime::Impl::loadFont(const std::string& requestedName) {
    const std::string name = lower(requestedName);
    const auto cached = fonts.find(name);
    if (cached != fonts.end()) {
        return cached->second;
    }
    const auto trimFontLine = [](const std::string& value) {
        const std::size_t first = value.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            return std::string();
        }
        const std::size_t last = value.find_last_not_of(" \t\r\n");
        return value.substr(first, last - first + 1);
    };
    const auto readFontLines = [&trimFontLine](const std::filesystem::path& path) {
        std::vector<std::string> lines;
        std::ifstream input(path);
        std::string line;
        while (std::getline(input, line)) {
            lines.push_back(trimFontLine(line));
        }
        return lines;
    };
    const auto parseInteger = [&trimFontLine](const std::string& value, int& result) {
        try {
            const std::string normalized = trimFontLine(value);
            std::size_t consumed = 0;
            const int parsed = std::stoi(normalized, &consumed);
            if (consumed != normalized.size()) {
                return false;
            }
            result = parsed;
            return true;
        } catch (const std::exception&) {
            return false;
        }
    };
    std::vector<std::filesystem::path> fontDirectories;
    std::filesystem::path sourceDirectory = configuration.sourcePath.parent_path();
    while (!sourceDirectory.empty()) {
        fontDirectories.push_back(sourceDirectory / "Fonts");
        const std::filesystem::path parentDirectory = sourceDirectory.parent_path();
        if (parentDirectory == sourceDirectory) {
            break;
        }
        sourceDirectory = parentDirectory;
    }
    std::filesystem::path fontPath;
    std::error_code error;
    for (const auto& directory : fontDirectories) {
        if (!std::filesystem::is_directory(directory, error)) {
            continue;
        }
        for (const auto& entry : std::filesystem::directory_iterator(directory, error)) {
            if (!entry.is_regular_file(error) ||
                lower(entry.path().extension().string()) != ".oft") {
                continue;
            }
            const std::vector<std::string> lines = readFontLines(entry.path());
            for (std::size_t lineIndex = 0; lineIndex < lines.size(); ++lineIndex) {
                if (lower(lines[lineIndex]) != "[newfont]" || lineIndex + 5 >= lines.size()) {
                    continue;
                }
                if (lower(lines[lineIndex + 1]) == name) {
                    fontPath = entry.path();
                    break;
                }
            }
            if (!fontPath.empty()) {
                break;
            }
        }
        if (!fontPath.empty()) {
            break;
        }
    }
    if (fontPath.empty()) {
        fonts.emplace(name, nullptr);
        return nullptr;
    }

    auto font = std::make_shared<FontAsset>();
    const std::vector<std::string> lines = readFontLines(fontPath);
    for (std::size_t lineIndex = 0; lineIndex < lines.size(); ++lineIndex) {
        if (lines[lineIndex] != "[newfont]" && lines[lineIndex] != "[char]") {
            if (lines[lineIndex].size() > 2 && lines[lineIndex].front() == '[' &&
                lines[lineIndex].back() == ']') {
                openbus::config::logUnhandledConfigurationTag(
                    "font (.oft)", fontPath, lineIndex + 1,
                    lines[lineIndex].substr(1, lines[lineIndex].size() - 2),
                    "font section is not implemented");
            }
            continue;
        }
        const std::size_t valueCount = lines[lineIndex] == "[newfont]" ? 5 : 4;
        if (lineIndex + valueCount >= lines.size()) {
            continue;
        }
        const bool isNewFont = lines[lineIndex] == "[newfont]";
        std::vector<std::string> values(lines.begin() + lineIndex + 1,
                                        lines.begin() + lineIndex + valueCount + 1);
        lineIndex += valueCount;
        if (isNewFont) {
            int height = 0;
            int horizontalGap = 0;
            if (!parseInteger(values[3], height) || !parseInteger(values[4], horizontalGap) ||
                height <= 0) {
                continue;
            }
            font->height = height;
            font->horizontalGap = horizontalGap;
            if (!openbus::rendering::TextureLoader::readImage(
                    fontPath.parent_path() / trimFontLine(values[1]), font->colorImage) ||
                !openbus::rendering::TextureLoader::readImage(
                    fontPath.parent_path() / trimFontLine(values[2]), font->alphaImage)) {
                continue;
            }
        } else {
            int left = 0;
            int right = 0;
            int top = 0;
            if (!parseInteger(values[1], left) || !parseInteger(values[2], right) ||
                !parseInteger(values[3], top) || right <= left || left < 0 || top < 0) {
                continue;
            }
            // Whitespace trimming leaves the literal space glyph empty.
            int code = values[0].empty() ? ' ' : static_cast<unsigned char>(values[0][0]);
            int numericCode = 0;
            // OFT [char] stores a literal character: "0" is the digit,
            // not codepoint zero. Keep multi-digit numeric extensions only.
            if (values[0].size() > 1 && parseInteger(values[0], numericCode) && numericCode >= 0 &&
                numericCode <= 255) {
                code = numericCode;
            }
            font->glyphs[static_cast<std::size_t>(code)] = {left, right, top, true};
        }
    }
    if (font->height <= 0 || font->colorImage.width <= 0 || font->alphaImage.width <= 0) {
        fonts.emplace(name, nullptr);
        return nullptr;
    }
    fonts.emplace(name, font);
    return font;
}

ScriptRuntime::Impl::ScriptTexture& ScriptRuntime::Impl::scriptTexture(Impl* runtime, int index) {
    return runtime->scriptTextures[index];
}

bool ScriptRuntime::Impl::popSystemInt(Impl* runtime, int& value) {
    return floatToInt(popSystemFloat(runtime), value);
}

bool ScriptRuntime::Impl::popSystemTextureIndex(Impl* runtime, int& index) {
    return popSystemInt(runtime, index) && validTextureIndex(index);
}

bool ScriptRuntime::Impl::resizeScriptTexture(ScriptTexture& texture, int requiredWidth,
                                              int requiredHeight) {
    if (requiredWidth <= texture.width && requiredHeight <= texture.height) {
        return true;
    }
    const int width = std::max(texture.width, requiredWidth);
    const int height = std::max(texture.height, requiredHeight);
    const std::size_t byteSize = openbus::scripting::scriptTextureByteSize(width, height);
    if (byteSize == 0 || !canReplaceTextureBytes(texture.pixels.size(), byteSize)) {
        return false;
    }
    std::vector<std::uint8_t> pixels(byteSize, 0);
    for (int y = 0; y < texture.height; ++y) {
        const auto sourceOffset =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(texture.width) * 4U;
        const auto targetOffset =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width) * 4U;
        const std::size_t rowBytes = static_cast<std::size_t>(texture.width) * 4U;
        std::copy_n(texture.pixels.begin() + sourceOffset, rowBytes, pixels.begin() + targetOffset);
    }
    texture.width = width;
    texture.height = height;
    texture.pixels = std::move(pixels);
    return true;
}

std::size_t ScriptRuntime::Impl::pixelOffset(const ScriptTexture& texture, int x, int y) {
    return (static_cast<std::size_t>(y) * texture.width + x) * 4;
}

void ScriptRuntime::Impl::setPixel(ScriptTexture& texture, int x, int y) {
    if (texture.locked || x < 0 || y < 0 || x >= openbus::scripting::maxScriptTextureDimension ||
        y >= openbus::scripting::maxScriptTextureDimension ||
        !resizeScriptTexture(texture, x + 1, y + 1)) {
        return;
    }
    const std::size_t offset = pixelOffset(texture, x, y);
    texture.pixels[offset] = texture.color[1];
    texture.pixels[offset + 1] = texture.color[2];
    texture.pixels[offset + 2] = texture.color[3];
    texture.pixels[offset + 3] = texture.color[0];
    ++texture.revision;
}

void ScriptRuntime::Impl::fillRect(ScriptTexture& texture, int x1, int y1, int x2, int y2) {
    if (texture.locked) {
        return;
    }
    const int left = std::max(0, std::min(x1, x2));
    const int top = std::max(0, std::min(y1, y2));
    const int right = std::min(openbus::scripting::maxScriptTextureDimension - 1, std::max(x1, x2));
    const int bottom =
        std::min(openbus::scripting::maxScriptTextureDimension - 1, std::max(y1, y2));
    if (left > right || top > bottom) {
        return;
    }
    const int width = right - left + 1;
    const int height = bottom - top + 1;
    const std::size_t pixelCount =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (pixelCount > openbus::scripting::maxScriptTexturePixels ||
        !resizeScriptTexture(texture, right + 1, bottom + 1)) {
        return;
    }
    for (int y = top; y <= bottom; ++y) {
        for (int x = left; x <= right; ++x) {
            const std::size_t offset = pixelOffset(texture, x, y);
            std::copy(texture.color.begin(), texture.color.end(), texture.pixels.begin() + offset);
        }
    }
    ++texture.revision;
}

void ScriptRuntime::Impl::clearTexture(ScriptTexture& texture) {
    texture.pixels.assign(
        static_cast<std::size_t>(texture.width) * static_cast<std::size_t>(texture.height) * 4, 0);
}

void ScriptRuntime::Impl::drawText(ScriptTexture& texture, const std::string& value, int x, int y,
                                   std::array<std::uint8_t, 4> color, int letterSpacing,
                                   int maximumScale, TextAlignment alignment,
                                   bool centerVertically) {
    if (texture.locked || value.empty() || value.size() > openbus::scripting::maxScriptTextLength ||
        !openbus::scripting::validScriptTextureSize(texture.width, texture.height)) {
        return;
    }
    std::size_t lineCount = 1;
    std::size_t longestLine = 0;
    std::size_t currentLineLength = 0;
    std::vector<std::size_t> lineLengths(1, 0);
    for (const char character : value) {
        if (character == '\n') {
            longestLine = std::max(longestLine, currentLineLength);
            currentLineLength = 0;
            ++lineCount;
            lineLengths.emplace_back(0);
        } else if (character != '\r') {
            ++currentLineLength;
            ++lineLengths.back();
        }
    }
    longestLine = std::max(longestLine, currentLineLength);
    const int heightScale = std::max(1, (texture.height - 2) / 7);
    const int spacing = std::max(0, letterSpacing);
    const int availableWidth = std::max(
        1, texture.width - 2 - spacing * static_cast<int>(longestLine > 0 ? longestLine - 1 : 0));
    const int widthScale =
        std::max(1, availableWidth / std::max(1, 6 * static_cast<int>(longestLine)));
    const int multilineHeight = std::max(1, (texture.height - 2) / static_cast<int>(7 * lineCount));
    const int scale = std::max(1, std::min({maximumScale, heightScale, widthScale}));
    const int advance = 6 * scale + letterSpacing;
    const int lineScale = std::max(1, std::min(scale, multilineHeight));
    const int textBlockHeight =
        static_cast<int>((lineCount - 1) * 8 * lineScale + 6 * lineScale + scale);
    const int verticalOffset =
        centerVertically ? std::max(0, (texture.height - textBlockHeight) / 2) : 0;
    int line = 0;
    int columnIndex = 0;
    for (const char character : value) {
        if (character == '\n') {
            ++line;
            columnIndex = 0;
            continue;
        }
        if (character == '\r') {
            continue;
        }
        const auto rows = glyph(character);
        const std::size_t lineLength = lineLengths[static_cast<std::size_t>(line)];
        const int lineWidth =
            lineLength == 0 ? 0 : static_cast<int>((lineLength - 1) * advance + 5 * scale);
        const int remainingWidth = std::max(0, texture.width - lineWidth);
        const int horizontalOffset = alignment == TextAlignment::Center  ? remainingWidth / 2
                                     : alignment == TextAlignment::Right ? remainingWidth
                                                                         : 0;
        const int originX = x + horizontalOffset + columnIndex * advance;
        const int originY = y + verticalOffset + line * 8 * lineScale;
        for (int row = 0; row < 7; ++row) {
            for (int column = 0; column < 5; ++column) {
                if ((rows[static_cast<std::size_t>(row)] & (1 << (4 - column))) == 0) {
                    continue;
                }
                for (int offsetY = 0; offsetY < scale; ++offsetY) {
                    for (int offsetX = 0; offsetX < scale; ++offsetX) {
                        const int pixelX = originX + column * scale + offsetX;
                        const int pixelY = originY + row * lineScale + offsetY;
                        if (pixelX < 0 || pixelY < 0 || pixelX >= texture.width ||
                            pixelY >= texture.height) {
                            continue;
                        }
                        const std::size_t offset = pixelOffset(texture, pixelX, pixelY);
                        std::copy(color.begin(), color.end(), texture.pixels.begin() + offset);
                    }
                }
            }
        }
        ++columnIndex;
    }
    ++texture.revision;
}

bool ScriptRuntime::Impl::drawBitmapFontText(ScriptTexture& texture, const std::string& value,
                                             int x, int y, const TextTextureDefinition& definition,
                                             const FontAsset& font) {
    if (value.size() > openbus::scripting::maxScriptTextLength ||
        !openbus::scripting::validScriptTextureSize(texture.width, texture.height)) {
        return false;
    }
    int fallbackCode = -1;
    for (std::size_t index = 0; index < font.glyphs.size(); ++index) {
        if (font.glyphs[index].defined) {
            fallbackCode = static_cast<int>(index);
            break;
        }
    }
    if (fallbackCode < 0 || font.colorImage.width <= 0 || font.alphaImage.width <= 0) {
        return false;
    }
    std::vector<std::vector<const FontGlyph*>> lines(1);
    for (std::size_t characterIndex = 0; characterIndex < value.size(); ++characterIndex) {
        const unsigned char character = static_cast<unsigned char>(value[characterIndex]);
        if (character == '@' || character == '\n') {
            lines.emplace_back();
            continue;
        }
        if (character == '\r') {
            continue;
        }
        if (character == 0xef && characterIndex + 3 < value.size() &&
            static_cast<unsigned char>(value[characterIndex + 1]) == 0xbf &&
            static_cast<unsigned char>(value[characterIndex + 2]) == 0xbd &&
            value[characterIndex + 3] == 'c' && font.glyphs[176].defined) {
            // Some legacy CP1252 scripts contain UTF-8 replacement bytes where a
            // degree sign was lost. Only the exact replacement-byte + lowercase-c suffix
            // is unambiguous.
            lines.back().push_back(&font.glyphs[176]);
            characterIndex += 2;
            continue;
        }
        int code = character;
        if (character == 0xc2 && characterIndex + 1 < value.size() &&
            static_cast<unsigned char>(value[characterIndex + 1]) == 0xb0) {
            code = 176;
            ++characterIndex;
        }
        const std::size_t glyphIndex = font.glyphs[static_cast<std::size_t>(code)].defined
                                           ? static_cast<std::size_t>(code)
                                           : static_cast<std::size_t>(fallbackCode);
        lines.back().push_back(&font.glyphs[glyphIndex]);
    }

    const int textBlockHeight = static_cast<int>(lines.size()) * font.height;
    const int horizontalGap = font.horizontalGap + definition.gridSpacing;
    const int verticalOffset = std::max(0, (texture.height - textBlockHeight) / 2);
    for (std::size_t lineIndex = 0; lineIndex < lines.size(); ++lineIndex) {
        const std::vector<const FontGlyph*>& line = lines[lineIndex];
        int lineWidth = 0;
        for (std::size_t glyphIndex = 0; glyphIndex < line.size(); ++glyphIndex) {
            lineWidth += line[glyphIndex]->right - line[glyphIndex]->left;
            if (glyphIndex + 1 < line.size()) {
                lineWidth += horizontalGap;
            }
        }
        lineWidth = std::max(0, lineWidth);
        const int remainingWidth = std::max(0, texture.width - lineWidth);
        int cursorX = x;
        if (definition.alignment == TextAlignment::Center) {
            cursorX += remainingWidth / 2;
        } else if (definition.alignment == TextAlignment::Right) {
            cursorX += remainingWidth;
        }
        const int cursorY = y + verticalOffset + static_cast<int>(lineIndex) * font.height;

        for (const FontGlyph* glyph : line) {
            const int glyphWidth = glyph->right - glyph->left;
            for (int row = 0; row < font.height; ++row) {
                const int sourceY = glyph->top + row;
                if (sourceY < 0 || sourceY >= font.alphaImage.height ||
                    sourceY >= font.colorImage.height) {
                    continue;
                }
                for (int column = 0; column < glyphWidth; ++column) {
                    const int sourceX = glyph->left + column;
                    if (sourceX < 0 || sourceX >= font.colorImage.width ||
                        sourceX >= font.alphaImage.width || cursorX + column < 0 ||
                        cursorY + row < 0 || cursorX + column >= texture.width ||
                        cursorY + row >= texture.height) {
                        continue;
                    }
                    const std::size_t sourceOffset =
                        (static_cast<std::size_t>(sourceY) * font.alphaImage.width + sourceX) * 4;
                    const std::size_t colorOffset =
                        (static_cast<std::size_t>(sourceY) * font.colorImage.width + sourceX) * 4;
                    const std::size_t destinationOffset =
                        pixelOffset(texture, cursorX + column, cursorY + row);
                    if (!definition.useFontColor) {
                        texture.pixels[destinationOffset] = definition.color[0];
                        texture.pixels[destinationOffset + 1] = definition.color[1];
                        texture.pixels[destinationOffset + 2] = definition.color[2];
                    } else {
                        texture.pixels[destinationOffset] = font.colorImage.rgba[colorOffset];
                        texture.pixels[destinationOffset + 1] =
                            font.colorImage.rgba[colorOffset + 1];
                        texture.pixels[destinationOffset + 2] =
                            font.colorImage.rgba[colorOffset + 2];
                    }
                    texture.pixels[destinationOffset + 3] = font.alphaImage.rgba[sourceOffset];
                }
            }
            cursorX += glyphWidth + horizontalGap;
        }
    }
    ++texture.revision;
    return true;
}

void ScriptRuntime::Impl::updateTextTextures(bool force) {
    if (!force && localState.get("refresh_strings") == 0.0) {
        return;
    }
    for (auto& entry : textTextureDefinitions) {
        const int index = entry.first;
        TextTextureDefinition& definition = entry.second;
        const std::size_t byteSize =
            openbus::scripting::scriptTextureByteSize(definition.width, definition.height);
        if (!validTextureIndex(index) || byteSize == 0) {
            continue;
        }
        const auto existing = textTextures.find(index);
        const std::size_t oldBytes =
            existing == textTextures.end() ? 0U : existing->second.pixels.size();
        if (!canReplaceTextureBytes(oldBytes, byteSize)) {
            continue;
        }
        const std::string value = localState.getString(definition.variable);
        if (definition.hasRenderedValue && value == definition.lastValue) {
            continue;
        }
        definition.lastValue = value;
        definition.hasRenderedValue = true;
        ScriptTexture& texture = textTextures[index];
        texture.width = definition.width;
        texture.height = definition.height;
        clearTexture(texture);
        const int maximumScale = 8;
        const std::shared_ptr<FontAsset> font = loadFont(definition.font);
        const bool bitmapFontRendered =
            font != nullptr && drawBitmapFontText(texture, value, 0, 0, definition, *font);
        if (!bitmapFontRendered && !value.empty()) {
            drawText(texture, value, 0, 0, definition.color, definition.gridSpacing, maximumScale,
                     definition.alignment, true);
        } else if (!bitmapFontRendered) {
            ++texture.revision;
        }
    }
    // Refresh_Strings is a one-shot request, consumed after the script pass
    // even if all configured strings were unchanged.
    localState.set("refresh_strings", 0.0);
}

bool ScriptRuntime::Impl::loadScriptTexture(int index, const std::string& requestedPath) {
    if (!validTextureIndex(index)) {
        return false;
    }
    const std::filesystem::path path =
        openbus::assets::resolveReferencedPath(configuration.sourcePath, requestedPath);
    openbus::rendering::Image image;
    if (!openbus::rendering::TextureLoader::readImage(path, image) || image.rgba.empty()) {
        return false;
    }
    const std::size_t byteSize =
        openbus::scripting::scriptTextureByteSize(image.width, image.height);
    const auto existing = scriptTextures.find(index);
    const std::size_t oldBytes =
        existing == scriptTextures.end() ? 0U : existing->second.pixels.size();
    if (byteSize == 0 || image.rgba.size() != byteSize ||
        !canReplaceTextureBytes(oldBytes, byteSize)) {
        return false;
    }
    ScriptTexture texture;
    texture.width = image.width;
    texture.height = image.height;
    texture.pixels = std::move(image.rgba);
    texture.revision = ++revisionCounter;
    scriptTextures[index] = std::move(texture);
    return true;
}

int ScriptRuntime::Impl::safeScriptTextureAction(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int index = -1;
    if (!popSystemTextureIndex(runtime, index)) {
        return 0;
    }
    ScriptTexture& texture = scriptTexture(runtime, index);
    texture.locked = true;
    texture.filtered = false;
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureNew(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int index = -1;
    if (!popSystemTextureIndex(runtime, index)) {
        return 0;
    }
    runtime->scriptTextures[index] = runtime->configuredScriptTexture(index);
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureLock(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int index = -1;
    if (popSystemTextureIndex(runtime, index)) {
        scriptTexture(runtime, index).locked = true;
    }
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureUnlock(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int index = -1;
    if (popSystemTextureIndex(runtime, index)) {
        scriptTexture(runtime, index).locked = false;
    }
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureFilter(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int index = -1;
    if (!popSystemTextureIndex(runtime, index)) {
        return 0;
    }
    ScriptTexture& texture = scriptTexture(runtime, index);
    if (!texture.locked) {
        if (!texture.filtered) {
            texture.filtered = true;
            ++texture.revision;
        }
    }
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureColor(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    const std::uint8_t blue = colorChannel(popSystemFloat(runtime));
    const std::uint8_t green = colorChannel(popSystemFloat(runtime));
    const std::uint8_t red = colorChannel(popSystemFloat(runtime));
    const std::uint8_t alpha = colorChannel(popSystemFloat(runtime));
    int index = -1;
    if (popSystemTextureIndex(runtime, index)) {
        scriptTexture(runtime, index).color = {alpha, red, green, blue};
    }
    return 0;
}

int ScriptRuntime::Impl::safeScriptTexturePixel(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int y = 0;
    int x = 0;
    int index = -1;
    const bool validY = popSystemInt(runtime, y);
    const bool validX = popSystemInt(runtime, x);
    const bool validIndex = popSystemTextureIndex(runtime, index);
    if (validY && validX && validIndex) {
        runtime->setPixel(scriptTexture(runtime, index), x, y);
    }
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureReadPixel(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int y = 0;
    int x = 0;
    int index = -1;
    const bool validY = popSystemInt(runtime, y);
    const bool validX = popSystemInt(runtime, x);
    const bool validIndex = popSystemTextureIndex(runtime, index);
    if (!validY || !validX || !validIndex) {
        return 0;
    }
    ScriptTexture& texture = scriptTexture(runtime, index);
    if (x >= 0 && y >= 0 && x < texture.width && y < texture.height) {
        const std::size_t offset = pixelOffset(texture, x, y);
        texture.color = {texture.pixels[offset + 3], texture.pixels[offset],
                         texture.pixels[offset + 1], texture.pixels[offset + 2]};
    }
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureRect(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int y2 = 0;
    int x2 = 0;
    int y1 = 0;
    int x1 = 0;
    int index = -1;
    const bool validY2 = popSystemInt(runtime, y2);
    const bool validX2 = popSystemInt(runtime, x2);
    const bool validY1 = popSystemInt(runtime, y1);
    const bool validX1 = popSystemInt(runtime, x1);
    const bool validIndex = popSystemTextureIndex(runtime, index);
    if (validY2 && validX2 && validY1 && validX1 && validIndex) {
        runtime->fillRect(scriptTexture(runtime, index), x1, y1, x2, y2);
    }
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureText(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int letterSpacing = 0;
    const bool validSpacing = popSystemInt(runtime, letterSpacing);
    const std::uint32_t colorValue = packedColor(popSystemFloat(runtime));
    popSystemFloat(runtime);
    int y = 0;
    int x = 0;
    int index = -1;
    const bool validY = popSystemInt(runtime, y);
    const bool validX = popSystemInt(runtime, x);
    const bool validIndex = popSystemTextureIndex(runtime, index);
    const std::string value = popSystemString(runtime);
    const std::array<std::uint8_t, 4> color = {
        static_cast<std::uint8_t>((colorValue >> 16) & 0xffU),
        static_cast<std::uint8_t>((colorValue >> 8) & 0xffU),
        static_cast<std::uint8_t>(colorValue & 0xffU),
        static_cast<std::uint8_t>((colorValue >> 24) & 0xffU)};
    if (validSpacing && validY && validX && validIndex && validDrawingOrigin(x) &&
        validDrawingOrigin(y) && letterSpacing >= -openbus::scripting::maxScriptTextureDimension &&
        letterSpacing <= openbus::scripting::maxScriptTextureDimension &&
        value.size() <= openbus::scripting::maxScriptTextLength) {
        drawText(scriptTexture(runtime, index), value, x, y, color, letterSpacing);
    }
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureCopyColor(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int destination = -1;
    int origin = -1;
    const bool validDestination = popSystemTextureIndex(runtime, destination);
    const bool validOrigin = popSystemTextureIndex(runtime, origin);
    if (validDestination && validOrigin) {
        scriptTexture(runtime, destination).color = scriptTexture(runtime, origin).color;
    }
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureLoad(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    int index = -1;
    const bool validIndex = popSystemTextureIndex(runtime, index);
    const std::string path = popSystemString(runtime);
    if (validIndex) {
        runtime->loadScriptTexture(index, path);
    }
    return 0;
}

int ScriptRuntime::Impl::safeScriptTextureChannel(lua_State* lua, std::size_t channel) {
    Impl* runtime = runtimeFor(lua);
    int index = -1;
    if (!popSystemTextureIndex(runtime, index)) {
        return returnSystemFloat(lua, 0.0);
    }
    const ScriptTexture& texture = scriptTexture(runtime, index);
    return returnSystemFloat(lua, texture.color[channel]);
}

int ScriptRuntime::Impl::safeScriptTextureRed(lua_State* lua) {
    return safeScriptTextureChannel(lua, 1);
}

int ScriptRuntime::Impl::safeScriptTextureGreen(lua_State* lua) {
    return safeScriptTextureChannel(lua, 2);
}

int ScriptRuntime::Impl::safeScriptTextureBlue(lua_State* lua) {
    return safeScriptTextureChannel(lua, 3);
}

int ScriptRuntime::Impl::safeScriptTextureAlpha(lua_State* lua) {
    return safeScriptTextureChannel(lua, 0);
}

int ScriptRuntime::Impl::safeFontIndex(lua_State* lua) {
    discardSystemStrings(runtimeFor(lua), 1);
    return returnSystemFloat(lua, 0.0);
}

int ScriptRuntime::Impl::safeTextLength(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    discardSystemFloats(runtime, 1);
    const std::string value = popSystemString(runtime);
    return returnSystemFloat(lua, static_cast<float>(value.size() * 6));
}

bool ScriptRuntime::Impl::copyScriptTexture(int index,
                                            ScriptRuntime::ScriptTextureSnapshot& snapshot) const {
    const auto found = scriptTextures.find(index);
    if (found == scriptTextures.end()) {
        return false;
    }
    snapshot.width = found->second.width;
    snapshot.height = found->second.height;
    snapshot.pixels = found->second.pixels;
    snapshot.filtered = found->second.filtered;
    snapshot.revision = found->second.revision;
    return true;
}

bool ScriptRuntime::Impl::copyTextTexture(int index,
                                          ScriptRuntime::ScriptTextureSnapshot& snapshot) const {
    const auto found = textTextures.find(index);
    if (found == textTextures.end()) {
        return false;
    }
    snapshot.width = found->second.width;
    snapshot.height = found->second.height;
    snapshot.pixels = found->second.pixels;
    snapshot.filtered = found->second.filtered;
    snapshot.revision = found->second.revision;
    return true;
}

bool ScriptRuntime::Impl::copyScriptTextureIfChanged(
    int index, std::uint64_t knownRevision, ScriptRuntime::ScriptTextureSnapshot& snapshot) const {
    const auto found = scriptTextures.find(index);
    if (found == scriptTextures.end()) {
        return false;
    }
    snapshot.width = found->second.width;
    snapshot.height = found->second.height;
    snapshot.filtered = found->second.filtered;
    snapshot.revision = found->second.revision;
    if (snapshot.revision == knownRevision) {
        std::vector<std::uint8_t>().swap(snapshot.pixels);
    } else {
        snapshot.pixels = found->second.pixels;
    }
    return true;
}

bool ScriptRuntime::Impl::copyTextTextureIfChanged(
    int index, std::uint64_t knownRevision, ScriptRuntime::ScriptTextureSnapshot& snapshot) const {
    const auto found = textTextures.find(index);
    if (found == textTextures.end()) {
        return false;
    }
    snapshot.width = found->second.width;
    snapshot.height = found->second.height;
    snapshot.filtered = found->second.filtered;
    snapshot.revision = found->second.revision;
    if (snapshot.revision == knownRevision) {
        std::vector<std::uint8_t>().swap(snapshot.pixels);
    } else {
        snapshot.pixels = found->second.pixels;
    }
    return true;
}
