#include "ScriptRuntime.h"

#include "Logger.h"
#include "Variables.h"
#include "Environment.h"
#include "PerfTrace.h"
#include "ScriptTextureLimits.h"
#include "TextureLoader.h"
#include "osc/OscConverter.h"

extern "C" {
#include "osc/lauxlib.h"
#include "osc/lua.h"
#include "osc/lualib.h"
}

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <string_view>
#include <unordered_map>

Logger scriptRuntimeLogger = Logger("ScriptRuntime");

namespace {

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::string scriptName(std::string value) {
    for (char& character : value) {
        const unsigned char byte = static_cast<unsigned char>(character);
        character = std::isalnum(byte) ? static_cast<char>(std::tolower(byte)) : '_';
    }
    return value;
}

std::shared_ptr<const OscProgram> cachedNativeProgram(const std::filesystem::path& sourcePath,
                                                      std::string& error, bool& compiledThisCall) {
    static std::mutex cacheMutex;
    static std::unordered_map<std::string, std::shared_ptr<const OscProgram>> cache;
    std::string key = std::filesystem::absolute(sourcePath).lexically_normal().generic_string();
    key = lower(std::move(key));
    std::lock_guard<std::mutex> lock(cacheMutex);
    const auto cached = cache.find(key);
    if (cached != cache.end()) {
        return cached->second;
    }
    OscProgram compiled;
    if (!compileOscToBytecode(sourcePath, compiled, error)) {
        return {};
    }
    auto program = std::make_shared<const OscProgram>(std::move(compiled));
    cache.emplace(std::move(key), program);
    compiledThisCall = true;
    return program;
}

} // namespace

struct ScriptRuntime::Impl {

    struct ScriptTexture {
        int width = 1;
        int height = 1;
        std::vector<std::uint8_t> pixels = std::vector<std::uint8_t>(4, 0);
        std::array<std::uint8_t, 4> color = {255, 0, 0, 0};
        bool locked = false;
        bool filtered = false;
        std::uint64_t revision = 0;
    };

    struct ScriptEnvironment {
        int reference = LUA_NOREF;
    };

    struct LuaFunctionBinding {
        std::string name;
        lua_CFunction function = nullptr;
    };

    lua_State* state = nullptr;
    VehicleConfig configuration;
    std::string scriptIdentity;
    Variables& localState;
    SimulationState& sharedState;
    std::function<void(const std::string&, const std::string&, float)> onSoundTrigger;
    ScriptEnvironment scriptEnvironment;
    std::vector<std::string> errors;
    std::vector<float> floatStack;
    std::vector<std::string> stringStack;
    std::unordered_map<int, ScriptTexture> scriptTextures;
    std::unordered_map<int, std::pair<int, int>> scriptTextureDimensions;
    enum class TextAlignment { Left, Center, Right };

    struct TextTextureDefinition {
        std::string variable;
        std::string font;
        int width = 0;
        int height = 0;
        TextAlignment alignment = TextAlignment::Center;
        int gridSpacing = 0;
        std::array<std::uint8_t, 4> color = {255, 255, 255, 255};
        bool useFontColor = false;
        std::string lastValue;
        bool hasRenderedValue = false;
    };
    std::unordered_map<int, TextTextureDefinition> textTextureDefinitions;
    std::unordered_map<int, ScriptTexture> textTextures;
    struct FontGlyph {
        int left = 0;
        int right = 0;
        int top = 0;
        bool defined = false;
    };
    struct FontAsset {
        int height = 0;
        int horizontalGap = 0;
        std::array<FontGlyph, 256> glyphs = {};
        openbus::rendering::Image colorImage;
        openbus::rendering::Image alphaImage;
    };
    std::unordered_map<std::string, std::shared_ptr<FontAsset>> fonts;
    std::uint64_t revisionCounter = 0;
    std::vector<std::unique_ptr<LuaFunctionBinding>> luaFunctionBindings;
    std::vector<std::shared_ptr<const OscProgram>> nativePrograms;
    std::array<float, 8> nativeRegisters = {};
    bool nativeBackend = false;

    static bool floatToInt(float value, int& result) {
        const double numericValue = static_cast<double>(value);
        if (!std::isfinite(numericValue) ||
            numericValue < static_cast<double>(std::numeric_limits<int>::min()) ||
            numericValue > static_cast<double>(std::numeric_limits<int>::max())) {
            return false;
        }
        result = static_cast<int>(value);
        return true;
    }

    static bool validDrawingOrigin(int value) {
        return value >= -openbus::scripting::maxScriptTextureDimension &&
               value <= openbus::scripting::maxScriptTextureDimension;
    }

    static std::uint8_t colorChannel(float value) {
        if (!std::isfinite(value)) {
            return 0;
        }
        return static_cast<std::uint8_t>(std::clamp(value, 0.0F, 255.0F));
    }

    static std::uint32_t packedColor(float value) {
        if (!std::isfinite(value) || value <= 0.0F) {
            return 0;
        }
        return static_cast<std::uint32_t>(
            std::min(static_cast<double>(value),
                     static_cast<double>(std::numeric_limits<std::uint32_t>::max())));
    }

    static bool validTextureIndex(int index) {
        return openbus::scripting::validScriptTextureIndex(index);
    }

    std::size_t currentTextureBytes() const {
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

    bool canReplaceTextureBytes(std::size_t oldBytes, std::size_t newBytes) const {
        const std::size_t currentBytes = currentTextureBytes();
        return oldBytes <= currentBytes &&
               newBytes <= openbus::scripting::maxScriptTextureBytesPerSurface &&
               newBytes <= openbus::scripting::maxScriptTextureBytesPerRuntime &&
               currentBytes - oldBytes <=
                   openbus::scripting::maxScriptTextureBytesPerRuntime - newBytes;
    }

    ScriptTexture configuredScriptTexture(int index) {
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

    static std::array<std::uint8_t, 7> glyph(char character) {
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

    std::shared_ptr<FontAsset> loadFont(const std::string& requestedName) {
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
                    }
                    break;
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
                if (values[0].size() > 1 && parseInteger(values[0], numericCode) &&
                    numericCode >= 0 && numericCode <= 255) {
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

    float nativePopFloat() {
        if (floatStack.empty()) {
            return 0.0F;
        }
        const float value = floatStack.back();
        floatStack.pop_back();
        return value;
    }

    float nativePeekFloat() const {
        return floatStack.empty() ? 0.0F : floatStack.back();
    }

    float soundControlValue(const std::string& name) const {
        float value = nativePeekFloat();
        std::string positionVariable;
        std::string lastPositionVariable;
        if (name == "ev_cabwindow_cls" || name == "ev_cabwindow_opn") {
            positionVariable = "cp_cabwindow_pos";
            lastPositionVariable = "cp_cabwindow_lastpos";
        } else if (name == "ev_cabwindow2_cls" || name == "ev_cabwindow2_opn") {
            positionVariable = "cp_cabwindow2_pos";
            lastPositionVariable = "cp_cabwindow2_lastpos";
        }
        if (!positionVariable.empty()) {
            const float timegap = sharedState.sharedVariables().get("timegap");
            if (timegap > 0.0F) {
                value = (std::max)(value, std::abs(localState.get(positionVariable) -
                                                   localState.get(lastPositionVariable)) /
                                              timegap);
            }
        }
        return value;
    }

    std::string nativePopString() {
        if (stringStack.empty()) {
            return {};
        }
        std::string value = std::move(stringStack.back());
        stringStack.pop_back();
        return value;
    }

    const std::string& nativePeekString() const {
        static const std::string empty;
        return stringStack.empty() ? empty : stringStack.back();
    }

    void executeSystemMacro(const std::string& name) {
        const auto popFloats = [this](std::size_t count) {
            while (count-- > 0 && !floatStack.empty()) {
                floatStack.pop_back();
            }
        };
        const auto popStrings = [this](std::size_t count) {
            while (count-- > 0 && !stringStack.empty()) {
                stringStack.pop_back();
            }
        };
        const auto pushNumber = [this](float value) { floatStack.push_back(value); };
        const auto pushString = [this] { stringStack.emplace_back(); };

        if (name == "getterminusindex" || name == "getbusstopindex" || name == "getrouteindex") {
            popFloats(1);
            pushNumber(-1.0);
        } else if (name == "getterminusstring" || name == "getbusstopstring" ||
                   name == "getroutebusstopident") {
            popFloats(2);
            pushString();
        } else if (name == "getttbusstopname" || name == "getticketname" ||
                   name == "getdepotstringglobal" || name == "getarrbusline" ||
                   name == "getarrbusterminus") {
            popFloats(1);
            pushString();
        } else if (name == "givechangecoin") {
            popFloats(1);
        } else if (name == "getheightabovepoint") {
            popFloats(3);
            pushNumber(0.0);
        } else if (name == "nrspecrandom") {
            const unsigned int seed = static_cast<unsigned int>(nativePopFloat());
            std::minstd_rand random(seed);
            pushNumber(static_cast<float>(random()) / static_cast<float>(std::minstd_rand::max()));
        } else if (name == "getterminuscode" || name == "getrouteterminusindex" ||
                   name == "getbusstopcount" || name == "getttlinestring" ||
                   name == "getttterminusindex" || name == "getttbusstopcount" ||
                   name == "getttbusstopindex" || name == "getttdelay" ||
                   name == "getticketvalue" || name == "gethumancountonpathlink" ||
                   name == "gethumancountonseat" || name == "getarrbustimediff") {
            popFloats(1);
            pushNumber(0.0);
        } else if (name == "stnewtex") {
            int index = -1;
            if (floatToInt(nativePopFloat(), index) && validTextureIndex(index)) {
                scriptTextures[index] = configuredScriptTexture(index);
            }
        } else if (name == "stlock" || name == "stunlock" || name == "stfilter") {
            int index = -1;
            if (!floatToInt(nativePopFloat(), index) || !validTextureIndex(index)) {
                return;
            }
            ScriptTexture& texture = scriptTextures[index];
            if (name == "stlock")
                texture.locked = true;
            if (name == "stunlock")
                texture.locked = false;
            if (name == "stfilter" && !texture.locked && !texture.filtered) {
                texture.filtered = true;
                ++texture.revision;
            }
        } else if (name == "stsetcolor") {
            const auto channel = [this] { return colorChannel(nativePopFloat()); };
            const std::uint8_t blue = channel();
            const std::uint8_t green = channel();
            const std::uint8_t red = channel();
            const std::uint8_t alpha = channel();
            int index = -1;
            if (floatToInt(nativePopFloat(), index) && validTextureIndex(index)) {
                scriptTextures[index].color = {alpha, red, green, blue};
            }
        } else if (name == "stdrawpixel") {
            int y = 0;
            int x = 0;
            int index = -1;
            const bool validY = floatToInt(nativePopFloat(), y);
            const bool validX = floatToInt(nativePopFloat(), x);
            const bool validIndex = floatToInt(nativePopFloat(), index);
            if (validY && validX && validIndex && validTextureIndex(index)) {
                setPixel(scriptTextures[index], x, y);
            }
        } else if (name == "stdrawrect") {
            int y2 = 0;
            int x2 = 0;
            int y1 = 0;
            int x1 = 0;
            int index = -1;
            const bool validY2 = floatToInt(nativePopFloat(), y2);
            const bool validX2 = floatToInt(nativePopFloat(), x2);
            const bool validY1 = floatToInt(nativePopFloat(), y1);
            const bool validX1 = floatToInt(nativePopFloat(), x1);
            const bool validIndex = floatToInt(nativePopFloat(), index);
            if (validY2 && validX2 && validY1 && validX1 && validIndex &&
                validTextureIndex(index)) {
                fillRect(scriptTextures[index], x1, y1, x2, y2);
            }
        } else if (name == "sttextout") {
            int letterSpacing = 0;
            const bool validSpacing = floatToInt(nativePopFloat(), letterSpacing);
            const std::uint32_t colorValue = packedColor(nativePopFloat());
            nativePopFloat();
            int y = 0;
            int x = 0;
            int index = -1;
            const bool validY = floatToInt(nativePopFloat(), y);
            const bool validX = floatToInt(nativePopFloat(), x);
            const bool validIndex = floatToInt(nativePopFloat(), index);
            const std::string value = nativePopString();
            const std::array<std::uint8_t, 4> color = {
                static_cast<std::uint8_t>((colorValue >> 16) & 0xffU),
                static_cast<std::uint8_t>((colorValue >> 8) & 0xffU),
                static_cast<std::uint8_t>(colorValue & 0xffU),
                static_cast<std::uint8_t>((colorValue >> 24) & 0xffU)};
            if (validSpacing && validY && validX && validIndex && validTextureIndex(index) &&
                validDrawingOrigin(x) && validDrawingOrigin(y) &&
                letterSpacing >= -openbus::scripting::maxScriptTextureDimension &&
                letterSpacing <= openbus::scripting::maxScriptTextureDimension &&
                value.size() <= openbus::scripting::maxScriptTextLength) {
                drawText(scriptTextures[index], value, x, y, color, letterSpacing);
            }
        } else if (name == "streadpixel") {
            int y = 0;
            int x = 0;
            int index = -1;
            const bool validY = floatToInt(nativePopFloat(), y);
            const bool validX = floatToInt(nativePopFloat(), x);
            const bool validIndex = floatToInt(nativePopFloat(), index);
            if (validY && validX && validIndex && validTextureIndex(index)) {
                ScriptTexture& texture = scriptTextures[index];
                if (x >= 0 && y >= 0 && x < texture.width && y < texture.height) {
                    const std::size_t offset = pixelOffset(texture, x, y);
                    texture.color = {texture.pixels[offset + 3], texture.pixels[offset],
                                     texture.pixels[offset + 1], texture.pixels[offset + 2]};
                }
            }
        } else if (name == "stcopycolor") {
            int destination = -1;
            int origin = -1;
            const bool validDestination = floatToInt(nativePopFloat(), destination);
            const bool validOrigin = floatToInt(nativePopFloat(), origin);
            if (validDestination && validOrigin && validTextureIndex(destination) &&
                validTextureIndex(origin)) {
                scriptTextures[destination].color = scriptTextures[origin].color;
            }
        } else if (name == "stloadtex") {
            int index = -1;
            const bool validIndex = floatToInt(nativePopFloat(), index);
            const std::string path = nativePopString();
            if (validIndex && validTextureIndex(index)) {
                loadScriptTexture(index, path);
            }
        } else if (name == "stgetr" || name == "stgetg" || name == "stgetb" || name == "stgeta") {
            int index = -1;
            const bool validIndex = floatToInt(nativePopFloat(), index);
            const std::size_t channel = name == "stgeta"   ? 0
                                        : name == "stgetr" ? 1
                                        : name == "stgetg" ? 2
                                                           : 3;
            pushNumber(validIndex && validTextureIndex(index) ? scriptTextures[index].color[channel]
                                                              : 0.0F);
        } else if (name == "getfontindex") {
            popStrings(1);
            pushNumber(0.0);
        } else if (name == "textlength") {
            nativePopFloat();
            const std::string value = nativePopString();
            pushNumber(static_cast<float>(value.size() * 6));
        }
    }

    bool executeNativeFunction(const std::string& functionName, int callDepth = 0) {
        openbus::rendering::TraceScope trace("osc_native", functionName.c_str());
        if (callDepth > 64) {
            errors.push_back("native OSC call depth exceeded in " + functionName);
            return false;
        }
        for (const std::shared_ptr<const OscProgram>& program : nativePrograms) {
            const auto found = program->functions.find(functionName);
            if (found == program->functions.end()) {
                continue;
            }

            const std::vector<OscInstruction>& code = found->second;
            std::size_t instructionPointer = 0;
            while (instructionPointer < code.size()) {
                const OscInstruction& instruction = code[instructionPointer++];
                const auto binary = [this](auto operation) {
                    const float right = nativePopFloat();
                    const float left = nativePopFloat();
                    floatStack.push_back(operation(left, right));
                };
                switch (instruction.opcode) {
                case OscOpcode::PushNumber:
                    floatStack.push_back(instruction.number);
                    break;
                case OscOpcode::PushString:
                    stringStack.push_back(instruction.name);
                    break;
                case OscOpcode::LoadLocal:
                    floatStack.push_back(localState.get(instruction.name));
                    break;
                case OscOpcode::LoadLocalString:
                    stringStack.push_back(localState.getString(instruction.name));
                    break;
                case OscOpcode::StoreLocal:
                    localState.set(instruction.name, nativePeekFloat());
                    break;
                case OscOpcode::StoreLocalString:
                    localState.setString(instruction.name, nativePopString());
                    break;
                case OscOpcode::LoadSystem:
                    floatStack.push_back(sharedState.sharedVariables().get(instruction.name));
                    break;
                case OscOpcode::StoreSystem:
                    sharedState.sharedVariables().set(instruction.name, nativePeekFloat());
                    break;
                case OscOpcode::LoadConstant: {
                    const auto constant = configuration.constants.find(instruction.name);
                    const float value =
                        constant == configuration.constants.end() ? 0.0F : constant->second;
                    floatStack.push_back(value);
                    break;
                }
                case OscOpcode::CallCurve: {
                    const float value = nativePopFloat();
                    const auto curve = configuration.curves.find(instruction.name);
                    if (curve == configuration.curves.end() || curve->second.points.empty()) {
                        floatStack.push_back(value);
                        break;
                    }
                    const auto& points = curve->second.points;
                    if (value <= points.front().x) {
                        floatStack.push_back(points.front().y);
                        break;
                    }
                    for (std::size_t index = 1; index < points.size(); ++index) {
                        const float rightX = points[index].x;
                        if (value <= rightX) {
                            const auto& left = points[index - 1];
                            const auto& right = points[index];
                            const float leftX = left.x;
                            const float leftY = left.y;
                            const float rightY = right.y;
                            const float fraction = (value - leftX) / (rightX - leftX);
                            floatStack.push_back(leftY + fraction * (rightY - leftY));
                            break;
                        }
                        if (index + 1 == points.size()) {
                            floatStack.push_back(points.back().y);
                        }
                    }
                    break;
                }
                case OscOpcode::LoadRegister:
                    floatStack.push_back(
                        nativeRegisters[static_cast<std::size_t>(instruction.index)]);
                    break;
                case OscOpcode::StoreRegister:
                    nativeRegisters[static_cast<std::size_t>(instruction.index)] =
                        nativePeekFloat();
                    break;
                case OscOpcode::Duplicate:
                    floatStack.push_back(nativePeekFloat());
                    break;
                case OscOpcode::Add:
                    binary([](float left, float right) { return left + right; });
                    break;
                case OscOpcode::Subtract:
                    binary([](float left, float right) { return left - right; });
                    break;
                case OscOpcode::Multiply:
                    binary([](float left, float right) { return left * right; });
                    break;
                case OscOpcode::Divide:
                    binary([](float left, float right) {
                        return right == 0.0F ? 0.0F : left / right;
                    });
                    break;
                case OscOpcode::Modulo:
                    binary([](float left, float right) {
                        return right == 0.0F ? 0.0F : left - std::floor(left / right) * right;
                    });
                    break;
                case OscOpcode::Negate:
                    floatStack.push_back(-nativePopFloat());
                    break;
                case OscOpcode::LogicalNot:
                    floatStack.push_back(nativePopFloat() == 0.0F ? 1.0F : 0.0F);
                    break;
                case OscOpcode::Equal:
                    binary([](float left, float right) { return left == right ? 1.0F : 0.0F; });
                    break;
                case OscOpcode::NotEqual:
                    binary([](float left, float right) { return left != right ? 1.0F : 0.0F; });
                    break;
                case OscOpcode::Less:
                    binary([](float left, float right) { return left < right ? 1.0F : 0.0F; });
                    break;
                case OscOpcode::LessEqual:
                    binary([](float left, float right) { return left <= right ? 1.0F : 0.0F; });
                    break;
                case OscOpcode::Greater:
                    binary([](float left, float right) { return left > right ? 1.0F : 0.0F; });
                    break;
                case OscOpcode::GreaterEqual:
                    binary([](float left, float right) { return left >= right ? 1.0F : 0.0F; });
                    break;
                case OscOpcode::LogicalAnd:
                    binary([](float left, float right) {
                        return left != 0.0F && right != 0.0F ? 1.0F : 0.0F;
                    });
                    break;
                case OscOpcode::LogicalOr:
                    binary([](float left, float right) {
                        return left != 0.0F || right != 0.0F ? 1.0F : 0.0F;
                    });
                    break;
                case OscOpcode::Absolute:
                    floatStack.push_back(std::abs(nativePopFloat()));
                    break;
                case OscOpcode::Minimum:
                    binary([](float left, float right) { return std::min(left, right); });
                    break;
                case OscOpcode::Maximum:
                    binary([](float left, float right) { return std::max(left, right); });
                    break;
                case OscOpcode::Floor:
                    floatStack.push_back(std::floor(nativePopFloat()));
                    break;
                case OscOpcode::Ceiling:
                    floatStack.push_back(std::ceil(nativePopFloat()));
                    break;
                case OscOpcode::Sine:
                    floatStack.push_back(std::sin(nativePopFloat()));
                    break;
                case OscOpcode::Cosine:
                    floatStack.push_back(std::cos(nativePopFloat()));
                    break;
                case OscOpcode::Tangent:
                    floatStack.push_back(std::tan(nativePopFloat()));
                    break;
                case OscOpcode::ArcTangent:
                    floatStack.push_back(std::atan(nativePopFloat()));
                    break;
                case OscOpcode::SquareRoot:
                    floatStack.push_back(std::sqrt(std::max(0.0F, nativePopFloat())));
                    break;
                case OscOpcode::ArcSine:
                    floatStack.push_back(std::asin(std::clamp(nativePopFloat(), -1.0F, 1.0F)));
                    break;
                case OscOpcode::Exponential:
                    floatStack.push_back(std::exp(nativePopFloat()));
                    break;
                case OscOpcode::Square: {
                    const float value = nativePopFloat();
                    floatStack.push_back(value * value);
                    break;
                }
                case OscOpcode::Sign: {
                    const float value = nativePopFloat();
                    floatStack.push_back(value > 0.0 ? 1.0 : value < 0.0 ? -1.0 : 0.0);
                    break;
                }
                case OscOpcode::Random: {
                    const int limit =
                        std::max(0, static_cast<int>(std::floor(nativePopFloat())) - 1);
                    floatStack.push_back(
                        limit == 0 ? 0.0F : static_cast<float>(std::rand() % (limit + 1)));
                    break;
                }
                case OscOpcode::StringDuplicate:
                    stringStack.push_back(nativePeekString());
                    break;
                case OscOpcode::StringConcat: {
                    const std::string right = nativePopString();
                    const std::string left = nativePopString();
                    stringStack.push_back(left + right);
                    break;
                }
                case OscOpcode::StringRepeat: {
                    const int length = std::max(0, static_cast<int>(std::floor(nativePopFloat())));
                    const std::string value = nativePopString();
                    std::string result;
                    if (!value.empty()) {
                        while (static_cast<int>(result.size() + value.size()) <= length) {
                            result += value;
                        }
                    }
                    stringStack.push_back(std::move(result));
                    break;
                }
                case OscOpcode::StringLength:
                    floatStack.push_back(static_cast<float>(nativePeekString().size()));
                    break;
                case OscOpcode::StringCutBegin: {
                    const int count = std::max(0, static_cast<int>(std::floor(nativePopFloat())));
                    const std::string value = nativePopString();
                    stringStack.push_back(
                        count < static_cast<int>(value.size()) ? value.substr(count) : "");
                    break;
                }
                case OscOpcode::StringCutEnd: {
                    const int count = std::max(0, static_cast<int>(std::floor(nativePopFloat())));
                    const std::string value = nativePopString();
                    stringStack.push_back(count > 0 && count < static_cast<int>(value.size())
                                              ? value.substr(0, value.size() - count)
                                              : value);
                    break;
                }
                case OscOpcode::StringSetLengthLeft:
                case OscOpcode::StringSetLengthRight:
                case OscOpcode::StringSetLengthCenter: {
                    const int length = std::max(0, static_cast<int>(std::floor(nativePeekFloat())));
                    const std::string value = nativePopString();
                    if (static_cast<int>(value.size()) > length) {
                        if (instruction.opcode == OscOpcode::StringSetLengthRight) {
                            stringStack.push_back(value.substr(value.size() - length));
                        } else if (instruction.opcode == OscOpcode::StringSetLengthCenter) {
                            const std::size_t start = (value.size() - length) / 2;
                            stringStack.push_back(value.substr(start, length));
                        } else {
                            stringStack.push_back(value.substr(0, length));
                        }
                    } else if (static_cast<int>(value.size()) < length) {
                        const std::string padding(static_cast<std::size_t>(length) - value.size(),
                                                  ' ');
                        stringStack.push_back(
                            instruction.opcode == OscOpcode::StringSetLengthRight ? padding + value
                            : instruction.opcode == OscOpcode::StringSetLengthCenter
                                ? padding.substr(0, padding.size() / 2) + value +
                                      padding.substr(padding.size() / 2)
                                : value + padding);
                    } else {
                        stringStack.push_back(value);
                    }
                    break;
                }
                case OscOpcode::IntegerToString:
                    stringStack.push_back(
                        std::to_string(static_cast<long long>(std::floor(nativePeekFloat()))));
                    break;
                case OscOpcode::IntegerToStringEnhanced: {
                    const std::string format = nativePopString();
                    const long long value = static_cast<long long>(std::floor(nativePopFloat()));
                    if (format.size() < 2) {
                        stringStack.emplace_back("ERROR");
                        break;
                    }
                    const char fill = format.front();
                    int digits = 0;
                    try {
                        digits = std::stoi(format.substr(1));
                    } catch (...) {
                        stringStack.emplace_back("ERROR");
                        break;
                    }
                    std::string result = std::to_string(value);
                    if (static_cast<int>(result.size()) < digits) {
                        result.insert(result.begin(), digits - result.size(), fill);
                    }
                    stringStack.push_back(std::move(result));
                    break;
                }
                case OscOpcode::StringToFloat: {
                    const std::string value = nativePopString();
                    try {
                        floatStack.push_back(std::stof(value));
                    } catch (...) {
                        floatStack.push_back(-1.0);
                    }
                    break;
                }
                case OscOpcode::RemoveSpaces: {
                    std::string value = nativePopString();
                    const auto first = value.find_first_not_of(" \t\r\n");
                    const auto last = value.find_last_not_of(" \t\r\n");
                    stringStack.push_back(
                        first == std::string::npos ? "" : value.substr(first, last - first + 1));
                    break;
                }
                case OscOpcode::StringEqual:
                case OscOpcode::StringLess:
                case OscOpcode::StringGreater:
                case OscOpcode::StringLessEqual:
                case OscOpcode::StringGreaterEqual: {
                    const std::string right = nativePopString();
                    const std::string left = nativePopString();
                    bool result = false;
                    if (instruction.opcode == OscOpcode::StringEqual)
                        result = left == right;
                    if (instruction.opcode == OscOpcode::StringLess)
                        result = left < right;
                    if (instruction.opcode == OscOpcode::StringGreater)
                        result = left > right;
                    if (instruction.opcode == OscOpcode::StringLessEqual)
                        result = left <= right;
                    if (instruction.opcode == OscOpcode::StringGreaterEqual)
                        result = left >= right;
                    floatStack.push_back(result ? 1.0 : 0.0);
                    break;
                }
                case OscOpcode::StringNoOp:
                    break;
                case OscOpcode::DebugString:
                    log("omsi_debug(" + nativePeekString() + ")");
                    break;
                case OscOpcode::StackDump:
                    break;
                case OscOpcode::JumpIfFalse:
                    if (nativePopFloat() == 0.0) {
                        instructionPointer = static_cast<std::size_t>(instruction.index);
                    }
                    break;
                case OscOpcode::Jump:
                    instructionPointer = static_cast<std::size_t>(instruction.index);
                    break;
                case OscOpcode::CallFunction:
                    if (!executeNativeFunction(instruction.name, callDepth + 1)) {
                        return false;
                    }
                    break;
                case OscOpcode::CallSystemMacro:
                    executeSystemMacro(instruction.name);
                    break;
                case OscOpcode::SoundTrigger:
                    if (onSoundTrigger) {
                        onSoundTrigger(instruction.name, {}, soundControlValue(instruction.name));
                    }
                    break;
                case OscOpcode::SoundTriggerFile: {
                    const std::string file = nativePopString();
                    if (onSoundTrigger) {
                        onSoundTrigger(instruction.name, file, soundControlValue(instruction.name));
                    }
                    break;
                }
                }
            }
            return true;
        }
        return false;
    }

    bool tryInitializeNativeBackend() {
        const char* backend = openbus::getEnvironment("OPENBUS_SCRIPT_BACKEND");
        if (backend && lower(backend) == "lua") {
            scriptRuntimeLogger.Log("Lua script backend explicitly selected");
            return false;
        }

        bool compiledAnyScript = false;
        for (const std::string& referencedPath : configuration.scripts) {
            std::string normalized = referencedPath;
            std::replace(normalized.begin(), normalized.end(), '\\', '/');
            const std::filesystem::path sourcePath =
                configuration.sourcePath.parent_path() / normalized;
            std::string error;
            std::shared_ptr<const OscProgram> program =
                cachedNativeProgram(sourcePath, error, compiledAnyScript);
            if (!program) {
                scriptRuntimeLogger.Log("Native OSC backend falling back to Lua for " +
                                        sourcePath.string() + ": " + error);
                nativePrograms.clear();
                return false;
            }
            nativePrograms.push_back(std::move(program));
        }
        nativeBackend = !nativePrograms.empty();
        if (nativeBackend && compiledAnyScript) {
            scriptRuntimeLogger.Log("Native OSC backend enabled for " +
                                    std::to_string(nativePrograms.size()) + " script(s)");
        }
        return nativeBackend;
    }

    static bool compileOnlyRequested() {
        const char* value = openbus::getEnvironment("OPENBUS_SCRIPT_COMPILE_ONLY");
        return value && std::string(value) == "1";
    }

    Impl(const VehicleConfig& source, Variables& local, SimulationState& shared)
        : configuration(source),
          scriptIdentity(source.sourcePath.empty() ? "unknown"
                                                   : source.sourcePath.filename().string()),
          localState(local), sharedState(shared) {
        for (const std::string& variable : configuration.floatVariables) {
            localState.declare(variable);
        }
        for (const std::string& variable : configuration.stringVariables) {
            localState.declareString(variable);
        }
        if (localState.objectKind() == ScriptObjectKind::Vehicle) {
            for (const char* variable : {"ident", "number", "act_route", "act_busstop", "setlineto",
                                         "yard", "file_schedule"}) {
                localState.declareString(variable);
            }
        }
        if (!configuration.selectedRegistration.empty() ||
            !configuration.selectedVehicleNumber.empty()) {
            // The plate text surface reads `ident`; scripts and fleet-number
            // surfaces read `number`.
            localState.setString("ident", configuration.selectedRegistration);
            localState.setString("number", configuration.selectedVehicleNumber.empty()
                                               ? configuration.selectedRegistration
                                               : configuration.selectedVehicleNumber);
        }

        if (tryInitializeNativeBackend()) {
            return;
        }

        state = luaL_newstate();
        if (!state) {
            errors.push_back("unable to create Lua state");
            return;
        }
        lua_atpanic(state, &Impl::panicHandler);
        luaL_openlibs(state);
        registerFunctions();
        loadScripts();
    }

    void log(const std::string& message) const {
        scriptRuntimeLogger.Log("[" + scriptIdentity + "] " + message);
    }

    static int panicHandler(lua_State* lua) {
        const char* message = lua_tostring(lua, -1);
        const std::string detail = message ? message : "non-string Lua error object";
        const std::string output =
            "Lua panic: " + detail + " (stack=" + std::to_string(lua_gettop(lua)) + ")";
        scriptRuntimeLogger.Log(output);
        std::cerr << output << '\n';
        return 0;
    }

    ~Impl() {
        if (state) {
            luaL_unref(state, LUA_REGISTRYINDEX, scriptEnvironment.reference);
            lua_close(state);
        }
    }

    Impl* self(lua_State* lua) {
        return static_cast<Impl*>(lua_touserdata(lua, lua_upvalueindex(1)));
    }

    static float numericArgument(lua_State* lua, int index) {
        if (lua_isboolean(lua, index)) {
            return lua_toboolean(lua, index) != 0 ? 1.0F : 0.0F;
        }
        return static_cast<float>(luaL_checknumber(lua, index));
    }

    static int oscFloat32(lua_State* lua) {
        lua_pushnumber(lua, static_cast<float>(luaL_checknumber(lua, 1)));
        return 1;
    }

    static int getLocal(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const char* name = luaL_checkstring(lua, 1);
        const float value = runtime->localState.get(name);
        // runtime->log(lua, "get_local_var(" + std::string(name) + "): " + std::to_string(value));
        lua_pushnumber(lua, value);
        return 1;
    }

    static int setLocal(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const char* name = luaL_checkstring(lua, 1);
        const float value = numericArgument(lua, 2);
        // runtime->log(lua, "set_local_var(" + std::string(name) + ", " + std::to_string(value) +
        // ")");
        runtime->localState.set(name, value);
        return 0;
    }

    static int getLocalString(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // runtime->log(lua, "get_local_str(" + std::string(luaL_checkstring(lua, 1)) + ")");
        const std::string value = runtime->localState.getString(luaL_checkstring(lua, 1));
        lua_pushstring(lua, value.c_str());
        return 1;
    }

    static int setLocalString(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // runtime->log(lua, "set_local_str(" + std::string(luaL_checkstring(lua, 1)) + ", " +
        // std::string(luaL_checkstring(lua, 2)) + ")");
        runtime->localState.setString(luaL_checkstring(lua, 1), luaL_checkstring(lua, 2));
        return 0;
    }

    static int getSystem(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // runtime->log(lua, "get_sys_var(" + std::string(luaL_checkstring(lua, 1)) + ")");
        lua_pushnumber(lua, runtime->sharedState.sharedVariables().get(luaL_checkstring(lua, 1)));
        return 1;
    }

    static int setSystem(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // runtime->log(lua, "set_sys_var(" + std::string(luaL_checkstring(lua, 1)) + ", " +
        // std::to_string(numericArgument(lua, 2)) + ")");
        runtime->sharedState.sharedVariables().set(luaL_checkstring(lua, 1),
                                                   numericArgument(lua, 2));
        return 0;
    }

    static int getConstant(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // runtime->log(lua, "get_const(" + std::string(luaL_checkstring(lua, 1)) + ")");
        const auto found = runtime->configuration.constants.find(luaL_checkstring(lua, 1));
        lua_pushnumber(lua, found == runtime->configuration.constants.end() ? 0.0F : found->second);
        return 1;
    }

    static int callFunction(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // runtime->log(lua, "call_func(" + std::string(luaL_checkstring(lua, 1)) + ", " +
        // std::to_string(numericArgument(lua, 2)) + ")");
        const std::string name = luaL_checkstring(lua, 1);
        const float value = numericArgument(lua, 2);
        const auto found = runtime->configuration.curves.find(name);
        if (found != runtime->configuration.curves.end() && !found->second.points.empty()) {
            const ConstantCurve& curve = found->second;
            if (value <= curve.points.front().x) {
                lua_pushnumber(lua, curve.points.front().y);
                return 1;
            }
            for (std::size_t index = 1; index < curve.points.size(); ++index) {
                const float rightX = curve.points[index].x;
                if (value <= rightX) {
                    const ConstantCurvePoint& left = curve.points[index - 1];
                    const ConstantCurvePoint& right = curve.points[index];
                    const float leftX = left.x;
                    const float leftY = left.y;
                    const float rightY = right.y;
                    const float fraction = (value - leftX) / (rightX - leftX);
                    lua_pushnumber(lua, leftY + fraction * (rightY - leftY));
                    return 1;
                }
            }
            lua_pushnumber(lua, curve.points.back().y);
            return 1;
        }
        lua_pushnumber(lua, value);
        return 1;
    }

    static int pushFloat(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // // runtime->log(lua, "_pushf(" + std::to_string(numericArgument(lua, 1)) + ")");
        runtime->floatStack.push_back(numericArgument(lua, 1));
        return 0;
    }

    static int popFloat(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // // runtime->log(lua, "_popf()");
        if (runtime->floatStack.empty()) {
            lua_pushnumber(lua, 0.0);
        } else {
            lua_pushnumber(lua, runtime->floatStack.back());
            runtime->floatStack.pop_back();
        }
        return 1;
    }

    static int peekFloat(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // // runtime->log(lua, "_peekf(" + std::to_string(runtime->floatStack.empty() ? 0.0 :
        // runtime->floatStack.back()) + ")");
        lua_pushnumber(lua, runtime->floatStack.empty() ? 0.0 : runtime->floatStack.back());
        return 1;
    }

    static int pushString(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // // runtime->log(lua, "_pushs(" + std::string(luaL_checkstring(lua, 1)) + ")");
        runtime->stringStack.emplace_back(luaL_checkstring(lua, 1));
        return 0;
    }

    static int popString(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        if (runtime->stringStack.empty()) {
            lua_pushliteral(lua, "");
        } else {
            lua_pushstring(lua, runtime->stringStack.back().c_str());
            runtime->stringStack.pop_back();
        }
        return 1;
    }

    static int peekString(lua_State* lua) {
        // scriptRuntimeLogger.Log("_peeks()");
        Impl* runtime = runtimeFor(lua);
        lua_pushstring(lua,
                       runtime->stringStack.empty() ? "" : runtime->stringStack.back().c_str());
        return 1;
    }

    static int soundTrigger(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const std::string name = luaL_checkstring(lua, 1);
        if (runtime->onSoundTrigger) {
            runtime->onSoundTrigger(name, {}, runtime->soundControlValue(name));
        }
        return 0;
    }

    static int soundTriggerFile(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const std::string name = luaL_checkstring(lua, 1);
        const std::string file = luaL_checkstring(lua, 2);
        if (runtime->onSoundTrigger) {
            runtime->onSoundTrigger(name, file, runtime->soundControlValue(name));
        }
        return 0;
    }

    static int debug(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        runtime->log("omsi_debug(" + std::string(luaL_checkstring(lua, 1)) + ")");
        return 0;
    }

    // TODO: Replace safe system-macro fallbacks with HOF, timetable, and vehicle data.
    static float popSystemFloat(Impl* runtime) {
        if (runtime->floatStack.empty()) {
            return 0.0F;
        }
        const float value = runtime->floatStack.back();
        runtime->floatStack.pop_back();
        return value;
    }

    static std::string popSystemString(Impl* runtime) {
        if (runtime->stringStack.empty()) {
            return {};
        }
        std::string value = std::move(runtime->stringStack.back());
        runtime->stringStack.pop_back();
        return value;
    }

    static int returnSystemFloat(lua_State* lua, float value) {
        Impl* runtime = runtimeFor(lua);
        runtime->floatStack.push_back(value);
        return 0;
    }

    static int returnSystemString(lua_State* lua, const char* value = "") {
        Impl* runtime = runtimeFor(lua);
        runtime->stringStack.emplace_back(value);
        return 0;
    }

    // TODO: Implement the corresponding HOF, timetable, or passenger lookup.
    static int safeNumericLookup(lua_State* lua) {
        popSystemFloat(runtimeFor(lua));
        return returnSystemFloat(lua, 0.0);
    }

    // TODO: Implement the corresponding HOF lookup and return the actual missing index.
    static int safeMissingIndex(lua_State* lua) {
        popSystemFloat(runtimeFor(lua));
        return returnSystemFloat(lua, -1.0);
    }

    // TODO: Implement the corresponding HOF lookup and return the requested string.
    static int safeStringLookup(lua_State* lua) {
        popSystemFloat(runtimeFor(lua));
        popSystemFloat(runtimeFor(lua));
        return returnSystemString(lua);
    }

    // TODO: Implement route bus-stop lists from the active HOF data.
    static int safeRouteBusstopIdent(lua_State* lua) {
        popSystemFloat(runtimeFor(lua));
        popSystemFloat(runtimeFor(lua));
        return returnSystemString(lua);
    }

    // TODO: Implement current line number
    static int safeCurrentLineNumber(lua_State* lua) {
        popSystemFloat(runtimeFor(lua));
        return returnSystemFloat(lua, 0.0);
    }

    // TODO: Implement vehicle-relative ground ray casting.
    static int safeGetHeightAbovePoint(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        popSystemFloat(runtime);
        popSystemFloat(runtime);
        popSystemFloat(runtime);
        return returnSystemFloat(lua, 0.0);
    }

    // TODO: Implement deterministic fleet-number and seed-based randomization.
    static int safeNrSpecRandom(lua_State* lua) {
        const float seed = popSystemFloat(runtimeFor(lua));
        std::minstd_rand rand(static_cast<unsigned int>(seed));
        return returnSystemFloat(lua, static_cast<float>(rand()) /
                                          static_cast<float>(std::minstd_rand::max()));
    }

    // TODO: Implement currency configuration and change-coin issuance.
    static int safeGiveChangeCoin(lua_State* lua) {
        popSystemFloat(runtimeFor(lua));
        return 0;
    }

    // TODO: Implement ticket-pack and global-string lookups.
    static int safeTicketName(lua_State* lua) {
        popSystemFloat(runtimeFor(lua));
        return returnSystemString(lua);
    }

    static void discardSystemFloats(Impl* runtime, std::size_t count) {
        while (count-- > 0) {
            popSystemFloat(runtime);
        }
    }

    static void discardSystemStrings(Impl* runtime, std::size_t count) {
        while (count-- > 0 && !runtime->stringStack.empty()) {
            runtime->stringStack.pop_back();
        }
    }

    // TODO: Implement arrival-board line and destination data from the parent bus stop.
    static int safeArrivalString(lua_State* lua) {
        popSystemFloat(runtimeFor(lua));
        return returnSystemString(lua);
    }

    // TODO: Implement arrival-board time differences from the parent bus stop.
    static int safeArrivalTime(lua_State* lua) {
        popSystemFloat(runtimeFor(lua));
        return returnSystemFloat(lua, 0.0);
    }

    static ScriptTexture& scriptTexture(Impl* runtime, int index) {
        return runtime->scriptTextures[index];
    }

    static bool popSystemInt(Impl* runtime, int& value) {
        return floatToInt(popSystemFloat(runtime), value);
    }

    static bool popSystemTextureIndex(Impl* runtime, int& index) {
        return popSystemInt(runtime, index) && validTextureIndex(index);
    }

    bool resizeScriptTexture(ScriptTexture& texture, int requiredWidth, int requiredHeight) {
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
            std::copy_n(texture.pixels.begin() + sourceOffset, rowBytes,
                        pixels.begin() + targetOffset);
        }
        texture.width = width;
        texture.height = height;
        texture.pixels = std::move(pixels);
        return true;
    }

    static std::size_t pixelOffset(const ScriptTexture& texture, int x, int y) {
        return (static_cast<std::size_t>(y) * texture.width + x) * 4;
    }

    void setPixel(ScriptTexture& texture, int x, int y) {
        if (texture.locked || x < 0 || y < 0 ||
            x >= openbus::scripting::maxScriptTextureDimension ||
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

    void fillRect(ScriptTexture& texture, int x1, int y1, int x2, int y2) {
        if (texture.locked) {
            return;
        }
        const int left = std::max(0, std::min(x1, x2));
        const int top = std::max(0, std::min(y1, y2));
        const int right =
            std::min(openbus::scripting::maxScriptTextureDimension - 1, std::max(x1, x2));
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
                std::copy(texture.color.begin(), texture.color.end(),
                          texture.pixels.begin() + offset);
            }
        }
        ++texture.revision;
    }

    static void clearTexture(ScriptTexture& texture) {
        texture.pixels.assign(static_cast<std::size_t>(texture.width) *
                                  static_cast<std::size_t>(texture.height) * 4,
                              0);
    }

    static void drawText(ScriptTexture& texture, const std::string& value, int x, int y,
                         std::array<std::uint8_t, 4> color, int letterSpacing = 0,
                         int maximumScale = 8, TextAlignment alignment = TextAlignment::Left,
                         bool centerVertically = false) {
        if (texture.locked || value.empty() ||
            value.size() > openbus::scripting::maxScriptTextLength ||
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
        const int availableWidth =
            std::max(1, texture.width - 2 -
                            spacing * static_cast<int>(longestLine > 0 ? longestLine - 1 : 0));
        const int widthScale =
            std::max(1, availableWidth / std::max(1, 6 * static_cast<int>(longestLine)));
        const int multilineHeight =
            std::max(1, (texture.height - 2) / static_cast<int>(7 * lineCount));
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

    static bool drawBitmapFontText(ScriptTexture& texture, const std::string& value, int x, int y,
                                   const TextTextureDefinition& definition, const FontAsset& font) {
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
                            (static_cast<std::size_t>(sourceY) * font.alphaImage.width + sourceX) *
                            4;
                        const std::size_t colorOffset =
                            (static_cast<std::size_t>(sourceY) * font.colorImage.width + sourceX) *
                            4;
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

    void updateTextTextures(bool force = false) {
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
                drawText(texture, value, 0, 0, definition.color, definition.gridSpacing,
                         maximumScale, definition.alignment, true);
            } else if (!bitmapFontRendered) {
                ++texture.revision;
            }
        }
        // Refresh_Strings is a one-shot request, consumed after the script pass
        // even if all configured strings were unchanged.
        localState.set("refresh_strings", 0.0);
    }

    bool loadScriptTexture(int index, const std::string& requestedPath) {
        if (!validTextureIndex(index)) {
            return false;
        }
        std::string normalized = requestedPath;
        std::replace(normalized.begin(), normalized.end(), '\\', '/');
        const std::filesystem::path path = configuration.sourcePath.parent_path() / normalized;
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

    static int safeScriptTextureAction(lua_State* lua) {
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

    static int safeScriptTextureNew(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        int index = -1;
        if (!popSystemTextureIndex(runtime, index)) {
            return 0;
        }
        runtime->scriptTextures[index] = runtime->configuredScriptTexture(index);
        return 0;
    }

    static int safeScriptTextureLock(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        int index = -1;
        if (popSystemTextureIndex(runtime, index)) {
            scriptTexture(runtime, index).locked = true;
        }
        return 0;
    }

    static int safeScriptTextureUnlock(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        int index = -1;
        if (popSystemTextureIndex(runtime, index)) {
            scriptTexture(runtime, index).locked = false;
        }
        return 0;
    }

    static int safeScriptTextureFilter(lua_State* lua) {
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

    static int safeScriptTextureColor(lua_State* lua) {
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

    static int safeScriptTexturePixel(lua_State* lua) {
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

    static int safeScriptTextureReadPixel(lua_State* lua) {
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

    static int safeScriptTextureRect(lua_State* lua) {
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

    static int safeScriptTextureText(lua_State* lua) {
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
            validDrawingOrigin(y) &&
            letterSpacing >= -openbus::scripting::maxScriptTextureDimension &&
            letterSpacing <= openbus::scripting::maxScriptTextureDimension &&
            value.size() <= openbus::scripting::maxScriptTextLength) {
            drawText(scriptTexture(runtime, index), value, x, y, color, letterSpacing);
        }
        return 0;
    }

    static int safeScriptTextureCopyColor(lua_State* lua) {
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

    static int safeScriptTextureLoad(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        int index = -1;
        const bool validIndex = popSystemTextureIndex(runtime, index);
        const std::string path = popSystemString(runtime);
        if (validIndex) {
            runtime->loadScriptTexture(index, path);
        }
        return 0;
    }

    static int safeScriptTextureChannel(lua_State* lua, std::size_t channel) {
        Impl* runtime = runtimeFor(lua);
        int index = -1;
        if (!popSystemTextureIndex(runtime, index)) {
            return returnSystemFloat(lua, 0.0);
        }
        const ScriptTexture& texture = scriptTexture(runtime, index);
        return returnSystemFloat(lua, texture.color[channel]);
    }

    static int safeScriptTextureRed(lua_State* lua) {
        return safeScriptTextureChannel(lua, 1);
    }

    static int safeScriptTextureGreen(lua_State* lua) {
        return safeScriptTextureChannel(lua, 2);
    }

    static int safeScriptTextureBlue(lua_State* lua) {
        return safeScriptTextureChannel(lua, 3);
    }

    static int safeScriptTextureAlpha(lua_State* lua) {
        return safeScriptTextureChannel(lua, 0);
    }

    static int safeFontIndex(lua_State* lua) {
        discardSystemStrings(runtimeFor(lua), 1);
        return returnSystemFloat(lua, 0.0);
    }

    static int safeTextLength(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        discardSystemFloats(runtime, 1);
        const std::string value = popSystemString(runtime);
        return returnSystemFloat(lua, static_cast<float>(value.size() * 6));
    }

    static int tracedLuaFunction(lua_State* lua) {
        const auto* binding =
            static_cast<const LuaFunctionBinding*>(lua_touserdata(lua, lua_upvalueindex(2)));
        openbus::rendering::TraceScope trace("lua_callback", binding->name.c_str());
        return binding->function(lua);
    }

    static int genericSystemMacro(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const auto* binding =
            static_cast<const LuaFunctionBinding*>(lua_touserdata(lua, lua_upvalueindex(2)));
        runtime->executeSystemMacro(binding->name.substr(10));
        return 0;
    }

    static Impl* runtimeFor(lua_State* lua) {
        return static_cast<Impl*>(lua_touserdata(lua, lua_upvalueindex(1)));
    }

    void registerFunction(const char* name, lua_CFunction function) {
        auto binding = std::make_unique<LuaFunctionBinding>();
        binding->name = name;
        binding->function = function;
        LuaFunctionBinding* bindingPointer = binding.get();
        luaFunctionBindings.push_back(std::move(binding));
        lua_pushlightuserdata(state, this);
        lua_pushlightuserdata(state, bindingPointer);
        lua_pushcclosure(state, tracedLuaFunction, 2);
        lua_setglobal(state, name);
    }

    void registerSystemMacro(const char* name, lua_CFunction) {
        if (localState.supportsSystemMacro(name)) {
            registerFunction((std::string("sys_macro_") + name).c_str(), genericSystemMacro);
        }
    }

    void registerFunctions() {
        lua_pushcfunction(state, oscFloat32);
        lua_setglobal(state, "osc_f32");
        registerFunction("get_local_var", getLocal);
        registerFunction("set_local_var", setLocal);
        registerFunction("get_local_str", getLocalString);
        registerFunction("set_local_str", setLocalString);
        registerFunction("get_sys_var", getSystem);
        registerFunction("set_sys_var", setSystem);
        registerFunction("get_const", getConstant);
        registerFunction("call_func", callFunction);
        registerFunction("_pushf", pushFloat);
        registerFunction("_popf", popFloat);
        registerFunction("_peekf", peekFloat);
        registerFunction("_pushs", pushString);
        registerFunction("_pops", popString);
        registerFunction("_peeks", peekString);
        registerFunction("sound_trigger", soundTrigger);
        registerFunction("sound_trigger_file", soundTriggerFile);
        registerFunction("omsi_debug", debug);

        registerSystemMacro("getterminusindex", safeMissingIndex);
        registerSystemMacro("getterminuscode", safeNumericLookup);
        registerSystemMacro("getterminusstring", safeStringLookup);
        registerSystemMacro("getbusstopindex", safeMissingIndex);
        registerSystemMacro("getbusstopstring", safeStringLookup);
        registerSystemMacro("getrouteindex", safeMissingIndex);
        registerSystemMacro("getrouteterminusindex", safeNumericLookup);
        registerSystemMacro("getbusstopcount", safeNumericLookup);
        registerSystemMacro("getroutebusstopident", safeRouteBusstopIdent);
        registerSystemMacro("getttlinestring", safeCurrentLineNumber);
        registerSystemMacro("getttterminusindex", safeNumericLookup);
        registerSystemMacro("getttbusstopcount", safeNumericLookup);
        registerSystemMacro("getttbusstopindex", safeNumericLookup);
        registerSystemMacro("getttdelay", safeNumericLookup);
        registerSystemMacro("getttbusstopname", safeTicketName);
        registerSystemMacro("getttbusstoparr", safeNumericLookup);
        registerSystemMacro("getttbusstopdep", safeNumericLookup);
        registerSystemMacro("givechangecoin", safeGiveChangeCoin);
        registerSystemMacro("getticketname", safeTicketName);
        registerSystemMacro("getticketvalue", safeNumericLookup);
        registerSystemMacro("gethumancountonpathlink", safeNumericLookup);
        registerSystemMacro("nrspecrandom", safeNrSpecRandom);
        registerSystemMacro("getheightabovepoint", safeGetHeightAbovePoint);
        registerSystemMacro("getdepotstringglobal", safeTicketName);
        registerSystemMacro("gethumancountonseat", safeNumericLookup);
        registerSystemMacro("getarrbusline", safeArrivalString);
        registerSystemMacro("getarrbusterminus", safeArrivalString);
        registerSystemMacro("getarrbustimediff", safeArrivalTime);
        registerSystemMacro("stnewtex", safeScriptTextureNew);
        registerSystemMacro("stlock", safeScriptTextureLock);
        registerSystemMacro("stunlock", safeScriptTextureUnlock);
        registerSystemMacro("stfilter", safeScriptTextureFilter);
        registerSystemMacro("stsetcolor", safeScriptTextureColor);
        registerSystemMacro("stdrawpixel", safeScriptTexturePixel);
        registerSystemMacro("stdrawrect", safeScriptTextureRect);
        registerSystemMacro("sttextout", safeScriptTextureText);
        registerSystemMacro("streadpixel", safeScriptTextureReadPixel);
        registerSystemMacro("stcopycolor", safeScriptTextureCopyColor);
        registerSystemMacro("stloadtex", safeScriptTextureLoad);
        registerSystemMacro("stgetr", safeScriptTextureRed);
        registerSystemMacro("stgetg", safeScriptTextureGreen);
        registerSystemMacro("stgetb", safeScriptTextureBlue);
        registerSystemMacro("stgeta", safeScriptTextureAlpha);
        registerSystemMacro("getfontindex", safeFontIndex);
        registerSystemMacro("textlength", safeTextLength);
    }

    void loadScripts() {
        lua_newtable(state);
        lua_newtable(state);
        lua_pushvalue(state, LUA_GLOBALSINDEX);
        lua_setfield(state, -2, "__index");
        lua_setmetatable(state, -2);
        scriptEnvironment.reference = luaL_ref(state, LUA_REGISTRYINDEX);

        for (const std::string& referencedPath : configuration.scripts) {
            std::string normalized = referencedPath;
            std::replace(normalized.begin(), normalized.end(), '\\', '/');
            std::filesystem::path sourcePath = configuration.sourcePath.parent_path() / normalized;
            try {
                sourcePath = generatedLuaPath(sourcePath);
            } catch (const std::exception& exception) {
                errors.push_back("unable to hash script " + sourcePath.string() + ": " +
                                 exception.what());
                continue;
            }
            if (!std::filesystem::exists(sourcePath)) {
                errors.push_back("generated Lua script not found: " + sourcePath.string());
                continue;
            }
            if (luaL_loadfile(state, sourcePath.string().c_str()) != 0) {
                errors.push_back(lua_tostring(state, -1));
                lua_pop(state, 1);
                continue;
            }
            lua_rawgeti(state, LUA_REGISTRYINDEX, scriptEnvironment.reference);
            lua_setfenv(state, -2);
            if (lua_pcall(state, 0, 0, 0) != 0) {
                errors.push_back(lua_tostring(state, -1));
                lua_pop(state, 1);
                continue;
            }
        }
    }

    bool invoke(const char* functionName) {
        openbus::rendering::TraceScope trace("lua", functionName);

        // scriptRuntimeLogger.Log(std::string(functionName) + "()");
        bool invoked = false;
        lua_rawgeti(state, LUA_REGISTRYINDEX, scriptEnvironment.reference);
        lua_getfield(state, -1, functionName);
        if (lua_isfunction(state, -1)) {
            invoked = true;
            if (lua_pcall(state, 0, 0, 0) != 0) {
                errors.push_back(std::string(functionName) + ": " + lua_tostring(state, -1));
                lua_pop(state, 1);
            }
        } else {
            lua_pop(state, 1);
        }
        lua_pop(state, 1);
        return invoked;
    }

    void invokeFrame(bool isAiVehicle) {
        openbus::rendering::TraceScope trace("lua", "frame");

        // scriptRuntimeLogger.Log("invokeFrame(" + std::string(isAiVehicle ? "true" : "false") +
        // ")");
        lua_rawgeti(state, LUA_REGISTRYINDEX, scriptEnvironment.reference);
        const char* functionName = "frame";
        lua_getfield(state, -1, "frame_ai");
        if (isAiVehicle && lua_isfunction(state, -1)) {
            functionName = "frame_ai";
        } else {
            lua_pop(state, 1);
            lua_getfield(state, -1, "frame");
        }
        const bool hasFunction = lua_isfunction(state, -1) != 0;
        if (hasFunction) {
            if (lua_pcall(state, 0, 0, 0) != 0) {
                errors.push_back(std::string(functionName) + ": " + lua_tostring(state, -1));
                lua_pop(state, 1);
            }
        } else {
            lua_pop(state, 1);
        }
        lua_pop(state, 1);
    }

    bool copyScriptTexture(int index, ScriptRuntime::ScriptTextureSnapshot& snapshot) const {
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

    bool copyTextTexture(int index, ScriptRuntime::ScriptTextureSnapshot& snapshot) const {
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
};

ScriptRuntime::ScriptRuntime(
    const VehicleConfig& configuration, Variables& localState, SimulationState& sharedState,
    std::function<void(const std::string&, const std::string&, float)> soundTrigger)
    : impl_(std::make_unique<Impl>(configuration, localState, sharedState)) {
    impl_->onSoundTrigger = std::move(soundTrigger);
}

ScriptRuntime::~ScriptRuntime() = default;

void ScriptRuntime::configureScriptTextures(const std::vector<ModelScriptTexture>& definitions) {
    if (!impl_) {
        return;
    }
    for (const ModelScriptTexture& definition : definitions) {
        if (!openbus::scripting::validScriptTextureIndex(definition.slot) ||
            !openbus::scripting::validScriptTextureSize(definition.width, definition.height)) {
            continue;
        }
        const std::size_t byteSize =
            openbus::scripting::scriptTextureByteSize(definition.width, definition.height);
        const auto existing = impl_->scriptTextures.find(definition.slot);
        const std::size_t oldBytes =
            existing == impl_->scriptTextures.end() ? 0U : existing->second.pixels.size();
        if (!impl_->canReplaceTextureBytes(oldBytes, byteSize)) {
            continue;
        }
        impl_->scriptTextureDimensions[definition.slot] = {definition.width, definition.height};
        impl_->scriptTextures[definition.slot] = impl_->configuredScriptTexture(definition.slot);
    }
}

void ScriptRuntime::configureTextTextures(const std::vector<ModelTextTexture>& definitions) {
    if (!impl_) {
        return;
    }
    for (const ModelTextTexture& definition : definitions) {
        if (!openbus::scripting::validScriptTextureIndex(definition.slot) ||
            definition.values.size() < 4) {
            continue;
        }
        ScriptRuntime::Impl::TextTextureDefinition configured;
        configured.variable = definition.values[0];
        configured.font = lower(definition.values[1]);
        try {
            configured.width = std::stoi(definition.values[2]);
            configured.height = std::stoi(definition.values[3]);
        } catch (const std::exception&) {
            continue;
        }
        if (configured.variable.empty() ||
            !openbus::scripting::validScriptTextureSize(configured.width, configured.height)) {
            continue;
        }
        const std::size_t pixelBytes =
            openbus::scripting::scriptTextureByteSize(configured.width, configured.height);
        const auto existing = impl_->textTextures.find(definition.slot);
        const std::size_t oldBytes =
            existing == impl_->textTextures.end() ? 0U : existing->second.pixels.size();
        if (!impl_->canReplaceTextureBytes(oldBytes, pixelBytes)) {
            continue;
        }
        if (definition.values.size() > 4) {
            try {
                configured.useFontColor = std::stoi(definition.values[4]) != 0;
            } catch (const std::exception&) {
                configured.useFontColor = false;
            }
        }
        // Enhanced alignment values are two sets of the same horizontal modes:
        // 0/3=center, 1/4=left, and 2/5=right. The following field is the
        // additional per-character grid spacing used by enhanced displays.
        if (definition.enhanced && definition.values.size() > 8) {
            try {
                const int alignment = std::stoi(definition.values[8]);
                switch (alignment) {
                case 1:
                case 4:
                    configured.alignment = ScriptRuntime::Impl::TextAlignment::Left;
                    break;
                case 2:
                case 5:
                    configured.alignment = ScriptRuntime::Impl::TextAlignment::Right;
                    break;
                case 0:
                case 3:
                default:
                    configured.alignment = ScriptRuntime::Impl::TextAlignment::Center;
                    break;
                }
            } catch (const std::exception&) {
                configured.alignment = ScriptRuntime::Impl::TextAlignment::Center;
            }
        }
        if (definition.enhanced && definition.values.size() > 9) {
            try {
                std::size_t consumed = 0;
                const int spacing = std::stoi(definition.values[9], &consumed);
                if (consumed == definition.values[9].size()) {
                    configured.gridSpacing = std::clamp(spacing, 0, 64);
                }
            } catch (const std::exception&) {
                configured.gridSpacing = 0;
            }
        }
        for (std::size_t channel = 0; channel < 3 && channel + 5 < definition.values.size();
             ++channel) {
            try {
                const float value = std::stof(definition.values[channel + 5]);
                configured.color[channel] = ScriptRuntime::Impl::colorChannel(value);
            } catch (const std::exception&) {
                continue;
            }
        }
        impl_->textTextureDefinitions[definition.slot] = configured;
        ScriptRuntime::Impl::ScriptTexture texture;
        texture.width = configured.width;
        texture.height = configured.height;
        texture.pixels.assign(pixelBytes, 0);
        texture.revision = ++impl_->revisionCounter;
        impl_->textTextures[definition.slot] = std::move(texture);
    }
}

bool ScriptRuntime::valid() const {
    return impl_ && impl_->errors.empty();
}

const std::vector<std::string>& ScriptRuntime::errors() const {
    return impl_->errors;
}

void ScriptRuntime::initialize() {
    if (impl_ && (impl_->nativeBackend || impl_->state)) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        if (impl_->nativeBackend) {
            impl_->executeNativeFunction("init");
        } else {
            impl_->invoke("init");
        }
        impl_->updateTextTextures(true);
    }
}

void ScriptRuntime::invokeEntryPoint(const std::string& functionName) {
    if (impl_ && (impl_->nativeBackend || impl_->state) && !functionName.empty()) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: entry point " + functionName + " not found");
        }
    }
}

void ScriptRuntime::invokeSystemTrigger(const std::string& triggerName) {
    if (impl_ && (impl_->nativeBackend || impl_->state) &&
        impl_->localState.supportsSystemTrigger(triggerName)) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const std::string functionName = "trigger_" + scriptName(triggerName);
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: system trigger " + triggerName + " -> " + functionName +
                       " not found");
        }
    }
}

void ScriptRuntime::invokeInputEvent(const std::string& keyName, bool pressed) {
    if (!impl_ || keyName.empty()) {
        return;
    }
    const std::string normalized = lower(keyName);
    impl_->localState.set("key_pressed", pressed ? 1.0 : 0.0);
    impl_->localState.setString("key_name", normalized);
    const std::string functionName =
        "key_" + scriptName(normalized) + (pressed ? "_pressed" : "_released");
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: key event " + normalized + (pressed ? " pressed" : " released") +
                       " -> " + functionName + " not found");
        }
    }
}

void ScriptRuntime::invokeKeyBinding(const std::string& bindingName, bool pressed) {
    if (!impl_ || bindingName.empty()) {
        return;
    }
    const std::string normalized = lower(bindingName);
    impl_->localState.set("key_pressed", pressed ? 1.0 : 0.0);
    impl_->localState.setString("key_name", normalized);
    bool invoked = false;
    const std::string functionName = "trigger_" + scriptName(normalized) + (pressed ? "" : "_off");
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                       : impl_->invoke(functionName.c_str());
        if (!invoked) {
            if (hasScriptEntryPoint(functionName)) {
                impl_->log("warning: key binding " + normalized +
                           (pressed ? " pressed" : " released") + " -> " + functionName +
                           " not found");
            }
        } else {
            impl_->log("trigger ran: " + functionName + " (key binding " + normalized +
                       (pressed ? " pressed)" : " released)"));
            if (pressed &&
                (normalized == "cp_batterietrennschalter_toggle" || normalized == "kw_s_plus" ||
                 normalized == "kw_wipermode_up" || normalized == "cp_wischer_intervall_toggle")) {
                impl_->log("wiper/power state after " + normalized +
                           ": mode=" + std::to_string(impl_->localState.get("wiper_mode")) +
                           " pos=" + std::to_string(impl_->localState.get("wiperpos")) +
                           " busbar=" + std::to_string(impl_->localState.get("elec_busbar_main")) +
                           " busbar_switch=" +
                           std::to_string(impl_->localState.get("elec_busbar_main_sw")));
            }
        }
    }
    // if (!pressed && normalized == "horn" && !invoked && impl_->onSoundTrigger) {
    //     impl_->onSoundTrigger("horn_off", {}, 0.0);
    // }
}

void ScriptRuntime::invokeMouseEvent(const std::string& eventName) {
    if (!impl_ || eventName.empty()) {
        return;
    }
    const std::string normalized = lower(eventName);
    impl_->localState.setString("mouse_event", normalized);
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const std::string functionName = "trigger_" + scriptName(normalized);
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: mouse event " + normalized + " -> " + functionName + " not found");
        } else {
            impl_->log("trigger ran: " + functionName + " (mouse event " + normalized + ")");
        }
    }
}

bool ScriptRuntime::hasScriptEntryPoint(const std::string& functionName) const {
    if (!impl_ || functionName.empty()) {
        return false;
    }
    if (impl_->nativeBackend) {
        for (const std::shared_ptr<const OscProgram>& program : impl_->nativePrograms) {
            if (program->functions.find(functionName) != program->functions.end()) {
                return true;
            }
        }
        return false;
    }
    if (!impl_->state) {
        return false;
    }
    lua_rawgeti(impl_->state, LUA_REGISTRYINDEX, impl_->scriptEnvironment.reference);
    lua_getfield(impl_->state, -1, functionName.c_str());
    const bool found = lua_isfunction(impl_->state, -1) != 0;
    lua_pop(impl_->state, 2);
    return found;
}

void ScriptRuntime::invokeMouseRelease(const std::string& eventName) {
    if (!impl_ || eventName.empty()) {
        return;
    }
    const std::string normalized = lower(eventName);
    impl_->localState.setString("mouse_event", normalized);
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const std::string functionName = "trigger_" + scriptName(normalized) + "_off";
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: mouse release " + normalized + " -> " + functionName +
                       " not found");
        } else {
            impl_->log("trigger ran: " + functionName + " (mouse release " + normalized + ")");
        }
    }
}

void ScriptRuntime::invokeMouseDrag(const std::string& eventName, float deltaX, float deltaY,
                                    float cursorX, float cursorY) {
    if (!impl_ || eventName.empty()) {
        return;
    }
    const std::string normalized = lower(eventName);
    impl_->localState.set("mouse_drag_x", deltaX);
    impl_->localState.set("mouse_drag_y", deltaY);
    impl_->localState.set("mouse_cursor_x", cursorX);
    impl_->localState.set("mouse_cursor_y", cursorY);
    // GLFW and the authored OMSI drag handlers use screen-space Y deltas: positive
    // means moving down. Keep that convention unchanged for every mouse event;
    // object scripts provide any control-specific scale or sign themselves.
    impl_->sharedState.sharedVariables().set("mouse_x", deltaX);
    impl_->sharedState.sharedVariables().set("mouse_y", deltaY);
    impl_->localState.setString("mouse_event", normalized);
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const std::string functionName = "trigger_" + scriptName(normalized) + "_drag";
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: mouse drag " + normalized + " -> " + functionName + " not found");
        }
    }
}

bool ScriptRuntime::copyScriptTexture(int index, ScriptTextureSnapshot& snapshot) const {
    return impl_ && impl_->copyScriptTexture(index, snapshot);
}

bool ScriptRuntime::copyTextTexture(int index, ScriptTextureSnapshot& snapshot) const {
    return impl_ && impl_->copyTextTexture(index, snapshot);
}

void ScriptRuntime::update(bool isAiVehicle) {
    if (impl_ && (impl_->nativeBackend || impl_->state)) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        if (impl_->nativeBackend) {
            if (isAiVehicle && !impl_->executeNativeFunction("frame_ai")) {
                impl_->executeNativeFunction("frame");
            } else if (!isAiVehicle) {
                impl_->executeNativeFunction("frame");
            }
        } else {
            impl_->invokeFrame(isAiVehicle);
        }
        impl_->updateTextTextures();
    }
}
