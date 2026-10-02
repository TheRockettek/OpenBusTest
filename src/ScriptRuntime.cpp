#include "ScriptRuntime.h"

#include "Logger.h"
#include "Variables.h"
#include "PerfTrace.h"
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
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
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
    std::function<void(const std::string&, const std::string&, double)> onSoundTrigger;
    ScriptEnvironment scriptEnvironment;
    std::vector<std::string> errors;
    std::vector<double> floatStack;
    std::vector<std::string> stringStack;
    std::unordered_map<int, ScriptTexture> scriptTextures;
    std::unordered_map<int, std::pair<int, int>> scriptTextureDimensions;
    struct TextTextureDefinition {
        std::string variable;
        int width = 0;
        int height = 0;
        std::array<std::uint8_t, 4> color = {255, 255, 255, 255};
        std::string lastValue;
        bool hasRenderedValue = false;
    };
    std::unordered_map<int, TextTextureDefinition> textTextureDefinitions;
    std::unordered_map<int, ScriptTexture> textTextures;
    std::uint64_t revisionCounter = 0;
    std::vector<std::unique_ptr<LuaFunctionBinding>> luaFunctionBindings;
    std::vector<OscProgram> nativePrograms;
    std::array<double, 8> nativeRegisters = {};
    bool nativeBackend = false;

    ScriptTexture configuredScriptTexture(int index) {
        ScriptTexture texture;
        texture.revision = ++revisionCounter;
        const auto found = scriptTextureDimensions.find(index);
        if (found == scriptTextureDimensions.end()) {
            return texture;
        }
        texture.width = found->second.first;
        texture.height = found->second.second;
        texture.pixels.assign(static_cast<std::size_t>(texture.width) *
                                  static_cast<std::size_t>(texture.height) * 4,
                              0);
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

    double nativePopFloat() {
        if (floatStack.empty()) {
            return 0.0;
        }
        const double value = floatStack.back();
        floatStack.pop_back();
        return value;
    }

    double nativePeekFloat() const {
        return floatStack.empty() ? 0.0 : floatStack.back();
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
        const auto pushNumber = [this](double value) { floatStack.push_back(value); };
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
            pushNumber(static_cast<double>(random()) /
                       static_cast<double>(std::minstd_rand::max()));
        } else if (name == "getterminuscode" || name == "getrouteterminusindex" ||
                   name == "getbusstopcount" || name == "getttlinestring" ||
                   name == "getttterminusindex" || name == "getttbusstopcount" ||
                   name == "getttbusstopindex" || name == "getttdelay" ||
                   name == "getticketvalue" || name == "gethumancountonpathlink" ||
                   name == "gethumancountonseat" || name == "getarrbustimediff") {
            popFloats(1);
            pushNumber(0.0);
        } else if (name == "stnewtex") {
            const int index = static_cast<int>(nativePopFloat());
            scriptTextures[index] = configuredScriptTexture(index);
        } else if (name == "stlock" || name == "stunlock" || name == "stfilter") {
            const int index = static_cast<int>(nativePopFloat());
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
            const auto channel = [this] {
                return static_cast<std::uint8_t>(std::clamp(nativePopFloat(), 0.0, 255.0));
            };
            const std::uint8_t blue = channel();
            const std::uint8_t green = channel();
            const std::uint8_t red = channel();
            const std::uint8_t alpha = channel();
            const int index = static_cast<int>(nativePopFloat());
            scriptTextures[index].color = {alpha, red, green, blue};
        } else if (name == "stdrawpixel") {
            const int y = static_cast<int>(nativePopFloat());
            const int x = static_cast<int>(nativePopFloat());
            const int index = static_cast<int>(nativePopFloat());
            setPixel(scriptTextures[index], x, y);
        } else if (name == "stdrawrect") {
            const int y2 = static_cast<int>(nativePopFloat());
            const int x2 = static_cast<int>(nativePopFloat());
            const int y1 = static_cast<int>(nativePopFloat());
            const int x1 = static_cast<int>(nativePopFloat());
            const int index = static_cast<int>(nativePopFloat());
            ScriptTexture& texture = scriptTextures[index];
            for (int y = std::min(y1, y2); y <= std::max(y1, y2); ++y) {
                for (int x = std::min(x1, x2); x <= std::max(x1, x2); ++x) {
                    setPixel(texture, x, y);
                }
            }
        } else if (name == "sttextout") {
            const int letterSpacing = static_cast<int>(nativePopFloat());
            const std::uint32_t packedColor =
                static_cast<std::uint32_t>(std::max(0.0, nativePopFloat()));
            nativePopFloat();
            const int y = static_cast<int>(nativePopFloat());
            const int x = static_cast<int>(nativePopFloat());
            const int index = static_cast<int>(nativePopFloat());
            const std::string value = nativePopString();
            const std::array<std::uint8_t, 4> color = {
                static_cast<std::uint8_t>((packedColor >> 16) & 0xffU),
                static_cast<std::uint8_t>((packedColor >> 8) & 0xffU),
                static_cast<std::uint8_t>(packedColor & 0xffU),
                static_cast<std::uint8_t>((packedColor >> 24) & 0xffU)};
            drawText(scriptTextures[index], value, x, y, color, letterSpacing);
        } else if (name == "streadpixel") {
            popFloats(3);
        } else if (name == "stcopycolor") {
            const int destination = static_cast<int>(nativePopFloat());
            const int origin = static_cast<int>(nativePopFloat());
            scriptTextures[destination].color = scriptTextures[origin].color;
        } else if (name == "stloadtex") {
            const int index = static_cast<int>(nativePopFloat());
            loadScriptTexture(index, nativePopString());
        } else if (name == "stgetr" || name == "stgetg" || name == "stgetb" || name == "stgeta") {
            const int index = static_cast<int>(nativePopFloat());
            const std::size_t channel = name == "stgeta"   ? 0
                                        : name == "stgetr" ? 1
                                        : name == "stgetg" ? 2
                                                           : 3;
            pushNumber(scriptTextures[index].color[channel]);
        } else if (name == "getfontindex") {
            popStrings(1);
            pushNumber(0.0);
        } else if (name == "textlength") {
            nativePopFloat();
            const std::string value = nativePopString();
            pushNumber(static_cast<double>(value.size() * 6));
        }
    }

    const OscInstruction* nativeFindInstruction(const std::vector<OscInstruction>& code,
                                                std::size_t index) const {
        return index < code.size() ? &code[index] : nullptr;
    }

    bool executeNativeFunction(const std::string& functionName, int callDepth = 0) {
        openbus::rendering::TraceScope trace("osc_native", functionName.c_str());
        if (callDepth > 64) {
            errors.push_back("native OSC call depth exceeded in " + functionName);
            return false;
        }
        for (const OscProgram& program : nativePrograms) {
            const auto found = program.functions.find(functionName);
            if (found == program.functions.end()) {
                continue;
            }

            const std::vector<OscInstruction>& code = found->second;
            std::size_t instructionPointer = 0;
            while (instructionPointer < code.size()) {
                const OscInstruction& instruction = code[instructionPointer++];
                const auto binary = [this](auto operation) {
                    const double right = nativePopFloat();
                    const double left = nativePopFloat();
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
                    floatStack.push_back(
                        constant == configuration.constants.end() ? 0.0 : constant->second);
                    break;
                }
                case OscOpcode::CallCurve: {
                    const double value = nativePopFloat();
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
                        if (value <= points[index].x) {
                            const auto& left = points[index - 1];
                            const auto& right = points[index];
                            const double fraction = (value - left.x) / (right.x - left.x);
                            floatStack.push_back(left.y + fraction * (right.y - left.y));
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
                    binary([](double left, double right) { return left + right; });
                    break;
                case OscOpcode::Subtract:
                    binary([](double left, double right) { return left - right; });
                    break;
                case OscOpcode::Multiply:
                    binary([](double left, double right) { return left * right; });
                    break;
                case OscOpcode::Divide:
                    binary([](double left, double right) {
                        return right == 0.0 ? 0.0 : left / right;
                    });
                    break;
                case OscOpcode::Modulo:
                    binary([](double left, double right) {
                        return right == 0.0 ? 0.0 : left - std::floor(left / right) * right;
                    });
                    break;
                case OscOpcode::Negate:
                    floatStack.push_back(-nativePopFloat());
                    break;
                case OscOpcode::LogicalNot:
                    floatStack.push_back(nativePopFloat() == 0.0 ? 1.0 : 0.0);
                    break;
                case OscOpcode::Equal:
                    binary([](double left, double right) { return left == right ? 1.0 : 0.0; });
                    break;
                case OscOpcode::NotEqual:
                    binary([](double left, double right) { return left != right ? 1.0 : 0.0; });
                    break;
                case OscOpcode::Less:
                    binary([](double left, double right) { return left < right ? 1.0 : 0.0; });
                    break;
                case OscOpcode::LessEqual:
                    binary([](double left, double right) { return left <= right ? 1.0 : 0.0; });
                    break;
                case OscOpcode::Greater:
                    binary([](double left, double right) { return left > right ? 1.0 : 0.0; });
                    break;
                case OscOpcode::GreaterEqual:
                    binary([](double left, double right) { return left >= right ? 1.0 : 0.0; });
                    break;
                case OscOpcode::LogicalAnd:
                    binary([](double left, double right) {
                        return left != 0.0 && right != 0.0 ? 1.0 : 0.0;
                    });
                    break;
                case OscOpcode::LogicalOr:
                    binary([](double left, double right) {
                        return left != 0.0 || right != 0.0 ? 1.0 : 0.0;
                    });
                    break;
                case OscOpcode::Absolute:
                    floatStack.push_back(std::abs(nativePopFloat()));
                    break;
                case OscOpcode::Minimum:
                    binary([](double left, double right) { return std::min(left, right); });
                    break;
                case OscOpcode::Maximum:
                    binary([](double left, double right) { return std::max(left, right); });
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
                    floatStack.push_back(std::sqrt(std::max(0.0, nativePopFloat())));
                    break;
                case OscOpcode::ArcSine:
                    floatStack.push_back(std::asin(std::clamp(nativePopFloat(), -1.0, 1.0)));
                    break;
                case OscOpcode::Exponential:
                    floatStack.push_back(std::exp(nativePopFloat()));
                    break;
                case OscOpcode::Square: {
                    const double value = nativePopFloat();
                    floatStack.push_back(value * value);
                    break;
                }
                case OscOpcode::Sign: {
                    const double value = nativePopFloat();
                    floatStack.push_back(value > 0.0 ? 1.0 : value < 0.0 ? -1.0 : 0.0);
                    break;
                }
                case OscOpcode::Random: {
                    const int limit =
                        std::max(0, static_cast<int>(std::floor(nativePopFloat())) - 1);
                    floatStack.push_back(
                        limit == 0 ? 0.0 : static_cast<double>(std::rand() % (limit + 1)));
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
                    floatStack.push_back(static_cast<double>(nativePeekString().size()));
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
                        floatStack.push_back(std::stod(value));
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
                        onSoundTrigger(instruction.name, {}, nativePeekFloat());
                    }
                    break;
                case OscOpcode::SoundTriggerFile: {
                    const std::string file = nativePopString();
                    if (onSoundTrigger) {
                        onSoundTrigger(instruction.name, file, nativePeekFloat());
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
        const char* backend = std::getenv("OPENBUS_SCRIPT_BACKEND");
        if (!backend || std::string(backend) != "native") {
            return false;
        }

        for (const std::string& referencedPath : configuration.scripts) {
            std::string normalized = referencedPath;
            std::replace(normalized.begin(), normalized.end(), '\\', '/');
            const std::filesystem::path sourcePath =
                configuration.sourcePath.parent_path() / normalized;
            OscProgram program;
            std::string error;
            if (!compileOscToBytecode(sourcePath, program, error)) {
                scriptRuntimeLogger.Log("Native OSC backend falling back to Lua for " +
                                        sourcePath.string() + ": " + error);
                nativePrograms.clear();
                return false;
            }
            nativePrograms.push_back(std::move(program));
        }
        nativeBackend = !nativePrograms.empty();
        return nativeBackend;
    }

    static bool compileOnlyRequested() {
        const char* value = std::getenv("OPENBUS_SCRIPT_COMPILE_ONLY");
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
        for (const char* variable : {"ident", "number", "act_route", "act_busstop", "setlineto",
                                     "yard", "file_schedule"}) {
            localState.declareString(variable);
        }
        if (!configuration.selectedRegistration.empty()) {
            localState.setString("number", configuration.selectedRegistration);
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

    static double numericArgument(lua_State* lua, int index) {
        if (lua_isboolean(lua, index)) {
            return lua_toboolean(lua, index) != 0 ? 1.0 : 0.0;
        }
        return luaL_checknumber(lua, index);
    }

    static int getLocal(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const char* name = luaL_checkstring(lua, 1);
        const double value = runtime->localState.get(name);
        // runtime->log(lua, "get_local_var(" + std::string(name) + "): " + std::to_string(value));
        lua_pushnumber(lua, value);
        return 1;
    }

    static int setLocal(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const char* name = luaL_checkstring(lua, 1);
        const double value = numericArgument(lua, 2);
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
        lua_pushnumber(lua, found == runtime->configuration.constants.end() ? 0.0 : found->second);
        return 1;
    }

    static int callFunction(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        // runtime->log(lua, "call_func(" + std::string(luaL_checkstring(lua, 1)) + ", " +
        // std::to_string(numericArgument(lua, 2)) + ")");
        const std::string name = luaL_checkstring(lua, 1);
        const double value = numericArgument(lua, 2);
        const auto found = runtime->configuration.curves.find(name);
        if (found != runtime->configuration.curves.end() && !found->second.points.empty()) {
            const ConstantCurve& curve = found->second;
            if (value <= curve.points.front().x) {
                lua_pushnumber(lua, curve.points.front().y);
                return 1;
            }
            for (std::size_t index = 1; index < curve.points.size(); ++index) {
                if (value <= curve.points[index].x) {
                    const ConstantCurvePoint& left = curve.points[index - 1];
                    const ConstantCurvePoint& right = curve.points[index];
                    const double fraction = (value - left.x) / (right.x - left.x);
                    lua_pushnumber(lua, left.y + fraction * (right.y - left.y));
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
            runtime->onSoundTrigger(name, {},
                                    runtime->floatStack.empty() ? 0.0 : runtime->floatStack.back());
        }
        return 0;
    }

    static int soundTriggerFile(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const std::string name = luaL_checkstring(lua, 1);
        const std::string file = luaL_checkstring(lua, 2);
        if (runtime->onSoundTrigger) {
            runtime->onSoundTrigger(name, file,
                                    runtime->floatStack.empty() ? 0.0 : runtime->floatStack.back());
        }
        return 0;
    }

    static int debug(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        runtime->log("omsi_debug(" + std::string(luaL_checkstring(lua, 1)) + ")");
        return 0;
    }

    // TODO: Replace safe system-macro fallbacks with HOF, timetable, and vehicle data.
    static double popSystemFloat(Impl* runtime) {
        if (runtime->floatStack.empty()) {
            return 0.0;
        }
        const double value = runtime->floatStack.back();
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

    static int returnSystemFloat(lua_State* lua, double value) {
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
        double seed = popSystemFloat(runtimeFor(lua));
        std::minstd_rand rand(static_cast<unsigned int>(seed));
        return returnSystemFloat(lua, static_cast<double>(rand()) /
                                          static_cast<double>(std::minstd_rand::max()));
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

    static void resizeScriptTexture(ScriptTexture& texture, int requiredWidth, int requiredHeight) {
        if (requiredWidth <= texture.width && requiredHeight <= texture.height) {
            return;
        }
        const int width = std::max(texture.width, requiredWidth);
        const int height = std::max(texture.height, requiredHeight);
        std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height * 4, 0);
        for (int y = 0; y < texture.height; ++y) {
            const auto sourceOffset = static_cast<std::size_t>(y) * texture.width * 4;
            const auto targetOffset = static_cast<std::size_t>(y) * width * 4;
            std::copy_n(texture.pixels.begin() + sourceOffset, texture.width * 4,
                        pixels.begin() + targetOffset);
        }
        texture.width = width;
        texture.height = height;
        texture.pixels = std::move(pixels);
    }

    static std::size_t pixelOffset(const ScriptTexture& texture, int x, int y) {
        return (static_cast<std::size_t>(y) * texture.width + x) * 4;
    }

    static void setPixel(ScriptTexture& texture, int x, int y) {
        if (texture.locked || x < 0 || y < 0) {
            return;
        }
        resizeScriptTexture(texture, x + 1, y + 1);
        const std::size_t offset = pixelOffset(texture, x, y);
        texture.pixels[offset] = texture.color[1];
        texture.pixels[offset + 1] = texture.color[2];
        texture.pixels[offset + 2] = texture.color[3];
        texture.pixels[offset + 3] = texture.color[0];
        ++texture.revision;
    }

    static void clearTexture(ScriptTexture& texture) {
        texture.pixels.assign(static_cast<std::size_t>(texture.width) *
                                  static_cast<std::size_t>(texture.height) * 4,
                              0);
    }

    static void drawText(ScriptTexture& texture, const std::string& value, int x, int y,
                         std::array<std::uint8_t, 4> color, int letterSpacing = 0) {
        if (texture.locked || value.empty() || texture.width <= 0 || texture.height <= 0) {
            return;
        }
        const int heightScale = std::max(1, (texture.height - 2) / 7);
        const int availableWidth = std::max(
            1, texture.width - 2 - std::max(0, letterSpacing) * static_cast<int>(value.size()));
        const int widthScale =
            std::max(1, availableWidth / std::max(1, 6 * static_cast<int>(value.size())));
        const int scale = std::max(1, std::min({8, heightScale, widthScale}));
        const int advance = 6 * scale + letterSpacing;
        for (std::size_t characterIndex = 0; characterIndex < value.size(); ++characterIndex) {
            const auto rows = glyph(value[characterIndex]);
            const int originX = x + static_cast<int>(characterIndex) * advance;
            for (int row = 0; row < 7; ++row) {
                for (int column = 0; column < 5; ++column) {
                    if ((rows[static_cast<std::size_t>(row)] & (1 << (4 - column))) == 0) {
                        continue;
                    }
                    for (int offsetY = 0; offsetY < scale; ++offsetY) {
                        for (int offsetX = 0; offsetX < scale; ++offsetX) {
                            const int pixelX = originX + column * scale + offsetX;
                            const int pixelY = y + row * scale + offsetY;
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
        }
        ++texture.revision;
    }

    void updateTextTextures() {
        for (auto& entry : textTextureDefinitions) {
            const int index = entry.first;
            TextTextureDefinition& definition = entry.second;
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
            drawText(texture, value, 0, 0, definition.color);
            if (value.empty()) {
                ++texture.revision;
            }
        }
    }

    bool loadScriptTexture(int index, const std::string& requestedPath) {
        std::string normalized = requestedPath;
        std::replace(normalized.begin(), normalized.end(), '\\', '/');
        const std::filesystem::path path = configuration.sourcePath.parent_path() / normalized;
        openbus::rendering::Image image;
        if (!openbus::rendering::TextureLoader::readImage(path, image) || image.width <= 0 ||
            image.height <= 0 || image.rgba.empty()) {
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
        const int index = static_cast<int>(popSystemFloat(runtime));
        ScriptTexture& texture = scriptTexture(runtime, index);
        texture.locked = true;
        texture.filtered = false;
        return 0;
    }

    static int safeScriptTextureNew(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const int index = static_cast<int>(popSystemFloat(runtime));
        runtime->scriptTextures[index] = runtime->configuredScriptTexture(index);
        return 0;
    }

    static int safeScriptTextureLock(lua_State* lua) {
        const int index = static_cast<int>(popSystemFloat(runtimeFor(lua)));
        scriptTexture(runtimeFor(lua), index).locked = true;
        return 0;
    }

    static int safeScriptTextureUnlock(lua_State* lua) {
        const int index = static_cast<int>(popSystemFloat(runtimeFor(lua)));
        scriptTexture(runtimeFor(lua), index).locked = false;
        return 0;
    }

    static int safeScriptTextureFilter(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const int index = static_cast<int>(popSystemFloat(runtime));
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
        const std::uint8_t blue =
            static_cast<std::uint8_t>(std::clamp(popSystemFloat(runtime), 0.0, 255.0));
        const std::uint8_t green =
            static_cast<std::uint8_t>(std::clamp(popSystemFloat(runtime), 0.0, 255.0));
        const std::uint8_t red =
            static_cast<std::uint8_t>(std::clamp(popSystemFloat(runtime), 0.0, 255.0));
        const std::uint8_t alpha =
            static_cast<std::uint8_t>(std::clamp(popSystemFloat(runtime), 0.0, 255.0));
        const int index = static_cast<int>(popSystemFloat(runtime));
        scriptTexture(runtime, index).color = {alpha, red, green, blue};
        return 0;
    }

    static int safeScriptTexturePixel(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const int y = static_cast<int>(popSystemFloat(runtime));
        const int x = static_cast<int>(popSystemFloat(runtime));
        const int index = static_cast<int>(popSystemFloat(runtime));
        setPixel(scriptTexture(runtime, index), x, y);
        return 0;
    }

    static int safeScriptTextureReadPixel(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const int y = static_cast<int>(popSystemFloat(runtime));
        const int x = static_cast<int>(popSystemFloat(runtime));
        const int index = static_cast<int>(popSystemFloat(runtime));
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
        const int y2 = static_cast<int>(popSystemFloat(runtime));
        const int x2 = static_cast<int>(popSystemFloat(runtime));
        const int y1 = static_cast<int>(popSystemFloat(runtime));
        const int x1 = static_cast<int>(popSystemFloat(runtime));
        const int index = static_cast<int>(popSystemFloat(runtime));
        ScriptTexture& texture = scriptTexture(runtime, index);
        for (int y = std::min(y1, y2); y <= std::max(y1, y2); ++y) {
            for (int x = std::min(x1, x2); x <= std::max(x1, x2); ++x) {
                setPixel(texture, x, y);
            }
        }
        return 0;
    }

    static int safeScriptTextureText(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const int letterSpacing = static_cast<int>(popSystemFloat(runtime));
        const std::uint32_t packedColor =
            static_cast<std::uint32_t>(std::max(0.0, popSystemFloat(runtime)));
        popSystemFloat(runtime);
        const int y = static_cast<int>(popSystemFloat(runtime));
        const int x = static_cast<int>(popSystemFloat(runtime));
        const int index = static_cast<int>(popSystemFloat(runtime));
        const std::string value = popSystemString(runtime);
        const std::array<std::uint8_t, 4> color = {
            static_cast<std::uint8_t>((packedColor >> 16) & 0xffU),
            static_cast<std::uint8_t>((packedColor >> 8) & 0xffU),
            static_cast<std::uint8_t>(packedColor & 0xffU),
            static_cast<std::uint8_t>((packedColor >> 24) & 0xffU)};
        drawText(scriptTexture(runtime, index), value, x, y, color, letterSpacing);
        return 0;
    }

    static int safeScriptTextureCopyColor(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const int destination = static_cast<int>(popSystemFloat(runtime));
        const int origin = static_cast<int>(popSystemFloat(runtime));
        scriptTexture(runtime, destination).color = scriptTexture(runtime, origin).color;
        return 0;
    }

    static int safeScriptTextureLoad(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const int index = static_cast<int>(popSystemFloat(runtime));
        runtime->loadScriptTexture(index, popSystemString(runtime));
        return 0;
    }

    static int safeScriptTextureChannel(lua_State* lua, std::size_t channel) {
        Impl* runtime = runtimeFor(lua);
        const int index = static_cast<int>(popSystemFloat(runtime));
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
        return returnSystemFloat(lua, static_cast<double>(value.size() * 6));
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
    std::function<void(const std::string&, const std::string&, double)> soundTrigger)
    : impl_(std::make_unique<Impl>(configuration, localState, sharedState)) {
    impl_->onSoundTrigger = std::move(soundTrigger);
}

ScriptRuntime::~ScriptRuntime() = default;

void ScriptRuntime::configureScriptTextures(const std::vector<ModelScriptTexture>& definitions) {
    if (!impl_) {
        return;
    }
    for (const ModelScriptTexture& definition : definitions) {
        if (definition.slot < 0 || definition.width <= 0 || definition.height <= 0) {
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
        if (definition.slot < 0 || definition.values.size() < 4) {
            continue;
        }
        ScriptRuntime::Impl::TextTextureDefinition configured;
        configured.variable = definition.values[0];
        try {
            configured.width = std::stoi(definition.values[2]);
            configured.height = std::stoi(definition.values[3]);
        } catch (const std::exception&) {
            continue;
        }
        if (configured.variable.empty() || configured.width <= 0 || configured.height <= 0) {
            continue;
        }
        for (std::size_t channel = 0; channel < 3 && channel + 5 < definition.values.size();
             ++channel) {
            try {
                double value = std::stod(definition.values[channel + 5]);
                if (value >= 0.0 && value <= 1.0) {
                    value *= 255.0;
                }
                configured.color[channel] =
                    static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
            } catch (const std::exception&) {
                continue;
            }
        }
        impl_->textTextureDefinitions[definition.slot] = configured;
        ScriptRuntime::Impl::ScriptTexture texture;
        texture.width = configured.width;
        texture.height = configured.height;
        impl_->textTextures[definition.slot] = std::move(texture);
    }
    impl_->updateTextTextures();
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
        impl_->updateTextTextures();
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
    const std::string functionName = "trigger_" + scriptName(normalized) + (pressed ? "" : "_off");
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: key binding " + normalized + (pressed ? " pressed" : " released") +
                       " -> " + functionName + " not found");
        }
    }
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
        }
        if (normalized == "alcolockelec" || normalized == "alcolockcheck") {
            if (invoked) {
                impl_->log(
                    "alcolock state elec_busbar_main=" +
                    std::to_string(impl_->localState.get("elec_busbar_main")) +
                    " alcolock_elec=" + std::to_string(impl_->localState.get("alcolock_elec")) +
                    " alcolock_ready=" + std::to_string(impl_->localState.get("alcolock_ready")) +
                    " alcolock_pass=" + std::to_string(impl_->localState.get("alcolock_pass")) +
                    " alcolock_blow=" + std::to_string(impl_->localState.get("alcolock_blow")) +
                    " alcodisp=" + impl_->localState.getString("alcodisp"));
            }
        }
    }
}

bool ScriptRuntime::hasScriptEntryPoint(const std::string& functionName) const {
    if (!impl_ || functionName.empty()) {
        return false;
    }
    if (impl_->nativeBackend) {
        for (const OscProgram& program : impl_->nativePrograms) {
            if (program.functions.find(functionName) != program.functions.end()) {
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
        }
    }
}

void ScriptRuntime::invokeMouseDrag(const std::string& eventName, double deltaX, double deltaY,
                                    double cursorX, double cursorY) {
    if (!impl_ || eventName.empty()) {
        return;
    }
    const std::string normalized = lower(eventName);
    impl_->localState.set("mouse_drag_x", deltaX);
    impl_->localState.set("mouse_drag_y", deltaY);
    impl_->localState.set("mouse_cursor_x", cursorX);
    impl_->localState.set("mouse_cursor_y", cursorY);
    // E400/OMSI drag handlers apply their own axis sign (for example, the two
    // paired cab windows use mouse_x / 500 and mouse_x / -500).
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
