// Native OSC bytecode backend: stack helpers, system macros and functions.

#include "scripting/ScriptRuntimeImpl.h"

float ScriptRuntime::Impl::nativePopFloat() {
    if (floatStack.empty()) {
        return 0.0F;
    }
    const float value = floatStack.back();
    floatStack.pop_back();
    return value;
}

float ScriptRuntime::Impl::nativePeekFloat() const {
    return floatStack.empty() ? 0.0F : floatStack.back();
}

float ScriptRuntime::Impl::soundControlValue() const {
    float value = nativePeekFloat();
    std::string positionVariable;
    std::string lastPositionVariable;
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

std::string ScriptRuntime::Impl::nativePopString() {
    if (stringStack.empty()) {
        return {};
    }
    std::string value = std::move(stringStack.back());
    stringStack.pop_back();
    return value;
}

const std::string& ScriptRuntime::Impl::nativePeekString() const {
    static const std::string empty;
    return stringStack.empty() ? empty : stringStack.back();
}

void ScriptRuntime::Impl::executeSystemMacro(const std::string& name) {
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
               name == "getttbusstopindex" || name == "getttdelay" || name == "getticketvalue" ||
               name == "gethumancountonpathlink" || name == "gethumancountonseat" ||
               name == "getarrbustimediff") {
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
        if (validY2 && validX2 && validY1 && validX1 && validIndex && validTextureIndex(index)) {
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

bool ScriptRuntime::Impl::executeNativeFunction(const std::string& functionName, int callDepth) {
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
                floatStack.push_back(nativeRegisters[static_cast<std::size_t>(instruction.index)]);
                break;
            case OscOpcode::StoreRegister:
                nativeRegisters[static_cast<std::size_t>(instruction.index)] = nativePeekFloat();
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
                binary([](float left, float right) { return right == 0.0F ? 0.0F : left / right; });
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
                const int limit = std::max(0, static_cast<int>(std::floor(nativePopFloat())) - 1);
                floatStack.push_back(limit == 0 ? 0.0F
                                                : static_cast<float>(std::rand() % (limit + 1)));
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
                stringStack.push_back(count < static_cast<int>(value.size()) ? value.substr(count)
                                                                             : "");
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
                    const std::string padding(static_cast<std::size_t>(length) - value.size(), ' ');
                    stringStack.push_back(instruction.opcode == OscOpcode::StringSetLengthRight
                                              ? padding + value
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
                if (nativePeekFloat() == 0.0F) {
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
                    onSoundTrigger(instruction.name, {}, soundControlValue());
                }
                break;
            case OscOpcode::SoundTriggerFile: {
                const std::string file = nativePopString();
                if (onSoundTrigger) {
                    onSoundTrigger(instruction.name, file, soundControlValue());
                }
                break;
            }
            }
        }
        return true;
    }
    return false;
}

bool ScriptRuntime::Impl::tryInitializeNativeBackend() {
    const char* backend = openbus::getEnvironment("OPENBUS_SCRIPT_BACKEND");
    if (backend && lower(backend) == "lua") {
        scriptRuntimeLogger.Log("Lua script backend explicitly selected");
        return false;
    }

    bool compiledAnyScript = false;
    for (const std::string& referencedPath : configuration.scripts) {
        const std::filesystem::path sourcePath =
            openbus::assets::resolveReferencedPath(configuration.sourcePath, referencedPath);
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

bool ScriptRuntime::Impl::compileOnlyRequested() {
    const char* value = openbus::getEnvironment("OPENBUS_SCRIPT_COMPILE_ONLY");
    return value && std::string(value) == "1";
}
