#include "RenderLoop.h"

#include "AssetRequestManager.h"
#include "BusConfiguration.h"
#include "BusConfigLoader.h"
#include "BusModelLoader.h"
#include "BusSimulation.h"
#include "CameraMath.h"
#include "CoreRenderer.h"
#include "Environment.h"
#include "InteriorLighting.h"
#include "Logger.h"
#include "MapRenderer.h"
#include "MouseControlMapping.h"
#include "ObjLoader.h"
#include "OpenGLFunctions.h"
#include "PerfTrace.h"
#include "RenderPrimitives.h"
#include "RendererReflection.h"
#include "RoadFeatures.h"
#include "ScriptRuntime.h"
#include "ScreenshotWriter.h"
#include "SoundEngine.h"
#include "VehicleSoundBank.h"
#include "TextureAssetLoader.h"
#include "TextureLoader.h"
#include "Viewpoint.h"
#include "Variables.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#include <GL/gl.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <gli/gli.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#define GL_STATIC_DRAW 0x88E4
#endif

#ifndef GL_COMPRESSED_RGBA_S3TC_DXT5_EXT
#define GL_COMPRESSED_RGBA_S3TC_DXT5_EXT 0x83F3
#endif

#ifndef GL_TEXTURE_2D_ARRAY
#define GL_TEXTURE_2D_ARRAY 0x8C1A
#endif

#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#define GL_COLOR_ATTACHMENT0 0x8CE0
#define GL_DEPTH_ATTACHMENT 0x8D00
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#define GL_DEPTH_COMPONENT24 0x81A6
#endif

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

#ifndef GL_CLAMP_TO_BORDER
#define GL_CLAMP_TO_BORDER 0x812D
#endif

#ifndef GL_MIRRORED_REPEAT
#define GL_MIRRORED_REPEAT 0x8370
#endif

#ifndef GL_MIRROR_CLAMP_TO_EDGE
#define GL_MIRROR_CLAMP_TO_EDGE 0x8743
#endif

#ifndef GL_TEXTURE0
#define GL_TEXTURE0 0x84C0
#define GL_TEXTURE1 0x84C1
#define GL_TEXTURE_ENV 0x2300
#define GL_TEXTURE_ENV_MODE 0x2200
#define GL_COMBINE 0x8570
#define GL_REPLACE 0x1E01
#define GL_MODULATE 0x2100
#define GL_COMBINE_RGB 0x8571
#define GL_COMBINE_ALPHA 0x8572
#define GL_SOURCE0_RGB 0x8580
#define GL_SOURCE0_ALPHA 0x8588
#define GL_SOURCE1_ALPHA 0x8589
#define GL_OPERAND0_RGB 0x8590
#define GL_OPERAND0_ALPHA 0x8598
#define GL_OPERAND1_ALPHA 0x8599
#define GL_PREVIOUS 0x8578
#define GL_SRC_ALPHA 0x0302
#define GL_TEXTURE 0x1702
#endif

Logger gameLog = Logger("Game");
Logger textureLog = Logger("Texture");

using Matrix4 = openbus::rendering::Matrix4;

namespace {

using RenderViewContext = openbus::rendering::ViewpointContext;

enum class VehicleRenderPass {
    All,
    Opaque,
    Transparent,
};

constexpr double ENVIRONMENT_MAP_OPACITY = 0.1;
constexpr int MAX_SCRIPT_CATCH_UP_TICKS = 8;
constexpr double DEFAULT_FIELD_OF_VIEW = 60.0;
constexpr double MIN_FIELD_OF_VIEW = 20.0;
constexpr double MAX_FIELD_OF_VIEW = 120.0;
constexpr double GROUND_SHADOW_OFFSET_Z = 0.015;
constexpr int COORDINATE_HUD_TEXTURE_WIDTH = 224;
constexpr int COORDINATE_HUD_TEXTURE_HEIGHT = 24;

std::array<std::uint8_t, 7> coordinateHudGlyph(char character) {
    switch (character) {
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
    default:
        return {};
    }
}

std::vector<std::uint8_t> makeCoordinateHudPixels(std::string_view text) {
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(COORDINATE_HUD_TEXTURE_WIDTH) *
                                     COORDINATE_HUD_TEXTURE_HEIGHT * 4);
    for (std::size_t index = 0; index < pixels.size(); index += 4) {
        pixels[index] = 10;
        pixels[index + 1] = 16;
        pixels[index + 2] = 24;
        pixels[index + 3] = 196;
    }
    constexpr int glyphAdvance = 6;
    constexpr int glyphTop = (COORDINATE_HUD_TEXTURE_HEIGHT - 7) / 2;
    constexpr int horizontalPadding = 6;
    for (std::size_t glyphIndex = 0; glyphIndex < text.size(); ++glyphIndex) {
        const int glyphLeft = horizontalPadding + static_cast<int>(glyphIndex) * glyphAdvance;
        if (glyphLeft + 5 > COORDINATE_HUD_TEXTURE_WIDTH) {
            break;
        }
        const auto glyph = coordinateHudGlyph(text[glyphIndex]);
        for (int row = 0; row < static_cast<int>(glyph.size()); ++row) {
            const int y = COORDINATE_HUD_TEXTURE_HEIGHT - 1 - (glyphTop + row);
            for (int column = 0; column < 5; ++column) {
                if ((glyph[static_cast<std::size_t>(row)] & (1U << (4 - column))) == 0) {
                    continue;
                }
                const int x = glyphLeft + column;
                const std::size_t pixelIndex =
                    (static_cast<std::size_t>(y) * COORDINATE_HUD_TEXTURE_WIDTH + x) * 4;
                pixels[pixelIndex] = 245;
                pixels[pixelIndex + 1] = 249;
                pixels[pixelIndex + 2] = 255;
                pixels[pixelIndex + 3] = 255;
            }
        }
    }
    return pixels;
}

struct VehicleKeyBinding {
    const char* action;
    int key;
    int flags;
};

constexpr int kBindingHeld = 1;
constexpr int kBindingShift = 2;
constexpr int kBindingControl = 4;
constexpr int kBindingAlt = 8;

void logVariableSet(const std::string& scope, const Variables& variables) {
    std::vector<std::string> numericNames;
    numericNames.reserve(variables.numericValues().size());
    for (const auto& [name, value] : variables.numericValues()) {
        (void)value;
        numericNames.push_back(name);
    }
    std::sort(numericNames.begin(), numericNames.end());
    gameLog.Log(scope + " numeric variables: " + std::to_string(numericNames.size()));
    for (const std::string& name : numericNames) {
        std::ostringstream line;
        line << scope << '.' << name << " = " << std::setprecision(12)
             << variables.numericValues().at(name);
        gameLog.Log(line.str());
    }

    std::vector<std::string> stringNames;
    stringNames.reserve(variables.stringValues().size());
    for (const auto& [name, value] : variables.stringValues()) {
        (void)value;
        stringNames.push_back(name);
    }
    std::sort(stringNames.begin(), stringNames.end());
    gameLog.Log(scope + " string variables: " + std::to_string(stringNames.size()));
    for (const std::string& name : stringNames) {
        gameLog.Log(scope + "." + name + " = " + variables.stringValues().at(name));
    }
}

// OpenBus vehicle defaults. Cashdesk, IBIS, and rollband actions are
// intentionally omitted until their dedicated input surfaces are wired.
const std::vector<VehicleKeyBinding>& defaultVehicleKeyBindings() {
    static const std::vector<VehicleKeyBinding> bindings = {
        {"throttle", GLFW_KEY_KP_8, kBindingHeld},
        {"brake", GLFW_KEY_KP_2, kBindingHeld},
        {"throttle_amplify", GLFW_KEY_KP_ADD, kBindingHeld},
        {"clutch", GLFW_KEY_TAB, kBindingHeld},
        {"steering_left", GLFW_KEY_KP_4, kBindingHeld},
        {"steering_neutral", GLFW_KEY_KP_5, kBindingHeld},
        {"steering_right", GLFW_KEY_KP_6, kBindingHeld},
        {"ticket_give", GLFW_KEY_T, 0},
        {"change_give", GLFW_KEY_T, kBindingControl},
        {"change_take", GLFW_KEY_T, kBindingShift},
        {"parking_brake_toggle", GLFW_KEY_PERIOD, 0},
        {"blinker_left_set", GLFW_KEY_KP_7, kBindingHeld},
        {"blinker_right_set", GLFW_KEY_KP_9, kBindingHeld},
        {"blinker_off", GLFW_KEY_KP_DECIMAL, 0},
        {"blinker_warn_toggle", GLFW_KEY_B, 0},
        {"kw_scheinwerfer_toggle", GLFW_KEY_L, 0},
        {"kw_standlicht_toggle", GLFW_KEY_L, kBindingShift},
        {"kw_fernlicht_toggle", GLFW_KEY_F, 0},
        {"kw_m_enginestart", GLFW_KEY_M, 0},
        {"kw_wipermode_up", GLFW_KEY_W, 0},
        {"horn", GLFW_KEY_H, kBindingHeld},
        {"cp_microphone", GLFW_KEY_Q, kBindingHeld},
        {"cp_fahrerlicht_toggle", GLFW_KEY_6, 0},
        {"cp_licht_untenrechts_toggle", GLFW_KEY_7, 0},
        {"cp_licht_oberdeck_toggle", GLFW_KEY_8, 0},
        {"cp_licht_unterdeck_toggle", GLFW_KEY_9, 0},
        {"kw_s_R", GLFW_KEY_R, 0},
        {"kw_s_N", GLFW_KEY_N, 0},
        {"kw_s_1", GLFW_KEY_1, 0},
        {"kw_s_2", GLFW_KEY_2, 0},
        {"kw_s_3", GLFW_KEY_3, 0},
        {"kw_s_4", GLFW_KEY_4, 0},
        {"kw_s_5", GLFW_KEY_5, 0},
        {"kw_s_6", GLFW_KEY_6, 0},
        {"automatic_1", GLFW_KEY_1, kBindingShift},
        {"automatic_2", GLFW_KEY_2, kBindingShift},
        {"automatic_D", GLFW_KEY_LEFT_SHIFT, kBindingShift},
        {"automatic_R", GLFW_KEY_LEFT_ALT, 0},
        {"automatic_N", GLFW_KEY_LEFT_CONTROL, kBindingControl},
        {"bus_doorfront0", GLFW_KEY_KP_DIVIDE, 0},
        {"bus_doorfront1", GLFW_KEY_KP_MULTIPLY, 0},
        {"bus_dooraft", GLFW_KEY_KP_SUBTRACT, 0},
        {"bus_20h-switch", GLFW_KEY_SCROLL_LOCK, 0},
        {"bus_doorfront5", GLFW_KEY_KP_ADD, 0},
        {"bus_linie_plus", GLFW_KEY_F8, 0},
        {"bus_linie_minus", GLFW_KEY_F5, 0},
        {"bus_ziel_plus", GLFW_KEY_F7, 0},
        {"bus_ziel_minus", GLFW_KEY_F6, 0},
        {"debug_dump_variables", GLFW_KEY_F9, 0},
        {"cp_wischer_intervall_toggle", GLFW_KEY_W, kBindingShift},
        {"cp_wischer_wascher_button", GLFW_KEY_W, kBindingControl},
        {"cp_batterietrennschalter_toggle", GLFW_KEY_E, 0},
        {"taster_nebelschluss", GLFW_KEY_F, kBindingControl},
        {"cp_schalter_kinderwagen", GLFW_KEY_F12, 0},
        {"kw_s_plus", GLFW_KEY_LEFT_BRACKET, 0},
        {"kw_s_minus", GLFW_KEY_SLASH, 0},
        {"mouse_control_toggle", GLFW_KEY_O, 0},
    };
    return bindings;
}

int glfwKeyFromKeyboardConfigCode(int code) {
    if (code >= 2 && code <= 11) {
        return GLFW_KEY_1 + (code - 2);
    }
    switch (code) {
    case 15:
        return GLFW_KEY_TAB;
    case 16:
        return GLFW_KEY_Q;
    case 17:
        return GLFW_KEY_W;
    case 18:
        return GLFW_KEY_E;
    case 19:
        return GLFW_KEY_R;
    case 20:
        return GLFW_KEY_T;
    case 24:
        return GLFW_KEY_O;
    case 26:
        return GLFW_KEY_LEFT_BRACKET;
    case 29:
        return GLFW_KEY_LEFT_CONTROL;
    case 32:
        return GLFW_KEY_D;
    case 33:
        return GLFW_KEY_F;
    case 35:
        return GLFW_KEY_H;
    case 38:
        return GLFW_KEY_L;
    case 42:
        return GLFW_KEY_LEFT_SHIFT;
    case 48:
        return GLFW_KEY_B;
    case 49:
        return GLFW_KEY_N;
    case 50:
        return GLFW_KEY_M;
    case 52:
        return GLFW_KEY_PERIOD;
    case 53:
        return GLFW_KEY_SLASH;
    case 55:
        return GLFW_KEY_KP_MULTIPLY;
    case 56:
        return GLFW_KEY_LEFT_ALT;
    case 63:
        return GLFW_KEY_F5;
    case 64:
        return GLFW_KEY_F6;
    case 65:
        return GLFW_KEY_F7;
    case 66:
        return GLFW_KEY_F8;
    case 70:
        return GLFW_KEY_SCROLL_LOCK;
    case 71:
        return GLFW_KEY_KP_7;
    case 72:
        return GLFW_KEY_KP_8;
    case 73:
        return GLFW_KEY_KP_9;
    case 74:
        return GLFW_KEY_KP_SUBTRACT;
    case 75:
        return GLFW_KEY_KP_4;
    case 76:
        return GLFW_KEY_KP_5;
    case 77:
        return GLFW_KEY_KP_6;
    case 78:
        return GLFW_KEY_KP_ADD;
    case 80:
        return GLFW_KEY_KP_2;
    case 83:
        return GLFW_KEY_KP_DECIMAL;
    case 88:
        return GLFW_KEY_F12;
    case 181:
        return GLFW_KEY_KP_DIVIDE;
    default:
        return GLFW_KEY_UNKNOWN;
    }
}

std::string trimKeyBindingText(std::string value) {
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

void inheritVehicleKeyBindingsFromOmsi(std::vector<VehicleKeyBinding>& bindings) {
    const std::filesystem::path configPath = omsiRootPath() / "Inputs" / "keyboard.cfg";
    std::ifstream input(configPath);
    if (!input) {
        gameLog.Log("OMSI keybindings not found, using OpenBus defaults: " +
                    configPath.generic_string());
        return;
    }

    bool inVehiclesSection = false;
    bool readingEntry = false;
    int valueIndex = 0;
    std::string action;
    int keyCode = 0;
    int flags = 0;
    std::string line;
    while (std::getline(input, line)) {
        line = trimKeyBindingText(line);
        if (line.empty()) {
            continue;
        }
        if (line.front() == '[' && line.back() == ']') {
            const std::string section = line.substr(1, line.size() - 2);
            if (section == "vehicles") {
                inVehiclesSection = true;
                readingEntry = false;
            } else if (section == "entry") {
                readingEntry = inVehiclesSection;
            } else {
                inVehiclesSection = false;
                readingEntry = false;
            }
            valueIndex = 0;
            continue;
        }
        if (!inVehiclesSection || !readingEntry) {
            continue;
        }
        if (valueIndex == 0) {
            action = line;
        } else if (valueIndex == 1) {
            try {
                keyCode = std::stoi(line);
            } catch (const std::exception&) {
                readingEntry = false;
                continue;
            }
        } else if (valueIndex == 2) {
            try {
                flags = std::stoi(line);
            } catch (const std::exception&) {
                readingEntry = false;
                continue;
            }
            const auto found = std::find_if(
                bindings.begin(), bindings.end(),
                [&action](const VehicleKeyBinding& binding) { return action == binding.action; });
            if (found != bindings.end()) {
                found->key = glfwKeyFromKeyboardConfigCode(keyCode);
                found->flags = flags;
            }
            readingEntry = false;
            continue;
        }
        ++valueIndex;
    }
    gameLog.Log("Inherited OMSI vehicle keybindings from " + configPath.generic_string());
}

const std::vector<VehicleKeyBinding>& vehicleKeyBindings() {
    static const std::vector<VehicleKeyBinding> bindings = [] {
        std::vector<VehicleKeyBinding> result = defaultVehicleKeyBindings();
        inheritVehicleKeyBindingsFromOmsi(result);
        return result;
    }();
    return bindings;
}

const VehicleKeyBinding* findVehicleKeyBinding(const char* action) {
    const auto& bindings = vehicleKeyBindings();
    const auto found =
        std::find_if(bindings.begin(), bindings.end(), [action](const VehicleKeyBinding& binding) {
            return std::string_view(binding.action) == action;
        });
    return found == bindings.end() ? nullptr : &*found;
}

bool vehicleBindingPressed(GLFWwindow* window, const VehicleKeyBinding& binding) {
    if (glfwGetKey(window, binding.key) != GLFW_PRESS) {
        return false;
    }
    const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    const bool control = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                         glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
    const bool alt = glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
    const bool primaryShift =
        binding.key == GLFW_KEY_LEFT_SHIFT || binding.key == GLFW_KEY_RIGHT_SHIFT;
    const bool primaryControl =
        binding.key == GLFW_KEY_LEFT_CONTROL || binding.key == GLFW_KEY_RIGHT_CONTROL;
    const bool primaryAlt = binding.key == GLFW_KEY_LEFT_ALT || binding.key == GLFW_KEY_RIGHT_ALT;
    return shift == (((binding.flags & kBindingShift) != 0) || primaryShift) &&
           control == (((binding.flags & kBindingControl) != 0) || primaryControl) &&
           alt == (((binding.flags & kBindingAlt) != 0) || primaryAlt);
}

GLenum textureAddressModeToGl(TextureAddressMode mode) {
    switch (mode) {
    case TextureAddressMode::Clamp:
        return GL_CLAMP_TO_EDGE;
    case TextureAddressMode::Border:
        return GL_CLAMP_TO_BORDER;
    case TextureAddressMode::Mirror:
        return GL_MIRRORED_REPEAT;
    case TextureAddressMode::MirrorOnce:
        return GL_MIRROR_CLAMP_TO_EDGE;
    case TextureAddressMode::Repeat:
    default:
        return GL_REPEAT;
    }
}

Matrix4 identityMatrix() {
    return {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

Matrix4 multiplyMatrix4(const Matrix4& left, const Matrix4& right) {
    Matrix4 result = {};
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            for (int inner = 0; inner < 4; ++inner) {
                result[row + column * 4] += left[row + inner * 4] * right[inner + column * 4];
            }
        }
    }
    return result;
}

std::array<double, 4> transformPoint(const Matrix4& matrix, const std::array<double, 4>& point) {
    std::array<double, 4> result = {};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            result[row] += matrix[row + column * 4] * point[column];
        }
    }
    return result;
}

using FrustumPlanes = std::array<std::array<double, 4>, 6>;
using FrustumPlaneLengths = std::array<double, 6>;

struct ViewFrustum {
    FrustumPlanes planes = {};
    FrustumPlaneLengths planeLengths = {};
};

ViewFrustum buildViewFrustum(const Matrix4& projection) {
    const std::array<std::array<double, 4>, 6> planeSigns = {{{{1.0, 0.0, 0.0, 1.0}},
                                                              {{-1.0, 0.0, 0.0, 1.0}},
                                                              {{0.0, 1.0, 0.0, 1.0}},
                                                              {{0.0, -1.0, 0.0, 1.0}},
                                                              {{0.0, 0.0, 1.0, 1.0}},
                                                              {{0.0, 0.0, -1.0, 1.0}}}};
    ViewFrustum frustum;
    for (std::size_t plane = 0; plane < planeSigns.size(); ++plane) {
        double lengthSquared = 0.0;
        for (int column = 0; column < 4; ++column) {
            double coefficient = 0.0;
            for (int row = 0; row < 4; ++row) {
                coefficient += planeSigns[plane][row] * projection[row + column * 4];
            }
            frustum.planes[plane][column] = coefficient;
            if (column < 3) {
                lengthSquared += coefficient * coefficient;
            }
        }
        frustum.planeLengths[plane] = std::sqrt(lengthSquared);
    }
    return frustum;
}

bool sphereOutsideFrustum(const ViewFrustum& frustum, const std::array<double, 4>& eye,
                          double radius) {
    for (std::size_t plane = 0; plane < frustum.planes.size(); ++plane) {
        const double planeDistance = frustum.planes[plane][0] * eye[0] +
                                     frustum.planes[plane][1] * eye[1] +
                                     frustum.planes[plane][2] * eye[2] + frustum.planes[plane][3];
        if (planeDistance < -radius * frustum.planeLengths[plane]) {
            return true;
        }
    }
    return false;
}

Matrix4 translationMatrix(const std::array<double, 3>& value) {
    Matrix4 result = identityMatrix();
    result[12] = value[0];
    result[13] = value[1];
    result[14] = value[2];
    return result;
}

Matrix4 poseMatrix(const BodyPose& pose) {
    return {pose.rotation[0], pose.rotation[3], pose.rotation[6], 0.0,
            pose.rotation[1], pose.rotation[4], pose.rotation[7], 0.0,
            pose.rotation[2], pose.rotation[5], pose.rotation[8], 0.0,
            pose.position[0], pose.position[1], pose.position[2], 1.0};
}

Matrix4 rotationMatrix(double angleDegrees, double x, double y, double z) {
    const double length = std::sqrt(x * x + y * y + z * z);
    if (length <= 1.0e-12) {
        return identityMatrix();
    }
    x /= length;
    y /= length;
    z /= length;
    const double angle = angleDegrees * 3.141592653589793 / 180.0;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double inverseCosine = 1.0 - cosine;
    return {cosine + x * x * inverseCosine,
            y * x * inverseCosine + z * sine,
            z * x * inverseCosine - y * sine,
            0.0,
            x * y * inverseCosine - z * sine,
            cosine + y * y * inverseCosine,
            z * y * inverseCosine + x * sine,
            0.0,
            x * z * inverseCosine + y * sine,
            y * z * inverseCosine - x * sine,
            cosine + z * z * inverseCosine,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0};
}

bool invertAffineMatrix(const Matrix4& matrix, Matrix4& inverse) {
    const double a = matrix[0];
    const double b = matrix[4];
    const double c = matrix[8];
    const double d = matrix[1];
    const double e = matrix[5];
    const double f = matrix[9];
    const double g = matrix[2];
    const double h = matrix[6];
    const double i = matrix[10];
    const double determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (std::abs(determinant) <= 1.0e-12) {
        inverse = identityMatrix();
        return false;
    }
    const double scale = 1.0 / determinant;
    inverse = identityMatrix();
    inverse[0] = (e * i - f * h) * scale;
    inverse[4] = (c * h - b * i) * scale;
    inverse[8] = (b * f - c * e) * scale;
    inverse[1] = (f * g - d * i) * scale;
    inverse[5] = (a * i - c * g) * scale;
    inverse[9] = (c * d - a * f) * scale;
    inverse[2] = (d * h - e * g) * scale;
    inverse[6] = (b * g - a * h) * scale;
    inverse[10] = (a * e - b * d) * scale;
    inverse[12] = -(inverse[0] * matrix[12] + inverse[4] * matrix[13] + inverse[8] * matrix[14]);
    inverse[13] = -(inverse[1] * matrix[12] + inverse[5] * matrix[13] + inverse[9] * matrix[14]);
    inverse[14] = -(inverse[2] * matrix[12] + inverse[6] * matrix[13] + inverse[10] * matrix[14]);
    return true;
}

std::array<double, 3> applyMeshAxis(const std::array<double, 3>& renderAxis,
                                    const std::array<double, 9>& meshRotation) {
    const std::array<double, 3> sourceAxis = {-renderAxis[1], renderAxis[0], renderAxis[2]};
    const std::array<double, 3> rotatedSourceAxis = {
        meshRotation[0] * sourceAxis[0] + meshRotation[3] * sourceAxis[1] +
            meshRotation[6] * sourceAxis[2],
        meshRotation[1] * sourceAxis[0] + meshRotation[4] * sourceAxis[1] +
            meshRotation[7] * sourceAxis[2],
        meshRotation[2] * sourceAxis[0] + meshRotation[5] * sourceAxis[1] +
            meshRotation[8] * sourceAxis[2]};
    return {rotatedSourceAxis[1], -rotatedSourceAxis[0], rotatedSourceAxis[2]};
}

Matrix4 makeMeshTransform(const std::array<double, 9>& meshRotation,
                          const std::array<double, 3>& origin) {
    Matrix4 result = identityMatrix();
    const std::array<std::array<double, 3>, 3> axes = {std::array<double, 3>{1.0, 0.0, 0.0},
                                                       std::array<double, 3>{0.0, 1.0, 0.0},
                                                       std::array<double, 3>{0.0, 0.0, 1.0}};
    for (int column = 0; column < 3; ++column) {
        const std::array<double, 3> transformed = applyMeshAxis(axes[column], meshRotation);
        result[column * 4] = transformed[0];
        result[column * 4 + 1] = transformed[1];
        result[column * 4 + 2] = transformed[2];
    }
    result[12] = origin[0];
    result[13] = origin[1];
    result[14] = origin[2];
    return result;
}

struct SharedVehicleDefinition {
    std::string cacheKey;
    std::filesystem::path modelConfigPath;
    std::filesystem::path modelRoot;
    VehicleConfig vehicleConfiguration;
    openbus::rendering::BusModelLoadResult modelConfiguration;
    std::vector<std::string> modelNumericVariables;
    std::vector<std::string> modelStringVariables;
};

std::string normalizedConfigPath(const std::filesystem::path& path) {
    std::string key = std::filesystem::absolute(path).lexically_normal().generic_string();
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return key;
}

std::filesystem::path modelRootFor(const std::filesystem::path& modelConfigPath) {
    const std::filesystem::path directory = modelConfigPath.parent_path();
    std::string directoryName = directory.filename().string();
    std::transform(
        directoryName.begin(), directoryName.end(), directoryName.begin(),
        [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return directoryName == "configuration files" ? directory.parent_path() : directory;
}

std::shared_ptr<const SharedVehicleDefinition>
sharedVehicleDefinition(const std::filesystem::path& busConfigPath,
                        const std::filesystem::path& modelConfigPath) {
    static std::mutex cacheMutex;
    static std::unordered_map<std::string, std::shared_ptr<const SharedVehicleDefinition>> cache;
    const std::string key =
        normalizedConfigPath(busConfigPath) + "|" + normalizedConfigPath(modelConfigPath);
    std::lock_guard<std::mutex> lock(cacheMutex);
    const auto existing = cache.find(key);
    if (existing != cache.end()) {
        return existing->second;
    }

    auto definition = std::make_shared<SharedVehicleDefinition>();
    definition->cacheKey = key;
    definition->modelConfigPath = modelConfigPath;
    definition->modelRoot = modelRootFor(modelConfigPath);
    gameLog.Log("Loading bus model with config: " + modelConfigPath.string() +
                " and model root: " + definition->modelRoot.string());
    definition->vehicleConfiguration = loadBusConfig(busConfigPath);

    openbus::scripting::Vehicle modelVariables;
    definition->modelConfiguration =
        openbus::rendering::loadBusModel(modelConfigPath, definition->modelRoot, modelVariables);
    for (const auto& [name, value] : modelVariables.numericValues()) {
        (void)value;
        definition->modelNumericVariables.push_back(name);
    }
    for (const auto& [name, value] : modelVariables.stringValues()) {
        (void)value;
        definition->modelStringVariables.push_back(name);
    }
    for (const ConfigurationDiagnostic& diagnostic :
         definition->modelConfiguration.diagnostics.entries) {
        const std::string severity =
            diagnostic.severity == ConfigurationDiagnostic::Severity::Error ? "error" : "warning";
        gameLog.Log("CFG " + severity + " line " + std::to_string(diagnostic.line) + " [" +
                    diagnostic.keyword + "]: " + diagnostic.message);
    }

    std::shared_ptr<const SharedVehicleDefinition> shared = std::move(definition);
    cache.emplace(key, shared);
    return shared;
}

} // namespace

using openbus::rendering::applyPose;
using AssetRequestManager = openbus::rendering::AssetRequestManager;
using openbus::rendering::drawBox;
using openbus::rendering::drawEnvironmentBatch;
using openbus::rendering::drawGround;
using openbus::rendering::drawMaterialBatch;
using openbus::rendering::drawModelBatch;
using openbus::rendering::lookAt;
using openbus::rendering::makeGroundShadowTransform;
using openbus::rendering::makeModelRootPlacement;
using openbus::rendering::modelViewMatrix;
using openbus::rendering::multiplyMatrix;
using openbus::rendering::parseEnabledFlag;
using openbus::rendering::popMatrix;
using openbus::rendering::pushMatrix;
using openbus::rendering::rotate;
using openbus::rendering::scale;
using openbus::rendering::setPerspective;
using openbus::rendering::TraceScope;
using openbus::rendering::transformLocalPoint;
using openbus::rendering::translate;
using ObjIndex = openbus::rendering::ObjIndex;
using ObjNormal = openbus::rendering::ObjNormal;
using ObjPosition = openbus::rendering::ObjPosition;
using ObjTexCoord = openbus::rendering::ObjTexCoord;
using ObjTriangle = openbus::rendering::ObjTriangle;
using ParsedObj = openbus::rendering::ParsedObj;
using Image = openbus::rendering::Image;
using CompressedDds = openbus::rendering::CompressedDds;

void applyVehiclePlacement(const VehiclePlacement& placement) {
    translate(placement.position[0], placement.position[1], placement.position[2]);
    rotate(placement.yawDegrees, 0.0, 0.0, 1.0);
}

void selectRuntimeRegistration(VehicleConfig& configuration, const VehiclePlacement& placement) {
    if (const char* overrideValue = std::getenv("OPENBUS_REGISTRATION");
        overrideValue != nullptr && *overrideValue != '\0') {
        configuration.selectedRegistration = overrideValue;
        return;
    }

    const std::size_t registrationCount = registrationOptionCount(configuration);
    if (configuration.registrationFree && registrationCount > 0) {
        std::size_t selectedIndex = 0;
        if (const char* indexValue = std::getenv("OPENBUS_REGISTRATION_INDEX");
            indexValue != nullptr && *indexValue != '\0') {
            try {
                selectedIndex = std::stoull(indexValue);
            } catch (const std::exception&) {
                selectedIndex = 0;
            }
        }
        selectRegistrationAtIndex(configuration, selectedIndex);
        return;
    }

    if (configuration.registrationAutomatic) {
        // Keep automatic numbers repeatable for a vehicle placement while avoiding
        // process-global random state. The explicit override remains available for
        // multiplayer/replay workflows that need a prescribed registration.
        std::string seed = configuration.sourcePath.generic_string();
        seed += ':' + std::to_string(placement.position[0]);
        seed += ':' + std::to_string(placement.position[1]);
        seed += ':' + std::to_string(placement.position[2]);
        seed += ':' + std::to_string(placement.yawDegrees);
        std::uint32_t hash = 2166136261U;
        for (const unsigned char character : seed) {
            hash ^= character;
            hash *= 16777619U;
        }
        std::ostringstream generated;
        generated << configuration.registrationPrefix << std::setfill('0') << std::setw(3)
                  << (hash % 1000U);
        configuration.selectedRegistration = generated.str();
    }
}

struct Vehicle {
    ModelLoadingPolicy loadingPolicy;
    using TextureRequest = AssetRequestManager::TextureRequest;
    using TextureCacheEntry = AssetRequestManager::TextureCacheEntry;

    struct ObjRequest {
        std::shared_future<std::shared_ptr<openbus::rendering::ParsedObj>> future;
    };

    using MaterialState = openbus::rendering::BusModelMaterialState;

    struct Part : openbus::rendering::BusModelPart {
        std::shared_ptr<ObjRequest> objRequest;
    };

    struct Vertex {
        float x, y, z, u, v, layer, nx, ny, nz;
    };

    struct AuxiliaryTexture {
        std::string name;
        GLuint texture = 0;
        bool loadAttempted = false;
        std::shared_ptr<TextureCacheEntry> cacheEntry;
    };

    struct MaterialTextureSource {
        GLuint texture = 0;
        bool textureArray = false;
        std::size_t textureArrayLayers = 1;
        std::filesystem::path texturePath;
        std::filesystem::path textureRoot;
        std::string textureName;
        bool textureLoadAttempted = false;
        bool textureLoadStarted = false;
        std::shared_ptr<TextureRequest> textureRequest;
        std::shared_ptr<TextureCacheEntry> textureCacheEntry;
    };

    struct InteriorLightController {
        bool isNumeric = false;
        double numericValue = 0.0;
    };

    struct Batch {
        struct SharedVertices {
            std::shared_ptr<std::vector<Vertex>> values = std::make_shared<std::vector<Vertex>>();

            SharedVertices& operator=(std::vector<Vertex>&& vertices) {
                values = std::make_shared<std::vector<Vertex>>(std::move(vertices));
                return *this;
            }
            std::size_t size() const {
                return values->size();
            }
            bool empty() const {
                return values->empty();
            }
            Vertex* data() {
                return values->data();
            }
            const Vertex* data() const {
                return values->data();
            }
            Vertex& operator[](std::size_t index) {
                return (*values)[index];
            }
            const Vertex& operator[](std::size_t index) const {
                return (*values)[index];
            }
            auto begin() {
                return values->begin();
            }
            auto end() {
                return values->end();
            }
            auto begin() const {
                return values->begin();
            }
            auto end() const {
                return values->end();
            }
            void reserve(std::size_t size) {
                values->reserve(size);
            }
            template <typename Iterator>
            auto insert(typename std::vector<Vertex>::iterator position, Iterator first,
                        Iterator last) {
                return values->insert(position, first, last);
            }
        };

        GLuint buffer = 0;
        bool bufferTracked = false;
        bool sharedGeometryBuffer = false;
        GLuint texture = 0;
        bool textureArray = false;
        std::size_t textureArrayLayers = 1;
        std::filesystem::path baseTexturePath;
        std::string baseTextureName;
        std::filesystem::path texturePath;
        std::filesystem::path textureRoot;
        std::string textureName;
        std::string environmentTextureName;
        GLuint environmentTexture = 0;
        double environmentStrength = 0.0;
        bool environmentLoadAttempted = false;
        std::shared_ptr<TextureCacheEntry> environmentTextureCacheEntry;
        std::array<double, 3> color = {0.65, 0.65, 0.65};
        bool textured = false;
        GLenum textureWrapS = GL_REPEAT;
        GLenum textureWrapT = GL_REPEAT;
        bool hasNormals = false;
        bool textureLoadAttempted = false;
        bool textureLoadStarted = false;
        std::shared_ptr<TextureRequest> textureRequest;
        std::shared_ptr<TextureCacheEntry> textureCacheEntry;
        AuxiliaryTexture lightmap;
        AuxiliaryTexture nightmap;
        AuxiliaryTexture transmap;
        AuxiliaryTexture bumpmap;
        double bumpmapStrength = 0.0;
        std::array<int, 4> interiorLightIndexes = {-1, -1, -1, -1};
        std::string lightmapStrengthVariable;
        std::string freeTextureName;
        std::string freeTextureVariable;
        AuxiliaryTexture freeTextureAsset;
        int scriptTextureIndex = -1;
        int textTextureIndex = -1;
        std::string texcoordTransXVariable;
        std::string texcoordTransYVariable;
        GLuint freeTexture = 0;
        GLuint dynamicTexture = 0;
        int freeTextureIndex = -1;
        int freeTextureWidth = 0;
        int freeTextureHeight = 0;
        std::uint64_t freeTextureRevision = 0;
        int alphaMode = 0;
        bool noZwrite = false;
        bool noZcheck = false;
        std::string alphaScaleVariable;
        int baseTextureLayer = 0;
        int textureLayer = 0;
        SharedVertices vertices;
        std::vector<std::array<float, 4>> materialColors;
        std::vector<MaterialTextureSource> materialTextures;
        std::vector<GLuint> materialTextureIds;
        std::vector<bool> materialTextureFlips;
        std::vector<MaterialState::TextureChange> textureChanges;
        MaterialState baseMaterial;
        bool selectedMaterialItem = false;
        std::uint64_t materialSelectionGeneration = 0;
        std::size_t vertexCount = 0;
    };

    struct DisplayPart {
        struct AnimationState {
            double currentAmount = 0.0;
            double targetAmount = 0.0;
            bool initialized = false;
        };

        // Geometry metadata is immutable after loading; visibility is still
        // evaluated from live variables during each draw.
        std::vector<Batch> batches;
        int viewpoint;
        int renderType = 2;
        bool isShadow = false;
        bool transparent;
        int lodIndex;
        std::string meshName;
        std::string visibleVariable;
        int visibleValue = 0;
        std::string meshIdentifier;
        std::string animationParent;
        std::string mouseEvent;
        int odeWheelIndex = -1;
        bool backFaceCulling = false;
        std::array<double, 3> center;
        std::array<double, 3> size;
        double radius;
        std::vector<ModelAnimation> animations;
        std::array<int, 4> interiorLightIndexes = {-1, -1, -1, -1};
        std::vector<AnimationState> animationStates;
        std::vector<int> reflectionTextureIndices;
        bool pickOccluder = false;
        mutable std::uint64_t animationCacheGeneration = 0;
        mutable Matrix4 cachedAnimationTransform = {};
        mutable std::uint64_t viewDepthCacheGeneration = 0;
        mutable double cachedViewDepth = 0.0;
    };

    struct TransparentBatch {
        Batch* batch;
        DisplayPart* part;
        double depth;
        int renderType;
    };

    std::vector<DisplayPart> displayLists;
    std::vector<DisplayPart*> opaquePartsScratch;
    std::vector<TransparentBatch> noDepthOpaqueBatchesScratch;
    std::vector<TransparentBatch> transparentBatchesScratch;
    std::vector<bool> variableVisibleParts;
    std::vector<std::size_t> visibleClickablePartIndices;
    std::vector<std::size_t> visibleOpaquePickPartIndices;
    mutable std::size_t hoveredClickablePartIndex = std::numeric_limits<std::size_t>::max();
    mutable std::size_t hoveredClickableBatchIndex = std::numeric_limits<std::size_t>::max();
    mutable std::size_t hoveredClickableTriangle = 0;
    mutable std::vector<openbus::rendering::PrimitiveVertex> clickableDebugBoxVertices;
    mutable std::vector<openbus::rendering::PrimitiveVertex> clickableDebugWireVertices;
    mutable std::vector<openbus::rendering::PrimitiveVertex> clickableDebugTriangleVertices;
    std::unordered_set<int> visibleReflectionTextureIndices;
    std::unordered_map<int, int> reflectionRequiredSizes;
    openbus::scripting::Vehicle variables;
    std::unique_ptr<ScriptRuntime> scripts;
    SoundEngine* soundPlayback = nullptr;
    VehicleSoundBank soundBank;
    bool soundEventsEnabled = false;
    std::vector<Part> pendingParts;
    std::vector<ModelInteriorLight> interiorLights;
    std::vector<InteriorLightController> interiorLightControllers;
    Matrix4 interiorLightRootModelView = identityMatrix();
    std::string modelCacheKey;
    AssetRequestManager* assets;
    VehiclePlacement placement;
    double modelOffsetZ = 0.0;
    double textureScale = 1;
    double odometerMetres = 0.0;
    std::array<double, 2> odometerPosition = {};
    bool odometerPositionInitialized = false;
    bool frustumCulling = true;
    std::vector<double> lodThresholds;
    std::unordered_map<std::string, std::string> ctcTextureReplacements;
    std::size_t opaqueDisplayCount = 0;
    std::uint64_t viewDepthGeneration = 0;
    // The outer visibility pass advances this once; reflection views and the main
    // view then share unchanged material selections for the rest of that frame.
    std::uint64_t materialSelectionGeneration = 1;
    bool preparedDrawListsValid = false;
    RenderViewContext preparedDrawContext = RenderViewContext::PlayerExterior;
    bool preparedDrawReflection = false;
    bool preparedDrawTransparent = false;
    std::uint64_t preparedDrawAnimationGeneration = 0;
    std::uint64_t preparedDrawMaterialGeneration = 0;
    Matrix4 preparedDrawModelView = {};
    Matrix4 preparedDrawProjection = {};
    bool loaded = false;
    bool hasSharedGeometry = false;
    bool hasLoadedInitialView = false;
    bool loggedAllObjectsLoaded = false;
    bool logPlayerStartupState = false;
    int activeLod = -1;
    double animationTimeStep = 0.0;
    std::uint64_t animationGeneration = 1;
    std::uint64_t clickableStateRevision = 1;
    bool wheelsFromOde = false;
    const BusSimulation* odeSimulation = nullptr;
    std::chrono::steady_clock::time_point textureUploadStart;

    std::string resolveCtcTextureName(const std::string& textureName) const {
        const auto found = ctcTextureReplacements.find(lower(textureName));
        return found == ctcTextureReplacements.end() ? textureName : found->second;
    }

    void updateMaterialChange(Batch& batch) {
        if (batch.materialSelectionGeneration == materialSelectionGeneration) {
            return;
        }
        if (batch.materialSelectionGeneration != 0 && batch.textureChanges.empty()) {
            batch.materialSelectionGeneration = materialSelectionGeneration;
            return;
        }
        TraceScope phase("texture", "updateMaterialChange");
        const MaterialState& selected = selectModelMaterial(
            batch.baseMaterial, [this](const std::string& name) { return variables.get(name); });
        batch.selectedMaterialItem = &selected != &batch.baseMaterial;
        const auto auxiliary = [](AuxiliaryTexture& texture, const std::string& name) {
            if (texture.name != name) {
                texture = {};
                texture.name = name;
            }
        };
        auxiliary(batch.lightmap, selected.lightmapTextureName);
        auxiliary(batch.nightmap, selected.nightmapTextureName);
        auxiliary(batch.transmap, selected.transmapTextureName);
        auxiliary(batch.bumpmap, selected.bumpmapTextureName);
        batch.bumpmapStrength = selected.bumpmapStrength;
        if (batch.environmentTextureName != selected.environmentTextureName) {
            batch.environmentTexture = 0;
            batch.environmentTextureCacheEntry.reset();
            batch.environmentLoadAttempted = false;
        }
        batch.environmentTextureName = selected.environmentTextureName;
        batch.environmentStrength = selected.environmentStrength;
        batch.lightmapStrengthVariable = selected.lightmapStrengthVariable;
        if (batch.freeTextureName != selected.freeTextureName ||
            batch.freeTextureVariable != selected.freeTextureVariable ||
            batch.scriptTextureIndex != selected.scriptTextureIndex ||
            batch.textTextureIndex != selected.textTextureIndex) {
            // Text, script and file textures can reuse the same slot number,
            // but they are different sources. Never retain a previous binding.
            batch.freeTexture = 0;
            batch.freeTextureIndex = -1;
            batch.freeTextureRevision = 0;
            batch.freeTextureAsset = {};
        }
        batch.freeTextureName = selected.freeTextureName;
        batch.freeTextureVariable = selected.freeTextureVariable;
        batch.scriptTextureIndex = selected.scriptTextureIndex;
        batch.textTextureIndex = selected.textTextureIndex;
        batch.texcoordTransXVariable = selected.texcoordTransXVariable;
        batch.texcoordTransYVariable = selected.texcoordTransYVariable;
        batch.alphaMode = selected.alphaMode;
        batch.noZwrite = selected.noZwrite;
        batch.noZcheck = selected.noZcheck;
        batch.alphaScaleVariable = selected.alphaScaleVariable;
        batch.textureWrapS = textureAddressModeToGl(selected.textureAddressS);
        batch.textureWrapT = textureAddressModeToGl(selected.textureAddressT);

        // matl_change's index identifies a texture occurrence in the mesh,
        // not a DDS array layer or a replacement texture.
        const std::filesystem::path texturePath = batch.baseTexturePath;
        const std::string textureName = batch.baseTextureName;
        const int requestedLayer = batch.baseTextureLayer;
        const int layer =
            batch.textureArray
                ? std::clamp(requestedLayer, 0, static_cast<int>(batch.textureArrayLayers) - 1)
                : 0;
        const bool sameRequestedTexture =
            textureName == batch.textureName &&
            (texturePath == batch.texturePath || batch.textureLoadAttempted);
        if (sameRequestedTexture && layer == batch.textureLayer) {
            batch.materialSelectionGeneration = materialSelectionGeneration;
            return;
        }
        batch.textureName = textureName;
        batch.textureCacheEntry.reset();
        batch.textureRequest.reset();
        batch.texture = 0;
        batch.textured = false;
        batch.textureLoadAttempted = false;
        batch.textureLoadStarted = false;
        if (layer != batch.textureLayer) {
            batch.textureLayer = layer;
            for (Vertex& vertex : batch.vertices) {
                vertex.layer = static_cast<float>(layer);
            }
            if (!batch.vertices.empty()) {
                pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                pglBufferData(GL_ARRAY_BUFFER,
                              static_cast<std::ptrdiff_t>(batch.vertices.size() * sizeof(Vertex)),
                              batch.vertices.data(), GL_STATIC_DRAW);
                pglBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        }
        batch.materialSelectionGeneration = materialSelectionGeneration;
    }

    float alphaScale(const Batch& batch) const {
        if (batch.alphaScaleVariable.empty()) {
            return 1.0F;
        }
        return std::clamp(variables.get(batch.alphaScaleVariable), 0.0F, 1.0F);
    }

    void updateAnimationStates() {
        TraceScope trace("render", "Vehicle::updateAnimationStates");
        ++animationGeneration;
        const double timeStep = std::clamp(animationTimeStep, 0.0, 0.25);
        for (DisplayPart& part : displayLists) {
            if (part.animationStates.size() != part.animations.size()) {
                part.animationStates.resize(part.animations.size());
            }
            for (std::size_t index = 0; index < part.animations.size(); ++index) {
                const ModelAnimation& animation = part.animations[index];
                DisplayPart::AnimationState& state = part.animationStates[index];
                if (animation.type.empty()) {
                    state.currentAmount = 0.0;
                    state.targetAmount = 0.0;
                    state.initialized = true;
                    continue;
                }
                const double variableValue = variables.get(animation.variable);
                const double targetAmount = variableValue * animation.scale + animation.offset;
                const bool targetChanged = !state.initialized || targetAmount != state.targetAmount;
                if (animation.type == "anim_trans" && targetChanged &&
                    std::abs(targetAmount - state.currentAmount) > 0.5) {
                    gameLog.Log("Animation translation variable=" + animation.variable +
                                " value=" + std::to_string(variableValue) +
                                " scale=" + std::to_string(animation.scale) +
                                " offset=" + std::to_string(animation.offset) +
                                " target=" + std::to_string(targetAmount) +
                                " current=" + std::to_string(state.currentAmount) +
                                " mesh=" + part.meshIdentifier + " mouse_event=" + part.mouseEvent);
                }
                if (!state.initialized) {
                    state.currentAmount = targetAmount;
                    state.targetAmount = targetAmount;
                    state.initialized = true;
                    continue;
                }
                if (targetAmount != state.targetAmount) {
                    state.targetAmount = targetAmount;
                }
                double nextAmount = state.targetAmount;
                if (animation.delay > 0.0) {
                    const double interpolation = std::min(1.0, animation.delay * timeStep);
                    nextAmount = state.currentAmount +
                                 (state.targetAmount - state.currentAmount) * interpolation;
                }
                if (animation.maxSpeed > 0.0) {
                    const double maximumStep = animation.maxSpeed * timeStep;
                    state.currentAmount +=
                        std::clamp(nextAmount - state.currentAmount, -maximumStep, maximumStep);
                } else {
                    state.currentAmount = nextAmount;
                }
            }
        }
    }

    Matrix4 animationOriginMatrix(const ModelAnimation& animation) const {
        Matrix4 origin = identityMatrix();
        for (const ModelAnimationOrigin& operation : animation.originOperations) {
            Matrix4 operationMatrix = identityMatrix();
            switch (operation.type) {
            case ModelAnimationOriginType::Translation:
                operationMatrix = translationMatrix(operation.value);
                break;
            case ModelAnimationOriginType::RotationX:
                // OpenOMSI applies -angle around model X. Model X converts to
                // render -Y, which is equivalent to +angle around render +Y.
                operationMatrix = rotationMatrix(operation.value[0], 0.0, 1.0, 0.0);
                break;
            case ModelAnimationOriginType::RotationY:
                // Model Y converts to render +X.
                operationMatrix = rotationMatrix(-operation.value[0], 1.0, 0.0, 0.0);
                break;
            case ModelAnimationOriginType::RotationZ:
                // Model Z converts directly to render +Z.
                operationMatrix = rotationMatrix(-operation.value[0], 0.0, 0.0, 1.0);
                break;
            case ModelAnimationOriginType::FromMesh:
                if (animation.hasMeshTransform) {
                    operationMatrix = animation.meshTransform;
                }
                break;
            }
            origin = multiplyMatrix4(origin, operationMatrix);
        }
        if (animation.originOperations.empty() && animation.hasOrigin) {
            origin = translationMatrix(animation.origin);
        }
        return origin;
    }

    Matrix4 animationTransform(const ModelAnimation& animation, double amount) const {
        const Matrix4 origin = animationOriginMatrix(animation);
        Matrix4 inverseOrigin = identityMatrix();
        invertAffineMatrix(origin, inverseOrigin);
        // anim_rot and anim_trans both use the animation frame's local model
        // X axis. Model X converts to render -Y. OpenOMSI's -amount rotation
        // therefore becomes +amount around render +Y; translation converts to
        // negative render Y.
        const Matrix4 local = animation.type == "anim_rot" ? rotationMatrix(amount, 0.0, 1.0, 0.0)
                              : animation.type == "anim_trans"
                                  ? translationMatrix({0.0, -amount, 0.0})
                                  : identityMatrix();
        return multiplyMatrix4(multiplyMatrix4(origin, local), inverseOrigin);
    }

    Matrix4 animationTransformForPart(const DisplayPart& part,
                                      std::vector<const DisplayPart*>& active) const {
        if (part.animationCacheGeneration == animationGeneration) {
            return part.cachedAnimationTransform;
        }
        if (wheelsFromOde && odeSimulation != nullptr && part.odeWheelIndex >= 0) {
            const BodyPose chassis = odeSimulation->chassisPose();
            const BodyPose wheel =
                odeSimulation->wheelPose(static_cast<std::size_t>(part.odeWheelIndex));
            Matrix4 relative = identityMatrix();
            const std::array<double, 3> delta = {wheel.position[0] - chassis.position[0],
                                                 wheel.position[1] - chassis.position[1],
                                                 wheel.position[2] - chassis.position[2]};
            for (int row = 0; row < 3; ++row) {
                const double odeLocalPosition = chassis.rotation[row] * delta[0] +
                                                chassis.rotation[3 + row] * delta[1] +
                                                chassis.rotation[6 + row] * delta[2];
                const double modelLocalPosition =
                    part.center[row] + (row == 2 ? modelOffsetZ : 0.0);
                relative[12 + row] = odeLocalPosition - modelLocalPosition;
                for (int column = 0; column < 3; ++column) {
                    relative[column * 4 + row] =
                        chassis.rotation[row] * wheel.rotation[column] +
                        chassis.rotation[3 + row] * wheel.rotation[3 + column] +
                        chassis.rotation[6 + row] * wheel.rotation[6 + column];
                }
            }
            Matrix4 base = identityMatrix();
            base[5] = 0.0;
            base[6] = -1.0;
            base[9] = 1.0;
            base[10] = 0.0;
            Matrix4 inverseBase = identityMatrix();
            invertAffineMatrix(base, inverseBase);
            relative = multiplyMatrix4(relative, inverseBase);
            const Matrix4 toOrigin =
                translationMatrix({-part.center[0], -part.center[1], -part.center[2]});
            const Matrix4 fromOrigin =
                translationMatrix({part.center[0], part.center[1], part.center[2]});
            part.cachedAnimationTransform =
                multiplyMatrix4(fromOrigin, multiplyMatrix4(relative, toOrigin));
            part.animationCacheGeneration = animationGeneration;
            return part.cachedAnimationTransform;
        }
        Matrix4 local = identityMatrix();
        for (std::size_t index = 0; index < part.animations.size(); ++index) {
            const ModelAnimation& animation = part.animations[index];
            const double value = variables.get(animation.variable);
            const double amount = index < part.animationStates.size()
                                      ? part.animationStates[index].currentAmount
                                      : value * animation.scale + animation.offset;
            local = multiplyMatrix4(animationTransform(animation, amount), local);
        }
        if (part.animationParent.empty() ||
            std::find(active.begin(), active.end(), &part) != active.end()) {
            if (std::find(active.begin(), active.end(), &part) != active.end()) {
                return local;
            }
            part.cachedAnimationTransform = local;
            part.animationCacheGeneration = animationGeneration;
            return local;
        }
        const DisplayPart* parent = nullptr;
        const auto findParent = [&](const std::vector<DisplayPart>& parts) {
            const auto found =
                std::find_if(parts.begin(), parts.end(), [&](const DisplayPart& candidate) {
                    return candidate.meshIdentifier == part.animationParent;
                });
            return found == parts.end() ? nullptr : &*found;
        };
        parent = findParent(displayLists);
        if (parent == nullptr) {
            part.cachedAnimationTransform = local;
            part.animationCacheGeneration = animationGeneration;
            return local;
        }
        active.push_back(&part);
        const Matrix4 parentTransform = animationTransformForPart(*parent, active);
        active.pop_back();
        part.cachedAnimationTransform = multiplyMatrix4(parentTransform, local);
        part.animationCacheGeneration = animationGeneration;
        return part.cachedAnimationTransform;
    }

    Matrix4 animationTransformForPart(const DisplayPart& part) const {
        if (part.animationCacheGeneration == animationGeneration) {
            return part.cachedAnimationTransform;
        }
        std::vector<const DisplayPart*> active;
        return animationTransformForPart(part, active);
    }

    void applyAnimations(const DisplayPart& part) const {
        multiplyMatrix(animationTransformForPart(part));
    }

    void applyDrawTransform(const DisplayPart& part, const Matrix4* groundShadowTransform) const {
        if (part.isShadow && groundShadowTransform != nullptr) {
            multiplyMatrix(*groundShadowTransform);
        }
        applyAnimations(part);
    }

    std::array<double, 3> animatedPartCenter(const DisplayPart& part) const {
        const std::array<double, 4> local = {part.center[0], part.center[1], part.center[2], 1.0};
        const Matrix4 animation = animationTransformForPart(part);
        const std::array<double, 4> transformed = transformPoint(animation, local);
        return {transformed[0], transformed[1], transformed[2] + modelOffsetZ};
    }

    static void setBackFaceCulling(bool enabled) {
        glFrontFace(GL_CW);
        glCullFace(GL_BACK);
        if (enabled) {
            glEnable(GL_CULL_FACE);
        } else {
            glDisable(GL_CULL_FACE);
        }
    }

    void updateFrameVariables(bool isAiVehicle, double timeStep) {
        TraceScope trace("frame", "Vehicle::updateFrameVariables");
        // Frame-scoped values are refreshed before simulation and rendering run.
        variables.updateFrame();
        variables.set("ai", isAiVehicle ? 1.0 : 0.0);
        animationTimeStep = timeStep;
    }

    void updateScripts(bool isAiVehicle) {
        TraceScope trace("script", "Vehicle::updateScripts");
        if (scripts) {
            const std::size_t scriptErrorCount = scripts->errors().size();
            scripts->update(isAiVehicle);
            for (std::size_t index = scriptErrorCount; index < scripts->errors().size(); ++index) {
                gameLog.Log("Lua frame error: " + scripts->errors()[index]);
            }
        }
        if (!isAiVehicle && logPlayerStartupState) {
            gameLog.Log(
                "Player startup states: battery=" +
                std::to_string(variables.get("batterymasterswitchstate")) +
                " busbar=" + std::to_string(variables.get("elec_busbar_main")) +
                " busbar_sw=" + std::to_string(variables.get("elec_busbar_main_sw")) +
                " ignition=" + std::to_string(variables.get("mmc_ignitionswitchstate")) +
                " test_lights=" + std::to_string(variables.get("mmc_startupdashtestlights")) +
                " test_run=" + std::to_string(variables.get("mmc_startupdashtestrun")) +
                " screen=" + std::to_string(variables.get("dashscreenisinitialised")) +
                " marker=" + std::to_string(variables.get("markerlightstate")) +
                " dash_mode=" + std::to_string(variables.get("mmc_dashlightswitch_rot_mode")) +
                " dash_rot=" + std::to_string(variables.get("mmc_dashlightswitch_rot")) +
                " driver_light=" + std::to_string(variables.get("lights_fahrerlicht")) +
                " battery_v=" + std::to_string(variables.get("elec_v_battery")) +
                " battery_load=" + std::to_string(variables.get("elec_battery_load")) +
                " battery_avail=" + std::to_string(variables.get("elec_busbar_avail")) +
                " battery_failure=" + std::to_string(variables.get("elec_failure_general")));
            logPlayerStartupState = false;
        }
    }

    void initializePlayerSystems() {
        if (!scripts) {
            return;
        }
        // if (variables.get("batterymasterswitchstate") == 0.0 &&
        //     scripts->hasScriptEntryPoint("trigger_batterymasterswitch")) {
        //     scripts->invokeKeyBinding("batteryMasterSwitch", true);
        // }
        // // The authored ignition trigger advances StartMode and starts the
        // // dashboard test/electrical busbar sequence after the master switch.
        // if (variables.get("mmc_ignitionswitchstate") == 0.0 &&
        //     scripts->hasScriptEntryPoint("trigger_mmc_ignitionswitch")) {
        //     scripts->invokeKeyBinding("MMC_ignitionSwitch", true);
        // }
        logPlayerStartupState = true;
    }

    const std::vector<int>& reflectionTextureIndicesForPart(const DisplayPart& part) const {
        return part.reflectionTextureIndices;
    }

    void cacheReflectionTextureIndices(DisplayPart& part) const {
        std::unordered_set<int> uniqueReflectionIndices;
        for (const Batch& batch : part.batches) {
            const auto registerReflection = [&](const std::string& textureName) {
                const int reflectionIndex = openbus::rendering::reflectionTextureIndex(textureName);
                if (reflectionIndex >= 0) {
                    uniqueReflectionIndices.insert(reflectionIndex);
                }
            };
            registerReflection(batch.baseTextureName);
            registerReflection(batch.textureName);
            for (const MaterialTextureSource& source : batch.materialTextures) {
                registerReflection(source.textureName);
            }
            for (const MaterialState::TextureChange& change : batch.textureChanges) {
                registerReflection(change.textureName);
            }
        }
        part.reflectionTextureIndices.assign(uniqueReflectionIndices.begin(),
                                             uniqueReflectionIndices.end());
    }

    void prepareFrameVisibility(RenderViewContext context, int viewportWidth, int viewportHeight) {
        TraceScope trace("render", "Vehicle::prepareFrameVisibility");
        ++materialSelectionGeneration;
        {
            TraceScope phase("render", "Vehicle::prepareFrameVisibility.reset");
            variableVisibleParts.resize(displayLists.size());
            visibleClickablePartIndices.clear();
            visibleOpaquePickPartIndices.clear();
            visibleReflectionTextureIndices.clear();
            reflectionRequiredSizes.clear();
        }
        const auto& modelView = openbus::rendering::modelViewMatrix();
        const auto& projection = openbus::rendering::projectionMatrix();
        const double viewportWidthPixels = static_cast<double>(std::max(viewportWidth, 1));
        const double viewportHeightPixels = static_cast<double>(std::max(viewportHeight, 1));
        ViewFrustum frustum;
        {
            TraceScope phase("render", "Vehicle::prepareFrameVisibility.frustum");
            if (frustumCulling) {
                frustum = buildViewFrustum(projection);
            }
        }
        {
            TraceScope phase("render", "Vehicle::prepareFrameVisibility.scanParts");
            for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
                const DisplayPart& part = displayLists[partIndex];
                const bool visible =
                    part.visibleVariable.empty() ||
                    variables.get(part.visibleVariable) == static_cast<double>(part.visibleValue);
                variableVisibleParts[partIndex] = visible;
                if (!visible || !openbus::rendering::viewpointMatches(part.viewpoint, context)) {
                    continue;
                }
                const bool lodMatches =
                    activeLod < 0 || part.lodIndex < 0 || part.lodIndex == activeLod;
                if (lodMatches && !part.mouseEvent.empty()) {
                    visibleClickablePartIndices.push_back(partIndex);
                }
                if (lodMatches && part.pickOccluder) {
                    visibleOpaquePickPartIndices.push_back(partIndex);
                }
                const std::vector<int>& partReflectionIndices =
                    reflectionTextureIndicesForPart(part);
                if (partReflectionIndices.empty()) {
                    continue;
                }
                const std::array<double, 3> center = animatedPartCenter(part);
                const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
                const std::array<double, 4> eye = transformPoint(modelView, local);
                if (frustumCulling && sphereOutsideFrustum(frustum, eye, part.radius)) {
                    continue;
                }
                const Matrix4 animation = animationTransformForPart(part);
                const std::array<double, 3> halfSize = {std::max(part.size[0] * 0.5, 0.0),
                                                        std::max(part.size[1] * 0.5, 0.0),
                                                        std::max(part.size[2] * 0.5, 0.0)};
                double minimumNdcX = std::numeric_limits<double>::max();
                double maximumNdcX = std::numeric_limits<double>::lowest();
                double minimumNdcY = std::numeric_limits<double>::max();
                double maximumNdcY = std::numeric_limits<double>::lowest();
                bool hasProjectedCorner = false;
                bool intersectsNearPlane = false;

                for (int corner = 0; corner < 8; ++corner) {
                    const std::array<double, 4> cornerLocal = {
                        part.center[0] + ((corner & 1) == 0 ? -halfSize[0] : halfSize[0]),
                        part.center[1] + ((corner & 2) == 0 ? -halfSize[1] : halfSize[1]),
                        part.center[2] + ((corner & 4) == 0 ? -halfSize[2] : halfSize[2]), 1.0};
                    std::array<double, 4> animated = transformPoint(animation, cornerLocal);
                    animated[2] += modelOffsetZ;
                    const std::array<double, 4> cornerEye = transformPoint(modelView, animated);
                    const double depth = -cornerEye[2];
                    if (depth <= openbus::rendering::kReflectionNearPlane) {
                        intersectsNearPlane = true;
                        continue;
                    }
                    const double clipX =
                        projection[0] * cornerEye[0] + projection[4] * cornerEye[1] +
                        projection[8] * cornerEye[2] + projection[12] * cornerEye[3];
                    const double clipY =
                        projection[1] * cornerEye[0] + projection[5] * cornerEye[1] +
                        projection[9] * cornerEye[2] + projection[13] * cornerEye[3];
                    minimumNdcX = std::min(minimumNdcX, clipX / depth);
                    maximumNdcX = std::max(maximumNdcX, clipX / depth);
                    minimumNdcY = std::min(minimumNdcY, clipY / depth);
                    maximumNdcY = std::max(maximumNdcY, clipY / depth);
                    hasProjectedCorner = true;
                }
                if (!hasProjectedCorner) {
                    continue;
                }
                const double projectedWidth =
                    intersectsNearPlane
                        ? viewportWidthPixels
                        : std::clamp((maximumNdcX - minimumNdcX) * 0.5 * viewportWidthPixels, 0.0,
                                     viewportWidthPixels);
                const double projectedHeight =
                    intersectsNearPlane
                        ? viewportHeightPixels
                        : std::clamp((maximumNdcY - minimumNdcY) * 0.5 * viewportHeightPixels, 0.0,
                                     viewportHeightPixels);
                const double screenBoundedDiameter = std::max(projectedWidth, projectedHeight);
                const int requiredSize =
                    std::max(openbus::rendering::kMinReflectionTargetSize,
                             static_cast<int>(std::ceil(screenBoundedDiameter)));

                for (const int reflectionIndex : partReflectionIndices) {
                    visibleReflectionTextureIndices.insert(reflectionIndex);
                    reflectionRequiredSizes[reflectionIndex] =
                        std::max(reflectionRequiredSizes[reflectionIndex], requiredSize);
                }
            }
        }
    }

    bool needsReflectionTexture(std::size_t reflectionIndex) const {
        return visibleReflectionTextureIndices.find(static_cast<int>(reflectionIndex)) !=
               visibleReflectionTextureIndices.end();
    }

    int requiredReflectionSize(std::size_t reflectionIndex) const {
        const auto required = reflectionRequiredSizes.find(static_cast<int>(reflectionIndex));
        return required == reflectionRequiredSizes.end()
                   ? openbus::rendering::kMinReflectionTargetSize
                   : required->second;
    }

    void updateSimulationVariables(const BusSimulation& simulation, double throttle,
                                   double steering, double brake) {
        simulation.updateVariables(variables, throttle, steering, brake);
        const double currentX = simulation.positionX();
        const double currentY = simulation.positionY();
        if (!odometerPositionInitialized) {
            odometerPosition = {currentX, currentY};
            odometerPositionInitialized = true;
        } else {
            const double distance =
                std::hypot(currentX - odometerPosition[0], currentY - odometerPosition[1]);
            if (std::isfinite(distance) && distance <= 100.0) {
                odometerMetres += distance;
            }
            odometerPosition = {currentX, currentY};
        }
        odometerMetres = std::max(0.0, odometerMetres);
        variables.set("kmcounter_km", std::floor(odometerMetres / 1000.0));
        variables.set("kmcounter_m", std::fmod(odometerMetres, 1000.0));
    }

    void setOdeSimulation(const BusSimulation& simulation) {
        odeSimulation = &simulation;
    }

    bool clickablePartVisible(const DisplayPart& part, std::size_t partIndex,
                              RenderViewContext context) const {
        const bool visible =
            partIndex < variableVisibleParts.size()
                ? variableVisibleParts[partIndex]
                : part.visibleVariable.empty() ||
                      variables.get(part.visibleVariable) == static_cast<double>(part.visibleValue);
        if (!visible || !openbus::rendering::viewpointMatches(part.viewpoint, context)) {
            return false;
        }
        return activeLod < 0 || part.lodIndex < 0 || part.lodIndex == activeLod;
    }

    const DisplayPart* pickClickable(double cursorX, double cursorY, int viewportWidth,
                                     int viewportHeight, RenderViewContext context,
                                     const Batch** hitBatch = nullptr,
                                     std::size_t* hitTriangle = nullptr) const {
        TraceScope trace("input", "Vehicle::pickClickable");
        (void)context;
        if (hitBatch != nullptr) {
            *hitBatch = nullptr;
        }
        if (hitTriangle != nullptr) {
            *hitTriangle = 0;
        }
        if (viewportWidth <= 0 || viewportHeight <= 0) {
            return nullptr;
        }
        const auto& modelView = openbus::rendering::modelViewMatrix();
        const auto& projection = openbus::rendering::projectionMatrix();
        const Batch* selectedBatch = nullptr;
        std::size_t selectedTriangle = 0;
        const double normalizedX = cursorX / static_cast<double>(viewportWidth) * 2.0 - 1.0;
        const double normalizedY = 1.0 - cursorY / static_cast<double>(viewportHeight) * 2.0;
        struct ProjectedVertex {
            double x = 0.0;
            double y = 0.0;
            double depth = 0.0;
            bool valid = false;
        };
        const auto projectVertex = [&](const Matrix4& modelViewPart, const Vertex& vertex) {
            const std::array<double, 4> view =
                transformPoint(modelViewPart, {vertex.x, vertex.y, vertex.z, 1.0});
            const std::array<double, 4> clip = transformPoint(projection, view);
            if (clip[3] <= 1.0e-8) {
                return ProjectedVertex{};
            }
            return ProjectedVertex{clip[0] / clip[3], clip[1] / clip[3], -view[2], true};
        };
        const auto cross2d = [](double ax, double ay, double bx, double by) {
            return ax * by - ay * bx;
        };
        const auto triangleDepthAtCursor = [&](const Matrix4& modelViewPart,
                                               const DisplayPart& part, const Batch& batch,
                                               std::size_t index, double& depth) {
            const ProjectedVertex first = projectVertex(modelViewPart, batch.vertices[index]);
            const ProjectedVertex second = projectVertex(modelViewPart, batch.vertices[index + 1]);
            const ProjectedVertex third = projectVertex(modelViewPart, batch.vertices[index + 2]);
            if (!first.valid || !second.valid || !third.valid) {
                return false;
            }
            const double denominator = cross2d(second.x - first.x, second.y - first.y,
                                               third.x - first.x, third.y - first.y);
            if (std::abs(denominator) <= 1.0e-12 || (part.backFaceCulling && denominator >= 0.0)) {
                return false;
            }
            const double firstWeight = cross2d(second.x - normalizedX, second.y - normalizedY,
                                               third.x - normalizedX, third.y - normalizedY) /
                                       denominator;
            const double secondWeight = cross2d(third.x - normalizedX, third.y - normalizedY,
                                                first.x - normalizedX, first.y - normalizedY) /
                                        denominator;
            const double thirdWeight = 1.0 - firstWeight - secondWeight;
            constexpr double triangleTolerance = 1.0e-7;
            if (firstWeight < -triangleTolerance || secondWeight < -triangleTolerance ||
                thirdWeight < -triangleTolerance) {
                return false;
            }
            depth =
                firstWeight * first.depth + secondWeight * second.depth + thirdWeight * third.depth;
            return depth >= 0.0;
        };
        const auto cursorWithinProjectedBounds = [&](const Matrix4& modelViewPart,
                                                     const DisplayPart& part,
                                                     std::array<double, 4>& viewCenter,
                                                     std::array<double, 4>& clipCenter) {
            const std::array<double, 3> center = {part.center[0], part.center[1], part.center[2]};
            viewCenter = transformPoint(modelViewPart, {center[0], center[1], center[2], 1.0});
            clipCenter = transformPoint(projection, viewCenter);
            double minimumNdcX = std::numeric_limits<double>::max();
            double maximumNdcX = std::numeric_limits<double>::lowest();
            double minimumNdcY = std::numeric_limits<double>::max();
            double maximumNdcY = std::numeric_limits<double>::lowest();
            const std::array<double, 3> halfSize = {std::max(part.size[0] * 0.5, 0.0),
                                                    std::max(part.size[1] * 0.5, 0.0),
                                                    std::max(part.size[2] * 0.5, 0.0)};
            bool hasProjectedCorner = false;
            for (int corner = 0; corner < 8; ++corner) {
                const std::array<double, 4> local = {
                    center[0] + ((corner & 1) == 0 ? -halfSize[0] : halfSize[0]),
                    center[1] + ((corner & 2) == 0 ? -halfSize[1] : halfSize[1]),
                    center[2] + ((corner & 4) == 0 ? -halfSize[2] : halfSize[2]), 1.0};
                const std::array<double, 4> view = transformPoint(modelViewPart, local);
                const std::array<double, 4> clip = transformPoint(projection, view);
                if (!std::isfinite(clip[0]) || !std::isfinite(clip[1]) || !std::isfinite(clip[3]) ||
                    clip[3] <= 1.0e-8) {
                    // Retain eye-plane and invalid cases for the exact path.
                    return true;
                }
                const double ndcX = clip[0] / clip[3];
                const double ndcY = clip[1] / clip[3];
                if (!std::isfinite(ndcX) || !std::isfinite(ndcY)) {
                    return true;
                }
                minimumNdcX = std::min(minimumNdcX, ndcX);
                maximumNdcX = std::max(maximumNdcX, ndcX);
                minimumNdcY = std::min(minimumNdcY, ndcY);
                maximumNdcY = std::max(maximumNdcY, ndcY);
                hasProjectedCorner = true;
            }
            if (!hasProjectedCorner) {
                return true;
            }
            return normalizedX >= minimumNdcX && normalizedX <= maximumNdcX &&
                   normalizedY >= minimumNdcY && normalizedY <= maximumNdcY;
        };
        double closestOccluderDepth = std::numeric_limits<double>::max();
        for (const std::size_t partIndex : visibleOpaquePickPartIndices) {
            const DisplayPart& part = displayLists[partIndex];
            const Matrix4 modelViewPart = multiplyMatrix4(
                modelView, multiplyMatrix4(translationMatrix({0.0, 0.0, modelOffsetZ}),
                                           animationTransformForPart(part)));
            std::array<double, 4> viewCenter = {};
            std::array<double, 4> clipCenter = {};
            if (!cursorWithinProjectedBounds(modelViewPart, part, viewCenter, clipCenter)) {
                continue;
            }
            for (const Batch& batch : part.batches) {
                if (batch.alphaMode != 0 || batch.noZwrite || batch.noZcheck) {
                    continue;
                }
                for (std::size_t index = 0; index + 2 < batch.vertices.size(); index += 3) {
                    double triangleDepth = 0.0;
                    if (triangleDepthAtCursor(modelViewPart, part, batch, index, triangleDepth)) {
                        closestOccluderDepth = std::min(closestOccluderDepth, triangleDepth);
                    }
                }
            }
        }
        double closestDepth = std::numeric_limits<double>::max();
        const DisplayPart* selected = nullptr;
        for (const std::size_t partIndex : visibleClickablePartIndices) {
            const DisplayPart& part = displayLists[partIndex];
            const Matrix4 modelViewPart = multiplyMatrix4(
                modelView, multiplyMatrix4(translationMatrix({0.0, 0.0, modelOffsetZ}),
                                           animationTransformForPart(part)));
            std::array<double, 4> viewCenter = {};
            std::array<double, 4> clipCenter = {};
            if (!cursorWithinProjectedBounds(modelViewPart, part, viewCenter, clipCenter)) {
                continue;
            }
            if (std::abs(clipCenter[3]) <= 1.0e-8) {
                continue;
            }
            double partDistance = std::numeric_limits<double>::max();
            const Batch* partBatch = nullptr;
            std::size_t partTriangle = 0;
            for (const Batch& batch : part.batches) {
                for (std::size_t index = 0; index + 2 < batch.vertices.size(); index += 3) {
                    double triangleDistance = 0.0;
                    if (!triangleDepthAtCursor(modelViewPart, part, batch, index,
                                               triangleDistance)) {
                        continue;
                    }
                    if (triangleDistance < partDistance) {
                        partDistance = triangleDistance;
                        partBatch = &batch;
                        partTriangle = index;
                    }
                }
            }
            if (partDistance >= closestDepth) {
                continue;
            }
            if (partDistance == std::numeric_limits<double>::max()) {
                continue;
            }
            if (partDistance >
                closestOccluderDepth + 1.0e-4 * std::max(1.0, closestOccluderDepth)) {
                continue;
            }
            closestDepth = partDistance;
            selected = &part;
            selectedBatch = partBatch;
            selectedTriangle = partTriangle;
        }
        if (hitBatch != nullptr) {
            *hitBatch = selectedBatch;
        }
        if (hitTriangle != nullptr) {
            *hitTriangle = selectedTriangle;
        }
        return selected;
    }

    bool hasClickableAt(double cursorX, double cursorY, int viewportWidth, int viewportHeight,
                        RenderViewContext context) const {
        const Batch* selectedBatch = nullptr;
        std::size_t selectedTriangle = 0;
        const DisplayPart* selected = pickClickable(cursorX, cursorY, viewportWidth, viewportHeight,
                                                    context, &selectedBatch, &selectedTriangle);
        hoveredClickablePartIndex = selected == nullptr
                                        ? std::numeric_limits<std::size_t>::max()
                                        : static_cast<std::size_t>(selected - displayLists.data());
        hoveredClickableBatchIndex = std::numeric_limits<std::size_t>::max();
        hoveredClickableTriangle = selectedTriangle;
        if (selected != nullptr && selectedBatch != nullptr) {
            hoveredClickableBatchIndex =
                static_cast<std::size_t>(selectedBatch - selected->batches.data());
        }
        return selected != nullptr;
    }

    bool hasVisibleClickable(RenderViewContext context) const {
        (void)context;
        return !visibleClickablePartIndices.empty();
    }

    std::uint64_t clickableRevision() const {
        return clickableStateRevision + displayLists.size();
    }

    std::string clickableDebugDescription(const DisplayPart& part, std::size_t partIndex,
                                          RenderViewContext context) const {
        std::string scriptEvent = lower(part.mouseEvent);
        for (char& character : scriptEvent) {
            const unsigned char byte = static_cast<unsigned char>(character);
            character = std::isalnum(byte) ? static_cast<char>(std::tolower(byte)) : '_';
        }
        const std::string handler = "trigger_" + scriptEvent;
        const bool handlerFound = scripts != nullptr && scripts->hasScriptEntryPoint(handler);
        std::ostringstream description;
        description << "Clickable part[" << partIndex << "] mesh=\"" << part.meshName << "\"";
        if (!part.meshIdentifier.empty()) {
            description << " id=\"" << part.meshIdentifier << "\"";
        }
        description << " mouseevent=\"" << part.mouseEvent << "\" maps_to=" << handler
                    << " handler=" << (handlerFound ? "found" : "missing") << " visible="
                    << (clickablePartVisible(part, partIndex, context) ? "yes" : "no")
                    << " bounds_size_xyz=(" << part.size[0] << ',' << part.size[1] << ','
                    << part.size[2] << ") [model units]";
        return description.str();
    }

    void logClickableDebugInfo(RenderViewContext context) const {
        std::size_t clickableCount = 0;
        for (const DisplayPart& part : displayLists) {
            clickableCount += !part.mouseEvent.empty() ? 1U : 0U;
        }
        gameLog.Log("Clickable overlay: " + std::to_string(clickableCount) +
                    " mesh(es) with [mouseevent]; bounds are in model units");
        for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
            const DisplayPart& part = displayLists[partIndex];
            if (!part.mouseEvent.empty()) {
                gameLog.Log(clickableDebugDescription(part, partIndex, context));
            }
        }
    }

    void drawClickableDebug(RenderViewContext context) const {
        TraceScope trace("debug", "Vehicle::drawClickableDebug");
        static constexpr std::array<std::array<int, 2>, 12> boxEdges = {{{0, 1},
                                                                         {1, 3},
                                                                         {3, 2},
                                                                         {2, 0},
                                                                         {4, 5},
                                                                         {5, 7},
                                                                         {7, 6},
                                                                         {6, 4},
                                                                         {0, 4},
                                                                         {1, 5},
                                                                         {2, 6},
                                                                         {3, 7}}};
        clickableDebugBoxVertices.clear();
        clickableDebugWireVertices.clear();
        clickableDebugTriangleVertices.clear();
        clickableDebugBoxVertices.reserve(visibleClickablePartIndices.size() * 24);
        const auto appendVertex = [](std::vector<openbus::rendering::PrimitiveVertex>& vertices,
                                     const std::array<double, 4>& point,
                                     const std::array<float, 3>& color) {
            vertices.push_back({static_cast<float>(point[0]), static_cast<float>(point[1]),
                                static_cast<float>(point[2]), color[0], color[1], color[2]});
        };
        const auto transformVertex = [this](const Matrix4& animation, const Vertex& vertex) {
            std::array<double, 4> point =
                transformPoint(animation, {vertex.x, vertex.y, vertex.z, 1.0});
            point[2] += modelOffsetZ;
            return point;
        };
        const DisplayPart* selected = nullptr;
        const Batch* selectedBatch = nullptr;
        std::size_t selectedTriangle = hoveredClickableTriangle;
        if (hoveredClickablePartIndex < displayLists.size()) {
            const DisplayPart& part = displayLists[hoveredClickablePartIndex];
            if (clickablePartVisible(part, hoveredClickablePartIndex, context)) {
                selected = &part;
                if (hoveredClickableBatchIndex < part.batches.size()) {
                    selectedBatch = &part.batches[hoveredClickableBatchIndex];
                }
            }
        }
        for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
            const DisplayPart& part = displayLists[partIndex];
            if (part.mouseEvent.empty() || !clickablePartVisible(part, partIndex, context)) {
                continue;
            }
            const bool isSelected = &part == selected;
            const std::array<float, 3> boundsColor = isSelected
                                                         ? std::array<float, 3>{1.0f, 0.15f, 0.05f}
                                                         : std::array<float, 3>{0.0f, 0.75f, 0.95f};
            const Matrix4 animation = animationTransformForPart(part);
            const std::array<double, 3> half = {std::max(part.size[0] * 0.5, 0.0),
                                                std::max(part.size[1] * 0.5, 0.0),
                                                std::max(part.size[2] * 0.5, 0.0)};
            std::array<std::array<double, 4>, 8> corners = {};
            for (int corner = 0; corner < 8; ++corner) {
                const std::array<double, 4> local = {
                    part.center[0] + ((corner & 1) == 0 ? -half[0] : half[0]),
                    part.center[1] + ((corner & 2) == 0 ? -half[1] : half[1]),
                    part.center[2] + ((corner & 4) == 0 ? -half[2] : half[2]), 1.0};
                corners[static_cast<std::size_t>(corner)] = transformPoint(animation, local);
                corners[static_cast<std::size_t>(corner)][2] += modelOffsetZ;
            }
            for (const auto& edge : boxEdges) {
                appendVertex(clickableDebugBoxVertices, corners[static_cast<std::size_t>(edge[0])],
                             boundsColor);
                appendVertex(clickableDebugBoxVertices, corners[static_cast<std::size_t>(edge[1])],
                             boundsColor);
            }
            const std::array<float, 3> meshColor = {1.0f, 0.85f, 0.05f};
            for (const Batch& batch : part.batches) {
                for (std::size_t index = 0; index + 2 < batch.vertices.size(); index += 3) {
                    const auto first = transformVertex(animation, batch.vertices[index]);
                    const auto second = transformVertex(animation, batch.vertices[index + 1]);
                    const auto third = transformVertex(animation, batch.vertices[index + 2]);
                    appendVertex(clickableDebugWireVertices, first, meshColor);
                    appendVertex(clickableDebugWireVertices, second, meshColor);
                    appendVertex(clickableDebugWireVertices, second, meshColor);
                    appendVertex(clickableDebugWireVertices, third, meshColor);
                    appendVertex(clickableDebugWireVertices, third, meshColor);
                    appendVertex(clickableDebugWireVertices, first, meshColor);
                }
                if (isSelected && selectedBatch == &batch &&
                    selectedTriangle + 2 < batch.vertices.size()) {
                    const std::array<float, 3> fillColor = {1.0f, 0.15f, 0.02f};
                    const std::array<float, 3> outlineColor = {1.0f, 1.0f, 0.2f};
                    const auto first = transformVertex(animation, batch.vertices[selectedTriangle]);
                    const auto second =
                        transformVertex(animation, batch.vertices[selectedTriangle + 1]);
                    const auto third =
                        transformVertex(animation, batch.vertices[selectedTriangle + 2]);
                    appendVertex(clickableDebugTriangleVertices, first, fillColor);
                    appendVertex(clickableDebugTriangleVertices, second, fillColor);
                    appendVertex(clickableDebugTriangleVertices, third, fillColor);
                    appendVertex(clickableDebugWireVertices, first, outlineColor);
                    appendVertex(clickableDebugWireVertices, second, outlineColor);
                    appendVertex(clickableDebugWireVertices, second, outlineColor);
                    appendVertex(clickableDebugWireVertices, third, outlineColor);
                    appendVertex(clickableDebugWireVertices, third, outlineColor);
                    appendVertex(clickableDebugWireVertices, first, outlineColor);
                }
            }
        }
        if (clickableDebugBoxVertices.empty() && clickableDebugWireVertices.empty()) {
            return;
        }
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        if (!clickableDebugBoxVertices.empty()) {
            openbus::rendering::drawPrimitives(clickableDebugBoxVertices, GL_LINES);
        }
        if (!clickableDebugTriangleVertices.empty()) {
            openbus::rendering::drawPrimitives(clickableDebugTriangleVertices, GL_TRIANGLES);
        }
        if (!clickableDebugWireVertices.empty()) {
            openbus::rendering::drawPrimitives(clickableDebugWireVertices, GL_LINES, 1.5f);
        }
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }

    std::string mouseEventAt(double cursorX, double cursorY, int viewportWidth, int viewportHeight,
                             RenderViewContext context) const {
        const DisplayPart* selected =
            pickClickable(cursorX, cursorY, viewportWidth, viewportHeight, context);
        return selected == nullptr ? std::string() : selected->mouseEvent;
    }

    bool handleMouseClick(const std::string& eventName) {
        if (!scripts) {
            return false;
        }
        if (eventName.empty()) {
            return false;
        }
        if (scripts->hasScriptEntryPoint("trigger_" + lower(eventName))) {
            scripts->invokeMouseEvent(eventName);
        }
        return true;
    }

    void joinTextureWorkers() {
        assets->join();
    }

#ifdef _WIN32
    bool comInitialized = false;
#endif

    explicit Vehicle(const std::shared_ptr<const SharedVehicleDefinition>& definition,
                     const VehiclePlacement& configuredPlacement, double configuredModelOffsetZ,
                     ModelLoadingPolicy policy, AssetRequestManager& manager,
                     SimulationState& simulationState, SoundEngine& soundEngine,
                     openbus::rendering::ViewpointContext& soundViewpoint)
        : loadingPolicy(policy), variables(), soundPlayback(&soundEngine),
          modelCacheKey(definition->cacheKey), assets(&manager), placement(configuredPlacement),
          modelOffsetZ(configuredModelOffsetZ),
          wheelsFromOde(parseEnabledFlag(std::getenv("OPENBUS_WHEELS_FROM_ODE"))) {
        if (const char* scale = std::getenv("OPENBUS_TEXTURE_SCALE")) {
            try {
                textureScale = std::clamp(std::stod(scale), 0.25, 1.0);
            } catch (const std::exception&) {
                textureScale = 1.0;
            }
        }
        if (const char* culling = std::getenv("OPENBUS_FRUSTUM_CULLING")) {
            frustumCulling = std::string(culling) == "1";
        }
#ifdef _WIN32
        const HRESULT comResult = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        comInitialized = SUCCEEDED(comResult);
#endif
        VehicleConfig vehicleConfiguration = definition->vehicleConfiguration;
        selectRuntimeRegistration(vehicleConfiguration, configuredPlacement);
        odometerMetres =
            vehicleConfiguration.hasOdometerInitial
                ? std::max(0.0, vehicleConfiguration.odometerInitialKilometres) * 1000.0
                : 0.0;
        soundBank.load(vehicleConfiguration.soundConfigPath);
        scripts = std::make_unique<ScriptRuntime>(
            vehicleConfiguration, variables, simulationState,
            [this, &soundEngine, &soundViewpoint](const std::string& name, const std::string& file,
                                                  double controlValue) {
                if (!soundEventsEnabled) {
                    return;
                }
                if (file.empty() && !soundBank.hasTrigger(name) && name.size() > 4 &&
                    name.compare(name.size() - 4, 4, "_off") == 0) {
                    soundBank.stop(soundEngine, name.substr(0, name.size() - 4));
                } else {
                    soundBank.trigger(soundEngine, name, file, static_cast<float>(controlValue),
                                      variables, soundViewpoint);
                }
            });
        for (const std::string& error : scripts->errors()) {
            gameLog.Log("Lua script error: " + error);
        }
        for (const std::string& name : definition->modelNumericVariables) {
            variables.declare(name);
        }
        for (const std::string& name : definition->modelStringVariables) {
            variables.declareString(name);
        }
        load(*definition);
        scripts->initialize();
        for (const std::string& error : scripts->errors()) {
            gameLog.Log("Lua initialization error: " + error);
        }
        scripts->update(false);
        if (!hasSharedGeometry) {
            spawn();
            cacheSharedDisplayLists();
        } else {
            loggedAllObjectsLoaded = true;
        }
        // Suppress sound callbacks caused by scripts settling initial state.
        soundEventsEnabled = true;
    }

    ~Vehicle() {
        if (soundPlayback != nullptr) {
            soundBank.stopAllLoops(*soundPlayback);
        }
        joinTextureWorkers();
        const auto deleteBuffers = [](const std::vector<DisplayPart>& parts) {
            for (const DisplayPart& part : parts) {
                for (const Batch& batch : part.batches) {
                    if (batch.buffer && !batch.bufferTracked) {
                        pglDeleteBuffers(1, &batch.buffer);
                    }
                }
            }
        };
        deleteBuffers(displayLists);
#ifdef _WIN32
        if (comInitialized) {
            CoUninitialize();
        }
#endif
    }

    void applyInteriorLightMaterial(const Batch& batch,
                                    openbus::rendering::ModelMaterial& material) const {
        material.interiorLightCount = 0;
        material.interiorLightViewPositions = {};
        material.interiorLightColors = {};
        material.interiorLightStrengths = {};
        for (const int lightIndex : batch.interiorLightIndexes) {
            if (lightIndex < 0 || static_cast<std::size_t>(lightIndex) >= interiorLights.size()) {
                continue;
            }
            const std::size_t index = static_cast<std::size_t>(lightIndex);
            const ModelInteriorLight& light = interiorLights[index];
            const InteriorLightController& controller = interiorLightControllers[index];
            const double brightness =
                std::max(0.0, controller.isNumeric ? controller.numericValue
                                                   : variables.get(light.controller));
            const double strength = brightness * std::max(0.0, light.intensity);
            if (strength <= 0.0 || material.interiorLightCount >=
                                       static_cast<int>(openbus::rendering::MAX_INTERIOR_LIGHTS)) {
                continue;
            }
            const int outputIndex = material.interiorLightCount++;
            for (std::size_t channel = 0; channel < 3; ++channel) {
                material.interiorLightColors[outputIndex][channel] =
                    static_cast<float>(std::clamp(light.color[channel] / 255.0, 0.0, 1.0));
            }
            // Light locations are model-root positions, independent of the
            // receiving mesh's animation. Transform to view space once from
            // the vehicle root so the shader can compare them to view-space
            // fragment positions without inheriting a door/panel animation.
            const std::array<double, 3> viewPosition =
                openbus::rendering::interiorLightPositionInViewSpace(interiorLightRootModelView,
                                                                     light.position);
            material.interiorLightViewPositions[outputIndex] = {
                static_cast<float>(viewPosition[0]), static_cast<float>(viewPosition[1]),
                static_cast<float>(viewPosition[2])};
            material.interiorLightStrengths[outputIndex] = static_cast<float>(strength);
        }
        material.useInteriorLight = material.interiorLightCount > 0;
    }

    void drawBatch(Batch& batch, double alpha, bool forceUntextured = false,
                   const std::array<double, 3>* overrideColor = nullptr, int alphaModeOverride = -1,
                   bool forceDepthTest = false) {
        if (alpha <= 0.0) {
            return;
        }
        const int alphaMode = alphaModeOverride >= 0 ? alphaModeOverride : batch.alphaMode;
        const bool noZwrite = alphaModeOverride >= 0 ? false : batch.noZwrite;
        if (alphaMode == 2 || noZwrite) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
        } else if (alphaMode == 1) {
            glDisable(GL_BLEND);
            glDepthMask(GL_FALSE);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(-1.0f, -1.0f);
        } else {
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
        }
        // Ground-projected fake shadows must still be occluded by vehicle
        // geometry; keep their noZwrite behavior but honor the depth test.
        const bool noZcheck = batch.noZcheck && !forceDepthTest;
        if (noZcheck) {
            glDisable(GL_DEPTH_TEST);
        } else if (forceDepthTest) {
            glEnable(GL_DEPTH_TEST);
        }
        const bool materialBatch = !batch.materialTextures.empty() && !forceUntextured;
        const std::vector<GLuint>* materialTextureIds = nullptr;
        {
            TraceScope phase("render", "Vehicle::drawBatch.resolveTextures");
            if (materialBatch) {
                batch.materialTextureIds.resize(batch.materialTextures.size());
                batch.materialTextureFlips.resize(batch.materialTextures.size());
                std::fill(batch.materialTextureFlips.begin(), batch.materialTextureFlips.end(),
                          false);
                for (std::size_t sourceIndex = 0; sourceIndex < batch.materialTextures.size();
                     ++sourceIndex) {
                    MaterialTextureSource& source = batch.materialTextures[sourceIndex];
                    ensureMaterialTexture(source);
                    if (!openbus::rendering::reflectionPassActive()) {
                        const int reflectionIndex =
                            openbus::rendering::reflectionTextureIndex(source.textureName);
                        const unsigned int reflectionTexture =
                            openbus::rendering::reflectionTextureForIndex(reflectionIndex);
                        if (reflectionTexture != 0) {
                            source.texture = reflectionTexture;
                            source.textureArray = false;
                            batch.materialTextureFlips[sourceIndex] = true;
                        }
                    }
                    batch.materialTextureIds[sourceIndex] = source.texture;
                }
                materialTextureIds = &batch.materialTextureIds;
            } else {
                ensureTexture(batch);
            }
            ensureAuxiliaryTexture(batch, batch.lightmap);
            ensureAuxiliaryTexture(batch, batch.nightmap);
            ensureAuxiliaryTexture(batch, batch.transmap);
            ensureAuxiliaryTexture(batch, batch.bumpmap);
            updateFreeTexture(batch);
        }
        const std::array<double, 3>& color =
            overrideColor == nullptr ? batch.color : *overrideColor;
        TraceScope phase("render", "Vehicle::drawBatch.prepareAndSubmit");
        openbus::rendering::ModelMaterial material;
        material.texture = materialBatch ? 0 : batch.texture;
        material.textureArray = materialBatch ? false : batch.textureArray;
        material.textured = materialBatch ? false : batch.textured && !forceUntextured;
        material.textureWrapS = batch.textureWrapS;
        material.textureWrapT = batch.textureWrapT;
        material.lightmap = batch.lightmap.texture;
        material.nightmap = batch.nightmap.texture;
        material.transmap = batch.transmap.texture;
        material.bumpmap = batch.bumpmap.texture;
        material.freeTexture = batch.freeTexture;
        // Text textures use their own shader sampler. They share the dynamic
        // upload storage with script/free textures, but must not be left only
        // in the free-texture sampler or the shader will never display them.
        material.textTexture = batch.textTextureIndex >= 0 ? batch.freeTexture : 0;
        material.useLightmap = !forceUntextured && material.lightmap != 0;
        material.useNightmap = !forceUntextured && material.nightmap != 0;
        material.useTransmap = !forceUntextured && material.transmap != 0;
        material.useBumpmap = !forceUntextured && material.bumpmap != 0;
        material.useFreeTexture =
            !forceUntextured && batch.textTextureIndex < 0 && material.freeTexture != 0;
        material.useTextTexture =
            !forceUntextured && batch.textTextureIndex >= 0 && material.textTexture != 0;
        if (!forceUntextured) {
            applyInteriorLightMaterial(batch, material);
        }
        material.lightmapStrength = static_cast<float>(
            batch.lightmapStrengthVariable.empty()
                ? 1.0F
                : std::clamp(variables.get(batch.lightmapStrengthVariable), 0.0F, 1.0F));
        // Script-selected emissive items illuminate even in daytime. Ordinary
        // base nightmaps still follow the host's environmental lighting.
        const float nightlight =
            batch.selectedMaterialItem
                ? 1.0F
                : std::max(variables.get("nightlighta"), 1.0F - variables.get("envir_brightness"));
        material.nightmapStrength = std::clamp(nightlight, 0.0F, 1.0F);
        material.bumpmapStrength = static_cast<float>(std::clamp(batch.bumpmapStrength, 0.0, 1.0));
        material.texcoordOffsetX = static_cast<float>(
            batch.texcoordTransXVariable.empty() ? 0.0
                                                 : variables.get(batch.texcoordTransXVariable));
        material.texcoordOffsetY = static_cast<float>(
            batch.texcoordTransYVariable.empty() ? 0.0
                                                 : variables.get(batch.texcoordTransYVariable));
        if (!openbus::rendering::reflectionPassActive() && !materialBatch) {
            const int reflectionIndex = openbus::rendering::reflectionTextureIndex(
                batch.textureName.empty() ? batch.texturePath.string() : batch.textureName);
            const unsigned int reflectionTexture =
                openbus::rendering::reflectionTextureForIndex(reflectionIndex);
            if (reflectionTexture != 0) {
                material.texture = reflectionTexture;
                material.textureArray = false;
                material.textured = true;
                material.flipTextureY = true;
            }
        }
        if (materialBatch) {
            drawMaterialBatch(batch.buffer, batch.vertexCount, batch.materialColors,
                              *materialTextureIds, batch.materialTextureFlips);
        } else {
            drawModelBatch(batch.buffer, batch.vertexCount, material, color, alpha,
                           forceUntextured ? 0 : alphaMode);
        }
        if (noZcheck) {
            glEnable(GL_DEPTH_TEST);
        }
    }

    void sortTransparentBatch(Batch& batch) {
        if (batch.vertices.size() < 6 || batch.vertices.size() % 3 != 0) {
            return;
        }
        struct TriangleDepth {
            std::size_t index;
            double depth;
        };
        const Matrix4& modelView = openbus::rendering::modelViewMatrix();
        std::vector<TriangleDepth> triangles;
        triangles.reserve(batch.vertices.size() / 3);
        for (std::size_t index = 0; index < batch.vertices.size(); index += 3) {
            const auto& first = batch.vertices[index];
            const auto& second = batch.vertices[index + 1];
            const auto& third = batch.vertices[index + 2];
            const std::array<double, 4> center = {
                (static_cast<double>(first.x) + second.x + third.x) / 3.0,
                (static_cast<double>(first.y) + second.y + third.y) / 3.0,
                (static_cast<double>(first.z) + second.z + third.z) / 3.0, 1.0};
            const std::array<double, 4> viewCenter = transformPoint(modelView, center);
            triangles.push_back({index, -viewCenter[2]});
        }
        std::stable_sort(triangles.begin(), triangles.end(),
                         [](const TriangleDepth& first, const TriangleDepth& second) {
                             return first.depth > second.depth;
                         });
        std::vector<Vertex> sortedVertices;
        sortedVertices.reserve(batch.vertices.size());
        for (const TriangleDepth& triangle : triangles) {
            sortedVertices.insert(sortedVertices.end(), batch.vertices.begin() + triangle.index,
                                  batch.vertices.begin() + triangle.index + 3);
        }
        if (batch.sharedGeometryBuffer) {
            GLuint privateBuffer = 0;
            pglGenBuffers(1, &privateBuffer);
            if (privateBuffer == 0) {
                return;
            }
            batch.buffer = privateBuffer;
            batch.sharedGeometryBuffer = false;
            batch.bufferTracked = true;
            assets->trackBuffer(batch.buffer);
        }
        pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
        pglBufferData(GL_ARRAY_BUFFER,
                      static_cast<std::ptrdiff_t>(sortedVertices.size() * sizeof(Vertex)),
                      sortedVertices.data(), GL_STATIC_DRAW);
    }

    void draw(RenderViewContext context, VehicleRenderPass renderPass = VehicleRenderPass::All,
              const Matrix4* groundShadowTransform = nullptr) {
        TraceScope trace("render", "Vehicle::draw");
        if (!loaded) {
            return;
        }
        const bool reflectionPass = openbus::rendering::reflectionPassActive();
        const bool renderOpaque = renderPass != VehicleRenderPass::Transparent;
        const bool classifyTransparent =
            !reflectionPass || openbus::rendering::reflectionTransparentEnabled();
        const bool renderTransparent =
            renderPass != VehicleRenderPass::Opaque && classifyTransparent;
        if (!reflectionPass && renderPass != VehicleRenderPass::Transparent) {
            updateAnimationStates();
        }
        // Texture uploads must happen on the OpenGL thread, so decoding and GL
        // upload are deliberately split between the worker and draw paths.
        textureUploadStart = std::chrono::steady_clock::now();

        const auto& modelView = openbus::rendering::modelViewMatrix();
        const auto& projection = openbus::rendering::projectionMatrix();
        const bool reusePreparedDraw =
            renderPass == VehicleRenderPass::Transparent && preparedDrawListsValid &&
            preparedDrawContext == context && preparedDrawReflection == reflectionPass &&
            preparedDrawTransparent == classifyTransparent &&
            preparedDrawAnimationGeneration == animationGeneration &&
            preparedDrawMaterialGeneration == materialSelectionGeneration &&
            preparedDrawModelView == modelView && preparedDrawProjection == projection;
        ViewFrustum frustum;

        {
            TraceScope phase("render", "Vehicle::draw.setup");

            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);
            setBackFaceCulling(false);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(-1.0f, -1.0f);
        }

        {
            if (!reusePreparedDraw && frustumCulling) {
                TraceScope phase("render", "Vehicle::draw.frustumSetup");
                frustum = buildViewFrustum(openbus::rendering::projectionMatrix());
            }
        }

        const auto visible = [&](const DisplayPart& part, std::size_t partIndex) {
            // TraceScope phase("render", "Vehicle::draw.visible");
            if (partIndex >= variableVisibleParts.size() || !variableVisibleParts[partIndex]) {
                return false;
            }
            const std::array<double, 3> center = animatedPartCenter(part);
            const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
            const std::array<double, 4> eye = transformPoint(modelView, local);
            part.cachedViewDepth = -eye[2];
            part.viewDepthCacheGeneration = viewDepthGeneration;
            const double distance = std::sqrt(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
            if (!lodThresholds.empty() && part.lodIndex >= 0) {
                std::size_t selectedLod = lodThresholds.size() - 1;
                for (std::size_t index = 0; index < lodThresholds.size(); ++index) {
                    const double boundary = 25.0 / std::max(lodThresholds[index], 0.001);
                    if (distance <= boundary) {
                        selectedLod = index;
                        break;
                    }
                }
                if (static_cast<std::size_t>(part.lodIndex) != selectedLod) {
                    return false;
                }
            }
            if (!frustumCulling) {
                return true;
            }
            if (sphereOutsideFrustum(frustum, eye, part.radius)) {
                return false;
            }
            return true;
        };
        const auto viewDepth = [&](const DisplayPart& part) {
            // TraceScope phase("render", "Vehicle::draw.viewDepth");
            if (part.viewDepthCacheGeneration == viewDepthGeneration) {
                return part.cachedViewDepth;
            }
            const std::array<double, 3> center = animatedPartCenter(part);
            const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
            const double depth = -transformPoint(modelView, local)[2];
            part.cachedViewDepth = depth;
            part.viewDepthCacheGeneration = viewDepthGeneration;
            return depth;
        };
        std::vector<DisplayPart*>& opaqueParts = opaquePartsScratch;
        std::vector<TransparentBatch>& noDepthOpaqueBatches = noDepthOpaqueBatchesScratch;
        std::vector<TransparentBatch>& transparentBatches = transparentBatchesScratch;
        if (!reusePreparedDraw) {
            ++viewDepthGeneration;
            opaqueParts.clear();
            noDepthOpaqueBatches.clear();
            transparentBatches.clear();
            opaqueParts.reserve(displayLists.size());
            if (classifyTransparent) {
                transparentBatches.reserve(displayLists.size());
                noDepthOpaqueBatches.reserve(displayLists.size());
            }
            {
                TraceScope phase("render", "Vehicle::draw.classifyParts");
                for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
                    DisplayPart& part = displayLists[partIndex];
                    const bool viewpointMatches =
                        openbus::rendering::viewpointMatches(part.viewpoint, context);
                    if (!viewpointMatches || !visible(part, partIndex)) {
                        continue;
                    }
                    bool hasOpaqueBatch = false;
                    for (Batch& batch : part.batches) {
                        updateMaterialChange(batch);
                        hasOpaqueBatch =
                            hasOpaqueBatch || (batch.alphaMode == 0 && !batch.noZwrite);
                        if (classifyTransparent && (batch.alphaMode != 0 || batch.noZwrite)) {
                            transparentBatches.push_back(
                                {&batch, &part, viewDepth(part), part.renderType});
                        }
                    }
                    if (renderOpaque && hasOpaqueBatch) {
                        opaqueParts.push_back(&part);
                    }
                }
            }
            {
                TraceScope phase("render", "Vehicle::draw.sortBatches");
                std::sort(opaqueParts.begin(), opaqueParts.end(),
                          [&](const DisplayPart* first, const DisplayPart* second) {
                              return viewDepth(*first) < viewDepth(*second);
                          });
                std::stable_sort(transparentBatches.begin(), transparentBatches.end(),
                                 [](const TransparentBatch& first, const TransparentBatch& second) {
                                     if (first.renderType != second.renderType) {
                                         return first.renderType < second.renderType;
                                     }
                                     return first.depth > second.depth;
                                 });
                std::stable_sort(noDepthOpaqueBatches.begin(), noDepthOpaqueBatches.end(),
                                 [](const TransparentBatch& first, const TransparentBatch& second) {
                                     if (first.renderType != second.renderType) {
                                         return first.renderType < second.renderType;
                                     }
                                     return first.depth > second.depth;
                                 });
            }
            preparedDrawListsValid = renderPass == VehicleRenderPass::Opaque;
            if (preparedDrawListsValid) {
                preparedDrawContext = context;
                preparedDrawReflection = reflectionPass;
                preparedDrawTransparent = classifyTransparent;
                preparedDrawAnimationGeneration = animationGeneration;
                preparedDrawMaterialGeneration = materialSelectionGeneration;
                preparedDrawModelView = modelView;
                preparedDrawProjection = projection;
            }
        }
        pushMatrix();
        translate(0.0, 0.0, modelOffsetZ);
        interiorLightRootModelView = openbus::rendering::modelViewMatrix();
        if (renderOpaque) {
            TraceScope phase("render", "Vehicle::draw.opaquePass");
            for (DisplayPart* part : opaqueParts) {
                pushMatrix();
                applyDrawTransform(*part, groundShadowTransform);
                setBackFaceCulling(part->backFaceCulling);
                for (Batch& batch : part->batches) {
                    const double alpha = alphaScale(batch);
                    if (alpha <= 0.0) {
                        continue;
                    }
                    if (batch.alphaMode != 0 || batch.noZwrite) {
                        continue;
                    }
                    drawBatch(batch, alpha);
                    if (!reflectionPass) {
                        drawEnvironmentMap(batch, alpha);
                    }
                }
                popMatrix();
            }
        }
        if (renderTransparent) {
            {
                TraceScope phase("render", "Vehicle::draw.transparentDepthPrepass");
                glEnable(GL_DEPTH_TEST);
                glDepthFunc(GL_LESS);
                glDepthMask(GL_TRUE);
                glDisable(GL_BLEND);
                glDisable(GL_POLYGON_OFFSET_FILL);
                glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                for (const TransparentBatch& transparent : transparentBatches) {
                    Batch& batch = *transparent.batch;
                    if (batch.alphaMode != 2 || batch.transmap.name.empty() || batch.noZwrite ||
                        batch.noZcheck) {
                        continue;
                    }
                    pushMatrix();
                    applyDrawTransform(*transparent.part, groundShadowTransform);
                    setBackFaceCulling(transparent.part->backFaceCulling);
                    const double alpha = alphaScale(batch);
                    if (alpha > 0.0) {
                        drawBatch(batch, alpha, false, nullptr, 3);
                    }
                    popMatrix();
                }
                glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            }
            glDepthMask(GL_FALSE);
            glDisable(GL_BLEND);
            {
                TraceScope phase("render", "Vehicle::draw.noDepthPass");
                for (const TransparentBatch& noDepthOpaque : noDepthOpaqueBatches) {
                    Batch& batch = *noDepthOpaque.batch;
                    pushMatrix();
                    applyDrawTransform(*noDepthOpaque.part, groundShadowTransform);
                    setBackFaceCulling(noDepthOpaque.part->backFaceCulling);
                    const double alpha = alphaScale(batch);
                    if (alpha <= 0.0) {
                        popMatrix();
                        continue;
                    }
                    drawBatch(batch, alpha);
                    if (!reflectionPass) {
                        drawEnvironmentMap(batch, alpha);
                    }
                    popMatrix();
                }
            }
            glDepthMask(GL_FALSE);
            {
                TraceScope phase("render", "Vehicle::draw.transparentPass");
                for (const TransparentBatch& transparent : transparentBatches) {
                    Batch& batch = *transparent.batch;
                    pushMatrix();
                    applyDrawTransform(*transparent.part, groundShadowTransform);
                    setBackFaceCulling(transparent.part->backFaceCulling);
                    const double alpha = alphaScale(batch);
                    if (alpha <= 0.0) {
                        popMatrix();
                        continue;
                    }
                    if (batch.alphaMode == 1) {
                        glDisable(GL_BLEND);
                        glEnable(GL_POLYGON_OFFSET_FILL);
                        glPolygonOffset(-1.0f, -1.0f);
                    } else if (batch.alphaMode == 2) {
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        glDepthFunc(GL_LEQUAL);
                        glEnable(GL_POLYGON_OFFSET_FILL);
                        glPolygonOffset(-1.0f, -1.0f);
                    } else if (batch.noZwrite) {
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        glDisable(GL_POLYGON_OFFSET_FILL);
                    } else {
                        glDisable(GL_BLEND);
                        glDisable(GL_POLYGON_OFFSET_FILL);
                    }
                    if (batch.alphaMode == 2 && !batch.transmap.name.empty()) {
                        sortTransparentBatch(batch);
                    }
                    drawBatch(batch, alpha, false, nullptr, -1, transparent.part->isShadow);
                    if (!reflectionPass) {
                        drawEnvironmentMap(batch, alpha);
                    }
                    popMatrix();
                }
            }
        }
        {
            TraceScope phase("render", "Vehicle::draw.cleanup");
            glDepthMask(GL_TRUE);
            glDepthFunc(GL_LESS);
            glDisable(GL_BLEND);
            glDisable(GL_POLYGON_OFFSET_FILL);
            setBackFaceCulling(false);
            glDepthMask(GL_TRUE);
        }
        popMatrix();
    }

    bool areObjectsLoaded() const {
        return loaded && pendingParts.empty();
    }

    bool areTexturesLoaded() const {
        if (!areObjectsLoaded()) {
            return false;
        }
        std::unordered_set<const TextureCacheEntry*> trackedTextures;
        const auto texturesLoadedInParts = [&](const std::vector<DisplayPart>& parts) {
            for (const DisplayPart& displayPart : parts) {
                for (const Batch& batch : displayPart.batches) {
                    for (const MaterialTextureSource& source : batch.materialTextures) {
                        if (!source.textureCacheEntry) {
                            return false;
                        }
                        const TextureCacheEntry* sourceEntry = source.textureCacheEntry.get();
                        if (trackedTextures.insert(sourceEntry).second) {
                            std::lock_guard<std::mutex> sourceLock(sourceEntry->request->mutex);
                            if (!sourceEntry->request->complete) {
                                return false;
                            }
                        }
                    }
                    if (batch.materialTextures.empty() &&
                        (!batch.texturePath.empty() || !batch.textureName.empty())) {
                        if (!batch.textureCacheEntry) {
                            return false;
                        }
                        const TextureCacheEntry* entry = batch.textureCacheEntry.get();
                        if (trackedTextures.insert(entry).second) {
                            std::lock_guard<std::mutex> lock(entry->request->mutex);
                            if (!entry->request->complete) {
                                return false;
                            }
                        }
                    }
                    for (const AuxiliaryTexture* auxiliary :
                         {&batch.lightmap, &batch.nightmap, &batch.transmap, &batch.bumpmap}) {
                        if (auxiliary->name.empty()) {
                            continue;
                        }
                        if (!auxiliary->cacheEntry) {
                            return false;
                        }
                        const TextureCacheEntry* auxiliaryEntry = auxiliary->cacheEntry.get();
                        if (!trackedTextures.insert(auxiliaryEntry).second) {
                            continue;
                        }
                        std::lock_guard<std::mutex> auxiliaryLock(auxiliaryEntry->request->mutex);
                        if (!auxiliaryEntry->request->complete) {
                            return false;
                        }
                    }
                }
            }
            return true;
        };
        if (!texturesLoadedInParts(displayLists)) {
            return false;
        }
        return true;
    }

    bool isCaptureReady() const {
        return areObjectsLoaded() && areTexturesLoaded();
    }

    static std::mutex& sharedDisplayCacheMutex() {
        static std::mutex mutex;
        return mutex;
    }

    static std::unordered_map<std::string, std::vector<DisplayPart>>& sharedDisplayCache() {
        static std::unordered_map<std::string, std::vector<DisplayPart>> cache;
        return cache;
    }

    bool loadSharedDisplayLists() {
        std::lock_guard<std::mutex> lock(sharedDisplayCacheMutex());
        const auto cached = sharedDisplayCache().find(modelCacheKey);
        if (cached == sharedDisplayCache().end()) {
            return false;
        }
        displayLists = cached->second;
        for (DisplayPart& part : displayLists) {
            part.animationStates.assign(part.animations.size(), {});
            part.animationCacheGeneration = 0;
            part.cachedAnimationTransform = identityMatrix();
            part.viewDepthCacheGeneration = 0;
            part.cachedViewDepth = 0.0;
            for (Batch& batch : part.batches) {
                batch.sharedGeometryBuffer = true;
                batch.dynamicTexture = 0;
                batch.freeTexture = 0;
                batch.freeTextureIndex = -1;
                batch.freeTextureWidth = 0;
                batch.freeTextureHeight = 0;
                batch.freeTextureRevision = 0;
                batch.freeTextureAsset = {};
                batch.selectedMaterialItem = false;
                batch.materialSelectionGeneration = 0;
            }
        }
        loaded = !displayLists.empty();
        hasSharedGeometry = loaded;
        hasLoadedInitialView = loaded;
        loggedAllObjectsLoaded = loaded;
        if (loaded) {
            rebuildDisplayOrder();
            preloadTextures();
        }
        return loaded;
    }

    void cacheSharedDisplayLists() {
        if (displayLists.empty()) {
            return;
        }
        for (DisplayPart& part : displayLists) {
            for (Batch& batch : part.batches) {
                batch.sharedGeometryBuffer = batch.buffer != 0;
            }
        }
        std::lock_guard<std::mutex> lock(sharedDisplayCacheMutex());
        sharedDisplayCache().try_emplace(modelCacheKey, displayLists);
    }

    static void clearSharedDisplayCache() {
        std::lock_guard<std::mutex> lock(sharedDisplayCacheMutex());
        sharedDisplayCache().clear();
    }

    void spawn() {
        TraceScope trace("obj", "spawn");
        std::vector<std::pair<Part*, std::shared_future<std::shared_ptr<ParsedObj>>>> requests;
        requests.reserve(pendingParts.size());
        for (Part& part : pendingParts) {
            requests.emplace_back(&part, parsedObjFuture(part.objPath, part.bundleEntry));
        }
        for (const auto& request : requests) {
            loadObj(*request.first, request.second.get());
        }
        pendingParts.clear();
        rebuildDisplayOrder();
        preloadTextures();
        loaded = !displayLists.empty();
        hasLoadedInitialView = true;
        if (!loggedAllObjectsLoaded && loaded) {
            gameLog.Log("All objects loaded. bodyParts=" + std::to_string(displayLists.size()) +
                        " wheelParts=generic");
            loggedAllObjectsLoaded = true;
        }
    }

  private:
    static std::string trim(const std::string& value) {
        const std::size_t first = value.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            return {};
        }
        const std::size_t last = value.find_last_not_of(" \t\r\n");
        return value.substr(first, last - first + 1);
    }

    static std::string lower(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
        return value;
    }

    static double parseDouble(const std::string& value, double fallback) {
        try {
            return std::stod(trim(value));
        } catch (const std::exception&) {
            return fallback;
        }
    }

    GLuint uploadTexture(const std::filesystem::path& path, Image image) {
        TraceScope trace("texture", "uploadTexture");
        static const bool verboseTextureUploadLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
        if (verboseTextureUploadLogs) {
            gameLog.Log("Uploading texture from path: " + path.generic_string());
        }
        if (textureScale < 0.999) {
            const int scaledWidth =
                std::max(1, static_cast<int>(std::lround(image.width * textureScale)));
            const int scaledHeight =
                std::max(1, static_cast<int>(std::lround(image.height * textureScale)));
            std::size_t scaledSize = 0;
            if (!openbus::rendering::checkedTextureBufferSize(
                    static_cast<std::size_t>(scaledWidth), static_cast<std::size_t>(scaledHeight),
                    4, scaledSize)) {
                gameLog.Log("Texture scaling exceeds the decoded image size limit: " +
                            path.generic_string());
                return 0;
            }
            std::vector<std::uint8_t> scaled(scaledSize);
            for (int y = 0; y < scaledHeight; ++y) {
                const int sourceY = std::min(image.height - 1, static_cast<int>(y / textureScale));
                for (int x = 0; x < scaledWidth; ++x) {
                    const int sourceX =
                        std::min(image.width - 1, static_cast<int>(x / textureScale));
                    const std::size_t source =
                        (static_cast<std::size_t>(sourceY) * image.width + sourceX) * 4;
                    const std::size_t target = (static_cast<std::size_t>(y) * scaledWidth + x) * 4;
                    std::copy_n(image.rgba.data() + source, 4, scaled.data() + target);
                }
            }
            image.width = scaledWidth;
            image.height = scaledHeight;
            image.rgba = std::move(scaled);
        }
        GLuint texture = 0;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, image.rgba.data());
        openbus::rendering::invalidateTextureBindings();
        assets->trackTexture(texture);
        return texture;
    }

    GLuint uploadCompressedTexture(const std::filesystem::path& path, const gli::texture& image,
                                   bool& textureArray, std::size_t& textureArrayLayers) {
        TraceScope trace("texture", "uploadCompressedTexture");
        textureArray = image.layers() > 1;
        textureArrayLayers = std::max<std::size_t>(1, image.layers());
        gli::gl translator(gli::gl::PROFILE_GL33);
        const gli::gl::format format = translator.translate(image.format(), image.swizzles());
        if (format.Internal == 0 || !gli::is_compressed(image.format())) {
            return 0;
        }
        if (textureArray && pglCompressedTexImage3D == nullptr) {
            return 0;
        }
        GLuint texture = 0;
        glGenTextures(1, &texture);
        const GLenum target = textureArray ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D;
        glBindTexture(target, texture);
        glTexParameteri(target, GL_TEXTURE_MIN_FILTER,
                        image.levels() > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
        glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_REPEAT);
        for (std::size_t level = 0; level < image.levels(); ++level) {
            const auto extent = image.extent(level);
            if (textureArray) {
                pglCompressedTexImage3D(
                    GL_TEXTURE_2D_ARRAY, static_cast<GLint>(level), format.Internal,
                    static_cast<GLsizei>(extent.x), static_cast<GLsizei>(extent.y),
                    static_cast<GLsizei>(textureArrayLayers), 0,
                    static_cast<GLsizei>(image.size(level)), image.data(0, 0, level));
            } else {
                pglCompressedTexImage2D(
                    GL_TEXTURE_2D, static_cast<GLint>(level), format.Internal,
                    static_cast<GLsizei>(extent.x), static_cast<GLsizei>(extent.y), 0,
                    static_cast<GLsizei>(image.size(level)), image.data(0, 0, level));
            }
        }
        openbus::rendering::invalidateTextureBindings();
        assets->trackTexture(texture);
        static const bool verboseTextureUploadLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
        if (verboseTextureUploadLogs) {
            gameLog.Log("Uploaded compressed texture from path: " + path.generic_string());
        }
        return texture;
    }

    GLuint uploadCompressedDds(const std::filesystem::path& path, const CompressedDds& image) {
        TraceScope trace("texture", "uploadCompressedDds");
        if (pglCompressedTexImage2D == nullptr || image.levels.empty()) {
            return 0;
        }
        const GLubyte* extensionBytes = glGetString(GL_EXTENSIONS);
        const std::string extensions = extensionBytes != nullptr
                                           ? reinterpret_cast<const char*>(extensionBytes)
                                           : std::string();
        if (extensions.find("GL_EXT_texture_compression_s3tc") == std::string::npos &&
            extensions.find("GL_S3_s3tc") == std::string::npos) {
            return 0;
        }
        GLuint texture = 0;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                        image.levels.size() > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        int levelWidth = image.width;
        int levelHeight = image.height;
        for (std::size_t level = 0; level < image.levels.size(); ++level) {
            const auto& data = image.levels[level];
            pglCompressedTexImage2D(GL_TEXTURE_2D, static_cast<GLint>(level),
                                    GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, levelWidth, levelHeight, 0,
                                    static_cast<GLsizei>(data.size()), data.data());
            if (glGetError() != GL_NO_ERROR) {
                glDeleteTextures(1, &texture);
                return 0;
            }
            levelWidth = std::max(1, levelWidth / 2);
            levelHeight = std::max(1, levelHeight / 2);
        }
        openbus::rendering::invalidateTextureBindings();
        assets->trackTexture(texture);
        static const bool verboseTextureUploadLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_TEXTURE_UPLOAD"));
        if (verboseTextureUploadLogs) {
            gameLog.Log("Uploaded DXT5 texture from path: " + path.generic_string());
        }
        return texture;
    }

    static std::filesystem::path findTexture(const std::filesystem::path& root,
                                             const std::string& name) {
        TraceScope trace("texture", "findTexture");
        const std::string cleaned = trim(name);
        if (cleaned.empty()) {
            return {};
        }

        static const bool verboseTextureLookupLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_TEXTURE_LOOKUP"));

        const auto isImageExtension = [](const std::string& extension) {
            static const std::array<std::string, 6> imageExtensions = {".tga", ".bmp", ".png",
                                                                       ".dds", ".jpg", ".jpeg"};
            return std::find(imageExtensions.begin(), imageExtensions.end(), extension) !=
                   imageExtensions.end();
        };

        const auto textureFormatPriority = [](const std::filesystem::path& path) {
            const std::string extension = lower(path.extension().string());
            if (extension == ".dds") {
                return 3;
            }
            if (extension == ".tga") {
                return 2;
            }
            return 1;
        };

        const auto normalizedPathKey = [&](const std::filesystem::path& path) {
            return lower(std::filesystem::absolute(path).lexically_normal().generic_string());
        };

        static std::mutex resolutionMutex;
        static std::unordered_map<std::string, std::filesystem::path> resolutionCache;
        struct TextureDirectoryIndex {
            std::mutex mutex;
            bool built = false;
            std::unordered_map<std::string, std::filesystem::path> byFileName;
            std::unordered_map<std::string, std::filesystem::path> byStem;
        };
        static std::unordered_map<std::string, std::shared_ptr<TextureDirectoryIndex>>
            textureIndexes;

        const std::filesystem::path normalizedRequestPath =
            std::filesystem::path(cleaned).lexically_normal();
        const std::string requestPathKey = lower(normalizedRequestPath.generic_string());
        const std::string resolutionKey = normalizedPathKey(root) + "|" + requestPathKey;

        {
            std::lock_guard<std::mutex> lock(resolutionMutex);
            const auto cached = resolutionCache.find(resolutionKey);
            if (cached != resolutionCache.end()) {
                return cached->second;
            }
        }

        if (verboseTextureLookupLogs) {
            gameLog.Log("Resolving texture with name: " + name +
                        " in root: " + root.generic_string());
        }

        const auto cacheResult = [&](const std::filesystem::path& path) {
            std::lock_guard<std::mutex> lock(resolutionMutex);
            resolutionCache.emplace(resolutionKey, path);
            return path;
        };

        const auto preferCompressedSibling = [&](const std::filesystem::path& path) {
            if (lower(path.extension().string()) == ".dds") {
                return path;
            }
            std::filesystem::path compressed = path;
            compressed.replace_extension(".dds");
            return std::filesystem::exists(compressed) ? compressed : path;
        };

        const auto getIndexForTextureRoot = [&](const std::filesystem::path& textureRoot) {
            const std::string indexKey = normalizedPathKey(textureRoot);
            std::shared_ptr<TextureDirectoryIndex> index;
            {
                std::lock_guard<std::mutex> lock(resolutionMutex);
                const auto found = textureIndexes.find(indexKey);
                if (found != textureIndexes.end()) {
                    index = found->second;
                } else {
                    index = std::make_shared<TextureDirectoryIndex>();
                    textureIndexes.emplace(indexKey, index);
                }
            }
            {
                std::lock_guard<std::mutex> lock(index->mutex);
                if (!index->built) {
                    for (const auto& entry : std::filesystem::recursive_directory_iterator(
                             textureRoot,
                             std::filesystem::directory_options::skip_permission_denied)) {
                        if (!entry.is_regular_file()) {
                            continue;
                        }
                        const std::string extension = lower(entry.path().extension().string());
                        if (!isImageExtension(extension)) {
                            continue;
                        }
                        const std::string fileName = lower(entry.path().filename().string());
                        const std::string stem = lower(entry.path().stem().string());
                        index->byFileName.emplace(fileName, entry.path());
                        const auto existingStem = index->byStem.find(stem);
                        if (existingStem == index->byStem.end() ||
                            textureFormatPriority(entry.path()) >
                                textureFormatPriority(existingStem->second)) {
                            index->byStem[stem] = entry.path();
                        }
                    }
                    index->built = true;
                }
            }
            return index;
        };

        std::filesystem::path direct = root / cleaned;
        if (std::filesystem::exists(direct)) {
            return cacheResult(preferCompressedSibling(direct));
        }

        std::vector<std::filesystem::path> textureRoots;
        for (std::filesystem::path ancestor = root; !ancestor.empty();
             ancestor = ancestor.parent_path()) {
            const std::filesystem::path textureRoot = ancestor / "Texture";
            if (std::filesystem::exists(textureRoot)) {
                textureRoots.push_back(textureRoot);
                direct = textureRoot / cleaned;
                if (std::filesystem::exists(direct)) {
                    return cacheResult(preferCompressedSibling(direct));
                }
            }
            const std::filesystem::path parent = ancestor.parent_path();
            if (parent == ancestor) {
                break;
            }
        }

        const std::string basename = lower(std::filesystem::path(cleaned).filename().string());
        const std::string requestedStem = lower(std::filesystem::path(cleaned).stem().string());
        if (std::filesystem::exists(root)) {
            std::filesystem::path preferredDirect;
            for (const auto& entry : std::filesystem::directory_iterator(
                     root, std::filesystem::directory_options::skip_permission_denied)) {
                if (entry.is_regular_file() &&
                    lower(entry.path().stem().string()) == requestedStem &&
                    isImageExtension(lower(entry.path().extension().string()))) {
                    if (preferredDirect.empty() || textureFormatPriority(entry.path()) >
                                                       textureFormatPriority(preferredDirect)) {
                        preferredDirect = entry.path();
                    }
                }
            }
            if (!preferredDirect.empty()) {
                if (verboseTextureLookupLogs) {
                    gameLog.Log("Found texture directly at path: " +
                                preferredDirect.generic_string());
                }
                return cacheResult(preferredDirect);
            }
        }

        for (const std::filesystem::path& textureRoot : textureRoots) {
            const std::shared_ptr<TextureDirectoryIndex> index =
                getIndexForTextureRoot(textureRoot);
            std::lock_guard<std::mutex> lock(index->mutex);
            const auto byFileName = index->byFileName.find(basename);
            if (byFileName != index->byFileName.end()) {
                if (verboseTextureLookupLogs) {
                    gameLog.Log("Found texture at indexed filename path: " +
                                byFileName->second.generic_string());
                }
                return cacheResult(byFileName->second);
            }
            const auto byStem = index->byStem.find(requestedStem);
            if (byStem != index->byStem.end()) {
                if (verboseTextureLookupLogs) {
                    gameLog.Log("Found texture at indexed stem path: " +
                                byStem->second.generic_string());
                }
                return cacheResult(byStem->second);
            }
        }

        textureLog.Log("missing: " + cleaned);
        return cacheResult({});
    }

    std::shared_ptr<TextureCacheEntry> textureEntry(Batch& batch) {
        TraceScope phase("texture", "textureEntry");
        return assets->requestTexture(
            batch.textureRoot, batch.texturePath, batch.textureName,
            [](const std::filesystem::path& root, const std::filesystem::path& path,
               const std::string& name) {
                return Vehicle::findTexture(root, name.empty() ? path.string() : name);
            });
    }

    void ensureAuxiliaryTexture(Batch& batch, AuxiliaryTexture& auxiliary) {
        if (auxiliary.name.empty() || auxiliary.loadAttempted) {
            return;
        }
        if (!auxiliary.cacheEntry) {
            auxiliary.cacheEntry = assets->requestTexture(
                batch.textureRoot, {}, auxiliary.name,
                [](const std::filesystem::path& root, const std::filesystem::path& path,
                   const std::string& name) {
                    return Vehicle::findTexture(root, name.empty() ? path.string() : name);
                });
        }
        startTextureRequest(auxiliary.cacheEntry);
        const std::shared_ptr<TextureRequest>& request = auxiliary.cacheEntry->request;
        bool requestComplete = false;
        {
            std::lock_guard<std::mutex> lock(request->mutex);
            requestComplete = request->complete;
        }
        if (!requestComplete) {
            return;
        }
        std::lock_guard<std::mutex> lock(request->mutex);
        if (!auxiliary.cacheEntry->uploadAttempted) {
            auxiliary.cacheEntry->uploadAttempted = true;
            if (!request->resolvedPath.empty()) {
                if (request->compressedDds) {
                    auxiliary.cacheEntry->texture =
                        uploadCompressedDds(request->resolvedPath, *request->compressedDds);
                } else if (request->compressedTexture) {
                    auxiliary.cacheEntry->texture =
                        uploadCompressedTexture(request->resolvedPath, *request->compressedTexture,
                                                auxiliary.cacheEntry->textureArray,
                                                auxiliary.cacheEntry->textureArrayLayers);
                } else if (request->image) {
                    auxiliary.cacheEntry->texture =
                        uploadTexture(request->resolvedPath, *request->image);
                }
            }
        }
        auxiliary.texture = auxiliary.cacheEntry->texture;
        auxiliary.loadAttempted = true;
    }

    void updateFreeTexture(Batch& batch) {
        if (batch.textTextureIndex < 0 && batch.scriptTextureIndex < 0 &&
            !batch.freeTextureName.empty() && !batch.freeTextureVariable.empty()) {
            const std::string selectedName = variables.getString(batch.freeTextureVariable);
            const std::string textureName =
                selectedName.empty() ? batch.freeTextureName : selectedName;
            if (batch.freeTextureAsset.name != textureName) {
                batch.freeTextureAsset = {};
                batch.freeTextureAsset.name = textureName;
            }
            ensureAuxiliaryTexture(batch, batch.freeTextureAsset);
            batch.freeTexture = batch.freeTextureAsset.texture;
            return;
        }
        if ((batch.freeTextureVariable.empty() && batch.scriptTextureIndex < 0 &&
             batch.textTextureIndex < 0) ||
            !scripts) {
            return;
        }
        ScriptRuntime::ScriptTextureSnapshot snapshot;
        int index = -1;
        bool copied = false;
        if (batch.textTextureIndex >= 0) {
            index = batch.textTextureIndex;
            copied = scripts->copyTextTexture(index, snapshot);
        } else if (batch.scriptTextureIndex >= 0) {
            index = batch.scriptTextureIndex;
            copied = scripts->copyScriptTexture(index, snapshot);
        } else {
            index = static_cast<int>(std::lround(variables.get(batch.freeTextureVariable)));
            copied = scripts->copyScriptTexture(index, snapshot);
        }
        if (index < 0 || !copied || snapshot.width <= 0 || snapshot.height <= 0 ||
            snapshot.pixels.empty()) {
            return;
        }
        const bool dimensionsChanged =
            batch.freeTextureWidth != snapshot.width || batch.freeTextureHeight != snapshot.height;
        const bool textureChanged = batch.freeTexture == 0 || dimensionsChanged ||
                                    batch.freeTextureIndex != index ||
                                    batch.freeTextureRevision != snapshot.revision;
        if (!textureChanged) {
            return;
        }
        const bool textureCreated = batch.dynamicTexture == 0;
        if (textureCreated) {
            glGenTextures(1, &batch.dynamicTexture);
            assets->trackTexture(batch.dynamicTexture);
        }
        batch.freeTexture = batch.dynamicTexture;
        glBindTexture(GL_TEXTURE_2D, batch.freeTexture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        const GLint filter = snapshot.filtered ? GL_LINEAR : GL_NEAREST;
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        // Dynamic script/text textures replace the material texture but must
        // keep its address mode. OMSI O3D UVs commonly span 1..2; clamping
        // those out-of-range coordinates collapses glyph sampling to an edge.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, batch.textureWrapS);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, batch.textureWrapT);
        if (textureCreated || dimensionsChanged) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, snapshot.width, snapshot.height, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, snapshot.pixels.data());
            batch.freeTextureWidth = snapshot.width;
            batch.freeTextureHeight = snapshot.height;
        } else {
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, snapshot.width, snapshot.height, GL_RGBA,
                            GL_UNSIGNED_BYTE, snapshot.pixels.data());
        }
        openbus::rendering::invalidateTextureBindings();
        batch.freeTextureIndex = index;
        batch.freeTextureRevision = snapshot.revision;
    }

    void startTextureRequest(const std::shared_ptr<TextureCacheEntry>& entry) {
        assets->startTextureRequest(entry);
    }

    void ensureTexture(Batch& batch, bool visible = true) {
        if (batch.textureLoadAttempted && batch.textureChanges.empty()) {
            return;
        }
        TraceScope phase("texture", "ensureTexture");
        updateMaterialChange(batch);
        if (batch.textureLoadAttempted) {
            return;
        }
        if (!batch.textureCacheEntry) {
            batch.textureCacheEntry = textureEntry(batch);
            batch.textureRequest = batch.textureCacheEntry->request;
            batch.textureLoadStarted = true;
        }
        startTextureRequest(batch.textureCacheEntry);
        bool requestComplete = false;
        {
            std::lock_guard<std::mutex> lock(batch.textureCacheEntry->request->mutex);
            requestComplete = batch.textureCacheEntry->request->complete;
        }
        if (!requestComplete) {
            return;
        }
        std::lock_guard<std::mutex> lock(batch.textureCacheEntry->request->mutex);
        if (!batch.textureCacheEntry->request->complete) {
            return;
        }
        if (!visible) {
            return;
        }
        batch.texturePath = batch.textureCacheEntry->request->resolvedPath;
        if (!batch.textureCacheEntry->uploadAttempted) {
            if (loadingPolicy.textureMode == AssetLoadingMode::Deferred) {
                constexpr auto textureUploadBudget = std::chrono::milliseconds(2);
                if (std::chrono::steady_clock::now() - textureUploadStart >= textureUploadBudget) {
                    return;
                }
            }
            batch.textureCacheEntry->uploadAttempted = true;
            if (!batch.texturePath.empty()) {
                if (batch.textureCacheEntry->request->compressedDds) {
                    batch.textureCacheEntry->texture = uploadCompressedDds(
                        batch.texturePath, *batch.textureCacheEntry->request->compressedDds);
                    if (batch.textureCacheEntry->texture == 0) {
                        Image fallbackImage;
                        if (openbus::rendering::TextureLoader::readImage(batch.texturePath,
                                                                         fallbackImage)) {
                            batch.textureCacheEntry->texture =
                                uploadTexture(batch.texturePath, std::move(fallbackImage));
                        }
                    }
                } else if (batch.textureCacheEntry->request->compressedTexture) {
                    batch.textureCacheEntry->texture = uploadCompressedTexture(
                        batch.texturePath, *batch.textureCacheEntry->request->compressedTexture,
                        batch.textureCacheEntry->textureArray,
                        batch.textureCacheEntry->textureArrayLayers);
                } else {
                    batch.textureCacheEntry->texture =
                        uploadTexture(batch.texturePath, *batch.textureCacheEntry->request->image);
                }
            }
        }
        batch.texture = batch.textureCacheEntry->texture;
        batch.textureArray = batch.textureCacheEntry->textureArray;
        batch.textureArrayLayers = batch.textureCacheEntry->textureArrayLayers;
        batch.textureLoadAttempted = true;
        batch.textured = batch.texture != 0;
        if ((batch.texturePath.empty() || batch.texture == 0) &&
            !batch.textureCacheEntry->decodeFailureLogged) {
            batch.textureCacheEntry->decodeFailureLogged = true;
            textureLog.Log("decode-failed: " + batch.textureName +
                           " resolved=" + batch.texturePath.generic_string());
        }
        // if (!batch.environmentLoadAttempted && !batch.environmentTextureName.empty() &&
        //     batch.environmentStrength > 0.0) {
        //     batch.environmentLoadAttempted = true;
        //     Batch environmentBatch;
        //     environmentBatch.textureRoot = batch.textureRoot;
        //     environmentBatch.textureName = batch.environmentTextureName;
        //     ensureTexture(environmentBatch, visible);
        //     batch.environmentTexture = environmentBatch.texture;
        // }
    }

    void ensureMaterialTexture(MaterialTextureSource& source, bool visible = true) {
        if (source.textureLoadAttempted) {
            return;
        }
        TraceScope phase("texture", "ensureMaterialTexture");
        if (!source.textureCacheEntry) {
            source.textureCacheEntry = assets->requestTexture(
                source.textureRoot, source.texturePath, source.textureName,
                [](const std::filesystem::path& root, const std::filesystem::path& path,
                   const std::string& name) {
                    return Vehicle::findTexture(root, name.empty() ? path.string() : name);
                });
            source.textureRequest = source.textureCacheEntry->request;
            source.textureLoadStarted = true;
        }
        startTextureRequest(source.textureCacheEntry);
        bool requestComplete = false;
        {
            std::lock_guard<std::mutex> lock(source.textureCacheEntry->request->mutex);
            requestComplete = source.textureCacheEntry->request->complete;
        }
        if (!requestComplete || !visible) {
            return;
        }
        std::lock_guard<std::mutex> lock(source.textureCacheEntry->request->mutex);
        if (!source.textureCacheEntry->request->complete) {
            return;
        }
        source.texturePath = source.textureCacheEntry->request->resolvedPath;
        if (!source.textureCacheEntry->uploadAttempted) {
            if (loadingPolicy.textureMode == AssetLoadingMode::Deferred) {
                constexpr auto textureUploadBudget = std::chrono::milliseconds(2);
                if (std::chrono::steady_clock::now() - textureUploadStart >= textureUploadBudget) {
                    return;
                }
            }
            source.textureCacheEntry->uploadAttempted = true;
            if (!source.texturePath.empty()) {
                if (source.textureCacheEntry->request->compressedDds) {
                    source.textureCacheEntry->texture = uploadCompressedDds(
                        source.texturePath, *source.textureCacheEntry->request->compressedDds);
                    if (source.textureCacheEntry->texture == 0) {
                        Image fallbackImage;
                        if (openbus::rendering::TextureLoader::readImage(source.texturePath,
                                                                         fallbackImage)) {
                            source.textureCacheEntry->texture =
                                uploadTexture(source.texturePath, std::move(fallbackImage));
                        }
                    }
                } else if (source.textureCacheEntry->request->compressedTexture) {
                    source.textureCacheEntry->texture = uploadCompressedTexture(
                        source.texturePath, *source.textureCacheEntry->request->compressedTexture,
                        source.textureCacheEntry->textureArray,
                        source.textureCacheEntry->textureArrayLayers);
                } else if (source.textureCacheEntry->request->image) {
                    source.textureCacheEntry->texture = uploadTexture(
                        source.texturePath, *source.textureCacheEntry->request->image);
                }
            }
        }
        source.texture = source.textureCacheEntry->texture;
        source.textureArray = source.textureCacheEntry->textureArray;
        source.textureArrayLayers = source.textureCacheEntry->textureArrayLayers;
        source.textureLoadAttempted = true;
    }

    void ensureEnvironmentTexture(Batch& batch, bool visible = true) {
        if (batch.environmentLoadAttempted || batch.environmentTextureName.empty() ||
            batch.environmentStrength <= 0.0) {
            return;
        }
        TraceScope phase("texture", "ensureEnvironmentTexture");
        if (!batch.environmentTextureCacheEntry) {
            Batch request;
            request.textureRoot = batch.textureRoot;
            request.textureName = batch.environmentTextureName;
            batch.environmentTextureCacheEntry = textureEntry(request);
        }
        const std::shared_ptr<TextureCacheEntry>& entry = batch.environmentTextureCacheEntry;
        startTextureRequest(entry);
        bool requestComplete = false;
        {
            std::lock_guard<std::mutex> lock(entry->request->mutex);
            requestComplete = entry->request->complete;
        }
        if (!requestComplete || !visible) {
            return;
        }
        std::lock_guard<std::mutex> lock(entry->request->mutex);
        if (!entry->uploadAttempted) {
            entry->uploadAttempted = true;
            if (!entry->request->resolvedPath.empty()) {
                if (entry->request->compressedDds) {
                    entry->texture = uploadCompressedDds(entry->request->resolvedPath,
                                                         *entry->request->compressedDds);
                    if (entry->texture == 0) {
                        Image fallbackImage;
                        if (openbus::rendering::TextureLoader::readImage(
                                entry->request->resolvedPath, fallbackImage)) {
                            entry->texture = uploadTexture(entry->request->resolvedPath,
                                                           std::move(fallbackImage));
                        }
                    }
                } else if (entry->request->compressedTexture) {
                    entry->texture = uploadCompressedTexture(
                        entry->request->resolvedPath, *entry->request->compressedTexture,
                        entry->textureArray, entry->textureArrayLayers);
                } else if (entry->request->image) {
                    entry->texture =
                        uploadTexture(entry->request->resolvedPath, *entry->request->image);
                }
            }
        }
        batch.environmentTexture = entry->texture;
        batch.environmentLoadAttempted = true;
    }

    void drawEnvironmentMap(Batch& batch, double alpha) {
        TraceScope trace("render", "Vehicle::drawEnvironmentMap");
        ensureEnvironmentTexture(batch);
        if (batch.environmentTexture == 0 || !batch.hasNormals || alpha <= 0.0) {
            return;
        }
        const double reflectionStrength =
            std::clamp(alpha * batch.environmentStrength * ENVIRONMENT_MAP_OPACITY, 0.0, 1.0);
        if (reflectionStrength <= 0.0) {
            return;
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        drawEnvironmentBatch(batch.buffer, batch.vertexCount, batch.environmentTexture,
                             reflectionStrength);
        glDisable(GL_BLEND);
    }

    void preloadTextures() {
        const auto preload = [&](std::vector<DisplayPart>& parts) {
            for (DisplayPart& part : parts) {
                for (Batch& batch : part.batches) {
                    if (!batch.materialTextures.empty()) {
                        for (MaterialTextureSource& source : batch.materialTextures) {
                            ensureMaterialTexture(source, loadingPolicy.textureMode ==
                                                              AssetLoadingMode::Eager);
                        }
                    } else if (!batch.texturePath.empty() || !batch.textureName.empty()) {
                        ensureTexture(batch, loadingPolicy.textureMode == AssetLoadingMode::Eager);
                    }
                    ensureEnvironmentTexture(batch,
                                             loadingPolicy.textureMode == AssetLoadingMode::Eager);
                    ensureAuxiliaryTexture(batch, batch.lightmap);
                    ensureAuxiliaryTexture(batch, batch.nightmap);
                    ensureAuxiliaryTexture(batch, batch.transmap);
                    ensureAuxiliaryTexture(batch, batch.bumpmap);
                }
            }
        };
        preload(displayLists);
    }

    std::shared_future<std::shared_ptr<ParsedObj>>
    parsedObjFuture(const std::filesystem::path& path, const std::string& bundleEntry = {}) {
        return assets->requestObj(path, bundleEntry);
    }

    void loadObj(const Part& part, const std::shared_ptr<ParsedObj>& parsed) {
        TraceScope trace("obj", "loadObj");
        static const bool verboseObjLoadLogs =
            parseEnabledFlag(std::getenv("OPENBUS_VERBOSE_OBJ_LOAD"));
        static const bool materialBatchingEnabled =
            parseEnabledFlag(std::getenv("OPENBUS_MATERIAL_BATCHING"));
        if (verboseObjLoadLogs) {
            gameLog.Log("Loading OBJ model from path: " + part.objPath.generic_string());
        }
        if (!parsed) {
            return;
        }
        const auto& positions = parsed->positions;
        const auto& normals = parsed->normals;
        const auto& texCoords = parsed->texCoords;
        const auto& triangles = parsed->triangles;
        const auto& materials = parsed->materials;
        const auto& boundsCenter = parsed->boundsCenter;
        const auto& boundsSize = parsed->boundsSize;
        const double boundsRadius = parsed->boundsRadius;

        std::vector<ModelAnimation> animations = part.animations;
        for (ModelAnimation& animation : animations) {
            animation.variable = lower(animation.variable);
            if (animation.originFromMesh) {
                if (parsed->hasTransform) {
                    // The converter preserves OBJ vertices and stores this transform as
                    // metadata. Only its source-space translation supplies the mesh pivot;
                    // applying its orientation again would double-transform the mesh.
                    if (!animation.hasOrigin) {
                        animation.origin = {parsed->transform[13], -parsed->transform[12],
                                            parsed->transform[14]};
                        animation.hasOrigin = true;
                    }
                    animation.meshRotation = {
                        parsed->transform[0], parsed->transform[1], parsed->transform[2],
                        parsed->transform[4], parsed->transform[5], parsed->transform[6],
                        parsed->transform[8], parsed->transform[9], parsed->transform[10]};
                    animation.hasMeshRotation = true;
                    const std::array<double, 3> meshOrigin = {
                        parsed->transform[13], -parsed->transform[12], parsed->transform[14]};
                    animation.meshTransform = makeMeshTransform(animation.meshRotation, meshOrigin);
                    animation.hasMeshTransform = true;
                } else if (!animation.hasOrigin) {
                    animation.origin = boundsCenter;
                    animation.hasOrigin = true;
                }
            }
        }
        if (verboseObjLoadLogs && !animations.empty()) {
            for (const ModelAnimation& animation : animations) {
                gameLog.Log("Model animation: " + part.objPath.filename().string() +
                            " type=" + animation.type + " variable=" + animation.variable +
                            " scale=" + std::to_string(animation.scale) + " origin=(" +
                            std::to_string(animation.origin[0]) + ',' +
                            std::to_string(animation.origin[1]) + ',' +
                            std::to_string(animation.origin[2]) +
                            ") hasOrigin=" + (animation.hasOrigin ? "true" : "false") +
                            " value=" + std::to_string(variables.get(animation.variable)));
            }
        }

        std::vector<DisplayPart>* destination = &displayLists;
        auto makeBatch = [&](const std::vector<const ObjTriangle*>& source,
                             const std::filesystem::path& texturePath,
                             const std::string& textureName, const std::array<double, 3>& color,
                             const std::string& environmentTextureName, double environmentStrength,
                             int alphaMode, bool noZwrite, bool noZcheck,
                             const std::string& alphaScaleVariable,
                             const std::vector<MaterialState::TextureChange>& textureChanges,
                             const MaterialState& materialState) {
            TraceScope batchTrace("obj", "loadObj.makeBatch");
            std::vector<Vertex> vertices;
            if (source.size() > std::numeric_limits<std::size_t>::max() / 3) {
                throw std::runtime_error("triangle vertex count overflow");
            }
            vertices.reserve(source.size() * 3);
            const auto convertPosition = [](const ObjPosition& position) {
                return std::array<double, 3>{position.y, -position.x, position.z};
            };
            const auto normalizeVector = [](std::array<double, 3> value) {
                const double length =
                    std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
                if (length <= 1.0e-8) {
                    return std::array<double, 3>{0.0, 0.0, 1.0};
                }
                for (double& component : value) {
                    component /= length;
                }
                return value;
            };
            const auto validIndex = [](int index, std::size_t count) {
                return index > 0 && static_cast<std::size_t>(index) <= count;
            };
            {
                TraceScope vertexTrace("obj", "loadObj.buildVertices");
                for (const ObjTriangle* triangle : source) {
                    std::array<double, 3> fallbackNormal = {0.0, 0.0, 1.0};
                    if (validIndex(triangle->indices[0].position, positions.size()) &&
                        validIndex(triangle->indices[1].position, positions.size()) &&
                        validIndex(triangle->indices[2].position, positions.size())) {
                        const std::array<double, 3> first = convertPosition(
                            positions[static_cast<std::size_t>(triangle->indices[0].position - 1)]);
                        const std::array<double, 3> second = convertPosition(
                            positions[static_cast<std::size_t>(triangle->indices[1].position - 1)]);
                        const std::array<double, 3> third = convertPosition(
                            positions[static_cast<std::size_t>(triangle->indices[2].position - 1)]);
                        const std::array<double, 3> edgeA = {
                            second[0] - first[0], second[1] - first[1], second[2] - first[2]};
                        const std::array<double, 3> edgeB = {
                            third[0] - first[0], third[1] - first[1], third[2] - first[2]};
                        fallbackNormal =
                            normalizeVector({edgeA[1] * edgeB[2] - edgeA[2] * edgeB[1],
                                             edgeA[2] * edgeB[0] - edgeA[0] * edgeB[2],
                                             edgeA[0] * edgeB[1] - edgeA[1] * edgeB[0]});
                    }
                    for (const ObjIndex& index : triangle->indices) {
                        if (!validIndex(index.position, positions.size())) {
                            continue;
                        }
                        const ObjPosition& position =
                            positions[static_cast<std::size_t>(index.position - 1)];
                        std::array<double, 3> normal = fallbackNormal;
                        if (validIndex(index.normal, normals.size())) {
                            const ObjNormal& sourceNormal =
                                normals[static_cast<std::size_t>(index.normal - 1)];
                            normal =
                                normalizeVector({sourceNormal.y, -sourceNormal.x, sourceNormal.z});
                        }
                        const ObjTexCoord* texCoord = nullptr;
                        if (validIndex(index.texCoord, texCoords.size())) {
                            texCoord = &texCoords[static_cast<std::size_t>(index.texCoord - 1)];
                        }
                        vertices.push_back(
                            {static_cast<float>(position.y), static_cast<float>(-position.x),
                             static_cast<float>(position.z),
                             texCoord ? static_cast<float>(texCoord->u) : 0.0f,
                             texCoord ? static_cast<float>(1.0 - texCoord->v) : 0.0f, 0.0f,
                             static_cast<float>(normal[0]), static_cast<float>(normal[1]),
                             static_cast<float>(normal[2])});
                    }
                }
            }
            if (vertices.empty())
                return;
            Batch batch;
            batch.vertices = std::move(vertices);
            batch.baseTexturePath = texturePath;
            batch.baseTextureName = resolveCtcTextureName(textureName);
            batch.texturePath = texturePath;
            batch.textureRoot = part.objPath.parent_path();
            batch.textureName = batch.baseTextureName;
            batch.environmentTextureName = environmentTextureName;
            batch.environmentStrength = environmentStrength;
            batch.lightmap.name = materialState.lightmapTextureName;
            batch.nightmap.name = materialState.nightmapTextureName;
            batch.transmap.name = materialState.transmapTextureName;
            batch.bumpmap.name = materialState.bumpmapTextureName;
            batch.bumpmapStrength = materialState.bumpmapStrength;
            batch.interiorLightIndexes = part.interiorLightIndexes;
            batch.lightmapStrengthVariable = lower(materialState.lightmapStrengthVariable);
            batch.freeTextureName = materialState.freeTextureName;
            batch.freeTextureVariable = lower(materialState.freeTextureVariable);
            batch.scriptTextureIndex = materialState.scriptTextureIndex;
            batch.textTextureIndex = materialState.textTextureIndex;
            batch.texcoordTransXVariable = lower(materialState.texcoordTransXVariable);
            batch.texcoordTransYVariable = lower(materialState.texcoordTransYVariable);
            batch.color = color;
            batch.hasNormals = true;
            batch.textureWrapS = textureAddressModeToGl(materialState.textureAddressS);
            batch.textureWrapT = textureAddressModeToGl(materialState.textureAddressT);
            batch.alphaMode = alphaMode;
            batch.noZwrite = noZwrite;
            batch.noZcheck = noZcheck;
            batch.alphaScaleVariable = lower(alphaScaleVariable);
            batch.textureChanges = textureChanges;
            batch.baseMaterial = materialState;
            for (MaterialState::TextureChange& change : batch.textureChanges) {
                change.activationVariable = lower(change.activationVariable);
            }
            batch.vertexCount = batch.vertices.size();

            const auto canMergeOpaqueBatches = [](const Batch& first, const Batch& second) {
                return first.alphaMode == 0 && second.alphaMode == 0 && !first.noZwrite &&
                       !second.noZwrite && !first.noZcheck && !second.noZcheck &&
                       first.texturePath == second.texturePath &&
                       first.textureRoot == second.textureRoot &&
                       first.textureName == second.textureName &&
                       first.environmentTextureName == second.environmentTextureName &&
                       first.environmentStrength == second.environmentStrength &&
                       first.color == second.color && first.textured == second.textured &&
                       first.hasNormals == second.hasNormals &&
                       first.textureWrapS == second.textureWrapS &&
                       first.textureWrapT == second.textureWrapT &&
                       first.lightmap.name == second.lightmap.name &&
                       first.nightmap.name == second.nightmap.name &&
                       first.transmap.name == second.transmap.name &&
                       first.bumpmap.name == second.bumpmap.name &&
                       first.bumpmapStrength == second.bumpmapStrength &&
                       first.interiorLightIndexes == second.interiorLightIndexes &&
                       first.lightmapStrengthVariable == second.lightmapStrengthVariable &&
                       first.freeTextureName == second.freeTextureName &&
                       first.freeTextureVariable == second.freeTextureVariable &&
                       first.scriptTextureIndex == second.scriptTextureIndex &&
                       first.textTextureIndex == second.textTextureIndex &&
                       first.texcoordTransXVariable == second.texcoordTransXVariable &&
                       first.texcoordTransYVariable == second.texcoordTransYVariable &&
                       first.alphaScaleVariable == second.alphaScaleVariable &&
                       first.baseTextureLayer == second.baseTextureLayer &&
                       first.textureLayer == second.textureLayer && first.textureChanges.empty() &&
                       second.textureChanges.empty();
            };
            std::vector<Batch>& batches = destination->back().batches;
            const auto compatibleBatch =
                std::find_if(batches.begin(), batches.end(), [&](const Batch& existing) {
                    return canMergeOpaqueBatches(existing, batch);
                });
            if (compatibleBatch != batches.end()) {
                Batch& merged = *compatibleBatch;
                merged.vertices.insert(merged.vertices.end(), batch.vertices.begin(),
                                       batch.vertices.end());
                merged.vertexCount = merged.vertices.size();
                TraceScope uploadTrace("obj", "loadObj.mergeBatch");
                pglBindBuffer(GL_ARRAY_BUFFER, merged.buffer);
                pglBufferData(GL_ARRAY_BUFFER,
                              static_cast<std::ptrdiff_t>(merged.vertices.size() * sizeof(Vertex)),
                              merged.vertices.data(), GL_STATIC_DRAW);
                return;
            }
            {
                TraceScope uploadTrace("obj", "loadObj.uploadVbo");
                pglGenBuffers(1, &batch.buffer);
                pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
                pglBufferData(GL_ARRAY_BUFFER,
                              static_cast<std::ptrdiff_t>(batch.vertices.size() * sizeof(Vertex)),
                              batch.vertices.data(), GL_STATIC_DRAW);
            }
            batches.push_back(std::move(batch));
        };

        const auto canUseMaterialBatch = [](const Batch& batch) {
            return batch.alphaMode == 0 && !batch.noZwrite && !batch.noZcheck &&
                   !batch.textureArray && batch.textureChanges.empty() &&
                   batch.textureWrapS == GL_REPEAT && batch.textureWrapT == GL_REPEAT &&
                   batch.lightmap.name.empty() && batch.nightmap.name.empty() &&
                   batch.transmap.name.empty() && batch.freeTextureVariable.empty() &&
                   batch.bumpmap.name.empty() &&
                   std::all_of(batch.interiorLightIndexes.begin(), batch.interiorLightIndexes.end(),
                               [](int index) { return index < 0; }) &&
                   batch.freeTextureName.empty() && batch.scriptTextureIndex < 0 &&
                   batch.textTextureIndex < 0 && batch.texcoordTransXVariable.empty() &&
                   batch.texcoordTransYVariable.empty() && batch.alphaScaleVariable.empty();
        };
        const auto makeMaterialTexture = [](const Batch& source) {
            MaterialTextureSource result;
            result.texturePath = source.texturePath;
            result.textureRoot = source.textureRoot;
            result.textureName = source.textureName;
            return result;
        };
        const auto sameMaterialBatch = [&](const Batch& first, const Batch& second) {
            return canUseMaterialBatch(first) && canUseMaterialBatch(second) &&
                   first.environmentTextureName == second.environmentTextureName &&
                   first.environmentStrength == second.environmentStrength &&
                   first.textured == second.textured && first.hasNormals == second.hasNormals &&
                   first.baseTextureLayer == second.baseTextureLayer &&
                   first.textureLayer == second.textureLayer && second.materialColors.empty() &&
                   second.materialTextures.empty();
        };
        const auto consolidateMaterialBatches = [&](std::vector<Batch>& batches) {
            if (!materialBatchingEnabled) {
                return;
            }
            std::vector<Batch> consolidated;
            consolidated.reserve(batches.size());
            for (Batch& source : batches) {
                auto target = std::find_if(consolidated.begin(), consolidated.end(),
                                           [&](const Batch& candidate) {
                                               return candidate.materialTextures.size() < 8 &&
                                                      sameMaterialBatch(candidate, source);
                                           });
                if (target == consolidated.end()) {
                    if (canUseMaterialBatch(source)) {
                        for (Vertex& vertex : source.vertices) {
                            vertex.layer = 0.0f;
                        }
                        source.materialColors.push_back({static_cast<float>(source.color[0]),
                                                         static_cast<float>(source.color[1]),
                                                         static_cast<float>(source.color[2]),
                                                         1.0f});
                        source.materialTextures.push_back(makeMaterialTexture(source));
                    }
                    consolidated.push_back(std::move(source));
                    continue;
                }

                const std::size_t materialIndex = target->materialColors.size();
                for (Vertex& vertex : source.vertices) {
                    vertex.layer = static_cast<float>(materialIndex);
                }
                target->vertices.insert(target->vertices.end(), source.vertices.begin(),
                                        source.vertices.end());
                target->vertexCount = target->vertices.size();
                target->materialColors.push_back({static_cast<float>(source.color[0]),
                                                  static_cast<float>(source.color[1]),
                                                  static_cast<float>(source.color[2]), 1.0f});
                target->materialTextures.push_back(makeMaterialTexture(source));
                if (source.buffer != 0) {
                    pglDeleteBuffers(1, &source.buffer);
                    source.buffer = 0;
                }
                pglBindBuffer(GL_ARRAY_BUFFER, target->buffer);
                pglBufferData(GL_ARRAY_BUFFER,
                              static_cast<std::ptrdiff_t>(target->vertices.size() * sizeof(Vertex)),
                              target->vertices.data(), GL_STATIC_DRAW);
            }
            batches = std::move(consolidated);
            for (Batch& batch : batches) {
                if (batch.materialColors.size() < 2) {
                    batch.materialColors.clear();
                    batch.materialTextures.clear();
                }
            }
            const std::size_t materialBatchCount = static_cast<std::size_t>(
                std::count_if(batches.begin(), batches.end(),
                              [](const Batch& batch) { return batch.materialColors.size() >= 2; }));
            if (verboseObjLoadLogs && materialBatchCount != 0) {
                gameLog.Log("Material batches: " + std::to_string(materialBatchCount) + " from " +
                            std::to_string(batches.size()) + " total batches");
            }
        };

        std::unordered_map<std::string, std::vector<const ObjTriangle*>> groups;
        std::unordered_map<std::string, std::filesystem::path> groupTextures;
        std::unordered_map<std::string, std::string> groupTextureNames;
        std::unordered_map<std::string, std::string> groupEnvironmentNames;
        std::unordered_map<std::string, double> groupEnvironmentStrengths;
        std::unordered_map<std::string, std::array<double, 3>> groupColors;
        std::unordered_map<std::string, MaterialState> groupStates;
        std::vector<std::string> groupOrder;
        std::unordered_map<std::string, const MaterialState*> materialStatesByFilename;
        std::unordered_map<std::string, const MaterialState*> materialStatesByStem;
        std::unordered_map<std::string, std::vector<const MaterialState*>>
            materialStatesByStemOccurrence;
        std::unordered_map<int, const MaterialState*> implicitMaterialStatesByIndex;
        materialStatesByFilename.reserve(part.materialStates.size());
        materialStatesByStem.reserve(part.materialStates.size());
        for (const auto& entry : part.materialStates) {
            materialStatesByFilename.emplace(
                lower(std::filesystem::path(entry.first).filename().string()), &entry.second);
            materialStatesByStem.emplace(lower(std::filesystem::path(entry.first).stem().string()),
                                         &entry.second);
        }
        for (const MaterialState& state : part.materialStatesInOrder) {
            const std::filesystem::path statePath = state.textureName.empty()
                                                        ? state.texturePath
                                                        : std::filesystem::path(state.textureName);
            if (state.textureName.empty() && state.texturePath.empty() &&
                state.materialIndex >= 0) {
                implicitMaterialStatesByIndex.emplace(state.materialIndex, &state);
            }
            std::vector<const MaterialState*>& states =
                materialStatesByStemOccurrence[lower(statePath.stem().string())];
            if (state.materialIndex > 10000) {
                gameLog.Log("Ignoring out-of-range material index " +
                            std::to_string(state.materialIndex));
                continue;
            }
            if (state.materialIndex >= 0) {
                if (states.size() <= static_cast<std::size_t>(state.materialIndex)) {
                    states.resize(static_cast<std::size_t>(state.materialIndex) + 1, nullptr);
                }
                states[static_cast<std::size_t>(state.materialIndex)] = &state;
            } else {
                states.push_back(&state);
            }
        }
        std::vector<std::pair<int, std::string>> indexedMaterials;
        indexedMaterials.reserve(materials.size());
        for (const auto& entry : materials) {
            if (entry.second.materialIndex >= 0) {
                const std::filesystem::path materialPath =
                    entry.second.textureName.empty()
                        ? entry.second.texturePath
                        : std::filesystem::path(entry.second.textureName);
                indexedMaterials.push_back(
                    {entry.second.materialIndex, lower(materialPath.stem().string())});
            }
        }
        std::sort(indexedMaterials.begin(), indexedMaterials.end(),
                  [](const auto& first, const auto& second) { return first.first < second.first; });
        std::unordered_map<std::string, int> textureOccurrences;
        std::unordered_map<int, int> materialOccurrenceByIndex;
        for (const auto& indexedMaterial : indexedMaterials) {
            const int occurrence = textureOccurrences[indexedMaterial.second]++;
            materialOccurrenceByIndex[indexedMaterial.first] = occurrence;
        }
        const std::filesystem::path fallbackTexturePath = part.texturePath;
        const std::string fallbackTextureName = part.textureName;
        bool hasTransparentMaterial = false;
        {
            TraceScope materialTrace("obj", "loadObj.groupMaterials");
            for (const auto& triangle : triangles) {
                const auto material = materials.find(triangle.material);
                std::filesystem::path texturePath =
                    material != materials.end() && !material->second.texturePath.empty()
                        ? material->second.texturePath
                        : fallbackTexturePath;
                std::string textureName =
                    material != materials.end() && !material->second.textureName.empty()
                        ? material->second.textureName
                        : fallbackTextureName;
                std::array<double, 3> color =
                    material != materials.end() ? material->second.color : part.color;
                const std::string key =
                    triangle.material.empty() ? "__fallback__" : triangle.material;
                if (groups.find(key) == groups.end()) {
                    groupOrder.push_back(key);
                }
                MaterialState state;
                bool matchedMaterialState = false;
                if (material != materials.end() && !part.materialStatesInOrder.empty() &&
                    material->second.materialIndex >= 0) {
                    const auto occurrence =
                        materialOccurrenceByIndex.find(material->second.materialIndex);
                    const std::filesystem::path materialPath =
                        material->second.textureName.empty()
                            ? material->second.texturePath
                            : std::filesystem::path(material->second.textureName);
                    const auto states =
                        materialStatesByStemOccurrence.find(lower(materialPath.stem().string()));
                    if (occurrence != materialOccurrenceByIndex.end() &&
                        states != materialStatesByStemOccurrence.end() && occurrence->second >= 0 &&
                        static_cast<std::size_t>(occurrence->second) < states->second.size() &&
                        states->second[static_cast<std::size_t>(occurrence->second)] != nullptr) {
                        state = *states->second[static_cast<std::size_t>(occurrence->second)];
                    } else {
                        const auto implicitState =
                            implicitMaterialStatesByIndex.find(material->second.materialIndex);
                        if (implicitState != implicitMaterialStatesByIndex.end()) {
                            state = *implicitState->second;
                        }
                    }
                    matchedMaterialState = true;
                }
                if (!textureName.empty() || !texturePath.empty()) {
                    const std::filesystem::path statePath =
                        textureName.empty() ? texturePath : std::filesystem::path(textureName);
                    const std::string stateKey = lower(statePath.filename().string());
                    if (!matchedMaterialState) {
                        const auto exactState = materialStatesByFilename.find(stateKey);
                        if (exactState != materialStatesByFilename.end()) {
                            state = *exactState->second;
                        } else {
                            const std::string stateStem = lower(statePath.stem().string());
                            const auto matchingState = materialStatesByStem.find(stateStem);
                            if (matchingState != materialStatesByStem.end()) {
                                state = *matchingState->second;
                            } else {
                                const auto matchingMaterial =
                                    materialStatesByStem.find(lower(triangle.material));
                                if (matchingMaterial != materialStatesByStem.end()) {
                                    state = *matchingMaterial->second;
                                }
                            }
                        }
                    }
                }
                if (part.isShadow && state.alphaMode == 0) {
                    state.alphaMode = 2;
                }
                if (!state.textureName.empty()) {
                    textureName = state.textureName;
                }
                groups[key].push_back(&triangle);
                groupTextures[key] = texturePath;
                groupTextureNames[key] = textureName;
                groupEnvironmentNames[key] = state.environmentTextureName;
                groupEnvironmentStrengths[key] = state.environmentStrength;
                groupColors[key] = color;
                groupStates[key] = state;
                hasTransparentMaterial = hasTransparentMaterial || state.alphaMode != 0 ||
                                         state.noZwrite || state.noZcheck;
            }
        }
        DisplayPart displayPart;
        displayPart.viewpoint = part.viewpoint;
        displayPart.renderType = part.renderType;
        displayPart.isShadow = part.isShadow;
        displayPart.transparent = hasTransparentMaterial;
        displayPart.lodIndex = part.lodIndex;
        displayPart.meshName =
            part.bundleEntry.empty() ? part.objPath.filename().string() : part.bundleEntry;
        displayPart.visibleVariable = lower(part.visibleVariable);
        displayPart.visibleValue = part.visibleValue;
        displayPart.meshIdentifier = part.meshIdentifier;
        displayPart.animationParent = part.animationParent;
        displayPart.mouseEvent = part.mouseEvent;
        displayPart.backFaceCulling = parsed->backFaceCulling;
        displayPart.center = boundsCenter;
        displayPart.size = boundsSize;
        displayPart.radius = boundsRadius;
        displayPart.animations = std::move(animations);
        displayPart.interiorLightIndexes = part.interiorLightIndexes;
        const std::string wheelVariable = lower(part.wheelAnimation.rotationVariable);
        if (wheelVariable.rfind("wheel_rotation_", 0) == 0) {
            const std::size_t axleStart = std::string("wheel_rotation_").size();
            const std::size_t sideSeparator = wheelVariable.find('_', axleStart);
            if (sideSeparator != std::string::npos) {
                try {
                    const int axleIndex =
                        std::stoi(wheelVariable.substr(axleStart, sideSeparator - axleStart));
                    const std::string side = wheelVariable.substr(sideSeparator + 1);
                    if (axleIndex >= 0 && (side == "l" || side == "r")) {
                        displayPart.odeWheelIndex = axleIndex * 2 + (side == "r" ? 1 : 0);
                    }
                } catch (const std::exception&) {
                    displayPart.odeWheelIndex = -1;
                }
            }
        }
        destination->push_back(std::move(displayPart));
        for (const std::string& key : groupOrder) {
            const MaterialState& state = groupStates[key];
            makeBatch(groups[key], groupTextures[key], groupTextureNames[key], groupColors[key],
                      groupEnvironmentNames[key], groupEnvironmentStrengths[key], state.alphaMode,
                      state.noZwrite, state.noZcheck, state.alphaScaleVariable,
                      state.textureChanges, state);
        }
        consolidateMaterialBatches(destination->back().batches);
        for (Batch& batch : destination->back().batches) {
            if (batch.buffer != 0 && !batch.bufferTracked) {
                assets->trackBuffer(batch.buffer);
                batch.bufferTracked = true;
            }
        }
        cacheReflectionTextureIndices(destination->back());
        if (verboseObjLoadLogs) {
            for (const Batch& batch : destination->back().batches) {
                gameLog.Log("OBJ batch: " + part.objPath.filename().string() +
                            " vertices=" + std::to_string(batch.vertexCount) +
                            " alpha=" + std::to_string(batch.alphaMode) +
                            " noZwrite=" + (batch.noZwrite ? "true" : "false") +
                            " texture=" + batch.textureName + " transmap=" + batch.transmap.name);
            }
        }
    }

    int lodForDistance(double distance) const {
        if (lodThresholds.empty()) {
            return -1;
        }
        for (std::size_t index = 0; index < lodThresholds.size(); ++index) {
            const double boundary = 25.0 / std::max(lodThresholds[index], 0.001);
            if (distance <= boundary) {
                return static_cast<int>(index);
            }
        }
        return static_cast<int>(lodThresholds.size() - 1);
    }

    void rebuildDisplayOrder() {
        std::stable_sort(displayLists.begin(), displayLists.end(),
                         [](const DisplayPart& first, const DisplayPart& second) {
                             return first.transparent < second.transparent;
                         });
        ++clickableStateRevision;
        for (DisplayPart& part : displayLists) {
            part.pickOccluder =
                std::any_of(part.batches.begin(), part.batches.end(), [](const Batch& batch) {
                    return batch.alphaMode == 0 && !batch.noZwrite && !batch.noZcheck;
                });
        }
        opaqueDisplayCount = 0;
        while (opaqueDisplayCount < displayLists.size() &&
               !displayLists[opaqueDisplayCount].transparent) {
            ++opaqueDisplayCount;
        }
    }

    // Load visible OBJ geometry immediately; texture decoding remains asynchronous.
    void ensureLoaded(RenderViewContext context, double viewDistance) {
        TraceScope trace("obj", "ensureLoaded");
        const int selectedLod = lodForDistance(viewDistance);
        activeLod = selectedLod;
        bool loadedPart = false;
        const bool immediateLoad =
            !hasLoadedInitialView || loadingPolicy.modelMode == AssetLoadingMode::Eager;
        const auto loadStart = std::chrono::steady_clock::now();
        constexpr auto loadBudget = std::chrono::milliseconds(4);
        static const std::size_t maxObjWorkers = [] {
            if (const char* value = std::getenv("OPENBUS_OBJ_WORKERS")) {
                try {
                    return static_cast<std::size_t>(std::clamp(std::stoi(value), 1, 64));
                } catch (const std::exception&) {
                }
            }
            const unsigned int concurrency = std::thread::hardware_concurrency();
            if (concurrency == 0) {
                return static_cast<std::size_t>(8);
            }
            return static_cast<std::size_t>(std::clamp(static_cast<int>(concurrency), 4, 32));
        }();
        static const std::size_t maxObjLoadsPerFrame = [] {
            if (const char* value = std::getenv("OPENBUS_OBJ_LOADS_PER_FRAME")) {
                try {
                    return static_cast<std::size_t>(std::clamp(std::stoi(value), 1, 128));
                } catch (const std::exception&) {
                }
            }
            return static_cast<std::size_t>(8);
        }();
        const std::size_t maxLoadsThisFrame =
            immediateLoad ? pendingParts.size() : maxObjLoadsPerFrame;
        const auto budgetReached = [&] {
            return !immediateLoad && std::chrono::steady_clock::now() - loadStart >= loadBudget;
        };

        std::size_t activeObjWorkers = 0;
        for (const Part& part : pendingParts) {
            if (!part.objRequest) {
                continue;
            }
            if (part.objRequest->future.wait_for(std::chrono::milliseconds(0)) !=
                std::future_status::ready) {
                ++activeObjWorkers;
            }
        }

        for (Part& part : pendingParts) {
            const bool viewpointMatches =
                openbus::rendering::viewpointMatches(part.viewpoint, context);
            const bool lodMatches = selectedLod < 0 || part.lodIndex == selectedLod;
            if (part.objRequest || (viewpointMatches && lodMatches)) {
                continue;
            }
            if (activeObjWorkers >= maxObjWorkers) {
                continue;
            }
            part.objRequest = std::make_shared<ObjRequest>();
            part.objRequest->future = parsedObjFuture(part.objPath, part.bundleEntry);
            ++activeObjWorkers;
        }

        std::size_t loadedThisFrame = 0;
        auto part = pendingParts.begin();
        while (part != pendingParts.end() && loadedThisFrame < maxLoadsThisFrame) {
            if (budgetReached() && loadedThisFrame > 0) {
                break;
            }
            const bool viewpointMatches =
                openbus::rendering::viewpointMatches(part->viewpoint, context);
            const bool shouldLoad =
                viewpointMatches && (selectedLod < 0 || part->lodIndex == selectedLod);
            if (!shouldLoad) {
                ++part;
                continue;
            }
            std::shared_ptr<ParsedObj> parsed;
            if (!part->objRequest) {
                parsed = parsedObjFuture(part->objPath, part->bundleEntry).get();
            } else {
                parsed = part->objRequest->future.get();
            }
            loadObj(*part, parsed);
            part = pendingParts.erase(part);
            loadedPart = true;
            ++loadedThisFrame;
        }

        auto backgroundPart = pendingParts.begin();
        while (backgroundPart != pendingParts.end() && loadedThisFrame < maxLoadsThisFrame) {
            if (budgetReached() && loadedThisFrame > 0) {
                break;
            }
            const bool viewpointMatches =
                openbus::rendering::viewpointMatches(backgroundPart->viewpoint, context);
            const bool lodMatches = selectedLod < 0 || backgroundPart->lodIndex == selectedLod;
            if ((viewpointMatches && lodMatches) || !backgroundPart->objRequest ||
                backgroundPart->objRequest->future.wait_for(std::chrono::milliseconds(0)) !=
                    std::future_status::ready) {
                ++backgroundPart;
                continue;
            }
            const std::shared_ptr<ParsedObj> parsed = backgroundPart->objRequest->future.get();
            loadObj(*backgroundPart, parsed);
            backgroundPart = pendingParts.erase(backgroundPart);
            loadedPart = true;
            ++loadedThisFrame;
        }
        hasLoadedInitialView = true;
        if (loadedPart) {
            rebuildDisplayOrder();
        }
        loaded = !displayLists.empty() || !pendingParts.empty();
        if (!loggedAllObjectsLoaded && pendingParts.empty() && loaded) {
            gameLog.Log("All objects loaded. bodyParts=" + std::to_string(displayLists.size()) +
                        " wheelParts=generic");
            loggedAllObjectsLoaded = true;
        }
    }

    void load(const SharedVehicleDefinition& definition) {
        const openbus::rendering::BusModelLoadResult& result = definition.modelConfiguration;
        if (scripts) {
            scripts->configureScriptTextures(result.scriptTextures);
            scripts->configureTextTextures(result.textTextures);
        }
        interiorLights = result.interiorLights;
        interiorLightControllers.clear();
        interiorLightControllers.reserve(interiorLights.size());
        for (const ModelInteriorLight& light : interiorLights) {
            char* parsedEnd = nullptr;
            const double numericValue = std::strtod(light.controller.c_str(), &parsedEnd);
            interiorLightControllers.push_back(
                {parsedEnd != light.controller.c_str() && *parsedEnd == '\0', numericValue});
        }
        lodThresholds = result.lodThresholds;
        ctcTextureReplacements.clear();
        for (const ModelCtcTexture& texture : result.ctcTextures) {
            ctcTextureReplacements[lower(texture.slot)] = texture.textureName;
        }
        if (loadSharedDisplayLists()) {
            return;
        }

        std::vector<Part> parts;
        parts.reserve(result.parts.size());
        for (const auto& source : result.parts) {
            Part part;
            static_cast<openbus::rendering::BusModelPart&>(part) = source;
            parts.push_back(std::move(part));
        }
        pendingParts = std::move(parts);
        loaded = !pendingParts.empty();
    }
};

void RenderLoop::scrollCallback(GLFWwindow* window, double, double yOffset) {
    auto* renderer = static_cast<RenderLoop*>(glfwGetWindowUserPointer(window));
    if (renderer == nullptr) {
        return;
    }
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        renderer->fieldOfViewOffset_ =
            std::clamp(renderer->fieldOfViewOffset_ - yOffset * 2.0, -40.0, 60.0);
        return;
    }
    renderer->cameraDistance_ = std::clamp(renderer->cameraDistance_ - yOffset * 2.0, 6.0, 80.0);
}

RenderLoop::RenderLoop(int width, int height, const char* title)
    : window_(nullptr), assetRequestManager_(std::make_unique<AssetRequestManager>()),
      reflectionRenderer_(std::make_unique<openbus::rendering::ReflectionRenderer>()) {
    TraceScope trace("startup", "RenderLoop::RenderLoop");
    if (const char* scriptRate = std::getenv("OPENBUS_SCRIPT_HZ")) {
        try {
            scriptRateHz_ = std::clamp(std::stod(scriptRate), 0.0, 1000.0);
        } catch (const std::exception&) {
            scriptRateHz_ = 0.0;
        }
    }
    if (const char* steeringSmoothing = std::getenv("OPENBUS_STEERING_SMOOTHING")) {
        try {
            steeringSmoothingRate_ = std::clamp(std::stod(steeringSmoothing), 0.0, 1000.0);
        } catch (const std::exception&) {
            steeringSmoothingRate_ = 0.0;
        }
    }
    if (scriptRateHz_ > 0.0) {
        gameLog.Log("Script rate limited to " + std::to_string(scriptRateHz_) + " Hz");
    } else {
        gameLog.Log("Scripts follow the render rate");
    }
    gameLog.Log(steeringSmoothingRate_ > 0.0
                    ? "Steering smoothing limited to " + std::to_string(steeringSmoothingRate_) +
                          " input units/s"
                    : "Steering smoothing disabled");
    if (!glfwInit()) {
        gameLog.Log("Failed to initialize GLFW");
        throw std::runtime_error("Failed to initialize GLFW");
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    const bool benchmarkMode = parseEnabledFlag(openbus::getEnvironment("OPENBUS_BENCHMARK"));
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) {
        gameLog.Log("Failed to create OpenGL window");
        glfwTerminate();
        throw std::runtime_error("Failed to create OpenGL window");
    }
    if (benchmarkMode) {
        gameLog.Log(
            std::string("Benchmark window is ") +
            (glfwGetWindowAttrib(window_, GLFW_VISIBLE) == GLFW_TRUE ? "visible" : "hidden"));
    }
    glfwMakeContextCurrent(window_);
    glfwGetFramebufferSize(window_, &framebufferWidth_, &framebufferHeight_);
    clickableCursor_ = glfwCreateStandardCursor(GLFW_HAND_CURSOR);
    mouseSteeringCursor_ = glfwCreateStandardCursor(GLFW_CROSSHAIR_CURSOR);
    glfwSetWindowUserPointer(window_, this);
    glfwSetScrollCallback(window_, &RenderLoop::scrollCallback);
    const char* vsyncSetting = std::getenv("OPENBUS_VSYNC");
    const bool vsyncEnabled = vsyncSetting == nullptr || (std::string(vsyncSetting) != "0" &&
                                                          std::string(vsyncSetting) != "off" &&
                                                          std::string(vsyncSetting) != "false");
    glfwSwapInterval(vsyncEnabled ? 1 : 0);
    gameLog.Log(std::string("VSync ") + (vsyncEnabled ? "enabled" : "disabled"));
    if (!loadOpenGLFunctions()) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw std::runtime_error("OpenGL VBO functions are unavailable");
    }
    if (!openbus::rendering::initializeCoreRenderer()) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw std::runtime_error("Failed to initialize core OpenGL renderer");
    }
    pglActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &coordinateHudTexture_);
    if (coordinateHudTexture_ != 0) {
        glBindTexture(GL_TEXTURE_2D, coordinateHudTexture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        const std::vector<std::uint8_t> initialPixels =
            makeCoordinateHudPixels("X 0.0 Y 0.0 Z 0.0");
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, COORDINATE_HUD_TEXTURE_WIDTH,
                     COORDINATE_HUD_TEXTURE_HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                     initialPixels.data());
        openbus::rendering::invalidateTextureBindings();
        if (glGetError() != GL_NO_ERROR) {
            glDeleteTextures(1, &coordinateHudTexture_);
            coordinateHudTexture_ = 0;
            gameLog.Log("Coordinate HUD texture initialization failed");
        }
    } else {
        gameLog.Log("Coordinate HUD texture could not be allocated");
    }
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.45f, 0.65f, 0.88f, 1.0f);
    gameLog.Log("RenderLoop initialized");
}

RenderLoop::~RenderLoop() {
    gameLog.Log("RenderLoop shutting down");
    reflectionRenderer_->destroy();
    mapRenderer_.reset();
    playerVehicle_ = nullptr;
    assetRequestManager_->join();
    vehicles_.clear();
    Vehicle::clearSharedDisplayCache();
    if (coordinateHudTexture_ != 0) {
        glDeleteTextures(1, &coordinateHudTexture_);
        coordinateHudTexture_ = 0;
        openbus::rendering::invalidateTextureBindings();
    }
    openbus::rendering::shutdownCoreRenderer();
    assetRequestManager_.reset();

    if (clickableCursor_ != nullptr) {
        glfwDestroyCursor(clickableCursor_);
        clickableCursor_ = nullptr;
    }
    if (mouseSteeringCursor_ != nullptr) {
        glfwDestroyCursor(mouseSteeringCursor_);
        mouseSteeringCursor_ = nullptr;
    }

    if (window_) {
        glfwDestroyWindow(window_);
    }

    glfwTerminate();
}

Vehicle* RenderLoop::AddVehicle(const std::filesystem::path& busConfigPath,
                                const std::filesystem::path& modelConfigPath,
                                const VehiclePlacement& placement,
                                ModelLoadingPolicy loadingPolicy) {
    const std::filesystem::path resolvedBusConfigPath = busConfigurationPathFor(busConfigPath);
    const std::filesystem::path resolvedModelConfigPath =
        modelConfigurationPathForBus(resolvedBusConfigPath, modelConfigPath);
    const std::shared_ptr<const SharedVehicleDefinition> definition =
        sharedVehicleDefinition(resolvedBusConfigPath, resolvedModelConfigPath);
    const VehicleConfig& vehicleConfiguration = definition->vehicleConfiguration;
    if (vehicleCameras_.empty()) {
        for (const VehicleCamera& camera : vehicleConfiguration.cameras) {
            if (camera.kind == VehicleCameraKind::Driver ||
                camera.kind == VehicleCameraKind::Passenger ||
                camera.kind == VehicleCameraKind::Reflexion ||
                camera.kind == VehicleCameraKind::Reflexion2) {
                vehicleCameras_.push_back(camera);
            }
        }
        if (!vehicleCameras_.empty()) {
            int driverIndex = 0;
            int selectedCamera = -1;
            for (std::size_t index = 0; index < vehicleCameras_.size(); ++index) {
                if (vehicleCameras_[index].kind != VehicleCameraKind::Driver) {
                    continue;
                }
                if (driverIndex == vehicleConfiguration.standardDriverCamera) {
                    selectedCamera = static_cast<int>(index);
                    break;
                }
                ++driverIndex;
            }
            cameraView_ = selectedCamera >= 0 ? selectedCamera + 1 : 1;
        }
        reflectionRenderer_->initialize(vehicleCameras_);
    }
    const double modelOffsetZ = -vehicleConfiguration.centerOfGravityHeight;
    auto model = std::make_unique<Vehicle>(definition, placement, modelOffsetZ, loadingPolicy,
                                           *assetRequestManager_, simulationState_, soundEngine_,
                                           soundViewpoint_);
    Vehicle* result = model.get();
    vehicles_.push_back(std::move(model));
    return result;
}

void RenderLoop::SetPlayerVehicle(Vehicle* model) {
    playerVehicle_ = model;
    if (playerVehicle_ != nullptr) {
        playerVehicle_->initializePlayerSystems();
    }
}

void RenderLoop::SetMap(const openbus::map::MapDefinition& map, std::size_t spawnEntryPointIndex,
                        std::size_t groundTextureIndex, const std::filesystem::path& omsiRoot) {
    TraceScope trace("map", "RenderLoop::SetMap");
    if (spawnEntryPointIndex >= map.entryPoints.size()) {
        throw std::out_of_range("Selected map spawn entrypoint is outside [entrypoints]");
    }
    mapRenderer_ = std::make_unique<openbus::rendering::MapRenderer>(
        map, static_cast<std::size_t>(map.entryPoints[spawnEntryPointIndex].tileIndex),
        groundTextureIndex, omsiRoot, simulationState_);
}

void RenderLoop::logDiagnosticVariables() const {
    gameLog.Log("Variable dump requested (F9)");
    if (playerVehicle_ != nullptr) {
        logVariableSet("vehicle", playerVehicle_->variables);
    } else {
        gameLog.Log("No player vehicle is available for the variable dump");
    }
    logVariableSet("system", simulationState_.sharedVariables());
}

void RenderLoop::updatePlayerVariables(const BusSimulation& simulation, double throttle,
                                       double steering, double brake) {
    TraceScope trace("frame", "RenderLoop::updatePlayerVariables");
    soundViewpoint_ =
        isExteriorView() ? RenderViewContext::PlayerExterior : RenderViewContext::PlayerInterior;
    smoothedSteering_ = openbus::input::smoothSteeringInput(
        smoothedSteering_, steering, std::clamp(frameTimeStep_, 0.0, 0.25), steeringSmoothingRate_);
    if (playerVehicle_ != nullptr) {
        playerVehicle_->updateSimulationVariables(simulation, throttle, smoothedSteering_, brake);
    }
    updateScripts();
}

void RenderLoop::updatePostPhysicsVariables(const BusSimulation& simulation) {
    if (playerVehicle_ != nullptr) {
        playerVehicle_->updateSimulationVariables(
            simulation, playerVehicle_->variables.get("throttle"),
            playerVehicle_->variables.get("steering"), playerVehicle_->variables.get("brake"));
        playerVehicle_->soundBank.updateAmbient(soundEngine_, playerVehicle_->variables,
                                                soundViewpoint_);
    }
}

void RenderLoop::updateScripts() {
    TraceScope trace("script", "RenderLoop::updateScripts");
    const auto updateAllScripts = [&] {
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            vehicle->updateScripts(vehicle.get() != playerVehicle_);
        }
        if (mapRenderer_ != nullptr) {
            mapRenderer_->updateScripts();
        }
    };
    const double renderTimeStep = std::clamp(frameTimeStep_, 0.0, 0.25);
    if (scriptRateHz_ <= 0.0) {
        simulationState_.sharedVariables().set("timegap", renderTimeStep);
        updateAllScripts();
        return;
    }

    const double scriptTimeStep = 1.0 / scriptRateHz_;
    scriptAccumulator_ += renderTimeStep;
    int ticks = 0;
    while (scriptAccumulator_ >= scriptTimeStep && ticks < MAX_SCRIPT_CATCH_UP_TICKS) {
        scriptAccumulator_ -= scriptTimeStep;
        simulationState_.sharedVariables().set("timegap", scriptTimeStep);
        updateAllScripts();
        ++ticks;
    }
    if (ticks == MAX_SCRIPT_CATCH_UP_TICKS && scriptAccumulator_ >= scriptTimeStep) {
        scriptAccumulator_ = std::fmod(scriptAccumulator_, scriptTimeStep);
    }
}

bool RenderLoop::isExteriorView() const {
    if (cameraView_ == 0) {
        return true;
    }
    const VehicleCamera* camera = currentVehicleCamera();
    return camera != nullptr && camera->kind == VehicleCameraKind::Passenger &&
           camera->orbitDistance > 1.0;
}

const VehicleCamera* RenderLoop::currentVehicleCamera() const {
    if (cameraView_ <= 0 || static_cast<std::size_t>(cameraView_) > vehicleCameras_.size()) {
        return nullptr;
    }
    return &vehicleCameras_[static_cast<std::size_t>(cameraView_ - 1)];
}

void RenderLoop::selectVehicleCamera(int direction) {
    if (vehicleCameras_.empty() || direction == 0) {
        return;
    }
    const int cameraCount = static_cast<int>(vehicleCameras_.size());
    int next = cameraView_ == 0 ? (direction > 0 ? 0 : cameraCount - 1) : cameraView_ - 1;
    if (cameraView_ != 0) {
        next = (next + direction + cameraCount) % cameraCount;
    }
    cameraView_ = next + 1;
    viewLookYaw_ = 0.0;
    viewLookPitch_ = 0.0;
    gameLog.Log("Changed vehicle camera to " + std::to_string(next));
}

double RenderLoop::currentFieldOfView() const {
    const VehicleCamera* camera = currentVehicleCamera();
    const double baseFieldOfView = camera != nullptr && camera->fieldOfView > 0.0
                                       ? camera->fieldOfView
                                       : DEFAULT_FIELD_OF_VIEW;
    return std::clamp(baseFieldOfView + fieldOfViewOffset_, MIN_FIELD_OF_VIEW, MAX_FIELD_OF_VIEW);
}

void RenderLoop::renderReflectionViews(const BusSimulation& simulation) {
    if (renderingReflection_ || reflectionRenderer_->empty()) {
        return;
    }
    const int previousCameraView = cameraView_;
    const double previousFovOffset = fieldOfViewOffset_;
    const double previousLookYaw = viewLookYaw_;
    const double previousLookPitch = viewLookPitch_;
    renderingReflection_ = true;
    reflectionRenderer_->render(
        simulation, vehicleCameras_, viewport_,
        [this](std::size_t reflectionIndex) {
            openbus::rendering::ReflectionRequirement result;
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (vehicle->needsReflectionTexture(reflectionIndex)) {
                    result.needed = true;
                    result.size =
                        std::max(result.size, vehicle->requiredReflectionSize(reflectionIndex));
                }
            }
            return result;
        },
        [this](const BusSimulation& reflectionSimulation, std::size_t cameraIndex, int width,
               int height) {
            framebufferWidth_ = std::max(width, 1);
            framebufferHeight_ = std::max(height, 1);
            cameraView_ = static_cast<int>(cameraIndex + 1);
            fieldOfViewOffset_ = 0.0;
            viewLookYaw_ = 0.0;
            viewLookPitch_ = 0.0;
            setPerspective(static_cast<double>(width), static_cast<double>(height),
                           currentFieldOfView());
            draw(reflectionSimulation);
        },
        [this, previousCameraView, previousFovOffset, previousLookYaw,
         previousLookPitch](int width, int height) {
            framebufferWidth_ = std::max(width, 1);
            framebufferHeight_ = std::max(height, 1);
            cameraView_ = previousCameraView;
            fieldOfViewOffset_ = previousFovOffset;
            viewLookYaw_ = previousLookYaw;
            viewLookPitch_ = previousLookPitch;
            setPerspective(static_cast<double>(width), static_cast<double>(height),
                           currentFieldOfView());
        });
    renderingReflection_ = false;
}

bool RenderLoop::shouldClose() const {
    return glfwWindowShouldClose(window_) != 0;
}

void RenderLoop::requestClose() {
    if (window_ != nullptr) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}

std::array<int, 2> RenderLoop::windowSize() const {
    int width = 1;
    int height = 1;
    glfwGetWindowSize(window_, &width, &height);
    return {width, height};
}

std::array<int, 2> RenderLoop::framebufferSize() const {
    return {framebufferWidth_, framebufferHeight_};
}

RenderBenchmarkInput RenderLoop::setBenchmarkFrame(RenderBenchmarkPhase phase, int frameInPhase,
                                                   int phaseFrameCount) {
    const int frameCount = std::max(phaseFrameCount, 1);
    const int frame = std::clamp(frameInPhase, 0, frameCount - 1);
    const double progress = frameCount <= 1 ? 0.0 : static_cast<double>(frame) / (frameCount - 1);
    benchmarkClickDiscoveryPending_ = false;
    benchmarkClickQueued_ = false;

    RenderBenchmarkInput input;
    switch (phase) {
    case RenderBenchmarkPhase::Baseline:
        cameraView_ = 0;
        cameraYaw_ = -2.3;
        cameraPitch_ = 0.45;
        cameraDistance_ = 24.0;
        viewLookYaw_ = 0.0;
        viewLookPitch_ = 0.0;
        fieldOfViewOffset_ = 0.0;
        break;
    case RenderBenchmarkPhase::CameraCycle: {
        const auto isUserCamera = [](const VehicleCamera& camera) {
            return camera.kind == VehicleCameraKind::Driver ||
                   camera.kind == VehicleCameraKind::Passenger;
        };
        const std::size_t cameraCount = static_cast<std::size_t>(
            std::count_if(vehicleCameras_.begin(), vehicleCameras_.end(), isUserCamera));
        if (cameraCount == 0) {
            cameraView_ = 0;
            break;
        }
        const std::size_t selectedCamera =
            std::min(cameraCount - 1, static_cast<std::size_t>(frame) * cameraCount /
                                          static_cast<std::size_t>(frameCount));
        std::size_t userCameraIndex = 0;
        for (std::size_t index = 0; index < vehicleCameras_.size(); ++index) {
            if (!isUserCamera(vehicleCameras_[index])) {
                continue;
            }
            if (userCameraIndex == selectedCamera) {
                cameraView_ = static_cast<int>(index + 1);
                break;
            }
            ++userCameraIndex;
        }
        break;
    }
    case RenderBenchmarkPhase::ThirdPersonZoom: {
        cameraView_ = 0;
        const double zoomCycle = 0.5 - 0.5 * std::cos(progress * 2.0 * 3.141592653589793);
        cameraDistance_ = 7.0 + 60.0 * zoomCycle;
        cameraYaw_ = -2.3 + 0.25 * std::sin(progress * 2.0 * 3.141592653589793);
        cameraPitch_ = 0.40 + 0.10 * std::sin(progress * 4.0 * 3.141592653589793);
        break;
    }
    case RenderBenchmarkPhase::DrivingControls:
        cameraView_ = 0;
        cameraDistance_ = 24.0;
        input.throttle = 0.15;
        input.steering = std::sin(progress * 4.0 * 3.141592653589793);
        input.brake = progress >= 0.75 ? 0.25 : 0.0;
        break;
    case RenderBenchmarkPhase::DashboardInteraction: {
        const auto driverCamera = std::find_if(
            vehicleCameras_.begin(), vehicleCameras_.end(),
            [](const VehicleCamera& camera) { return camera.kind == VehicleCameraKind::Driver; });
        cameraView_ =
            driverCamera == vehicleCameras_.end()
                ? 0
                : static_cast<int>(std::distance(vehicleCameras_.begin(), driverCamera)) + 1;
        viewLookYaw_ = 0.35 * std::sin(progress * 2.0 * 3.141592653589793);
        viewLookPitch_ = 0.12 * std::sin(progress * 4.0 * 3.141592653589793);
        fieldOfViewOffset_ = 8.0 * std::sin(progress * 2.0 * 3.141592653589793);
        benchmarkClickDiscoveryPending_ = !benchmarkClickDiscoveryComplete_;
        const int clickInterval = std::max(frameCount / 5, 1);
        benchmarkClickQueued_ = benchmarkClickTargetFound_ && frame % clickInterval == 0;
        break;
    }
    }
    return input;
}

bool RenderLoop::benchmarkClickTargetFound() const {
    return benchmarkClickTargetFound_;
}

void RenderLoop::beginFrame(double fixedTimeStep) {
    TraceScope trace("frame", "RenderLoop::beginFrame");
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.pollEvents");
        glfwPollEvents();
    }
    const double currentTime = glfwGetTime();
    const double wallTimegap =
        hasPreviousVariableTime_ ? std::max(0.0, currentTime - previousVariableTime_) : 0.0;
    const double timegap =
        fixedTimeStep >= 0.0 && std::isfinite(fixedTimeStep) ? fixedTimeStep : wallTimegap;
    previousVariableTime_ = currentTime;
    hasPreviousVariableTime_ = true;
    frameTimeStep_ = timegap;
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.keyboardInput");
        const auto& bindings = vehicleKeyBindings();
        if (previousVehicleKeyStates_.size() != bindings.size()) {
            previousVehicleKeyStates_.assign(bindings.size(), false);
        }
        for (std::size_t index = 0; index < bindings.size(); ++index) {
            const VehicleKeyBinding& binding = bindings[index];
            const bool pressed = vehicleBindingPressed(window_, binding);
            if (pressed != previousVehicleKeyStates_[index]) {
                keyEvents_.push_back({binding.action, pressed, glfwGetTime()});
                const bool mouseControlToggle =
                    std::string_view(binding.action) == "mouse_control_toggle";
                const bool dumpVariables =
                    std::string_view(binding.action) == "debug_dump_variables";
                if (mouseControlToggle && pressed) {
                    mouseControlEnabled_ = !mouseControlEnabled_;
                    if (mouseControlEnabled_) {
                        pendingMouseClick_ = false;
                        clickableHoverCacheValid_ = false;
                        clickableHoverCacheHit_ = false;
                    }
                    gameLog.Log(std::string("Mouse bus control ") +
                                (mouseControlEnabled_ ? "enabled" : "disabled"));
                } else if (dumpVariables && pressed) {
                    logDiagnosticVariables();
                } else if (!mouseControlToggle && !dumpVariables && playerVehicle_ != nullptr &&
                           playerVehicle_->scripts) {
                    playerVehicle_->scripts->invokeKeyBinding(binding.action, pressed);
                }
                if (!mouseControlToggle) {
                    gameLog.Log(std::string("Key binding ") + binding.action +
                                (pressed ? " pressed" : " released"));
                }
                previousVehicleKeyStates_[index] = pressed;
            }
            if (pressed && std::string_view(binding.action) == "horn" &&
                playerVehicle_ != nullptr && playerVehicle_->scripts) {
                playerVehicle_->scripts->invokeKeyBinding(binding.action, true);
            }
        }
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.viewInput");
        const auto& bindings = vehicleKeyBindings();
        for (std::size_t index = 0; index < previousViewKeyStates_.size(); ++index) {
            const int key = GLFW_KEY_0 + static_cast<int>(index);
            const bool keyDown = glfwGetKey(window_, key) == GLFW_PRESS;
            bool vehicleBindingPressedForKey = false;
            for (std::size_t bindingIndex = 0;
                 bindingIndex < bindings.size() && bindingIndex < previousVehicleKeyStates_.size();
                 ++bindingIndex) {
                if (bindings[bindingIndex].key == key && previousVehicleKeyStates_[bindingIndex]) {
                    vehicleBindingPressedForKey = true;
                    break;
                }
            }
            if (keyDown && !previousViewKeyStates_[index] && !vehicleBindingPressedForKey) {
                if (index == 0 || index <= vehicleCameras_.size()) {
                    cameraView_ = static_cast<int>(index);
                    viewLookYaw_ = 0.0;
                    viewLookPitch_ = 0.0;
                    gameLog.Log("Changed camera view to " + std::to_string(cameraView_));
                }
            }
            previousViewKeyStates_[index] = keyDown;
        }
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.cameraNavigation");
        const std::array<int, 2> cameraNavigationKeys = {GLFW_KEY_LEFT, GLFW_KEY_RIGHT};
        for (std::size_t index = 0; index < cameraNavigationKeys.size(); ++index) {
            const bool pressed = glfwGetKey(window_, cameraNavigationKeys[index]) == GLFW_PRESS;
            if (pressed && !previousCameraNavigationStates_[index]) {
                selectVehicleCamera(index == 0 ? -1 : 1);
            }
            previousCameraNavigationStates_[index] = pressed;
        }
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.captureInput");
        const bool captureKeyPressed = glfwGetKey(window_, GLFW_KEY_F12) == GLFW_PRESS;
        if (captureKeyPressed && !previousCaptureKeyState_) {
            captureRequested_ = true;
        }
        previousCaptureKeyState_ = captureKeyPressed;
    }
    {
        const bool clickableDebugKeyPressed = glfwGetKey(window_, GLFW_KEY_I) == GLFW_PRESS;
        if (clickableDebugKeyPressed && !previousClickableDebugKeyState_) {
            clickableDebugOverlay_ = !clickableDebugOverlay_;
            gameLog.Log(std::string("Clickable component overlay ") +
                        (clickableDebugOverlay_ ? "enabled" : "disabled"));
            if (clickableDebugOverlay_ && playerVehicle_ != nullptr) {
                const RenderViewContext context = isExteriorView()
                                                      ? RenderViewContext::PlayerExterior
                                                      : RenderViewContext::PlayerInterior;
                playerVehicle_->logClickableDebugInfo(context);
            }
        }
        previousClickableDebugKeyState_ = clickableDebugKeyPressed;
    }
    {
        const bool collisionDebugKeyPressed = glfwGetKey(window_, GLFW_KEY_C) == GLFW_PRESS;
        if (collisionDebugKeyPressed && !previousCollisionDebugKeyState_) {
            collisionDebugOverlay_ = !collisionDebugOverlay_;
            gameLog.Log(std::string("Collision wireframe overlay ") +
                        (collisionDebugOverlay_ ? "enabled" : "disabled"));
        }
        previousCollisionDebugKeyState_ = collisionDebugKeyPressed;
    }
    {
        const bool raisePlayerKeyPressed = glfwGetKey(window_, GLFW_KEY_U) == GLFW_PRESS;
        if (raisePlayerKeyPressed && !previousRaisePlayerKeyState_) {
            raisePlayerRequestPending_ = true;
        }
        previousRaisePlayerKeyState_ = raisePlayerKeyPressed;
    }
    int width = 1;
    int height = 1;
    double cursorX = 0.0;
    double cursorY = 0.0;
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.windowAndVariables");
        glfwGetFramebufferSize(window_, &width, &height);
        framebufferWidth_ = std::max(width, 1);
        framebufferHeight_ = std::max(height, 1);
        glfwGetCursorPos(window_, &cursorX, &cursorY);
        if (mouseControlEnabled_) {
            int windowWidth = 1;
            int windowHeight = 1;
            glfwGetWindowSize(window_, &windowWidth, &windowHeight);
            const openbus::input::MouseControlInputs inputs = openbus::input::mouseControlInputs(
                cursorX, cursorY, static_cast<double>(windowWidth),
                static_cast<double>(windowHeight));
            mouseThrottle_ = inputs.throttle;
            mouseSteering_ = inputs.steering;
            mouseBrake_ = inputs.brake;
        }
        simulationState_.sharedVariables().updateFrame(timegap, currentTime, cursorX, cursorY);
        for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
            const bool isAiVehicle = vehicle.get() != playerVehicle_;
            vehicle->updateFrameVariables(isAiVehicle, timegap);
        }
    }
    {
        TraceScope phase("frame", "RenderLoop::beginFrame.mouseInput");
        const bool rightMouse = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        if (rightMouse && !draggingFov_) {
            previousFovCursorY_ = cursorY;
        } else if (rightMouse) {
            const double cursorDeltaY = cursorY - previousFovCursorY_;
            if (cameraView_ == 0) {
                cameraDistance_ = std::clamp(cameraDistance_ + cursorDeltaY * 0.1, 0.1, 80.0);
            } else {
                fieldOfViewOffset_ =
                    std::clamp(fieldOfViewOffset_ + cursorDeltaY * 0.15, -40.0, 60.0);
            }
        }
        draggingFov_ = rightMouse;
        previousFovCursorY_ = cursorY;
        const bool middleMouse =
            glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        if (middleMouse && !draggingCamera_) {
            previousCursorX_ = cursorX;
            previousCursorY_ = cursorY;
        } else if (middleMouse) {
            const double cursorDeltaX = cursorX - previousCursorX_;
            const double cursorDeltaY = cursorY - previousCursorY_;
            if (isExteriorView()) {
                cameraYaw_ -= cursorDeltaX * 0.005;
                cameraPitch_ -= cursorDeltaY * 0.005;
                cameraPitch_ = std::clamp(cameraPitch_, -1.35, 1.35);
            } else {
                viewLookYaw_ -= cursorDeltaX * 0.005;
                viewLookPitch_ -= cursorDeltaY * 0.005;
                viewLookPitch_ = std::clamp(viewLookPitch_, -1.35, 1.35);
            }
        }
        draggingCamera_ = middleMouse;
        previousCursorX_ = cursorX;
        previousCursorY_ = cursorY;
        const bool leftMouse = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        leftMousePressed_ = leftMouse;
        if (!mouseControlEnabled_ && leftMouse && !previousLeftMouseState_ && !rightMouse &&
            !middleMouse) {
            pendingMouseClick_ = true;
            pendingMouseClickX_ = cursorX;
            pendingMouseClickY_ = cursorY;
        }
        previousLeftMouseState_ = leftMouse;
    }
    {
        TraceScope phase("render", "RenderLoop::beginFrame.setupView");
        viewport_ = {0, 0, width, height};
        glViewport(viewport_[0], viewport_[1], viewport_[2], viewport_[3]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        setPerspective(static_cast<double>(width), static_cast<double>(height),
                       currentFieldOfView());
    }
}

void RenderLoop::draw(const BusSimulation& simulation) {
    TraceScope trace("frame", "RenderLoop::draw");
    if (!renderingReflection_) {
        openbus::rendering::resetFrameTriangleCount();
    }
    const BodyPose chassis = simulation.chassisPose();
    const ChassisCollisionBox collision = simulation.chassisCollisionBox();
    constexpr double radiansToDegrees = 180.0 / 3.14159265358979323846;
    const Matrix4 playerModelBase = multiplyMatrix4(
        poseMatrix(chassis),
        translationMatrix(
            {0.0, 0.0, playerVehicle_ != nullptr ? playerVehicle_->modelOffsetZ : 0.0}));
    const Matrix4 shadowGroundBase = multiplyMatrix4(
        translationMatrix({chassis.position[0], chassis.position[1], GROUND_SHADOW_OFFSET_Z}),
        rotationMatrix(simulation.yaw() * radiansToDegrees, 0.0, 0.0, 1.0));
    Matrix4 inversePlayerModelBase = identityMatrix();
    const bool canGroundClampShadow = invertAffineMatrix(playerModelBase, inversePlayerModelBase);
    const Matrix4 groundShadowTransform =
        canGroundClampShadow ? multiplyMatrix4(inversePlayerModelBase, shadowGroundBase)
                             : identityMatrix();
    {
        TraceScope phase("render", "RenderLoop::draw.camera");
        if (cameraView_ == 0) {
            const std::array<double, 3> target =
                transformLocalPoint(chassis, simulation.outsideCameraCenter());
            const double targetX = target[0];
            const double targetY = target[1];
            const double targetZ = target[2];
            const double horizontalDistance = cameraDistance_ * std::cos(cameraPitch_);
            const double orbitYaw = simulation.yaw() + cameraYaw_;
            const double eyeX = targetX + horizontalDistance * std::cos(orbitYaw);
            const double eyeY = targetY + horizontalDistance * std::sin(orbitYaw);
            const double eyeZ = targetZ + cameraDistance_ * std::sin(cameraPitch_);
            lookAt(eyeX, eyeY, eyeZ, targetX, targetY, targetZ);
        } else if (const VehicleCamera* camera = currentVehicleCamera(); camera != nullptr) {
            constexpr double DEGREES_TO_RADIANS = 3.141592653589793 / 180.0;
            const bool reflectionCamera = camera->kind == VehicleCameraKind::Reflexion ||
                                          camera->kind == VehicleCameraKind::Reflexion2;
            const double panSign = reflectionCamera ? 1.0 : -1.0;
            const double pan = panSign * camera->pan * DEGREES_TO_RADIANS + viewLookYaw_;
            const double tilt = camera->tilt * DEGREES_TO_RADIANS + viewLookPitch_;
            const double modelOffsetZ =
                playerVehicle_ != nullptr ? playerVehicle_->modelOffsetZ : 0.0;
            const std::array<double, 3> centerLocal = {camera->position[1], -camera->position[0],
                                                       camera->position[2] + modelOffsetZ};
            const std::array<double, 3> direction = {
                std::cos(tilt) * std::cos(pan), std::cos(tilt) * std::sin(pan), std::sin(tilt)};
            const std::array<double, 3> upLocal = {-std::sin(tilt) * std::cos(pan),
                                                   -std::sin(tilt) * std::sin(pan), std::cos(tilt)};
            const std::array<double, 3> eyeLocal = {
                centerLocal[0] - direction[0] * camera->orbitDistance,
                centerLocal[1] - direction[1] * camera->orbitDistance,
                centerLocal[2] - direction[2] * camera->orbitDistance};
            const std::array<double, 3> targetLocal = {eyeLocal[0] + direction[0] * 3.0,
                                                       eyeLocal[1] + direction[1] * 3.0,
                                                       eyeLocal[2] + direction[2] * 3.0};
            const std::array<double, 3> eye = transformLocalPoint(chassis, eyeLocal);
            const std::array<double, 3> target = transformLocalPoint(chassis, targetLocal);
            const std::array<double, 3> up = {
                chassis.rotation[0] * upLocal[0] + chassis.rotation[1] * upLocal[1] +
                    chassis.rotation[2] * upLocal[2],
                chassis.rotation[3] * upLocal[0] + chassis.rotation[4] * upLocal[1] +
                    chassis.rotation[5] * upLocal[2],
                chassis.rotation[6] * upLocal[0] + chassis.rotation[7] * upLocal[1] +
                    chassis.rotation[8] * upLocal[2]};
            lookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2], up[0], up[1], up[2]);
        } else {
            std::array<double, 3> eyeLocal = {5.65, 0.70, 0.70};
            std::array<double, 3> targetLocal = {8.65, 0.70, 0.72};
            switch (cameraView_) {
            case 2:
                eyeLocal = {4.25, -0.55, 0.78};
                targetLocal = {7.25, -0.55, 0.80};
                break;
            case 3:
                eyeLocal = {-2.75, 0.0, 0.72};
                targetLocal = {0.25, 0.0, 0.74};
                break;
            case 4:
                eyeLocal = {5.35, 1.05, 0.86};
                targetLocal = {5.35, 4.0, 0.82};
                break;
            case 5:
                eyeLocal = {5.35, -1.05, 0.86};
                targetLocal = {5.35, -4.0, 0.82};
                break;
            case 6:
                eyeLocal = {6.85, 0.0, 0.80};
                targetLocal = {3.85, 0.0, 0.80};
                break;
            case 7:
                eyeLocal = {-6.15, 0.0, 0.80};
                targetLocal = {-3.15, 0.0, 0.80};
                break;
            case 8:
                eyeLocal = {0.0, 1.05, 1.30};
                targetLocal = {0.0, 4.0, 1.30};
                break;
            case 9:
                eyeLocal = {0.0, -1.05, 1.30};
                targetLocal = {0.0, -4.0, 1.30};
                break;
            default:
                break;
            }
            const std::array<double, 3> eye = transformLocalPoint(chassis, eyeLocal);
            const double directionX = targetLocal[0] - eyeLocal[0];
            const double directionY = targetLocal[1] - eyeLocal[1];
            const double directionZ = targetLocal[2] - eyeLocal[2];
            const double horizontalLength = std::hypot(directionX, directionY);
            const double distance = std::sqrt(directionX * directionX + directionY * directionY +
                                              directionZ * directionZ);
            const double baseYaw = std::atan2(directionY, directionX);
            const double basePitch = std::atan2(directionZ, horizontalLength);
            const double lookYaw = baseYaw + viewLookYaw_;
            const double lookPitch = basePitch + viewLookPitch_;
            const std::array<double, 3> target = transformLocalPoint(
                chassis, {eyeLocal[0] + distance * std::cos(lookPitch) * std::cos(lookYaw),
                          eyeLocal[1] + distance * std::cos(lookPitch) * std::sin(lookYaw),
                          eyeLocal[2] + distance * std::sin(lookPitch)});
            lookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2]);
        }
        if (!renderingReflection_) {
            const auto& view = modelViewMatrix();
            const std::array<double, 3> eyeWorld = {
                -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]),
                -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]),
                -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14])};
            const std::array<double, 3> forwardWorld = {-view[2], -view[6], -view[10]};
            const std::array<double, 3> upWorld = {view[1], view[5], view[9]};
            const auto toLocalPoint = [&chassis](const std::array<double, 3>& world) {
                const double x = world[0] - chassis.position[0];
                const double y = world[1] - chassis.position[1];
                const double z = world[2] - chassis.position[2];
                return std::array<double, 3>{
                    chassis.rotation[0] * x + chassis.rotation[3] * y + chassis.rotation[6] * z,
                    chassis.rotation[1] * x + chassis.rotation[4] * y + chassis.rotation[7] * z,
                    chassis.rotation[2] * x + chassis.rotation[5] * y + chassis.rotation[8] * z};
            };
            const auto toLocalDirection = [&chassis](const std::array<double, 3>& world) {
                return std::array<double, 3>{
                    chassis.rotation[0] * world[0] + chassis.rotation[3] * world[1] +
                        chassis.rotation[6] * world[2],
                    chassis.rotation[1] * world[0] + chassis.rotation[4] * world[1] +
                        chassis.rotation[7] * world[2],
                    chassis.rotation[2] * world[0] + chassis.rotation[5] * world[1] +
                        chassis.rotation[8] * world[2]};
            };
            soundEngine_.setListenerPose(toLocalPoint(eyeWorld), toLocalDirection(forwardWorld),
                                         toLocalDirection(upWorld));
        }
    }
    {
        TraceScope phase("render", "RenderLoop::draw.visibility");
        if (!renderingReflection_) {
            const RenderViewContext context = isExteriorView() ? RenderViewContext::PlayerExterior
                                                               : RenderViewContext::PlayerInterior;
            if (playerVehicle_ != nullptr) {
                playerVehicle_->setOdeSimulation(simulation);
            }
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                pushMatrix();
                if (vehicle.get() == playerVehicle_) {
                    applyPose(chassis);
                    vehicle->prepareFrameVisibility(context, framebufferWidth_, framebufferHeight_);
                } else {
                    applyVehiclePlacement(
                        makeModelRootPlacement(vehicle->placement, vehicle->modelOffsetZ));
                    vehicle->prepareFrameVisibility(RenderViewContext::NonPlayer, framebufferWidth_,
                                                    framebufferHeight_);
                }
                popMatrix();
            }
        }
    }
    if (!renderingReflection_ && playerVehicle_ != nullptr) {
        TraceScope phase("input", "RenderLoop::draw.interaction");
        double cursorX = 0.0;
        double cursorY = 0.0;
        glfwGetCursorPos(window_, &cursorX, &cursorY);
        int windowWidth = 1;
        int windowHeight = 1;
        glfwGetWindowSize(window_, &windowWidth, &windowHeight);
        const double framebufferScaleX =
            static_cast<double>(framebufferWidth_) / static_cast<double>(std::max(windowWidth, 1));
        const double framebufferScaleY = static_cast<double>(framebufferHeight_) /
                                         static_cast<double>(std::max(windowHeight, 1));
        const double framebufferCursorX = cursorX * framebufferScaleX;
        const double framebufferCursorY = cursorY * framebufferScaleY;
        pushMatrix();
        applyPose(chassis);
        const RenderViewContext interactionContext = isExteriorView()
                                                         ? RenderViewContext::PlayerExterior
                                                         : RenderViewContext::PlayerInterior;
        if (!mouseControlEnabled_ && benchmarkClickDiscoveryPending_) {
            TraceScope trace("benchmark", "Benchmark.discoverClickable");
            benchmarkClickDiscoveryPending_ = false;
            benchmarkClickDiscoveryComplete_ = true;
            if (playerVehicle_->hasVisibleClickable(interactionContext)) {
                constexpr int columns = 40;
                constexpr int rows = 24;
                for (int row = 1; row < rows && !benchmarkClickTargetFound_; ++row) {
                    const double y = static_cast<double>(framebufferHeight_) * row / rows;
                    for (int column = 1; column < columns; ++column) {
                        const double x = static_cast<double>(framebufferWidth_) * column / columns;
                        if (playerVehicle_
                                ->mouseEventAt(x, y, framebufferWidth_, framebufferHeight_,
                                               interactionContext)
                                .empty()) {
                            continue;
                        }
                        benchmarkClickFramebufferX_ = x;
                        benchmarkClickFramebufferY_ = y;
                        benchmarkClickTargetFound_ = true;
                        break;
                    }
                }
            }
        }
        if (!mouseControlEnabled_ && benchmarkClickQueued_ && benchmarkClickTargetFound_) {
            pendingMouseClick_ = true;
            pendingMouseClickX_ = benchmarkClickFramebufferX_ / framebufferScaleX;
            pendingMouseClickY_ = benchmarkClickFramebufferY_ / framebufferScaleY;
        }
        benchmarkClickQueued_ = false;
        if (!mouseControlEnabled_) {
            const int interactionContextValue = static_cast<int>(interactionContext);
            const std::uint64_t clickableRevision = playerVehicle_->clickableRevision();
            // Pose changes every simulation step; while the pointer is stationary, refresh
            // animated/moving hit targets at 13 Hz instead of scanning all triangles per frame.
            constexpr double clickableHoverRefreshInterval = 0.075;
            const double currentTime = glfwGetTime();
            const bool hoverCacheMatches =
                clickableHoverCacheValid_ && clickableHoverCacheX_ == framebufferCursorX &&
                clickableHoverCacheY_ == framebufferCursorY &&
                clickableHoverCacheWidth_ == framebufferWidth_ &&
                clickableHoverCacheHeight_ == framebufferHeight_ &&
                clickableHoverCacheContext_ == interactionContextValue &&
                clickableHoverCacheCameraView_ == cameraView_ &&
                clickableHoverCacheCameraYaw_ == cameraYaw_ &&
                clickableHoverCacheCameraPitch_ == cameraPitch_ &&
                clickableHoverCacheLookYaw_ == viewLookYaw_ &&
                clickableHoverCacheLookPitch_ == viewLookPitch_ &&
                clickableHoverCacheRevision_ == clickableRevision &&
                currentTime - clickableHoverCacheTimestamp_ < clickableHoverRefreshInterval;
            if (!hoverCacheMatches) {
                clickableHoverCacheValid_ = true;
                clickableHoverCacheX_ = framebufferCursorX;
                clickableHoverCacheY_ = framebufferCursorY;
                clickableHoverCacheWidth_ = framebufferWidth_;
                clickableHoverCacheHeight_ = framebufferHeight_;
                clickableHoverCacheContext_ = interactionContextValue;
                clickableHoverCacheCameraView_ = cameraView_;
                clickableHoverCacheCameraYaw_ = cameraYaw_;
                clickableHoverCacheCameraPitch_ = cameraPitch_;
                clickableHoverCacheLookYaw_ = viewLookYaw_;
                clickableHoverCacheLookPitch_ = viewLookPitch_;
                clickableHoverCacheRevision_ = clickableRevision;
                clickableHoverCacheTimestamp_ = currentTime;
                clickableHoverCacheHit_ =
                    playerVehicle_->hasVisibleClickable(interactionContext) &&
                    playerVehicle_->hasClickableAt(framebufferCursorX, framebufferCursorY,
                                                   framebufferWidth_, framebufferHeight_,
                                                   interactionContext);
            }
        } else {
            pendingMouseClick_ = false;
            clickableHoverCacheValid_ = false;
            clickableHoverCacheHit_ = false;
        }
        const bool hoveringClickable = !mouseControlEnabled_ && clickableHoverCacheHit_;
        GLFWcursor* cursor = mouseControlEnabled_ ? mouseSteeringCursor_
                             : hoveringClickable  ? clickableCursor_
                                                  : nullptr;
        glfwSetCursor(window_, cursor);
        if (mouseControlEnabled_) {
            pendingMouseClick_ = false;
            if (!activeMouseEvent_.empty() && playerVehicle_->scripts) {
                playerVehicle_->scripts->invokeMouseRelease(activeMouseEvent_);
            }
            activeMouseEvent_.clear();
        } else if (pendingMouseClick_) {
            activeMouseEvent_ = playerVehicle_->mouseEventAt(
                pendingMouseClickX_ * framebufferScaleX, pendingMouseClickY_ * framebufferScaleY,
                framebufferWidth_, framebufferHeight_, interactionContext);
            playerVehicle_->handleMouseClick(activeMouseEvent_);
            previousMouseInteractionX_ = pendingMouseClickX_;
            previousMouseInteractionY_ = pendingMouseClickY_;
            pendingMouseClick_ = false;
        } else if (!leftMousePressed_) {
            if (!activeMouseEvent_.empty()) {
                playerVehicle_->scripts->invokeMouseRelease(activeMouseEvent_);
            }
            activeMouseEvent_.clear();
        } else if (!activeMouseEvent_.empty()) {
            const double deltaX = cursorX - previousMouseInteractionX_;
            const double deltaY = cursorY - previousMouseInteractionY_;
            if (deltaX != 0.0 || deltaY != 0.0) {
                playerVehicle_->scripts->invokeMouseDrag(activeMouseEvent_, deltaX, deltaY, cursorX,
                                                         cursorY);
                previousMouseInteractionX_ = cursorX;
                previousMouseInteractionY_ = cursorY;
            }
        }
        popMatrix();
    }
    {
        TraceScope phase("render", "RenderLoop::draw.reflections");
        renderReflectionViews(simulation);
    }
    {
        TraceScope phase("render", "RenderLoop::draw.ground");
        if (mapRenderer_ != nullptr) {
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glDisable(GL_CULL_FACE);
            glDisable(GL_POLYGON_OFFSET_FILL);
            mapRenderer_->draw();
        } else if (!captureMode_) {
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glDisable(GL_CULL_FACE);
            glDisable(GL_POLYGON_OFFSET_FILL);
            drawGround(simulation.roadBumps());
        }
    }
    const RenderViewContext context = (renderingReflection_ || isExteriorView())
                                          ? RenderViewContext::PlayerExterior
                                          : RenderViewContext::PlayerInterior;
    {
        TraceScope phase("render", "RenderLoop::draw.model");
        bool playerDrawn = false;
        const auto drawVehicles = [&](VehicleRenderPass renderPass) {
            for (const std::unique_ptr<Vehicle>& vehicle : vehicles_) {
                if (!vehicle->loaded || vehicle->displayLists.empty()) {
                    continue;
                }
                pushMatrix();
                const RenderViewContext vehicleContext =
                    vehicle.get() == playerVehicle_ ? context : RenderViewContext::NonPlayer;
                if (vehicle.get() == playerVehicle_) {
                    applyPose(chassis);
                    if (renderPass == VehicleRenderPass::Opaque) {
                        playerDrawn = true;
                    }
                } else {
                    applyVehiclePlacement(
                        makeModelRootPlacement(vehicle->placement, vehicle->modelOffsetZ));
                }
                Matrix4 aiGroundShadowTransform = {};
                const Matrix4* shadowTransform = nullptr;
                if (vehicle.get() == playerVehicle_) {
                    if (canGroundClampShadow) {
                        shadowTransform = &groundShadowTransform;
                    }
                } else {
                    const VehiclePlacement rootPlacement =
                        makeModelRootPlacement(vehicle->placement, vehicle->modelOffsetZ);
                    aiGroundShadowTransform = makeGroundShadowTransform(
                        rootPlacement, vehicle->modelOffsetZ, GROUND_SHADOW_OFFSET_Z);
                    shadowTransform = &aiGroundShadowTransform;
                }
                vehicle->draw(vehicleContext, renderPass, shadowTransform);
                popMatrix();
            }
        };
        drawVehicles(VehicleRenderPass::Opaque);
        drawVehicles(VehicleRenderPass::Transparent);
        if (!playerDrawn && collision.enabled && !collision.mesh) {
            pushMatrix();
            applyPose(chassis);
            translate(collision.offsetX, collision.offsetY, collision.offsetZ);
            drawBox(collision.length, collision.width, collision.height, 0.85, 0.70, 0.08);
            popMatrix();
        }
        if (!renderingReflection_ && !mouseControlEnabled_ && clickableDebugOverlay_ &&
            playerVehicle_ != nullptr) {
            pushMatrix();
            applyPose(chassis);
            playerVehicle_->drawClickableDebug(context);
            popMatrix();
        }
    }
    if (!renderingReflection_ && collisionDebugOverlay_) {
        const std::uint64_t collisionRevision = simulation.collisionDebugRevision();
        if (!collisionWireframeBuilt_ || collisionWireframeRevision_ != collisionRevision) {
            collisionWireframeVertices_.clear();
            const std::vector<StaticCollisionMesh> meshes = simulation.collisionDebugMeshes();
            std::size_t triangleCount = 0;
            for (const StaticCollisionMesh& mesh : meshes) {
                if (mesh.vertices.size() % 3 != 0 || mesh.indices.size() % 3 != 0) {
                    continue;
                }
                triangleCount += mesh.indices.size() / 3;
                for (std::size_t index = 0; index + 2 < mesh.indices.size(); index += 3) {
                    const std::array<int, 3> triangle = {
                        mesh.indices[index], mesh.indices[index + 1], mesh.indices[index + 2]};
                    for (int edge = 0; edge < 3; ++edge) {
                        for (const int vertexIndex :
                             {triangle[static_cast<std::size_t>(edge)],
                              triangle[static_cast<std::size_t>((edge + 1) % 3)]}) {
                            if (vertexIndex < 0 || static_cast<std::size_t>(vertexIndex) * 3 + 2 >=
                                                       mesh.vertices.size()) {
                                continue;
                            }
                            const std::size_t vertexOffset =
                                static_cast<std::size_t>(vertexIndex) * 3;
                            collisionWireframeVertices_.push_back(
                                {static_cast<float>(mesh.vertices[vertexOffset]),
                                 static_cast<float>(mesh.vertices[vertexOffset + 1]),
                                 static_cast<float>(mesh.vertices[vertexOffset + 2] + 0.025), 1.0F,
                                 0.58F, 0.08F});
                        }
                    }
                }
            }
            collisionWireframeBuilt_ = true;
            collisionWireframeRevision_ = collisionRevision;
            gameLog.Log("Collision wireframe cached: " + std::to_string(triangleCount) +
                        " triangles across " + std::to_string(meshes.size()) +
                        " active terrain/road meshes");
        }

        const GLboolean depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
        GLboolean depthWriteWasEnabled = GL_TRUE;
        glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWriteWasEnabled);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        openbus::rendering::drawStaticPrimitives(collisionWireframeBuffer_,
                                                 collisionWireframeVertices_, GL_LINES, 1.5F);
        if (collision.enabled && !collision.mesh) {
            pushMatrix();
            applyPose(chassis);
            translate(collision.offsetX, collision.offsetY, collision.offsetZ);
            openbus::rendering::drawBox(collision.length, collision.width, collision.height, 1.0,
                                        0.15, 0.12);
            popMatrix();
        }
        glDepthMask(depthWriteWasEnabled);
        if (!depthWasEnabled) {
            glDisable(GL_DEPTH_TEST);
        }
    }
    if (!renderingReflection_ && coordinateHudTexture_ != 0) {
        std::ostringstream coordinateText;
        coordinateText << "X " << std::fixed << std::setprecision(1) << simulation.positionX()
                       << " Y " << simulation.positionY() << " Z " << simulation.positionZ();
        const std::string currentCoordinates = coordinateText.str();
        if (currentCoordinates != coordinateHudText_) {
            const std::vector<std::uint8_t> pixels = makeCoordinateHudPixels(currentCoordinates);
            pglActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, coordinateHudTexture_);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, COORDINATE_HUD_TEXTURE_WIDTH,
                            COORDINATE_HUD_TEXTURE_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE,
                            pixels.data());
            openbus::rendering::invalidateTextureBindings();
            coordinateHudText_ = currentCoordinates;
        }

        const GLboolean depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
        const GLboolean blendWasEnabled = glIsEnabled(GL_BLEND);
        const GLboolean cullWasEnabled = glIsEnabled(GL_CULL_FACE);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        const float scale = std::min(
            {1.5F,
             static_cast<float>(std::max(framebufferWidth_ - 24, 1)) / COORDINATE_HUD_TEXTURE_WIDTH,
             static_cast<float>(std::max(framebufferHeight_ - 24, 1)) /
                 COORDINATE_HUD_TEXTURE_HEIGHT});
        const float hudWidth = COORDINATE_HUD_TEXTURE_WIDTH * scale;
        const float hudHeight = COORDINATE_HUD_TEXTURE_HEIGHT * scale;
        const float leftPixels = 12.0F;
        const float bottomPixels = framebufferHeight_ - 12.0F - hudHeight;
        openbus::rendering::drawTextureQuad(
            coordinateHudTexture_, -1.0F + 2.0F * leftPixels / framebufferWidth_,
            -1.0F + 2.0F * bottomPixels / framebufferHeight_, 2.0F * hudWidth / framebufferWidth_,
            2.0F * hudHeight / framebufferHeight_);
        if (depthWasEnabled) {
            glEnable(GL_DEPTH_TEST);
        }
        if (blendWasEnabled) {
            glEnable(GL_BLEND);
        } else {
            glDisable(GL_BLEND);
        }
        if (cullWasEnabled) {
            glEnable(GL_CULL_FACE);
        }
    }
    if (!renderingReflection_ && playerVehicle_ && glfwGetTime() - lastStatsTitleTime_ > 0.25) {
        const double speedMetresPerSecond = simulation.speed();
        const long speedMph = std::lround(speedMetresPerSecond * 2.2369362921);
        const long speedKmh = std::lround(speedMetresPerSecond * 3.6);
        const long throttlePercent =
            std::lround(std::clamp(playerVehicle_->variables.get("throttle"), 0.0F, 1.0F) * 100.0F);
        const long brakePercent =
            std::lround(std::clamp(playerVehicle_->variables.get("brake"), 0.0F, 1.0F) * 100.0F);
        std::ostringstream title;
        title << "OpenBus - " << speedMph << " mph / " << speedKmh << " km/h - Throttle "
              << throttlePercent << "% - Brake " << brakePercent << "% - "
              << openbus::rendering::frameTriangleCount() << " triangles/frame";
        glfwSetWindowTitle(window_, title.str().c_str());
        lastStatsTitleTime_ = glfwGetTime();
    }
}

void RenderLoop::endFrame() {
    TraceScope trace("frame", "RenderLoop::endFrame.swapBuffers");
    glfwSwapBuffers(window_);
}

void RenderLoop::captureViews(const BusSimulation& simulation,
                              const std::filesystem::path& directory) {
    TraceScope trace("capture", "RenderLoop::captureViews");
    std::filesystem::create_directories(directory);
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window_, &width, &height);
    framebufferWidth_ = std::max(width, 1);
    framebufferHeight_ = std::max(height, 1);

    const int previousCameraView = cameraView_;
    const double previousCameraYaw = cameraYaw_;
    const double previousCameraPitch = cameraPitch_;
    const double previousCameraDistance = cameraDistance_;
    const double previousViewLookYaw = viewLookYaw_;
    const double previousViewLookPitch = viewLookPitch_;
    captureMode_ = true;
    cameraView_ = 0;
    cameraDistance_ = 24.0;
    viewLookYaw_ = 0.0;
    viewLookPitch_ = 0.0;

    struct CaptureView {
        const char* name;
        int cameraView;
        double yaw;
        double pitch;
        double distance;
    };
    const std::array<CaptureView, 5> views = {{{"dashboard", 1, 0.0, 0.0, 0.0},
                                               {"three-quarter", 0, 0.55, 0.08, 11.0},
                                               {"front", 0, 0.0, 0.08, 10.5},
                                               {"left", 0, 1.5707963267948966, 0.08, 10.5},
                                               {"right", 0, -1.5707963267948966, 0.08, 10.5}}};
    for (const CaptureView& view : views) {
        TraceScope viewTrace("capture", "RenderLoop::captureViews.view");
        cameraView_ = view.cameraView;
        cameraYaw_ = view.yaw;
        cameraPitch_ = view.pitch;
        cameraDistance_ = view.distance;
        {
            TraceScope phase("capture", "RenderLoop::captureViews.render");
            viewport_ = {0, 0, width, height};
            glViewport(viewport_[0], viewport_[1], viewport_[2], viewport_[3]);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            setPerspective(static_cast<double>(width), static_cast<double>(height), 60.0);
            draw(simulation);
            glFinish();
        }
        const std::filesystem::path path = directory / (std::string(view.name) + ".png");
        {
            TraceScope phase("capture", "RenderLoop::captureViews.save");
            if (!openbus::rendering::saveFramebufferPng(path, width, height)) {
                gameLog.Log("Failed to save screenshot: " + path.string());
            } else {
                gameLog.Log("Saved screenshot: " + path.string());
            }
        }
    }

    cameraView_ = previousCameraView;
    cameraYaw_ = previousCameraYaw;
    cameraPitch_ = previousCameraPitch;
    cameraDistance_ = previousCameraDistance;
    viewLookYaw_ = previousViewLookYaw;
    viewLookPitch_ = previousViewLookPitch;
    captureMode_ = false;
}

bool RenderLoop::consumeCaptureRequest() {
    const bool requested = captureRequested_;
    captureRequested_ = false;
    return requested;
}

bool RenderLoop::isCaptureReady() const {
    return playerVehicle_ && playerVehicle_->isCaptureReady();
}

double RenderLoop::throttle() const {
    if (mouseControlEnabled_) {
        return mouseThrottle_;
    }
    const VehicleKeyBinding* binding = findVehicleKeyBinding("throttle");
    return binding != nullptr && vehicleBindingPressed(window_, *binding) ? 1.0 : 0.0;
}

double RenderLoop::steering() const {
    if (mouseControlEnabled_) {
        return mouseSteering_;
    }
    const VehicleKeyBinding* leftBinding = findVehicleKeyBinding("steering_left");
    const VehicleKeyBinding* rightBinding = findVehicleKeyBinding("steering_right");
    const bool left = leftBinding != nullptr && vehicleBindingPressed(window_, *leftBinding);
    const bool right = rightBinding != nullptr && vehicleBindingPressed(window_, *rightBinding);
    return static_cast<double>(right) - static_cast<double>(left);
}

double RenderLoop::brake() const {
    if (mouseControlEnabled_) {
        return mouseBrake_;
    }
    const VehicleKeyBinding* binding = findVehicleKeyBinding("brake");
    return binding != nullptr && vehicleBindingPressed(window_, *binding) ? 1.0 : 0.0;
}

double RenderLoop::physicsThrottle() const {
    return playerVehicle_ == nullptr ? throttle() : playerVehicle_->variables.get("throttle");
}

double RenderLoop::physicsWheelTorque() const {
    return playerVehicle_ == nullptr ? 0.0 : playerVehicle_->variables.get("m_wheel");
}

double RenderLoop::physicsSteering() const {
    return playerVehicle_ == nullptr ? smoothedSteering_
                                     : playerVehicle_->variables.get("steering");
}

std::vector<double> RenderLoop::physicsWheelBrakeForces(std::size_t axleCount) const {
    std::vector<double> forces(axleCount * 2, 0.0);
    if (playerVehicle_ == nullptr) {
        return forces;
    }
    for (std::size_t axle = 0; axle < axleCount; ++axle) {
        const std::string prefix = "axle_brakeforce_" + std::to_string(axle) + "_";
        forces[axle * 2] = playerVehicle_->variables.get(prefix + "l");
        forces[axle * 2 + 1] = playerVehicle_->variables.get(prefix + "r");
    }
    return forces;
}

std::vector<double> RenderLoop::physicsAxleSpringFactors(std::size_t axleCount) const {
    std::vector<double> factors(axleCount * 2, 1.0);
    if (playerVehicle_ == nullptr) {
        return factors;
    }
    for (std::size_t axle = 0; axle < axleCount; ++axle) {
        const std::string prefix = "axle_springfactor_" + std::to_string(axle) + "_";
        const std::string leftName = prefix + "l";
        const std::string rightName = prefix + "r";
        if (playerVehicle_->variables.has(leftName)) {
            factors[axle * 2] = playerVehicle_->variables.get(leftName);
        }
        if (playerVehicle_->variables.has(rightName)) {
            factors[axle * 2 + 1] = playerVehicle_->variables.get(rightName);
        }
    }
    return factors;
}

std::vector<KeyEvent> RenderLoop::consumeKeyEvents() {
    std::vector<KeyEvent> events;
    events.swap(keyEvents_);
    return events;
}

bool RenderLoop::consumeRaisePlayerRequest() {
    const bool requested = raisePlayerRequestPending_;
    raisePlayerRequestPending_ = false;
    return requested;
}
