#pragma once

// Internal to the scripting module: ScriptRuntime::Impl state shared by the
// ScriptRuntime*.cpp / Script*.cpp translation units.

#include "scripting/ScriptRuntime.h"

#include "assets/TextureLoader.h"
#include "config/ConfigurationParser.h"
#include "config/ScriptTextureLimits.h"
#include "core/AssetPath.h"
#include "core/Environment.h"
#include "core/Logger.h"
#include "core/PerfTrace.h"
#include "core/Variables.h"
#include "osc/OscConverter.h"

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string_view>
#include <unordered_map>

extern Logger scriptRuntimeLogger;

namespace script_detail {

std::string lower(std::string value);

std::string scriptName(std::string value);

std::shared_ptr<const OscProgram> cachedNativeProgram(const std::filesystem::path& sourcePath,
                                                      std::string& error, bool& compiledThisCall);

} // namespace script_detail

using script_detail::cachedNativeProgram;
using script_detail::lower;
using script_detail::scriptName;

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

    static bool floatToInt(float value, int& result);

    static bool validDrawingOrigin(int value);

    static std::uint8_t colorChannel(float value);

    static std::uint32_t packedColor(float value);

    static bool validTextureIndex(int index);

    std::size_t currentTextureBytes() const;

    bool canReplaceTextureBytes(std::size_t oldBytes, std::size_t newBytes) const;

    ScriptTexture configuredScriptTexture(int index);

    static std::array<std::uint8_t, 7> glyph(char character);

    std::shared_ptr<FontAsset> loadFont(const std::string& requestedName);

    float nativePopFloat();

    float nativePeekFloat() const;

    float soundControlValue() const;

    std::string nativePopString();

    const std::string& nativePeekString() const;

    void executeSystemMacro(const std::string& name);

    bool executeNativeFunction(const std::string& functionName, int callDepth = 0);

    bool tryInitializeNativeBackend();

    static bool compileOnlyRequested();

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

    void log(const std::string& message) const;

    static int panicHandler(lua_State* lua);

    ~Impl();

    Impl* self(lua_State* lua);

    static float numericArgument(lua_State* lua, int index);

    static int oscFloat32(lua_State* lua);

    static int getLocal(lua_State* lua);

    static int setLocal(lua_State* lua);

    static int getLocalString(lua_State* lua);

    static int setLocalString(lua_State* lua);

    static int getSystem(lua_State* lua);

    static int setSystem(lua_State* lua);

    static int getConstant(lua_State* lua);

    static int callFunction(lua_State* lua);

    static int pushFloat(lua_State* lua);

    static int popFloat(lua_State* lua);

    static int peekFloat(lua_State* lua);

    static int pushString(lua_State* lua);

    static int popString(lua_State* lua);

    static int peekString(lua_State* lua);

    static int soundTrigger(lua_State* lua);

    static int soundTriggerFile(lua_State* lua);

    static int debug(lua_State* lua);

    // TODO: Replace safe system-macro fallbacks with HOF, timetable, and vehicle data.
    static float popSystemFloat(Impl* runtime);

    static std::string popSystemString(Impl* runtime);

    static int returnSystemFloat(lua_State* lua, float value);

    static int returnSystemString(lua_State* lua, const char* value = "");

    // TODO: Implement the corresponding HOF, timetable, or passenger lookup.
    static int safeNumericLookup(lua_State* lua);

    // TODO: Implement the corresponding HOF lookup and return the actual missing index.
    static int safeMissingIndex(lua_State* lua);

    // TODO: Implement the corresponding HOF lookup and return the requested string.
    static int safeStringLookup(lua_State* lua);

    // TODO: Implement route bus-stop lists from the active HOF data.
    static int safeRouteBusstopIdent(lua_State* lua);

    // TODO: Implement current line number
    static int safeCurrentLineNumber(lua_State* lua);

    // TODO: Implement vehicle-relative ground ray casting.
    static int safeGetHeightAbovePoint(lua_State* lua);

    // TODO: Implement deterministic fleet-number and seed-based randomization.
    static int safeNrSpecRandom(lua_State* lua);

    // TODO: Implement currency configuration and change-coin issuance.
    static int safeGiveChangeCoin(lua_State* lua);

    // TODO: Implement ticket-pack and global-string lookups.
    static int safeTicketName(lua_State* lua);

    static void discardSystemFloats(Impl* runtime, std::size_t count);

    static void discardSystemStrings(Impl* runtime, std::size_t count);

    // TODO: Implement arrival-board line and destination data from the parent bus stop.
    static int safeArrivalString(lua_State* lua);

    // TODO: Implement arrival-board time differences from the parent bus stop.
    static int safeArrivalTime(lua_State* lua);

    static ScriptTexture& scriptTexture(Impl* runtime, int index);

    static bool popSystemInt(Impl* runtime, int& value);

    static bool popSystemTextureIndex(Impl* runtime, int& index);

    bool resizeScriptTexture(ScriptTexture& texture, int requiredWidth, int requiredHeight);

    static std::size_t pixelOffset(const ScriptTexture& texture, int x, int y);

    void setPixel(ScriptTexture& texture, int x, int y);

    void fillRect(ScriptTexture& texture, int x1, int y1, int x2, int y2);

    static void clearTexture(ScriptTexture& texture);

    static void drawText(ScriptTexture& texture, const std::string& value, int x, int y,
                         std::array<std::uint8_t, 4> color, int letterSpacing = 0,
                         int maximumScale = 8, TextAlignment alignment = TextAlignment::Left,
                         bool centerVertically = false);

    static bool drawBitmapFontText(ScriptTexture& texture, const std::string& value, int x, int y,
                                   const TextTextureDefinition& definition, const FontAsset& font);

    void updateTextTextures(bool force = false);

    bool loadScriptTexture(int index, const std::string& requestedPath);

    static int safeScriptTextureAction(lua_State* lua);

    static int safeScriptTextureNew(lua_State* lua);

    static int safeScriptTextureLock(lua_State* lua);

    static int safeScriptTextureUnlock(lua_State* lua);

    static int safeScriptTextureFilter(lua_State* lua);

    static int safeScriptTextureColor(lua_State* lua);

    static int safeScriptTexturePixel(lua_State* lua);

    static int safeScriptTextureReadPixel(lua_State* lua);

    static int safeScriptTextureRect(lua_State* lua);

    static int safeScriptTextureText(lua_State* lua);

    static int safeScriptTextureCopyColor(lua_State* lua);

    static int safeScriptTextureLoad(lua_State* lua);

    static int safeScriptTextureChannel(lua_State* lua, std::size_t channel);

    static int safeScriptTextureRed(lua_State* lua);

    static int safeScriptTextureGreen(lua_State* lua);

    static int safeScriptTextureBlue(lua_State* lua);

    static int safeScriptTextureAlpha(lua_State* lua);

    static int safeFontIndex(lua_State* lua);

    static int safeTextLength(lua_State* lua);

    static int tracedLuaFunction(lua_State* lua);

    static int genericSystemMacro(lua_State* lua);

    static Impl* runtimeFor(lua_State* lua);

    void registerFunction(const char* name, lua_CFunction function);

    void registerSystemMacro(const char* name, lua_CFunction);

    void registerFunctions();

    void loadScripts();

    bool invoke(const char* functionName);

    void invokeFrame(bool isAiVehicle);

    bool copyScriptTexture(int index, ScriptRuntime::ScriptTextureSnapshot& snapshot) const;

    bool copyTextTexture(int index, ScriptRuntime::ScriptTextureSnapshot& snapshot) const;

    bool copyScriptTextureIfChanged(int index, std::uint64_t knownRevision,
                                    ScriptRuntime::ScriptTextureSnapshot& snapshot) const;

    bool copyTextTextureIfChanged(int index, std::uint64_t knownRevision,
                                  ScriptRuntime::ScriptTextureSnapshot& snapshot) const;
};
