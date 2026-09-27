#include "ScriptRuntime.h"

#include "Logger.h"
#include "Variables.h"
#include "PerfTrace.h"
#include "osc/OscConverter.h"

extern "C" {
#include "osc/lauxlib.h"
#include "osc/lua.h"
#include "osc/lualib.h"
}

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <random>
#include <sstream>
#include <unordered_map>

Logger luaLogger = Logger("Lua");

namespace {

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
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
    };

    struct ScriptEnvironment {
        int reference = LUA_NOREF;
    };

    lua_State* state = nullptr;
    VehicleConfig configuration;
    VehicleState& localState;
    SimulationState& sharedState;
    ScriptEnvironment scriptEnvironment;
    std::vector<std::string> errors;
    std::vector<double> floatStack;
    std::vector<std::string> stringStack;
    std::unordered_map<int, ScriptTexture> scriptTextures;

    Impl(const VehicleConfig& source, VehicleState& local, SimulationState& shared)
        : configuration(source), localState(local), sharedState(shared) {
        state = luaL_newstate();
        if (!state) {
            errors.push_back("unable to create Lua state");
            return;
        }
        for (const std::string& variable : configuration.floatVariables) {
            localState.declare(variable);
        }
        for (const std::string& variable : configuration.stringVariables) {
            localStrings.emplace(lower(variable), std::string());
        }
        luaL_openlibs(state);
        registerFunctions();
        loadScripts();
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
        // luaLogger.Log("get_local_var(" + std::string(luaL_checkstring(lua, 1)) + ")");
        Impl* runtime = runtimeFor(lua);
        lua_pushnumber(lua, runtime->localState.get(luaL_checkstring(lua, 1)));
        return 1;
    }

    static int setLocal(lua_State* lua) {
        // luaLogger.Log("set_local_var(" + std::string(luaL_checkstring(lua, 1)) + ", " + std::to_string(numericArgument(lua, 2)) + ")");
        Impl* runtime = runtimeFor(lua);
        runtime->localState.set(luaL_checkstring(lua, 1), numericArgument(lua, 2));
        return 0;
    }

    static int getLocalString(lua_State* lua) {
        // luaLogger.Log("get_local_str(" + std::string(luaL_checkstring(lua, 1)) + ")");
        Impl* runtime = runtimeFor(lua);
        lua_pushstring(lua, runtime->localStrings[lower(luaL_checkstring(lua, 1))].c_str());
        return 1;
    }

    static int setLocalString(lua_State* lua) {
        // luaLogger.Log("set_local_str(" + std::string(luaL_checkstring(lua, 1)) + ", " + std::string(luaL_checkstring(lua, 2)) + ")");
        Impl* runtime = runtimeFor(lua);
        runtime->localStrings[lower(luaL_checkstring(lua, 1))] = luaL_checkstring(lua, 2);
        return 0;
    }

    static int getSystem(lua_State* lua) {
        // luaLogger.Log("get_sys_var(" + std::string(luaL_checkstring(lua, 1)) + ")");
        Impl* runtime = runtimeFor(lua);
        lua_pushnumber(lua, runtime->sharedState.sharedVariables().get(luaL_checkstring(lua, 1)));
        return 1;
    }

    static int setSystem(lua_State* lua) {
        // luaLogger.Log("set_sys_var(" + std::string(luaL_checkstring(lua, 1)) + ", " + std::to_string(numericArgument(lua, 2)) + ")");
        Impl* runtime = runtimeFor(lua);
        runtime->sharedState.sharedVariables().set(luaL_checkstring(lua, 1), numericArgument(lua, 2));
        return 0;
    }

    static int getConstant(lua_State* lua) {
        // luaLogger.Log("get_const(" + std::string(luaL_checkstring(lua, 1)) + ")");
        Impl* runtime = runtimeFor(lua);
        const auto found = runtime->configuration.constants.find(luaL_checkstring(lua, 1));
        lua_pushnumber(lua, found == runtime->configuration.constants.end() ? 0.0 : found->second);
        return 1;
    }

    static int callFunction(lua_State* lua) {
        // luaLogger.Log("call_func(" + std::string(luaL_checkstring(lua, 1)) + ", " + std::to_string(numericArgument(lua, 2)) + ")");
        Impl* runtime = runtimeFor(lua);
        const std::string name = luaL_checkstring(lua, 1);
        const double value = numericArgument(lua, 2);
        for (const ConstantCurve& curve : runtime->configuration.curves) {
            if (curve.name != name || curve.points.empty()) {
                continue;
            }
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
        // luaLogger.Log("_pushf(" + std::to_string(numericArgument(lua, 1)) + ")");
        runtimeFor(lua)->floatStack.push_back(numericArgument(lua, 1));
        return 0;
    }

    static int popFloat(lua_State* lua) {
        // luaLogger.Log("_popf()");
        Impl* runtime = runtimeFor(lua);
        if (runtime->floatStack.empty()) {
            lua_pushnumber(lua, 0.0);
        } else {
            lua_pushnumber(lua, runtime->floatStack.back());
            runtime->floatStack.pop_back();
        }
        return 1;
    }

    static int peekFloat(lua_State* lua) {
        // luaLogger.Log("_peekf(" + std::to_string(runtimeFor(lua)->floatStack.empty() ? 0.0: runtimeFor(lua)->floatStack.back()));
        Impl* runtime = runtimeFor(lua);
        lua_pushnumber(lua, runtime->floatStack.empty() ? 0.0 : runtime->floatStack.back());
        return 1;
    }

    static int pushString(lua_State* lua) {
        // luaLogger.Log("_pushs(" + std::string(luaL_checkstring(lua, 1)) + ")");
        runtimeFor(lua)->stringStack.emplace_back(luaL_checkstring(lua, 1));
        return 0;
    }

    static int popString(lua_State* lua) {
        // luaLogger.Log("_pops()");
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
        // luaLogger.Log("_peeks()");
        Impl* runtime = runtimeFor(lua);
        lua_pushstring(lua,
                       runtime->stringStack.empty() ? "" : runtime->stringStack.back().c_str());
        return 1;
    }

    static int soundTrigger(lua_State* lua) {
        luaLogger.Log("sound_trigger(" + std::string(luaL_checkstring(lua, 1)) + ")");
        return 0;
    }

    static int soundTriggerFile(lua_State* lua) {
        luaLogger.Log("sound_trigger_file(" + std::string(luaL_checkstring(lua, 1)) + ", " + std::string(luaL_checkstring(lua, 2)) + ")");
        return 0;
    }

    static int debug(lua_State* lua) {
        luaLogger.Log("omsi_debug(" + std::string(luaL_checkstring(lua, 1)) + ")");
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

    static int returnSystemFloat(lua_State* lua, double value) {
        runtimeFor(lua)->floatStack.push_back(value);
        return 0;
    }

    static int returnSystemString(lua_State* lua, const char* value = "") {
        runtimeFor(lua)->stringStack.emplace_back(value);
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
        return returnSystemFloat(lua, static_cast<double>(rand()) / RAND_MAX);
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

    static void resizeScriptTexture(ScriptTexture& texture, int requiredWidth,
                                    int requiredHeight) {
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
        if (x < 0 || y < 0) {
            return;
        }
        resizeScriptTexture(texture, x + 1, y + 1);
        const std::size_t offset = pixelOffset(texture, x, y);
        std::copy(texture.color.begin(), texture.color.end(), texture.pixels.begin() + offset);
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
        runtime->scriptTextures[index] = ScriptTexture();
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
            texture.filtered = true;
        }
        return 0;
    }

    static int safeScriptTextureColor(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const std::uint8_t blue = static_cast<std::uint8_t>(std::clamp(popSystemFloat(runtime), 0.0, 255.0));
        const std::uint8_t green = static_cast<std::uint8_t>(std::clamp(popSystemFloat(runtime), 0.0, 255.0));
        const std::uint8_t red = static_cast<std::uint8_t>(std::clamp(popSystemFloat(runtime), 0.0, 255.0));
        const std::uint8_t alpha = static_cast<std::uint8_t>(std::clamp(popSystemFloat(runtime), 0.0, 255.0));
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
            std::copy_n(texture.pixels.begin() + offset, 4, texture.color.begin());
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

    // TODO: Implement STTextOut using the configured font raster data.
    static int safeScriptTextureText(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        discardSystemFloats(runtime, 6);
        discardSystemStrings(runtime, 1);
        return 0;
    }

    static int safeScriptTextureCopyColor(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        const int destination = static_cast<int>(popSystemFloat(runtime));
        const int origin = static_cast<int>(popSystemFloat(runtime));
        scriptTexture(runtime, destination).color = scriptTexture(runtime, origin).color;
        return 0;
    }

    // TODO: Implement STLoadTex with asynchronous texture loading.
    static int safeScriptTextureLoad(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        discardSystemFloats(runtime, 1);
        discardSystemStrings(runtime, 1);
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

    // TODO: Implement font registration and exact internal-name lookup.
    static int safeFontIndex(lua_State* lua) {
        discardSystemStrings(runtimeFor(lua), 1);
        return returnSystemFloat(lua, 0.0);
    }

    // TODO: Implement text measurement using the selected font.
    static int safeTextLength(lua_State* lua) {
        Impl* runtime = runtimeFor(lua);
        discardSystemFloats(runtime, 1);
        discardSystemStrings(runtime, 1);
        return returnSystemFloat(lua, 0.0);
    }

    static Impl* runtimeFor(lua_State* lua) {
        return static_cast<Impl*>(lua_touserdata(lua, lua_upvalueindex(1)));
    }

    void registerFunction(const char* name, lua_CFunction function) {
        lua_pushlightuserdata(state, this);
        lua_pushcclosure(state, function, 1);
        lua_setglobal(state, name);
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

        registerFunction("sys_macro_getterminusindex", safeMissingIndex);
        registerFunction("sys_macro_getterminuscode", safeNumericLookup);
        registerFunction("sys_macro_getterminusstring", safeStringLookup);
        registerFunction("sys_macro_getbusstopindex", safeMissingIndex);
        registerFunction("sys_macro_getbusstopstring", safeStringLookup);
        registerFunction("sys_macro_getrouteindex", safeMissingIndex);
        registerFunction("sys_macro_getrouteterminusindex", safeNumericLookup);
        registerFunction("sys_macro_getbusstopcount", safeNumericLookup);
        registerFunction("sys_macro_getroutebusstopident", safeRouteBusstopIdent);
        registerFunction("sys_macro_getttlinestring", safeCurrentLineNumber);
        registerFunction("sys_macro_getttterminusindex", safeNumericLookup);
        registerFunction("sys_macro_getttbusstopcount", safeNumericLookup);
        registerFunction("sys_macro_getttbusstopindex", safeNumericLookup);
        registerFunction("sys_macro_getttdelay", safeNumericLookup);
        registerFunction("sys_macro_getttbusstopname", safeTicketName);
        registerFunction("sys_macro_getttbusstoparr", safeNumericLookup);
        registerFunction("sys_macro_getttbusstopdep", safeNumericLookup);
        registerFunction("sys_macro_givechangecoin", safeGiveChangeCoin);
        registerFunction("sys_macro_getticketname", safeTicketName);
        registerFunction("sys_macro_getticketvalue", safeNumericLookup);
        registerFunction("sys_macro_gethumancountonpathlink", safeNumericLookup);
        registerFunction("sys_macro_nrspecrandom", safeNrSpecRandom);
        registerFunction("sys_macro_getheightabovepoint", safeGetHeightAbovePoint);
        registerFunction("sys_macro_getdepotstringglobal", safeTicketName);
        registerFunction("sys_macro_gethumancountonseat", safeNumericLookup);
        registerFunction("sys_macro_getarrbusline", safeArrivalString);
        registerFunction("sys_macro_getarrbusterminus", safeArrivalString);
        registerFunction("sys_macro_getarrbustimediff", safeArrivalTime);
        registerFunction("sys_macro_stnewtex", safeScriptTextureNew);
        registerFunction("sys_macro_stlock", safeScriptTextureLock);
        registerFunction("sys_macro_stunlock", safeScriptTextureUnlock);
        registerFunction("sys_macro_stfilter", safeScriptTextureFilter);
        registerFunction("sys_macro_stsetcolor", safeScriptTextureColor);
        registerFunction("sys_macro_stdrawpixel", safeScriptTexturePixel);
        registerFunction("sys_macro_stdrawrect", safeScriptTextureRect);
        registerFunction("sys_macro_sttextout", safeScriptTextureText);
        registerFunction("sys_macro_streadpixel", safeScriptTextureReadPixel);
        registerFunction("sys_macro_stcopycolor", safeScriptTextureCopyColor);
        registerFunction("sys_macro_stloadtex", safeScriptTextureLoad);
        registerFunction("sys_macro_stgetr", safeScriptTextureRed);
        registerFunction("sys_macro_stgetg", safeScriptTextureGreen);
        registerFunction("sys_macro_stgetb", safeScriptTextureBlue);
        registerFunction("sys_macro_stgeta", safeScriptTextureAlpha);
        registerFunction("sys_macro_getfontindex", safeFontIndex);
        registerFunction("sys_macro_textlength", safeTextLength);
    }

    void loadScripts() {
        std::size_t loadedScriptCount = 0;
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
            ++loadedScriptCount;
        }
        luaLogger.Log("Loaded " + std::to_string(loadedScriptCount) +
                      " Lua scripts before init()");
    }

    bool invoke(const char* functionName) {
        openbus::rendering::TraceScope trace("lua", functionName);

        // luaLogger.Log(std::string(functionName) + "()");
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
            luaLogger.Log(std::string(functionName) + " not found");
            lua_pop(state, 1);
        }
        lua_pop(state, 1);
        return invoked;
    }

    void invokeFrame(bool isAiVehicle) {
        openbus::rendering::TraceScope trace("lua", "frame");

        // luaLogger.Log("invokeFrame(" + std::string(isAiVehicle ? "true" : "false") + ")");
        lua_rawgeti(state, LUA_REGISTRYINDEX, scriptEnvironment.reference);
        const char* functionName = "frame";
        lua_getfield(state, -1, "frame_ai");
        if (isAiVehicle && lua_isfunction(state, -1)) {
            functionName = "frame_ai";
        } else {
            lua_pop(state, 1);
            lua_getfield(state, -1, "frame");
        }
        if (lua_isfunction(state, -1) && lua_pcall(state, 0, 0, 0) != 0) {
            errors.push_back(std::string(functionName) + ": " + lua_tostring(state, -1));
            lua_pop(state, 1);
        } else if (!lua_isfunction(state, -1)) {
            lua_pop(state, 1);
        }
        lua_pop(state, 1);
    }

    std::unordered_map<std::string, std::string> localStrings;
};

ScriptRuntime::ScriptRuntime(const VehicleConfig& configuration, VehicleState& localState,
                             SimulationState& sharedState)
    : impl_(std::make_unique<Impl>(configuration, localState, sharedState)) {}

ScriptRuntime::~ScriptRuntime() = default;

bool ScriptRuntime::valid() const {
    return impl_ && impl_->errors.empty();
}

const std::vector<std::string>& ScriptRuntime::errors() const {
    return impl_->errors;
}

void ScriptRuntime::initialize() {
    if (impl_ && impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        impl_->invoke("init");
    }
}

void ScriptRuntime::invokeEntryPoint(const std::string& functionName) {
    if (impl_ && impl_->state && !functionName.empty()) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        impl_->invoke(functionName.c_str());
    }
}

void ScriptRuntime::update(bool isAiVehicle) {
    if (impl_ && impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        impl_->invokeFrame(isAiVehicle);
    }
}
