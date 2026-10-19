// Lua C API bindings exposed to converted OMSI scripts.

#include "scripting/ScriptRuntimeImpl.h"

int ScriptRuntime::Impl::panicHandler(lua_State* lua) {
    const char* message = lua_tostring(lua, -1);
    const std::string detail = message ? message : "non-string Lua error object";
    const std::string output =
        "Lua panic: " + detail + " (stack=" + std::to_string(lua_gettop(lua)) + ")";
    scriptRuntimeLogger.Log(output);
    std::cerr << output << '\n';
    return 0;
}

ScriptRuntime::Impl* ScriptRuntime::Impl::self(lua_State* lua) {
    return static_cast<Impl*>(lua_touserdata(lua, lua_upvalueindex(1)));
}

float ScriptRuntime::Impl::numericArgument(lua_State* lua, int index) {
    if (lua_isboolean(lua, index)) {
        return lua_toboolean(lua, index) != 0 ? 1.0F : 0.0F;
    }
    return static_cast<float>(luaL_checknumber(lua, index));
}

int ScriptRuntime::Impl::oscFloat32(lua_State* lua) {
    lua_pushnumber(lua, static_cast<float>(luaL_checknumber(lua, 1)));
    return 1;
}

int ScriptRuntime::Impl::getLocal(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    const char* name = luaL_checkstring(lua, 1);
    const float value = runtime->localState.get(name);
    // runtime->log(lua, "get_local_var(" + std::string(name) + "): " + std::to_string(value));
    lua_pushnumber(lua, value);
    return 1;
}

int ScriptRuntime::Impl::setLocal(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    const char* name = luaL_checkstring(lua, 1);
    const float value = numericArgument(lua, 2);
    // runtime->log(lua, "set_local_var(" + std::string(name) + ", " + std::to_string(value) +
    // ")");
    runtime->localState.set(name, value);
    return 0;
}

int ScriptRuntime::Impl::getLocalString(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    // runtime->log(lua, "get_local_str(" + std::string(luaL_checkstring(lua, 1)) + ")");
    const std::string value = runtime->localState.getString(luaL_checkstring(lua, 1));
    lua_pushstring(lua, value.c_str());
    return 1;
}

int ScriptRuntime::Impl::setLocalString(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    // runtime->log(lua, "set_local_str(" + std::string(luaL_checkstring(lua, 1)) + ", " +
    // std::string(luaL_checkstring(lua, 2)) + ")");
    runtime->localState.setString(luaL_checkstring(lua, 1), luaL_checkstring(lua, 2));
    return 0;
}

int ScriptRuntime::Impl::getSystem(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    // runtime->log(lua, "get_sys_var(" + std::string(luaL_checkstring(lua, 1)) + ")");
    lua_pushnumber(lua, runtime->sharedState.sharedVariables().get(luaL_checkstring(lua, 1)));
    return 1;
}

int ScriptRuntime::Impl::setSystem(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    // runtime->log(lua, "set_sys_var(" + std::string(luaL_checkstring(lua, 1)) + ", " +
    // std::to_string(numericArgument(lua, 2)) + ")");
    runtime->sharedState.sharedVariables().set(luaL_checkstring(lua, 1), numericArgument(lua, 2));
    return 0;
}

int ScriptRuntime::Impl::getConstant(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    // runtime->log(lua, "get_const(" + std::string(luaL_checkstring(lua, 1)) + ")");
    const auto found = runtime->configuration.constants.find(luaL_checkstring(lua, 1));
    lua_pushnumber(lua, found == runtime->configuration.constants.end() ? 0.0F : found->second);
    return 1;
}

int ScriptRuntime::Impl::callFunction(lua_State* lua) {
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

int ScriptRuntime::Impl::pushFloat(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    // // runtime->log(lua, "_pushf(" + std::to_string(numericArgument(lua, 1)) + ")");
    runtime->floatStack.push_back(numericArgument(lua, 1));
    return 0;
}

int ScriptRuntime::Impl::popFloat(lua_State* lua) {
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

int ScriptRuntime::Impl::peekFloat(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    // // runtime->log(lua, "_peekf(" + std::to_string(runtime->floatStack.empty() ? 0.0 :
    // runtime->floatStack.back()) + ")");
    lua_pushnumber(lua, runtime->floatStack.empty() ? 0.0 : runtime->floatStack.back());
    return 1;
}

int ScriptRuntime::Impl::pushString(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    // // runtime->log(lua, "_pushs(" + std::string(luaL_checkstring(lua, 1)) + ")");
    runtime->stringStack.emplace_back(luaL_checkstring(lua, 1));
    return 0;
}

int ScriptRuntime::Impl::popString(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    if (runtime->stringStack.empty()) {
        lua_pushliteral(lua, "");
    } else {
        lua_pushstring(lua, runtime->stringStack.back().c_str());
        runtime->stringStack.pop_back();
    }
    return 1;
}

int ScriptRuntime::Impl::peekString(lua_State* lua) {
    // scriptRuntimeLogger.Log("_peeks()");
    Impl* runtime = runtimeFor(lua);
    lua_pushstring(lua, runtime->stringStack.empty() ? "" : runtime->stringStack.back().c_str());
    return 1;
}

int ScriptRuntime::Impl::soundTrigger(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    const std::string name = luaL_checkstring(lua, 1);
    if (runtime->onSoundTrigger) {
        runtime->onSoundTrigger(name, {}, runtime->soundControlValue());
    }
    return 0;
}

int ScriptRuntime::Impl::soundTriggerFile(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    const std::string name = luaL_checkstring(lua, 1);
    const std::string file = luaL_checkstring(lua, 2);
    if (runtime->onSoundTrigger) {
        runtime->onSoundTrigger(name, file, runtime->soundControlValue());
    }
    return 0;
}

int ScriptRuntime::Impl::debug(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    runtime->log("omsi_debug(" + std::string(luaL_checkstring(lua, 1)) + ")");
    return 0;
}

float ScriptRuntime::Impl::popSystemFloat(Impl* runtime) {
    if (runtime->floatStack.empty()) {
        return 0.0F;
    }
    const float value = runtime->floatStack.back();
    runtime->floatStack.pop_back();
    return value;
}

std::string ScriptRuntime::Impl::popSystemString(Impl* runtime) {
    if (runtime->stringStack.empty()) {
        return {};
    }
    std::string value = std::move(runtime->stringStack.back());
    runtime->stringStack.pop_back();
    return value;
}

int ScriptRuntime::Impl::returnSystemFloat(lua_State* lua, float value) {
    Impl* runtime = runtimeFor(lua);
    runtime->floatStack.push_back(value);
    return 0;
}

int ScriptRuntime::Impl::returnSystemString(lua_State* lua, const char* value) {
    Impl* runtime = runtimeFor(lua);
    runtime->stringStack.emplace_back(value);
    return 0;
}

int ScriptRuntime::Impl::safeNumericLookup(lua_State* lua) {
    popSystemFloat(runtimeFor(lua));
    return returnSystemFloat(lua, 0.0);
}

int ScriptRuntime::Impl::safeMissingIndex(lua_State* lua) {
    popSystemFloat(runtimeFor(lua));
    return returnSystemFloat(lua, -1.0);
}

int ScriptRuntime::Impl::safeStringLookup(lua_State* lua) {
    popSystemFloat(runtimeFor(lua));
    popSystemFloat(runtimeFor(lua));
    return returnSystemString(lua);
}

int ScriptRuntime::Impl::safeRouteBusstopIdent(lua_State* lua) {
    popSystemFloat(runtimeFor(lua));
    popSystemFloat(runtimeFor(lua));
    return returnSystemString(lua);
}

int ScriptRuntime::Impl::safeCurrentLineNumber(lua_State* lua) {
    popSystemFloat(runtimeFor(lua));
    return returnSystemFloat(lua, 0.0);
}

int ScriptRuntime::Impl::safeGetHeightAbovePoint(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    popSystemFloat(runtime);
    popSystemFloat(runtime);
    popSystemFloat(runtime);
    return returnSystemFloat(lua, 0.0);
}

int ScriptRuntime::Impl::safeNrSpecRandom(lua_State* lua) {
    const float seed = popSystemFloat(runtimeFor(lua));
    std::minstd_rand rand(static_cast<unsigned int>(seed));
    return returnSystemFloat(lua, static_cast<float>(rand()) /
                                      static_cast<float>(std::minstd_rand::max()));
}

int ScriptRuntime::Impl::safeGiveChangeCoin(lua_State* lua) {
    popSystemFloat(runtimeFor(lua));
    return 0;
}

int ScriptRuntime::Impl::safeTicketName(lua_State* lua) {
    popSystemFloat(runtimeFor(lua));
    return returnSystemString(lua);
}

void ScriptRuntime::Impl::discardSystemFloats(Impl* runtime, std::size_t count) {
    while (count-- > 0) {
        popSystemFloat(runtime);
    }
}

void ScriptRuntime::Impl::discardSystemStrings(Impl* runtime, std::size_t count) {
    while (count-- > 0 && !runtime->stringStack.empty()) {
        runtime->stringStack.pop_back();
    }
}

int ScriptRuntime::Impl::safeArrivalString(lua_State* lua) {
    popSystemFloat(runtimeFor(lua));
    return returnSystemString(lua);
}

int ScriptRuntime::Impl::safeArrivalTime(lua_State* lua) {
    popSystemFloat(runtimeFor(lua));
    return returnSystemFloat(lua, 0.0);
}

int ScriptRuntime::Impl::tracedLuaFunction(lua_State* lua) {
    const auto* binding =
        static_cast<const LuaFunctionBinding*>(lua_touserdata(lua, lua_upvalueindex(2)));
    openbus::rendering::TraceScope trace("lua_callback", binding->name.c_str());
    return binding->function(lua);
}

int ScriptRuntime::Impl::genericSystemMacro(lua_State* lua) {
    Impl* runtime = runtimeFor(lua);
    const auto* binding =
        static_cast<const LuaFunctionBinding*>(lua_touserdata(lua, lua_upvalueindex(2)));
    runtime->executeSystemMacro(binding->name.substr(10));
    return 0;
}

ScriptRuntime::Impl* ScriptRuntime::Impl::runtimeFor(lua_State* lua) {
    return static_cast<Impl*>(lua_touserdata(lua, lua_upvalueindex(1)));
}

void ScriptRuntime::Impl::registerFunction(const char* name, lua_CFunction function) {
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

void ScriptRuntime::Impl::registerSystemMacro(const char* name, lua_CFunction) {
    if (localState.supportsSystemMacro(name)) {
        registerFunction((std::string("sys_macro_") + name).c_str(), genericSystemMacro);
    }
}

void ScriptRuntime::Impl::registerFunctions() {
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
