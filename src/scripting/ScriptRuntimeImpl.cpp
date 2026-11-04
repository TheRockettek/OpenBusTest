// ScriptRuntime::Impl lifecycle: script loading and entry-point invocation.

#include "scripting/ScriptRuntimeImpl.h"

#include <system_error>

Logger scriptRuntimeLogger = Logger("ScriptRuntime");

namespace script_detail {

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
    std::error_code canonicalError;
    std::filesystem::path normalizedPath =
        std::filesystem::weakly_canonical(sourcePath, canonicalError);
    if (canonicalError) {
        canonicalError.clear();
        normalizedPath = std::filesystem::absolute(sourcePath, canonicalError);
    }
    if (canonicalError) {
        normalizedPath = sourcePath.lexically_normal();
    }
    std::string key = normalizedPath.generic_string();
#ifdef _WIN32
    key = lower(std::move(key));
#endif
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

} // namespace script_detail

void ScriptRuntime::Impl::log(const std::string& message) const {
    log(LogLevel::Info, message);
}

void ScriptRuntime::Impl::log(LogLevel level, const std::string& message) const {
    scriptRuntimeLogger.Log(level, "[" + scriptIdentity + "] " + message);
}

void ScriptRuntime::Impl::loadHofData() {
    const std::string configuredPath = localState.getString("hof_file");
    if (configuredPath.empty()) {
        return;
    }

    try {
        hofData = openbus::config::loadHofData(configuredPath);
    } catch (const std::exception& error) {
        errors.push_back(error.what());
        log(LogLevel::Warning, error.what());
        return;
    }
    hasHofData = true;
    log("loaded selected HOF data: termini=" + std::to_string(hofData.termini.size()) +
        ", bus stops=" + std::to_string(hofData.busstops.size()) +
        ", routes=" + std::to_string(hofData.routes.size()));
}
ScriptRuntime::Impl::~Impl() {
    if (state) {
        luaL_unref(state, LUA_REGISTRYINDEX, scriptEnvironment.reference);
        lua_close(state);
    }
}

void ScriptRuntime::Impl::loadScripts() {
    lua_newtable(state);
    lua_newtable(state);
    lua_pushvalue(state, LUA_GLOBALSINDEX);
    lua_setfield(state, -2, "__index");
    lua_setmetatable(state, -2);
    scriptEnvironment.reference = luaL_ref(state, LUA_REGISTRYINDEX);

    for (const std::string& referencedPath : configuration.scripts) {
        std::filesystem::path sourcePath =
            openbus::assets::resolveReferencedPath(configuration.sourcePath, referencedPath);
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

bool ScriptRuntime::Impl::invoke(const char* functionName) {
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

void ScriptRuntime::Impl::invokeFrame(bool isAiVehicle) {
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
