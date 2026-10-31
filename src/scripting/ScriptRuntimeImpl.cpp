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

    const std::filesystem::path path(configuredPath);
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        const std::string error = "unable to open selected HOF file: " + path.string();
        errors.push_back(error);
        log(LogLevel::Warning, error);
        return;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        if (lines.empty() && line.starts_with("\xEF\xBB\xBF")) {
            line.erase(0, 3);
        }
        while (!line.empty() &&
               (line.back() == '\r' || line.back() == '\t' || line.back() == ' ')) {
            line.pop_back();
        }
        lines.push_back(std::move(line));
    }
    if (input.bad()) {
        const std::string error = "failed while reading selected HOF file: " + path.string();
        errors.push_back(error);
        log(LogLevel::Warning, error);
        return;
    }

    const auto sectionName = [](const std::string& value) {
        const std::string normalized = openbus::config::trim(value);
        if (normalized.size() < 3 || normalized.front() != '[' || normalized.back() != ']') {
            return std::string{};
        }
        return openbus::config::lower(normalized.substr(1, normalized.size() - 2));
    };
    const auto isSectionEnd = [&sectionName](const std::string& value) {
        return sectionName(value) == "end";
    };
    const auto nextPayload = [&lines](std::size_t& cursor) -> std::optional<std::string> {
        while (cursor < lines.size()) {
            std::string value = lines[cursor++];
            if (!openbus::config::trim(value).empty()) {
                return value;
            }
        }
        return std::nullopt;
    };
    const auto nextCountedPayload = [&lines](std::size_t& cursor) -> std::optional<std::string> {
        if (cursor >= lines.size()) {
            return std::nullopt;
        }
        return lines[cursor++];
    };
    const auto readCount = [&nextPayload](std::size_t& cursor, std::size_t& count) {
        const std::optional<std::string> value = nextPayload(cursor);
        int parsed = 0;
        if (!value || !openbus::config::parseInt(*value, parsed) || parsed < 0) {
            return false;
        }
        count = static_cast<std::size_t>(parsed);
        return true;
    };
    const auto splitTabs = [](const std::string& value) {
        std::vector<std::string> fields;
        std::size_t start = 0;
        while (true) {
            const std::size_t separator = value.find('\t', start);
            fields.push_back(value.substr(
                start, separator == std::string::npos ? std::string::npos : separator - start));
            if (separator == std::string::npos) {
                return fields;
            }
            start = separator + 1;
        }
    };

    std::size_t terminusStringCount = 0;
    std::size_t busstopStringCount = 0;
    for (std::size_t index = 0; index + 1 < lines.size(); ++index) {
        const std::string directive = openbus::config::lower(openbus::config::trim(lines[index]));
        if (directive != "stringcount_terminus" && directive != "stringcount_busstop") {
            continue;
        }
        std::size_t valueIndex = index + 1;
        while (valueIndex < lines.size() && openbus::config::trim(lines[valueIndex]).empty()) {
            ++valueIndex;
        }
        int count = 0;
        if (valueIndex < lines.size() && openbus::config::parseInt(lines[valueIndex], count) &&
            count >= 0) {
            if (directive == "stringcount_terminus") {
                terminusStringCount = static_cast<std::size_t>(count);
            } else {
                busstopStringCount = static_cast<std::size_t>(count);
            }
        }
    }

    HofData parsed;
    for (std::size_t index = 0; index < lines.size();) {
        const std::string section = sectionName(lines[index]);
        if (section.empty()) {
            ++index;
            continue;
        }
        std::size_t cursor = index + 1;
        if (section == "global_strings") {
            std::size_t count = 0;
            if (readCount(cursor, count)) {
                for (std::size_t stringIndex = 0; stringIndex < count; ++stringIndex) {
                    const std::optional<std::string> value = nextCountedPayload(cursor);
                    if (!value) {
                        break;
                    }
                    parsed.globalStrings.push_back(*value);
                }
            }
        } else if (section == "addterminus" || section == "addterminus_allexit") {
            const std::optional<std::string> codeValue = nextPayload(cursor);
            const std::optional<std::string> ident = nextPayload(cursor);
            int code = 0;
            if (codeValue && ident && openbus::config::parseInt(*codeValue, code)) {
                HofTerminus terminus;
                terminus.code = code;
                for (std::size_t stringIndex = 0; stringIndex < terminusStringCount;
                     ++stringIndex) {
                    const std::optional<std::string> value = nextCountedPayload(cursor);
                    if (!value) {
                        break;
                    }
                    terminus.strings.push_back(*value);
                }
                parsed.termini.push_back(std::move(terminus));
            }
        } else if (section == "addterminus_list") {
            while (cursor < lines.size() && !isSectionEnd(lines[cursor])) {
                const std::string row = lines[cursor++];
                if (openbus::config::trim(row).empty()) {
                    continue;
                }
                const std::vector<std::string> fields = splitTabs(row);
                if (fields.size() < 3) {
                    continue;
                }
                int code = 0;
                if (!openbus::config::parseInt(fields[1], code)) {
                    continue;
                }
                HofTerminus terminus;
                terminus.code = code;
                for (std::size_t stringIndex = 3; stringIndex < fields.size(); ++stringIndex) {
                    terminus.strings.push_back(fields[stringIndex]);
                }
                if (terminusStringCount > 0) {
                    terminus.strings.resize(terminusStringCount);
                }
                parsed.termini.push_back(std::move(terminus));
            }
            if (cursor < lines.size() && isSectionEnd(lines[cursor])) {
                ++cursor;
            }
        } else if (section == "addbusstop") {
            const std::optional<std::string> ident = nextPayload(cursor);
            if (ident) {
                HofBusstop busstop;
                busstop.ident = openbus::config::trim(*ident);
                for (std::size_t stringIndex = 0; stringIndex < busstopStringCount; ++stringIndex) {
                    const std::optional<std::string> value = nextCountedPayload(cursor);
                    if (!value) {
                        break;
                    }
                    busstop.strings.push_back(*value);
                }
                parsed.busstops.push_back(std::move(busstop));
            }
        } else if (section == "addbusstop_list") {
            while (cursor < lines.size() && !isSectionEnd(lines[cursor])) {
                const std::string row = lines[cursor++];
                if (openbus::config::trim(row).empty()) {
                    continue;
                }
                const std::vector<std::string> fields = splitTabs(row);
                HofBusstop busstop;
                busstop.ident = openbus::config::trim(fields.front());
                for (std::size_t stringIndex = 1; stringIndex < fields.size(); ++stringIndex) {
                    busstop.strings.push_back(fields[stringIndex]);
                }
                parsed.busstops.push_back(std::move(busstop));
            }
            if (cursor < lines.size() && isSectionEnd(lines[cursor])) {
                ++cursor;
            }
        } else if (section == "infosystem_trip") {
            const std::optional<std::string> code = nextPayload(cursor);
            const std::optional<std::string> name = nextPayload(cursor);
            const std::optional<std::string> terminusValue = nextPayload(cursor);
            const std::optional<std::string> lineValue = nextPayload(cursor);
            if (code && name && terminusValue && lineValue) {
                HofRoute route;
                route.code = openbus::config::trim(*code);
                route.name = openbus::config::trim(*name);
                route.line = openbus::config::trim(*lineValue);
                route.hasTerminusIndex =
                    openbus::config::parseInt(*terminusValue, route.terminusIndex);
                parsed.routes.push_back(std::move(route));
            }
        } else if (section == "infosystem_busstop_list") {
            std::size_t count = 0;
            if (readCount(cursor, count) && !parsed.routes.empty()) {
                HofRoute& route = parsed.routes.back();
                for (std::size_t stopIndex = 0; stopIndex < count; ++stopIndex) {
                    const std::optional<std::string> value = nextPayload(cursor);
                    if (!value) {
                        break;
                    }
                    route.busstopIdents.push_back(openbus::config::trim(*value));
                }
            }
        }
        index = (std::max)(cursor, index + 1);
    }

    hofData = std::move(parsed);
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
