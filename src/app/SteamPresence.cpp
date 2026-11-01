#include "app/SteamPresence.h"

#include "core/Logger.h"
#include "core/Utf8Path.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__linux__)
#include <dlfcn.h>
#endif

namespace openbus::app {
namespace {

Logger steamLog("Steam");

#if defined(_WIN32) || defined(__linux__)
using EnvironmentString = std::filesystem::path::string_type;

#ifdef _WIN32
constexpr const wchar_t* APP_ID = L"252530";
constexpr const wchar_t* APP_ID_NAME = L"SteamAppId";
constexpr const wchar_t* GAME_ID_NAME = L"SteamGameId";
constexpr const wchar_t* LIBRARY_NAME = L"OPENBUS_STEAM_API_LIBRARY";
constexpr const wchar_t* PRESENCE_NAME = L"OPENBUS_STEAM_PRESENCE";
constexpr const wchar_t* ROOT_NAME = L"OPENBUS_STEAM_ROOT";
constexpr const wchar_t* API_FILENAME =
    sizeof(void*) == 8 ? L"steam_api64.dll" : L"steam_api.dll";

bool readEnvironment(const wchar_t* name, std::optional<EnvironmentString>& value) {
    SetLastError(ERROR_SUCCESS);
    const DWORD length = GetEnvironmentVariableW(name, nullptr, 0);
    if (length == 0) {
        const DWORD error = GetLastError();
        if (error == ERROR_ENVVAR_NOT_FOUND) {
            value.reset();
            return true;
        }
        if (error == ERROR_SUCCESS) {
            value = EnvironmentString{};
            return true;
        }
        return false;
    }
    std::vector<wchar_t> buffer(length);
    SetLastError(ERROR_SUCCESS);
    const DWORD copied = GetEnvironmentVariableW(name, buffer.data(), length);
    if ((copied == 0 && GetLastError() != ERROR_SUCCESS) || copied >= length) {
        return false;
    }
    value = EnvironmentString(buffer.data(), copied);
    return true;
}

bool writeEnvironment(const wchar_t* name, const std::optional<EnvironmentString>& value) {
    if (_wputenv_s(name, value ? value->c_str() : L"") != 0) {
        return false;
    }
    return !value || !value->empty() || SetEnvironmentVariableW(name, L"") != 0;
}

void* loadLibrary(const std::filesystem::path& path, std::string& error) {
    std::error_code absoluteError;
    const auto absolutePath = std::filesystem::absolute(path, absoluteError);
    if (absoluteError) {
        error = absoluteError.message();
        return nullptr;
    }
    HMODULE library = LoadLibraryExW(absolutePath.c_str(), nullptr,
                                     LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR |
                                         LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
    if (!library) {
        error = std::system_category().message(static_cast<int>(GetLastError()));
    }
    return library;
}

template <typename Function>
Function loadFunction(void* library, const char* name) {
    return reinterpret_cast<Function>(GetProcAddress(static_cast<HMODULE>(library), name));
}

void closeLibrary(void* library) {
    if (!FreeLibrary(static_cast<HMODULE>(library))) {
        steamLog.Warning("Could not unload Steam API library: " +
                         std::system_category().message(static_cast<int>(GetLastError())));
    }
}

std::optional<std::filesystem::path> executableDirectory() {
    std::vector<wchar_t> buffer(512);
    while (buffer.size() <= 32768) {
        const DWORD length = GetModuleFileNameW(nullptr, buffer.data(),
                                                static_cast<DWORD>(buffer.size()));
        if (length == 0) {
            steamLog.Warning("Could not locate OpenBus executable: " +
                             std::system_category().message(static_cast<int>(GetLastError())));
            return std::nullopt;
        }
        if (length < buffer.size()) {
            return std::filesystem::path(std::wstring(buffer.data(), length)).parent_path();
        }
        buffer.resize(buffer.size() * 2);
    }
    steamLog.Warning("OpenBus executable path exceeds the supported Windows path length");
    return std::nullopt;
}
#else
constexpr const char* APP_ID = "252530";
constexpr const char* APP_ID_NAME = "SteamAppId";
constexpr const char* GAME_ID_NAME = "SteamGameId";
constexpr const char* LIBRARY_NAME = "OPENBUS_STEAM_API_LIBRARY";
constexpr const char* PRESENCE_NAME = "OPENBUS_STEAM_PRESENCE";
constexpr const char* ROOT_NAME = "OPENBUS_STEAM_ROOT";
constexpr const char* API_FILENAME = "libsteam_api.so";

bool readEnvironment(const char* name, std::optional<EnvironmentString>& output) {
    const char* value = std::getenv(name);
    output = value ? std::optional<EnvironmentString>(value) : std::nullopt;
    return true;
}

bool writeEnvironment(const char* name, const std::optional<EnvironmentString>& value) {
    return (value ? setenv(name, value->c_str(), 1) : unsetenv(name)) == 0;
}

void* loadLibrary(const std::filesystem::path& path, std::string& error) {
    void* library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!library) {
        const char* detail = dlerror();
        error = detail ? detail : "unknown dynamic loader error";
    }
    return library;
}

template <typename Function>
Function loadFunction(void* library, const char* name) {
    return reinterpret_cast<Function>(dlsym(library, name));
}

void closeLibrary(void* library) {
    if (dlclose(library) != 0) {
        const char* detail = dlerror();
        steamLog.Warning(std::string("Could not unload Steam API library: ") +
                         (detail ? detail : "unknown dynamic loader error"));
    }
}

std::optional<std::filesystem::path> executableDirectory() {
    std::error_code error;
    const auto executable = std::filesystem::read_symlink("/proc/self/exe", error);
    if (error) {
        steamLog.Warning("Could not locate OpenBus executable: " + error.message());
        return std::nullopt;
    }
    return executable.parent_path();
}
#endif
#endif

} // namespace

SteamPresence::~SteamPresence() {
    shutdown();
}

bool SteamPresence::initializeFromEnvironment() {
#if defined(_WIN32) || defined(__linux__)
    if constexpr (sizeof(void*) != 8) {
        steamLog.Warning("Steam presence is unavailable for 32-bit OpenBus builds");
        return false;
    }
    const auto directory = executableDirectory();
    return directory ? initializeFromDirectory(*directory) : false;
#else
    steamLog.Warning("Steam presence is supported only on Windows and Linux");
    return false;
#endif
}

bool SteamPresence::initializeFromDirectory(const std::filesystem::path& directory) {
#if defined(_WIN32) || defined(__linux__)
    std::optional<EnvironmentString> enabled;
    if (!readEnvironment(PRESENCE_NAME, enabled)) {
        steamLog.Warning("Could not read OPENBUS_STEAM_PRESENCE");
        return false;
    }
    if (enabled && !enabled->empty()) {
        std::string flag = openbus::pathToUtf8(std::filesystem::path(*enabled));
        std::transform(flag.begin(), flag.end(), flag.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        if (flag == "0" || flag == "false" || flag == "off") {
            steamLog.Log("Steam presence disabled by OPENBUS_STEAM_PRESENCE");
            return false;
        }
        if (flag != "1" && flag != "true" && flag != "on") {
            steamLog.Warning("Invalid OPENBUS_STEAM_PRESENCE; expected 0, 1, false, true, off, "
                             "or on");
            return false;
        }
    }
    std::optional<EnvironmentString> obsoleteLibrary;
    if (!readEnvironment(LIBRARY_NAME, obsoleteLibrary)) {
        steamLog.Warning("Could not read OPENBUS_STEAM_API_LIBRARY");
        return false;
    }
    std::optional<EnvironmentString> obsoleteRoot;
    if (!readEnvironment(ROOT_NAME, obsoleteRoot)) {
        steamLog.Warning("Could not read OPENBUS_STEAM_ROOT");
        return false;
    }
    if ((obsoleteLibrary && !obsoleteLibrary->empty()) || (obsoleteRoot && !obsoleteRoot->empty())) {
        steamLog.Warning("OPENBUS_STEAM_API_LIBRARY and OPENBUS_STEAM_ROOT are no longer used; "
                         "loading the bundled Steamworks runtime");
    }
    return initialize(directory / API_FILENAME);
#else
    (void)directory;
    steamLog.Warning("Steam presence is supported only on Windows and Linux");
    return false;
#endif
}

bool SteamPresence::initialize(const std::filesystem::path& libraryPath) {
    if (libraryPath.empty()) {
        return false;
    }
    if (active_ || library_ != nullptr) {
        steamLog.Warning("Steam integration was already initialized");
        return false;
    }
#if defined(_WIN32) || defined(__linux__)
    const std::string libraryPathCopy = openbus::pathToUtf8(libraryPath);
    std::error_code error;
    const bool hasAppIdFile = std::filesystem::exists("steam_appid.txt", error);
    if (error) {
        steamLog.Warning("Cannot inspect steam_appid.txt: " + error.message());
        return false;
    }
    if (hasAppIdFile) {
        std::ifstream input("steam_appid.txt");
        std::string appId;
        std::string extra;
        if (!(input >> appId) || (input >> extra) || appId != "252530") {
            steamLog.Warning("Steam integration requires steam_appid.txt, if present, to contain "
                             "only 252530; the file was not changed");
            return false;
        }
    }
    if (!readEnvironment(APP_ID_NAME, previousAppId_) ||
        !readEnvironment(GAME_ID_NAME, previousGameId_)) {
        steamLog.Warning("Could not read the previous Steam app ID environment");
        return false;
    }
    environmentChanged_ = true;
    if (!writeEnvironment(APP_ID_NAME, EnvironmentString(APP_ID)) ||
        !writeEnvironment(GAME_ID_NAME, EnvironmentString(APP_ID))) {
        restoreEnvironment();
        steamLog.Warning("Could not set the Steam app ID environment for OMSI 2");
        return false;
    }
    std::string loadError;
    library_ = loadLibrary(libraryPath, loadError);
    if (!library_) {
        restoreEnvironment();
        steamLog.Warning("Could not load native Steam API library '" + libraryPathCopy +
                         "': " + loadError);
        return false;
    }
    init_ = loadFunction<int (*)(SteamError*)>(library_, "SteamAPI_InitFlat");
    callbacks_ = loadFunction<void (*)()>(library_, "SteamAPI_RunCallbacks");
    shutdown_ = loadFunction<void (*)()>(library_, "SteamAPI_Shutdown");
    const auto getUtils = loadFunction<void* (*)()>(library_, "SteamAPI_SteamUtils_v011");
    const auto getAppId =
        loadFunction<std::uint32_t (*)(void*)>(library_, "SteamAPI_ISteamUtils_GetAppID");
    if (!init_ || !callbacks_ || !shutdown_ || !getUtils || !getAppId) {
        shutdown();
        steamLog.Warning("Steam API library is missing required initialization, callback, "
                         "shutdown, or SteamUtils v011 app ID exports: " + libraryPathCopy);
        return false;
    }
    SteamError detail = {};
    const int result = init_(&detail);
    detail[sizeof(detail) - 1] = '\0';
    if (result != 0) {
        shutdown();
        steamLog.Warning("Steam API initialization failed for app 252530 (result " +
                         std::to_string(result) + "): " +
                         (detail[0] ? std::string(detail) :
                                      "check that Steam is running under the same user and the "
                                      "account owns OMSI 2"));
        return false;
    }
    active_ = true;
    void* utils = getUtils();
    const std::uint32_t appId = utils ? getAppId(utils) : 0;
    if (appId != 252530) {
        shutdown();
        steamLog.Warning("Steam API returned app ID " + std::to_string(appId) +
                         " instead of 252530; Steam integration was shut down");
        return false;
    }
    steamLog.Log("Steam API initialized for OMSI 2 (app 252530) using " +
                 libraryPathCopy);
    return true;
#else
    steamLog.Warning("Steam presence is supported only on Windows and Linux");
    return false;
#endif
}

void SteamPresence::runCallbacks() const {
    if (active_) {
        callbacks_();
    }
}

void SteamPresence::shutdown() {
#if defined(_WIN32) || defined(__linux__)
    if (active_) {
        shutdown_();
        active_ = false;
        steamLog.Log("Steam API shut down");
    }
    if (library_) {
        closeLibrary(library_);
        library_ = nullptr;
    }
    init_ = nullptr;
    callbacks_ = nullptr;
    shutdown_ = nullptr;
    restoreEnvironment();
#endif
}

void SteamPresence::restoreEnvironment() {
#if defined(_WIN32) || defined(__linux__)
    if (!environmentChanged_) {
        return;
    }
    if (!writeEnvironment(APP_ID_NAME, previousAppId_)) {
        steamLog.Warning("Could not restore environment variable SteamAppId");
    }
    if (!writeEnvironment(GAME_ID_NAME, previousGameId_)) {
        steamLog.Warning("Could not restore environment variable SteamGameId");
    }
    environmentChanged_ = false;
#endif
}

} // namespace openbus::app
