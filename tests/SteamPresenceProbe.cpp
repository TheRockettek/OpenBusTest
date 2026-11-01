#include "app/SteamPresence.h"
#include "core/Logger.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool equalsEnvironment(const char* name, const char* expected) {
#ifdef _WIN32
    char* value = nullptr;
    std::size_t length = 0;
    if (_dupenv_s(&value, &length, name) != 0) {
        return false;
    }
    const bool equal = value && std::string(value) == expected;
    std::free(value);
    return equal;
#else
    const char* value = std::getenv(name);
    return value && std::string(value) == expected;
#endif
}

bool environmentUnset(const char* name) {
#ifdef _WIN32
    char* value = nullptr;
    std::size_t length = 0;
    if (_dupenv_s(&value, &length, name) != 0) {
        return false;
    }
    const bool unset = value == nullptr;
    std::free(value);
    return unset;
#else
    return std::getenv(name) == nullptr;
#endif
}

bool setEnvironment(const char* name, const char* value) {
#ifdef _WIN32
    return _putenv_s(name, value ? value : "") == 0;
#else
    return (value ? setenv(name, value, 1) : unsetenv(name)) == 0;
#endif
}

bool setEmptyGameId() {
#ifdef _WIN32
    return SetEnvironmentVariableW(L"SteamGameId", L"") != 0;
#else
    return setenv("SteamGameId", "", 1) == 0;
#endif
}

bool gameIdIsEmpty() {
#ifdef _WIN32
    SetLastError(ERROR_SUCCESS);
    return GetEnvironmentVariableW(L"SteamGameId", nullptr, 0) <= 1 &&
           GetLastError() == ERROR_SUCCESS;
#else
    const char* value = std::getenv("SteamGameId");
    return value && *value == '\0';
#endif
}

void* openFixture(const std::filesystem::path& path) {
#ifdef _WIN32
    return LoadLibraryW(path.c_str());
#else
    return dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
#endif
}

template <typename Function>
Function fixtureFunction(void* library, const char* name) {
#ifdef _WIN32
    return reinterpret_cast<Function>(GetProcAddress(static_cast<HMODULE>(library), name));
#else
    return reinterpret_cast<Function>(dlsym(library, name));
#endif
}

void closeFixture(void* library) {
#ifdef _WIN32
    FreeLibrary(static_cast<HMODULE>(library));
#else
    dlclose(library);
#endif
}

} // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--auto") {
        openbus::app::SteamPresence presence;
        const bool active = presence.initializeFromEnvironment();
        presence.runCallbacks();
        return active ? 0 : 1;
    }
    if (argc == 3 && std::string(argv[1]) == "--real") {
        openbus::app::SteamPresence presence;
        const bool active = presence.initialize(std::filesystem::path(argv[2]));
        presence.runCallbacks();
        return active ? 0 : 1;
    }
    if (argc != 3) {
        std::cerr << "Expected complete and incomplete fixture paths\n";
        return 1;
    }
    const auto sourceFixture = std::filesystem::absolute(argv[1]);
    const auto incompleteFixture = std::filesystem::absolute(argv[2]);
    const auto originalDirectory = std::filesystem::current_path();
    const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto directory = std::filesystem::temp_directory_path() /
                           ("openbus-steam-probe-" + std::to_string(unique));
    const auto runtimeDirectory = directory / "runtime";
#ifdef _WIN32
    const auto apiFilename = sizeof(void*) == 8 ? "steam_api64.dll" : "steam_api.dll";
#else
    const auto apiFilename = "libsteam_api.so";
#endif
    if (!std::filesystem::create_directory(directory) ||
        !std::filesystem::create_directory(runtimeDirectory) ||
        !setEnvironment("OPENBUS_LOG_FILE", (directory / "probe.log").string().c_str()) ||
        !setEnvironment("SteamAppId", "original-app") || !setEnvironment("SteamGameId", nullptr) ||
        !setEnvironment("OPENBUS_STEAM_API_LIBRARY", "ignored-untrusted-library") ||
        !setEnvironment("OPENBUS_STEAM_ROOT", "ignored-untrusted-root") ||
        !setEnvironment("OPENBUS_STEAM_PRESENCE", nullptr)) {
        std::cerr << "Could not set up probe directory or environment\n";
        return 1;
    }
    const auto fixturePath = runtimeDirectory / apiFilename;
    std::filesystem::copy_file(sourceFixture, fixturePath);
    std::filesystem::current_path(directory);
    void* fixture = openFixture(fixturePath);
    if (!fixture) {
        std::cerr << "Could not load fixture\n";
        return 1;
    }
    const auto setMode = fixtureFunction<void (*)(int)>(fixture, "FixtureSetMode");
    const auto initializations = fixtureFunction<int (*)()>(fixture, "FixtureInitializations");
    const auto callbacks = fixtureFunction<int (*)()>(fixture, "FixtureCallbacks");
    const auto shutdowns = fixtureFunction<int (*)()>(fixture, "FixtureShutdowns");
    const auto correctEnvironment = fixtureFunction<bool (*)()>(fixture, "FixtureCorrectEnvironment");
    if (!setMode || !initializations || !callbacks || !shutdowns || !correctEnvironment) {
        std::cerr << "Fixture exports are incomplete\n";
        return 1;
    }

    bool passed = true;
    {
        openbus::app::SteamPresence presence;
        passed &= expect(!presence.initialize({}), "empty library path remains disabled");
        presence.runCallbacks();
        passed &= expect(callbacks() == 0, "inactive callbacks do not run");
        passed &= expect(presence.initializeFromDirectory(runtimeDirectory),
                         "bundled library loads despite obsolete environment overrides");
        passed &= expect(presence.active() && correctEnvironment(), "OMSI environment is used");
        passed &= expect(!presence.initialize(fixturePath), "duplicate initialization rejected");
        presence.runCallbacks();
        presence.runCallbacks();
        passed &= expect(callbacks() == 2, "callbacks pumped");
    }
    passed &= expect(shutdowns() == 1, "destructor shuts down once");
    passed &= expect(equalsEnvironment("SteamAppId", "original-app") &&
                         environmentUnset("SteamGameId"),
                     "original environment restored");
    {
        openbus::app::SteamPresence presence;
        setMode(1);
        passed &= expect(!presence.initialize(fixturePath), "initialization failure reported");
        passed &= expect(!presence.active() && shutdowns() == 1,
                         "failed initialization is not shut down");
        setMode(2);
        passed &= expect(!presence.initialize(fixturePath), "unexpected app ID rejected");
        passed &= expect(shutdowns() == 2, "incorrect app ID is shut down");
        setMode(3);
        passed &= expect(!presence.initialize(fixturePath), "missing SteamUtils rejected");
        passed &= expect(shutdowns() == 3, "missing SteamUtils is shut down");
        setMode(0);
        passed &= expect(!presence.initialize(incompleteFixture), "missing symbol rejected");
        passed &= expect(!presence.initializeFromDirectory(directory / "missing"),
                         "missing bundled library rejected");
        passed &= expect(equalsEnvironment("SteamAppId", "original-app") &&
                             environmentUnset("SteamGameId"),
                         "environment restored after failures");
        const int before = initializations();
        {
            std::ofstream file("steam_appid.txt");
            file << "42\n";
        }
        passed &= expect(!presence.initialize(fixturePath) && initializations() == before,
                         "conflicting app ID file rejected before initialization");
        {
            std::ofstream file("steam_appid.txt");
            file << "252530\n";
        }
        passed &= expect(presence.initialize(fixturePath), "matching app ID file accepted");
        presence.shutdown();
        presence.shutdown();
        passed &= expect(shutdowns() == 4, "explicit shutdown is idempotent");
    }
    {
        openbus::app::SteamPresence presence;
        passed &= expect(setEmptyGameId(), "set empty SteamGameId");
        passed &= expect(presence.initialize(fixturePath), "initialize with empty original game ID");
        presence.shutdown();
        passed &= expect(gameIdIsEmpty(), "empty SteamGameId restored");
    }
    {
        openbus::app::SteamPresence presence;
        const int before = initializations();
        passed &= expect(setEnvironment("OPENBUS_STEAM_PRESENCE", "0"), "disable presence");
        passed &= expect(!presence.initializeFromDirectory(runtimeDirectory) &&
                             initializations() == before,
                         "disabled presence does not load library");
        passed &= expect(!presence.initializeFromEnvironment(),
                         "disabled presence also applies to executable directory");
        passed &= expect(setEnvironment("OPENBUS_STEAM_PRESENCE", "invalid"),
                         "set invalid presence flag");
        passed &= expect(!presence.initializeFromDirectory(runtimeDirectory),
                         "invalid presence flag rejected");
        passed &= expect(setEnvironment("OPENBUS_STEAM_PRESENCE", nullptr),
                         "clear presence flag");
    }
#ifdef _WIN32
    const auto unicodeDirectory = directory / L"steam-\u6d4b\u8bd5";
    std::filesystem::create_directory(unicodeDirectory);
    const auto unicodeFixture = unicodeDirectory / apiFilename;
    std::filesystem::copy_file(sourceFixture, unicodeFixture);
    {
        openbus::app::SteamPresence presence;
        passed &= expect(presence.initializeFromDirectory(unicodeDirectory),
                         "Unicode library directory loads");
    }
    std::filesystem::remove(unicodeFixture);
    std::filesystem::remove(unicodeDirectory);
#endif
    Logger::Flush();
    std::ifstream log(directory / "probe.log");
    const std::string contents{std::istreambuf_iterator<char>(log), std::istreambuf_iterator<char>()};
    passed &= expect(contents.find("Fixture initialization failed") != std::string::npos,
                     "Steam initialization error detail logged");
    log.close();
    closeFixture(fixture);
    std::filesystem::current_path(originalDirectory);
    std::filesystem::remove(directory / "steam_appid.txt");
    std::filesystem::remove(directory / "probe.log");
    std::filesystem::remove(fixturePath);
    std::filesystem::remove(runtimeDirectory);
    std::filesystem::remove(directory);
    return passed ? 0 : 1;
}
