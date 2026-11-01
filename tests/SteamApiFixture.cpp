#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cwchar>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace {
int mode = 0;
int initializationCount = 0;
int callbackCount = 0;
int shutdownCount = 0;
bool correctEnvironment = false;
} // namespace

extern "C" {

int SteamAPI_InitFlat(char (*error)[1024]) {
    ++initializationCount;
#ifdef _WIN32
    wchar_t appId[16] = {};
    wchar_t gameId[16] = {};
    correctEnvironment = GetEnvironmentVariableW(L"SteamAppId", appId, 16) > 0 &&
                         GetEnvironmentVariableW(L"SteamGameId", gameId, 16) > 0 &&
                         std::wcscmp(appId, L"252530") == 0 &&
                         std::wcscmp(gameId, L"252530") == 0;
#else
    const char* appId = std::getenv("SteamAppId");
    const char* gameId = std::getenv("SteamGameId");
    correctEnvironment = appId && gameId && std::strcmp(appId, "252530") == 0 &&
                         std::strcmp(gameId, "252530") == 0;
#endif
    if (mode == 1 || !correctEnvironment) {
        if (error) {
            std::strcpy(*error, "Fixture initialization failed");
        }
        return 1;
    }
    return 0;
}

#ifndef OPENBUS_FIXTURE_INCOMPLETE
void SteamAPI_RunCallbacks() {
    ++callbackCount;
}
#endif

void SteamAPI_Shutdown() {
    ++shutdownCount;
}

void* SteamAPI_SteamUtils_v011() {
    return mode == 3 ? nullptr : &mode;
}

std::uint32_t SteamAPI_ISteamUtils_GetAppID(void*) {
    return mode == 2 ? 42 : 252530;
}

void FixtureSetMode(int value) {
    mode = value;
}

int FixtureInitializations() {
    return initializationCount;
}

int FixtureCallbacks() {
    return callbackCount;
}

int FixtureShutdowns() {
    return shutdownCount;
}

bool FixtureCorrectEnvironment() {
    return correctEnvironment;
}

} // extern "C"
