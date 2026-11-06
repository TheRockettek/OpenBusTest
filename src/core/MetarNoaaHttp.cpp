#include "core/MetarNoaa.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winhttp.h>
#else
#include <curl/curl.h>
#include <limits>
#endif

namespace openbus {
namespace {

#ifdef _WIN32
struct HttpHandleDeleter {
    void operator()(void* handle) const {
        if (handle != nullptr) {
            WinHttpCloseHandle(handle);
        }
    }
};
using HttpHandle = std::unique_ptr<void, HttpHandleDeleter>;

[[noreturn]] void httpError(const char* operation) {
    throw std::runtime_error(std::string("NOAA METAR ") + operation + " failed (WinHTTP error " +
                             std::to_string(GetLastError()) + ")");
}
#else
struct CurlDeleter {
    void operator()(CURL* handle) const {
        curl_easy_cleanup(handle);
    }
};

struct ResponseBuffer {
    std::string body;
    std::size_t maximumBytes;
    bool exceededLimit = false;
    bool allocationFailed = false;
};

std::size_t receiveData(char* data, std::size_t size, std::size_t count, void* context) {
    auto& response = *static_cast<ResponseBuffer*>(context);
    if (size != 0 && count > std::numeric_limits<std::size_t>::max() / size) {
        response.exceededLimit = true;
        return 0;
    }
    const std::size_t bytes = size * count;
    if (bytes > response.maximumBytes - response.body.size()) {
        response.exceededLimit = true;
        return 0;
    }
    try {
        response.body.append(data, bytes);
    } catch (const std::bad_alloc&) {
        response.allocationFailed = true;
        return 0;
    }
    return bytes;
}
#endif

std::string fetchNoaaPath(const std::string& path, std::size_t maximumResponseBytes) {
#ifdef _WIN32
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{15};
    HttpHandle session{WinHttpOpen(L"OpenBus METAR/1.0", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
                                   WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0)};
    if (!session) {
        httpError("session creation");
    }
    if (!WinHttpSetTimeouts(session.get(), 5000, 5000, 5000, 5000)) {
        httpError("timeout configuration");
    }
    HttpHandle connection{
        WinHttpConnect(session.get(), L"tgftp.nws.noaa.gov", INTERNET_DEFAULT_HTTPS_PORT, 0)};
    if (!connection) {
        httpError("connection");
    }
    const std::wstring widePath(path.begin(), path.end());
    HttpHandle request{WinHttpOpenRequest(connection.get(), L"GET", widePath.c_str(), nullptr,
                                          WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES,
                                          WINHTTP_FLAG_SECURE)};
    if (!request) {
        httpError("request creation");
    }
    DWORD disabled = WINHTTP_DISABLE_REDIRECTS;
    if (!WinHttpSetOption(request.get(), WINHTTP_OPTION_DISABLE_FEATURE, &disabled,
                          sizeof(disabled))) {
        httpError("redirect configuration");
    }
    if (!WinHttpSendRequest(request.get(), WINHTTP_NO_ADDITIONAL_HEADERS, 0,
                            WINHTTP_NO_REQUEST_DATA, 0, 0, 0) ||
        !WinHttpReceiveResponse(request.get(), nullptr)) {
        httpError("request");
    }
    DWORD status = 0;
    DWORD size = sizeof(status);
    if (!WinHttpQueryHeaders(request.get(), WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                             WINHTTP_HEADER_NAME_BY_INDEX, &status, &size,
                             WINHTTP_NO_HEADER_INDEX)) {
        httpError("status query");
    }
    if (status != 200) {
        throw std::runtime_error("NOAA METAR request returned HTTP " + std::to_string(status));
    }
    std::string body;
    char buffer[4096];
    while (true) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (remaining.count() <= 0) {
            throw std::runtime_error("NOAA METAR request timed out");
        }
        const int timeout = static_cast<int>(std::min<std::int64_t>(remaining.count(), 5000));
        if (!WinHttpSetTimeouts(request.get(), timeout, timeout, timeout, timeout)) {
            httpError("timeout configuration");
        }
        DWORD received = 0;
        if (!WinHttpReadData(request.get(), buffer, sizeof(buffer), &received)) {
            httpError("response read");
        }
        if (received == 0) {
            break;
        }
        if (received > maximumResponseBytes - body.size()) {
            throw std::runtime_error("NOAA METAR response exceeds " +
                                     std::to_string(maximumResponseBytes) + " bytes");
        }
        body.append(buffer, received);
    }
    return body;
#else
    static const CURLcode initialization = curl_global_init(CURL_GLOBAL_DEFAULT);
    if (initialization != CURLE_OK) {
        throw std::runtime_error("NOAA METAR could not initialize libcurl");
    }
    std::unique_ptr<CURL, CurlDeleter> request{curl_easy_init()};
    if (!request) {
        throw std::runtime_error("NOAA METAR could not create a libcurl request");
    }
    const std::string url = "https://tgftp.nws.noaa.gov" + path;
    ResponseBuffer response{{}, maximumResponseBytes};
    const auto setOption = [&](CURLoption option, auto value) {
        const CURLcode result = curl_easy_setopt(request.get(), option, value);
        if (result != CURLE_OK) {
            throw std::runtime_error(std::string("NOAA METAR request configuration failed: ") +
                                     curl_easy_strerror(result));
        }
    };
    setOption(CURLOPT_URL, url.c_str());
    setOption(CURLOPT_USERAGENT, "OpenBus METAR/1.0");
    setOption(CURLOPT_CONNECTTIMEOUT_MS, 5000L);
    setOption(CURLOPT_TIMEOUT_MS, 15000L);
    setOption(CURLOPT_NOSIGNAL, 1L);
    setOption(CURLOPT_FOLLOWLOCATION, 0L);
    setOption(CURLOPT_WRITEFUNCTION, &receiveData);
    setOption(CURLOPT_WRITEDATA, &response);
    const CURLcode result = curl_easy_perform(request.get());
    if (response.exceededLimit) {
        throw std::runtime_error("NOAA METAR response exceeds " +
                                 std::to_string(maximumResponseBytes) + " bytes");
    }
    if (response.allocationFailed) {
        throw std::bad_alloc();
    }
    if (result != CURLE_OK) {
        throw std::runtime_error(std::string("NOAA METAR request failed: ") +
                                 curl_easy_strerror(result));
    }
    long status = 0;
    const CURLcode statusResult = curl_easy_getinfo(request.get(), CURLINFO_RESPONSE_CODE, &status);
    if (statusResult != CURLE_OK) {
        throw std::runtime_error(std::string("NOAA METAR status query failed: ") +
                                 curl_easy_strerror(statusResult));
    }
    if (status != 200) {
        throw std::runtime_error("NOAA METAR request returned HTTP " + std::to_string(status));
    }
    return response.body;
#endif
}

} // namespace

std::string fetchNoaaMetar(std::string_view station) {
    return fetchNoaaPath("/data/observations/metar/stations/" + noaaMetarStation(station) + ".TXT",
                         8192);
}

std::string fetchNoaaMetarDirectory() {
    return fetchNoaaPath("/data/observations/metar/stations/?C=M;O=D", 4 * 1024 * 1024);
}

} // namespace openbus
