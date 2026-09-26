#include "CrashHandler.h"

#ifdef _WIN32

#include <DbgHelp.h>
#include <Windows.h>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <mutex>

namespace {

std::atomic_flag reportInProgress = ATOMIC_FLAG_INIT;

void writeCrashReport(const char* reason, DWORD exceptionCode = 0,
                      void* exceptionAddress = nullptr) {
    if (reportInProgress.test_and_set()) {
        return;
    }

    std::ofstream report("game.crash", std::ios::out | std::ios::trunc);
    if (!report.is_open()) {
        return;
    }

    report << "OpenBus crash report\n"
           << "Reason: " << reason << "\n";
    if (exceptionCode != 0) {
        report << "Exception code: 0x" << std::hex << exceptionCode << std::dec << "\n";
    }
    if (exceptionAddress != nullptr) {
        report << "Exception address: " << exceptionAddress << "\n";
    }

    HANDLE process = GetCurrentProcess();
    SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME);
    if (SymInitialize(process, nullptr, TRUE)) {
        void* frames[64] = {};
        const USHORT frameCount = CaptureStackBackTrace(2, 64, frames, nullptr);
        report << "Stack trace:\n";
        for (USHORT index = 0; index < frameCount; ++index) {
            const DWORD64 address = reinterpret_cast<DWORD64>(frames[index]);
            alignas(SYMBOL_INFO) char symbolBuffer[sizeof(SYMBOL_INFO) + MAX_SYM_NAME] = {};
            auto* symbol = reinterpret_cast<PSYMBOL_INFO>(symbolBuffer);
            symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
            symbol->MaxNameLen = MAX_SYM_NAME;
            DWORD64 displacement = 0;
            if (SymFromAddr(process, address, &displacement, symbol)) {
                report << "  #" << index << " " << symbol->Name << "+0x" << std::hex << displacement
                       << std::dec << "\n";
            } else {
                report << "  #" << index << " 0x" << std::hex << address << std::dec << "\n";
            }
        }
        SymCleanup(process);
    } else {
        report << "Stack trace unavailable (SymInitialize failed).\n";
    }
    report.flush();
}

LONG WINAPI handleUnhandledException(EXCEPTION_POINTERS* exceptionInfo) {
    void* address = exceptionInfo != nullptr && exceptionInfo->ExceptionRecord != nullptr
                        ? exceptionInfo->ExceptionRecord->ExceptionAddress
                        : nullptr;
    const DWORD code = exceptionInfo != nullptr && exceptionInfo->ExceptionRecord != nullptr
                           ? exceptionInfo->ExceptionRecord->ExceptionCode
                           : 0;
    writeCrashReport("unhandled Windows exception", code, address);
    return EXCEPTION_EXECUTE_HANDLER;
}

void handleTerminate() {
    writeCrashReport("std::terminate called");
    std::_Exit(EXIT_FAILURE);
}

void handlePureCall() {
    writeCrashReport("pure virtual function call");
    std::_Exit(EXIT_FAILURE);
}

void handleInvalidParameter(const wchar_t*, const wchar_t*, const wchar_t*, unsigned int,
                            uintptr_t) {
    writeCrashReport("invalid parameter");
    std::_Exit(EXIT_FAILURE);
}

} // namespace

void installCrashHandler() {
    SetUnhandledExceptionFilter(handleUnhandledException);
    std::set_terminate(handleTerminate);
    _set_purecall_handler(handlePureCall);
    _set_invalid_parameter_handler(handleInvalidParameter);
}

#else

void installCrashHandler() {}

#endif