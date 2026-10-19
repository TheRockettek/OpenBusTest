#include "app/CrashHandler.h"

#ifdef _WIN32

#include "core/Logger.h"

#include <Windows.h>
#include <DbgHelp.h>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <mutex>

Logger crashLog = Logger("Crash");

namespace {

std::atomic_flag reportInProgress = ATOMIC_FLAG_INIT;

const char* exceptionName(DWORD code) {
    switch (code) {
    case EXCEPTION_ACCESS_VIOLATION:
        return "access violation";
    case EXCEPTION_ILLEGAL_INSTRUCTION:
        return "illegal instruction";
    case EXCEPTION_STACK_OVERFLOW:
        return "stack overflow";
    case EXCEPTION_INT_DIVIDE_BY_ZERO:
        return "integer divide by zero";
    case EXCEPTION_FLT_DIVIDE_BY_ZERO:
        return "floating-point divide by zero";
    default:
        return "unknown exception";
    }
}

void writeCrashReport(const char* reason, DWORD exceptionCode = 0, void* exceptionAddress = nullptr,
                      const EXCEPTION_POINTERS* exceptionInfo = nullptr) {
    if (reportInProgress.test_and_set()) {
        return;
    }

    try {
        crashLog.Log("Crash detected: " + std::string(reason));

        std::ofstream report("game.crash", std::ios::out | std::ios::trunc);
        if (!report.is_open()) {
            return;
        }

        report << "OpenBus crash report\n"
               << "Reason: " << reason << "\n";
        if (exceptionCode != 0) {
            report << "Exception: " << exceptionName(exceptionCode) << " (0x" << std::hex
                   << exceptionCode << std::dec << ")\n";
        }
        if (exceptionAddress != nullptr) {
            report << "Exception address: " << exceptionAddress << "\n";
        }
        if (exceptionInfo != nullptr && exceptionInfo->ExceptionRecord != nullptr &&
            exceptionCode == EXCEPTION_ACCESS_VIOLATION &&
            exceptionInfo->ExceptionRecord->NumberParameters >= 2) {
            const ULONG_PTR operation = exceptionInfo->ExceptionRecord->ExceptionInformation[0];
            const ULONG_PTR target = exceptionInfo->ExceptionRecord->ExceptionInformation[1];
            report << "Access: "
                   << (operation == 0   ? "read"
                       : operation == 1 ? "write"
                                        : "execute")
                   << " at " << reinterpret_cast<const void*>(target) << "\n";
        }

        if (exceptionInfo != nullptr && exceptionInfo->ExceptionRecord != nullptr) {
            const EXCEPTION_RECORD& record = *exceptionInfo->ExceptionRecord;
            report << "Exception flags: 0x" << std::hex << record.ExceptionFlags << std::dec
                   << "\n";
            report << "Exception parameters: " << record.NumberParameters << "\n";
            for (ULONG index = 0; index < record.NumberParameters; ++index) {
                report << "  parameter[" << index << "]: 0x" << std::hex
                       << record.ExceptionInformation[index] << std::dec << "\n";
            }
        }

        if (exceptionInfo != nullptr && exceptionInfo->ContextRecord != nullptr) {
            const CONTEXT& context = *exceptionInfo->ContextRecord;
            report << "Thread ID: " << GetCurrentThreadId() << "\n"
                   << "Registers: RIP=0x" << std::hex << context.Rip << " RSP=0x" << context.Rsp
                   << " RBP=0x" << context.Rbp << " RAX=0x" << context.Rax << " RBX=0x"
                   << context.Rbx << " RCX=0x" << context.Rcx << " RDX=0x" << context.Rdx
                   << " RSI=0x" << context.Rsi << " RDI=0x" << context.Rdi << " R8=0x" << context.R8
                   << " R9=0x" << context.R9 << " R10=0x" << context.R10 << " R11=0x" << context.R11
                   << " R12=0x" << context.R12 << " R13=0x" << context.R13 << " R14=0x"
                   << context.R14 << " R15=0x" << context.R15 << std::dec << "\n";
        }

        HANDLE process = GetCurrentProcess();
        SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
        if (SymInitialize(process, nullptr, TRUE)) {
            CONTEXT context = {};
            STACKFRAME64 frame = {};
            DWORD machineType = IMAGE_FILE_MACHINE_AMD64;
            if (exceptionInfo != nullptr && exceptionInfo->ContextRecord != nullptr) {
                context = *exceptionInfo->ContextRecord;
                frame.AddrPC.Offset = context.Rip;
                frame.AddrFrame.Offset = context.Rbp;
                frame.AddrStack.Offset = context.Rsp;
            } else {
                RtlCaptureContext(&context);
                frame.AddrPC.Offset = context.Rip;
                frame.AddrFrame.Offset = context.Rbp;
                frame.AddrStack.Offset = context.Rsp;
            }
            frame.AddrPC.Mode = AddrModeFlat;
            frame.AddrFrame.Mode = AddrModeFlat;
            frame.AddrStack.Mode = AddrModeFlat;
            report << "Stack trace:\n";
            for (int index = 0; index < 64; ++index) {
                const DWORD64 address = frame.AddrPC.Offset;
                if (address == 0) {
                    break;
                }
                alignas(SYMBOL_INFO) char symbolBuffer[sizeof(SYMBOL_INFO) + MAX_SYM_NAME] = {};
                auto* symbol = reinterpret_cast<PSYMBOL_INFO>(symbolBuffer);
                symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
                symbol->MaxNameLen = MAX_SYM_NAME;
                DWORD64 displacement = 0;
                IMAGEHLP_LINE64 line = {};
                line.SizeOfStruct = sizeof(line);
                DWORD lineDisplacement = 0;
                IMAGEHLP_MODULE64 module = {};
                module.SizeOfStruct = sizeof(module);
                const bool hasModule = SymGetModuleInfo64(process, address, &module) != FALSE;
                report << "  #" << index << " 0x" << std::hex << address << std::dec;
                if (SymFromAddr(process, address, &displacement, symbol)) {
                    report << " " << (hasModule ? module.ModuleName : "?") << "!" << symbol->Name
                           << "+0x" << std::hex << displacement << std::dec;
                    if (SymGetLineFromAddr64(process, address, &lineDisplacement, &line)) {
                        report << " (" << line.FileName << ":" << line.LineNumber << ")";
                    }
                }
                if (hasModule) {
                    report << " [" << module.ImageName << "]";
                }
                report << "\n";
                const DWORD64 previousAddress = frame.AddrPC.Offset;
                if (!StackWalk64(machineType, process, GetCurrentThread(), &frame, &context,
                                 nullptr, SymFunctionTableAccess64, SymGetModuleBase64, nullptr) ||
                    frame.AddrPC.Offset == previousAddress) {
                    break;
                }
            }
            SymCleanup(process);
        } else {
            report << "Stack trace unavailable (SymInitialize failed, error " << GetLastError()
                   << ").\n";
        }
        report.flush();

        HANDLE dumpFile = CreateFileW(L"game.crash.dmp", GENERIC_WRITE, FILE_SHARE_READ, nullptr,
                                      CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (dumpFile != INVALID_HANDLE_VALUE) {
            MINIDUMP_EXCEPTION_INFORMATION dumpException = {};
            dumpException.ThreadId = GetCurrentThreadId();
            dumpException.ExceptionPointers = const_cast<EXCEPTION_POINTERS*>(exceptionInfo);
            dumpException.ClientPointers = FALSE;
            const MINIDUMP_TYPE dumpType =
                static_cast<MINIDUMP_TYPE>(MiniDumpWithIndirectlyReferencedMemory |
                                           MiniDumpScanMemory | MiniDumpWithThreadInfo);
            MiniDumpWriteDump(process, GetCurrentProcessId(), dumpFile, dumpType,
                              exceptionInfo != nullptr ? &dumpException : nullptr, nullptr,
                              nullptr);
            CloseHandle(dumpFile);
        }
    } catch (...) {
        // Crash reporting must never replace or obscure the original failure.
    }
}

LONG WINAPI handleUnhandledException(EXCEPTION_POINTERS* exceptionInfo) {
    void* address = exceptionInfo != nullptr && exceptionInfo->ExceptionRecord != nullptr
                        ? exceptionInfo->ExceptionRecord->ExceptionAddress
                        : nullptr;
    const DWORD code = exceptionInfo != nullptr && exceptionInfo->ExceptionRecord != nullptr
                           ? exceptionInfo->ExceptionRecord->ExceptionCode
                           : 0;
    writeCrashReport("unhandled Windows exception", code, address, exceptionInfo);
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

#if defined(__unix__) || defined(__APPLE__)

#include <execinfo.h>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <exception>
#include <fcntl.h>
#include <signal.h>
#include <ucontext.h>
#if defined(__linux__)
#include <sys/syscall.h>
#endif
#include <unistd.h>

namespace {

volatile sig_atomic_t crashReportInProgress = 0;
alignas(16) unsigned char alternateSignalStack[64 * 1024];

void writeAll(int descriptor, const char* data, std::size_t size) {
    while (size != 0) {
        const ssize_t written = write(descriptor, data, size);
        if (written > 0) {
            data += written;
            size -= static_cast<std::size_t>(written);
        } else if (written < 0 && errno == EINTR) {
            continue;
        } else {
            break;
        }
    }
}

void writeCString(int descriptor, const char* text) {
    std::size_t size = 0;
    while (text[size] != '\0') {
        ++size;
    }
    writeAll(descriptor, text, size);
}

void writeHex(int descriptor, const char* label, std::uint64_t value) {
    char digits[16];
    std::size_t count = 0;
    do {
        const unsigned int digit = static_cast<unsigned int>(value & 0x0fU);
        digits[count++] = static_cast<char>(digit < 10 ? '0' + digit : 'a' + digit - 10);
        value >>= 4U;
    } while (value != 0);
    writeCString(descriptor, label);
    writeCString(descriptor, "0x");
    while (count != 0) {
        writeAll(descriptor, &digits[--count], 1);
    }
    writeCString(descriptor, "\n");
}

void writeDecimal(int descriptor, const char* label, unsigned long value) {
    char digits[3 * sizeof(value) + 1];
    std::size_t count = 0;
    do {
        digits[count++] = static_cast<char>('0' + value % 10UL);
        value /= 10UL;
    } while (value != 0);
    writeCString(descriptor, label);
    while (count != 0) {
        writeAll(descriptor, &digits[--count], 1);
    }
    writeCString(descriptor, "\n");
}

const char* signalName(int signalNumber) {
    switch (signalNumber) {
    case SIGABRT:
        return "SIGABRT";
    case SIGFPE:
        return "SIGFPE";
    case SIGILL:
        return "SIGILL";
    case SIGSEGV:
        return "SIGSEGV";
#ifdef SIGBUS
    case SIGBUS:
        return "SIGBUS";
#endif
    default:
        return "unknown signal";
    }
}

void handlePosixSignal(int signalNumber, siginfo_t* signalInfo, void* rawContext) {
    if (crashReportInProgress != 0) {
        _Exit(128 + signalNumber);
    }
    crashReportInProgress = 1;

    const int descriptor = open("game.crash", O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (descriptor >= 0) {
        writeCString(descriptor, "OpenBus fatal signal\nSignal: ");
        writeCString(descriptor, signalName(signalNumber));
        writeCString(descriptor, "\n");
        if (signalInfo != nullptr) {
            writeDecimal(descriptor,
                         "Signal code: ", static_cast<unsigned long>(signalInfo->si_code));
            if (signalNumber == SIGSEGV
#ifdef SIGBUS
                || signalNumber == SIGBUS
#endif
                || signalNumber == SIGILL || signalNumber == SIGFPE) {
                writeHex(descriptor,
                         "Fault address: ", reinterpret_cast<std::uintptr_t>(signalInfo->si_addr));
            }
        }
        writeDecimal(descriptor, "Process ID: ", static_cast<unsigned long>(getpid()));
#if defined(__linux__)
        writeDecimal(descriptor, "Thread ID: ", static_cast<unsigned long>(syscall(SYS_gettid)));
#endif

        const auto* context = static_cast<const ucontext_t*>(rawContext);
        std::uintptr_t instructionPointer = 0;
        std::uintptr_t stackPointer = 0;
        std::uintptr_t framePointer = 0;
#if defined(__x86_64__) && defined(REG_RIP) && defined(REG_RSP) && defined(REG_RBP)
        if (context != nullptr) {
            instructionPointer = static_cast<std::uintptr_t>(context->uc_mcontext.gregs[REG_RIP]);
            stackPointer = static_cast<std::uintptr_t>(context->uc_mcontext.gregs[REG_RSP]);
            framePointer = static_cast<std::uintptr_t>(context->uc_mcontext.gregs[REG_RBP]);
        }
#elif defined(__aarch64__)
        if (context != nullptr) {
            instructionPointer = static_cast<std::uintptr_t>(context->uc_mcontext.pc);
            stackPointer = static_cast<std::uintptr_t>(context->uc_mcontext.sp);
            framePointer = static_cast<std::uintptr_t>(context->uc_mcontext.regs[29]);
        }
#endif
        if (instructionPointer != 0) {
            writeHex(descriptor, "Instruction pointer: ", instructionPointer);
            writeHex(descriptor, "Stack pointer: ", stackPointer);
            writeHex(descriptor, "Frame pointer: ", framePointer);
        }

        writeCString(descriptor, "Best-effort stack trace (signal handlers limit unwinding):\n");
        void* frames[64] = {};
        const int frameCount = backtrace(frames, 64);
        backtrace_symbols_fd(frames, frameCount, descriptor);
        close(descriptor);
    }
    if (signalNumber == SIGABRT) {
        abort();
    }
    // SA_RESETHAND restores the default action before this handler runs. For a
    // synchronous fault, returning re-executes the faulting instruction and
    // lets the OS produce a normal core dump when core dumps are enabled.
}

void handleTerminate() {
    static constexpr char message[] = "OpenBus terminated unexpectedly; stack trace:\n";
    const int descriptor = open("game.crash", O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (descriptor >= 0) {
        writeAll(descriptor, message, sizeof(message) - 1);
        void* frames[64] = {};
        const int frameCount = backtrace(frames, 64);
        backtrace_symbols_fd(frames, frameCount, descriptor);
        close(descriptor);
    }
    _Exit(EXIT_FAILURE);
}

} // namespace

void installCrashHandler() {
    stack_t alternateStack = {};
    alternateStack.ss_sp = alternateSignalStack;
    alternateStack.ss_size = sizeof(alternateSignalStack);
    sigaltstack(&alternateStack, nullptr);

    struct sigaction action = {};
    action.sa_sigaction = handlePosixSignal;
    sigfillset(&action.sa_mask);
    action.sa_flags = SA_SIGINFO | SA_ONSTACK | SA_RESETHAND;
    sigaction(SIGABRT, &action, nullptr);
    sigaction(SIGFPE, &action, nullptr);
    sigaction(SIGILL, &action, nullptr);
    sigaction(SIGSEGV, &action, nullptr);
#ifdef SIGBUS
    sigaction(SIGBUS, &action, nullptr);
#endif
    std::set_terminate(handleTerminate);
}

#else

void installCrashHandler() {}

#endif

#endif