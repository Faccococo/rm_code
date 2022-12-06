#pragma once
#include "Common.hpp"
#include <exception>
#include <fmt/format.h>

using namespace std::chrono_literals;

void installFPEProbe();
void uninstallFPEProbe();

class ExceptionProbe final {
    const char* mFile;
    const char* mFunction;
    const uint32_t mLine;

    static constexpr auto highLatency = 50ms;
    Clock::time_point mStart;

public:
    ExceptionProbe(const char* file, const char* function, const uint32_t line)
        : mFile{ file }, mFunction{ function }, mLine{ line }, mStart{ Clock::now() } {
#ifdef ARTINXHUB_DEBUG
        installFPEProbe();
#endif
    }
    ExceptionProbe(const ExceptionProbe& rhs) = delete;
    ExceptionProbe& operator=(const ExceptionProbe& rhs) = delete;
    ExceptionProbe(ExceptionProbe&& rhs) = delete;
    ExceptionProbe& operator=(ExceptionProbe&& rhs) = delete;

    ~ExceptionProbe() {
#ifdef ARTINXHUB_DEBUG
        uninstallFPEProbe();

        if(std::uncaught_exceptions()) {
#ifdef ARTINXHUB_WINDOWS
            __debugbreak();
#else
            __builtin_trap();
#endif
        }
#else
        if(Clock::now() - mStart > highLatency) {
            logWarning(fmt::format("High latency detected {} {} {}", mFile, mFunction, mLine));
        }
#endif
    }
};

#define ACTOR_EXCEPTION_PROBE()          \
    ExceptionProbe __probe {             \
        __FILE__, __FUNCTION__, __LINE__ \
    }
