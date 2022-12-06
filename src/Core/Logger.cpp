#include "Common.hpp"
#include <iostream>
#include <mutex>

static std::mutex logMutex;

void logInfo(const std::string_view message) {
    std::lock_guard guard{ logMutex };
    std::cerr << "[INFO] " << message << std::endl;
}

void logWarning(const std::string_view message) {
    std::lock_guard guard{ logMutex };
    std::cerr << "[WARNING] " << message << std::endl;
}

void logError(const std::string_view message) {
    std::lock_guard guard{ logMutex };
    std::cerr << "[ERROR] " << message << std::endl;
}

[[noreturn]] void raiseError(const std::string_view message) {
    {
        std::lock_guard guard{ logMutex };
        std::cerr << "[ERROR] " << message << std::endl;
    }

    if(!std::uncaught_exceptions())
        throw std::runtime_error{ std::string{ message } };

    std::rethrow_exception(std::current_exception());
}
