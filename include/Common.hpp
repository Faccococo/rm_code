#pragma once
#include <exception>
#include <filesystem>

namespace fs = std::filesystem;

#if _WIN32
#define ARTINXHUB_WINDOWS
#elif __linux__ && !__ANDROID__
#define ARTINXHUB_LINUX
#else
#error "Unsupported platform"
#endif

class Uncopyable {
public:
    Uncopyable() = default;
    Uncopyable(const Uncopyable&) = delete;
    Uncopyable& operator=(const Uncopyable&) = delete;
    Uncopyable(Uncopyable&&) = default;
    Uncopyable& operator=(Uncopyable&&) = default;
    ~Uncopyable() = default;
};

class Unmovable {
public:
    Unmovable() = default;
    Unmovable(const Unmovable&) = delete;
    Unmovable& operator=(const Unmovable&) = delete;
    Unmovable(Unmovable&&) = delete;
    Unmovable& operator=(Unmovable&&) = delete;
    ~Unmovable() = default;
};

class NotImplemented final : public std::exception {};

void logInfo(std::string_view message);
void logWarning(std::string_view message);
void logError(std::string_view message);
[[noreturn]] void raiseError(std::string_view message);
