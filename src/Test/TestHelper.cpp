#include "Utility.hpp"

#include <fstream>
#include <mutex>
#include <string>

static std::ostream& getTestResult() {
    static std::ofstream out{ "testResult.log", std::ios::app };
    return out;
}

void appendTestResult(const std::string& message) {
    static std::mutex mutex;
    std::lock_guard guard{ mutex };
    getTestResult() << "========== " << globalConfigName << " ==========" << std::endl << message << std::endl;
}
