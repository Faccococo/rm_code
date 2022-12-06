#pragma once
#include "DataDesc.hpp"
#include "SuppressWarningBegin.hpp"

#include <caf/actor_addr.hpp>

#include "SuppressWarningEnd.hpp"
#include <chrono>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

using TimePoint = Clock::time_point;
using Duration = Clock::duration;

class SynchronizedClock final {
    std::optional<TimePoint> mSimulationTime;

public:
    void setSimulationTime(TimePoint tp);
    [[nodiscard]] TimePoint now() const;
    static SynchronizedClock& instance();
};

class Timer final {
    std::mutex mMutex;
    struct TimerInfo final {
        caf::actor_addr address;
        TimePoint deadline;
        Duration duration;
        bool operator<(const TimerInfo& rhs) const noexcept {
            return deadline > rhs.deadline;
        }
    };
    std::priority_queue<TimerInfo> mTimers;
    std::thread mThread;

public:
    Timer() = default;
    ~Timer() = default;
    Timer(const Timer& rhs) = delete;
    Timer(Timer&& rhs) = delete;
    Timer& operator=(const Timer& rhs) = delete;
    Timer& operator=(Timer&& rhs) = delete;

    void bindSystem(caf::actor_system& system);
    void stop();
    void addTimer(caf::actor_addr actor, Duration period);
    static Timer& instance();
};
