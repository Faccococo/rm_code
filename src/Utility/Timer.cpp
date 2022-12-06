#include "Timer.hpp"
#include "DataDesc.hpp"
#include "Utility.hpp"
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/scoped_actor.hpp>

#include "SuppressWarningEnd.hpp"

Timer& Timer::instance() {
    static Timer inst;
    return inst;
}

void Timer::bindSystem(caf::actor_system& system) {
    mThread = std::thread{ [&] {
        using namespace std::chrono_literals;
        while(globalStatus == RunStatus::running) {
            if(mTimers.empty()) {
                std::this_thread::sleep_for(10ms);
                continue;
            }

            std::unique_lock<std::mutex> guard{ mMutex, std::try_to_lock };
            if(!guard.owns_lock()) {
                std::this_thread::yield();
                continue;
            }

            const auto [actor, deadline, duration] = mTimers.top();
            mTimers.pop();
            mTimers.push({ actor, deadline + duration, duration });

            guard.unlock();

            std::this_thread::sleep_until(deadline);

            const caf::scoped_actor caller{ system };
            caller->send(caf::actor_cast<caf::actor>(actor), timer_atom_v);
        }
    } };
}

void Timer::stop() {
    if(mThread.joinable())
        mThread.join();
}

void Timer::addTimer(caf::actor_addr actor, Duration period) {
    std::lock_guard<std::mutex> guard{ mMutex };
    mTimers.push({ std::move(actor), Clock::now() + period, period });
}

TimePoint SynchronizedClock::now() const {
    if(mSimulationTime)
        return mSimulationTime.value();

    return Clock::now();
}

SynchronizedClock& SynchronizedClock::instance() {
    static SynchronizedClock clock;
    return clock;
}

void SynchronizedClock::setSimulationTime(TimePoint tp) {
    mSimulationTime = tp;
}
