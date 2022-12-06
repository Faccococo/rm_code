#pragma once
#include "Timer.hpp"

class FrameRateKeeper final {
    double mTargetFPS;

    Clock::duration mThreshold;
    std::deque<Clock::time_point> mLastFrames;

public:
    explicit FrameRateKeeper(double targetFPS);
    bool newFrame();
};
