#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <cmath>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>

#include "SuppressWarningEnd.hpp"

std::pair<double, double> PIDSimulator::step(const double dt, double target, const double maxV, const double period) noexcept {
    if(period > 0.0) {
        target = std::remainder(target, period);
        if(target < 0)
            target += period;
    }
    auto error = target - mCurrent;
    if(period > 0.0) {
        if(error >= 0) {
            if(error > period - error)
                error -= period;
        } else {
            if(-error >= period + error)
                error += period;
        }
    }
    mSumError += error;
    const auto diff = error - mLastError;
    mLastError = error;
    const auto v = mParameters.kp * error + mParameters.ki * mSumError + mParameters.kd * diff;
    const auto realV = std::copysign(std::fmin(maxV, std::fabs(v)), v);
    mCurrent += realV * dt;
    if(period > 0.0) {
        mCurrent = std::remainder(mCurrent, period);
        if(mCurrent < 0)
            mCurrent += period;
    }
    return std::make_pair(mCurrent, realV);
}

class PIDSimulatorTester final : public HubHelper<caf::event_based_actor, void> {
public:
    PIDSimulatorTester(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
            // period test
            PIDSimulator yawSimulator{ { 100.0, 0.0, 0.0 } };

            constexpr auto period = glm::two_pi<double>();
            constexpr auto count = 1048576;
            constexpr auto dt = 0.001;
            constexpr auto maxV = 100.0;

            const auto checkEq = [&](const double val, const double expected) {
                constexpr auto eps = 1e-3;
                if(std::fabs(val - expected) > eps && std::fabs(val - expected + period) > eps &&
                   std::fabs(val - expected - period) > eps) {
                    logInfo(fmt::format("value {:.3f} expected {:.3f}", val, expected));
                    logInfo("Test failed");
                    terminateSystem(*this, false);
                }
            };

            // ccw
            for(uint32_t idx = 0; idx < count * 2 && globalStatus == RunStatus::running; ++idx) {
                const auto target = period * (idx % count) / count;
                const auto current = yawSimulator.step(dt, target, maxV, period).first;
                checkEq(current, target);
            }
            // cw
            for(uint32_t idx = 0; idx < count * 2 && globalStatus == RunStatus::running; ++idx) {
                const auto target = period - period * (idx % count) / count;
                const auto current = yawSimulator.step(dt, target, maxV, period).first;
                checkEq(current, target);
            }
            if(globalStatus == RunStatus::running)
                terminateSystem(*this, true);
        } };
    }
};

HUB_REGISTER_CLASS(PIDSimulatorTester);
