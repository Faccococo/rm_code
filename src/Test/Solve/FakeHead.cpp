#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SimulatorWorldInfo.hpp"
#include "Utility.hpp"
#include <cstdint>
#include <queue>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>

#include "SuppressWarningEnd.hpp"

struct FakeHeadSettings final {
    double delay;
    double headPosStd;
    double headSpeedStd;
    double headMaxSpeed;
    double headHeightOffset;

    double kp, ki, kd;
    bool enablePID;
};

template <class Inspector>
bool inspect(Inspector& f, FakeHeadSettings& x) {
    return f.object(x).fields(
        f.field("delay", x.delay).fallback(0.0), f.field("headPosStd", x.headPosStd).fallback(0.0),
        f.field("headSpeedStd", x.headSpeedStd).fallback(0.0), f.field("headMaxSpeed", x.headMaxSpeed).fallback(0.0),
        f.field("headHeightOffset", x.headHeightOffset), f.field("kp", x.kp).fallback(0.0), f.field("ki", x.ki).fallback(0.0),
        f.field("kd", x.kd).fallback(0.0), f.field("enablePID", x.enablePID).fallback(false));
}

class FakeHead final : public HubHelper<caf::event_based_actor, FakeHeadSettings, update_head_atom> {
    TimePoint mCurrent = {};
    Duration mDelay;
    // std::queue<std::tuple<TimePoint, double, double>> mQueue;

    double mTargetYaw = 0.0, mTargetPitch = 0.0, mLastYaw = 0.0, mLastPitch = 0.0;
    PIDSimulator mYaw, mPitch;
    Identifier mKey;

public:
    FakeHead(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mDelay{ static_cast<Clock::rep>(mConfig.delay * Clock::period::den / Clock::period::num) },
          mYaw{ { mConfig.kp, mConfig.ki, mConfig.kd } }, mPitch{ { mConfig.kp, mConfig.ki, mConfig.kd } },  //
          mKey{ generateKey(this) } {
        Timer::instance().addTimer(this->address(), 1ms);
    }

    caf::behavior make_behavior() override {
        return { [&](timer_atom) {
                    ACTOR_PROTOCOL_CHECK(timer_atom);
                    // mQueue.push({ mCurrent, mTargetYaw, mTargetPitch });
                    sendAll(update_head_atom_v, 1U, TypedIdentifier<HeadInfo>{ mKey });
                },
                 [&](simulator_step_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(simulator_step_atom, TypedIdentifier<SimulatorWorldInfo>);
                     const auto data = BlackBoard::instance().get<SimulatorWorldInfo>(key).value();

                     const auto current = data.lastUpdate;
                     const auto diff =
                         static_cast<double>((current - mCurrent).count()) / Clock::period::den * Clock::period::num;
                     mCurrent = current;

                     /*
                     std::optional<std::tuple<TimePoint, double, double>> cur;
                     while(!mQueue.empty() && current - std::get<0>(mQueue.front()) > mDelay) {
                         cur = mQueue.front();
                         mQueue.pop();
                     }

                     if(!cur.has_value())
                         return;
                         */

                     // const auto [_, targetYaw, targetPitch] = cur.value();
                     const auto targetYaw = mTargetYaw, targetPitch = mTargetPitch;
                     double yaw, pitch;
                     if(mConfig.enablePID) {
                         auto [yawPID, yawSpeedPID] = mYaw.step(diff, targetYaw, mConfig.headMaxSpeed, glm::two_pi<double>());
                         auto [pitchPID, pitchSpeedPID] = mPitch.step(diff, targetPitch, mConfig.headMaxSpeed);
                         yaw = yawPID;
                         pitch = pitchPID;
                     } else {
                         yaw = targetYaw;
                         pitch = targetPitch;

                         mLastYaw = targetYaw;
                         mLastPitch = targetPitch;
                     }

                     if(mConfig.headPosStd > 1e-6) {
                         yaw += std::clamp(glm::gaussRand(0.0, mConfig.headPosStd), -3.0 * mConfig.headPosStd,
                                           3.0 * mConfig.headPosStd);
                         pitch += std::clamp(glm::gaussRand(0.0, mConfig.headPosStd), -3.0 * mConfig.headPosStd,
                                             3.0 * mConfig.headPosStd);
                     }

                     const HeadInfo info{ mCurrent,
                                          decltype(HeadInfo::tfRobot2Gun){ glm::lookAtRH(
                                              glm::dvec3{ 0.0, mConfig.headHeightOffset, 0.0 },
                                              glm::dvec3{ std::cos(yaw + glm::half_pi<double>()) * std::cos(pitch),
                                                          mConfig.headHeightOffset + std::sin(pitch),
                                                          -std::sin(yaw + glm::half_pi<double>()) * std::cos(pitch) },
                                              glm::dvec3{ 0.0, 1.0, 0.0 }) } };

                     sendAll(update_head_atom_v, 1U, BlackBoard::instance().updateSync(mKey, info));
                     sendMasked(update_head_atom_v, 1U, 1U, BlackBoard::instance().updateSync(mKey, info));
                     sendMasked(update_head_atom_v, 2U, 2U,
                                BlackBoard::instance().updateSync(Identifier{ mKey.val ^ 0xffffffff }, info));
                 },
                 [&](set_target_info_atom, GroupMask, Clock::rep, const double yaw, const double pitch, bool) {
                     ACTOR_PROTOCOL_CHECK(set_target_info_atom, GroupMask, Clock::rep, double, double, bool);
                     mTargetYaw = yaw;
                     mTargetPitch = pitch;
                 },
                 [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    }
};

HUB_REGISTER_CLASS(FakeHead);
