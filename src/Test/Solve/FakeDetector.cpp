#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SimulatorWorldInfo.hpp"
#include <cstdint>
#include <queue>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <glm/gtc/random.hpp>

#include "SuppressWarningEnd.hpp"

struct FakeDetectorSettings final {
    double delay;
    double detectLinearStd;
};

template <class Inspector>
bool inspect(Inspector& f, FakeDetectorSettings& x) {
    return f.object(x).fields(f.field("delay", x.delay).fallback(0.0),
                              f.field("detectLinearStd", x.detectLinearStd).fallback(0.0));
}

class FakeDetector final : public HubHelper<caf::event_based_actor, FakeDetectorSettings, detect_available_atom> {
    Identifier mKey, mHeadKey{};
    Duration mDelay;
    std::queue<SimulatorWorldInfo> mQueue;

public:
    FakeDetector(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, mDelay{ static_cast<Clock::rep>(
                                                                    mConfig.delay * Clock::period::den / Clock::period::num) } {}
    caf::behavior make_behavior() override {
        return { [&](simulator_step_atom, Identifier key) {
                    ACTOR_PROTOCOL_CHECK(simulator_step_atom, TypedIdentifier<SimulatorWorldInfo>);
                    mQueue.push(BlackBoard::instance().get<SimulatorWorldInfo>(key).value());

                    const auto headData = BlackBoard::instance().get<HeadInfo>(mHeadKey);
                    if(!headData.has_value())
                        return;

                    const TimePoint current = mQueue.back().lastUpdate;

                    std::optional<SimulatorWorldInfo> cur;
                    while(!mQueue.empty() && current - mQueue.front().lastUpdate > mDelay) {
                        cur = mQueue.front();
                        mQueue.pop();
                    }

                    if(!cur.has_value())
                        return;

                    const auto& info = cur.value();
                    const auto& head = headData.value();

                    DetectedTargetArray data;

                    data.lastUpdate = current;

                    const auto worldTrans = combine(info.tfGround2Robot, head.tfRobot2Gun) ;

                    for(const auto& target : info.targets) {
                        const auto pos = worldTrans(target);
                        auto noise = glm::zero<glm::dvec3>();
                        if(mConfig.detectLinearStd > 1e-3) {
                            noise = glm::gaussRand(glm::zero<glm::dvec3>(), glm::dvec3{ mConfig.detectLinearStd });
                            noise = glm::clamp(noise, glm::dvec3{ -3.0 * mConfig.detectLinearStd },
                                               glm::dvec3{ 3.0 * mConfig.detectLinearStd });
                        }

                        data.targets.push_back(DetectedTarget{ pos + Vector<UnitType::Distance, FrameOfRef::Gun>(noise), 0.0, 0,
                                                               ArmorType::Small,
                                                               decltype(DetectedTarget::velocity){ glm::zero<glm::dvec3>() } });
                    }

                    sendAll(detect_available_atom_v, mGroupMask, BlackBoard::instance().updateSync(mKey, data));
                },
                 [&](update_head_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
                     mHeadKey = key;
                 },
                 [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    }
};

HUB_REGISTER_CLASS(FakeDetector);
