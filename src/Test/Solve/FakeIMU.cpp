#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SimulatorWorldInfo.hpp"
#include <cstdint>
#include <queue>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <glm/gtx/matrix_decompose.hpp>

#include "SuppressWarningEnd.hpp"

struct FakeIMUSettings final {
    double delay;
    double imuLinearStd;
    double imuAngularStd;
};

template <class Inspector>
bool inspect(Inspector& f, FakeIMUSettings& x) {
    return f.object(x).fields(f.field("delay", x.delay).fallback(0.0), f.field("imuLinearStd", x.imuLinearStd).fallback(0.0),
                              f.field("imuAngularStd", x.imuAngularStd).fallback(0.0));
}

class FakeIMU final : public HubHelper<caf::event_based_actor, FakeIMUSettings, update_posture_atom> {
    Identifier mKey;
    Duration mDelay;
    std::queue<std::pair<TimePoint, decltype(PostureData::tfGround2Robot)>> mQueue;

    std::optional<PostureData> mLastData;

public:
    FakeIMU(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, mDelay{ static_cast<Clock::rep>(
                                                                    mConfig.delay * Clock::period::den / Clock::period::num) } {}
    caf::behavior make_behavior() override {
        return { [&](simulator_step_atom, Identifier key) {
                    ACTOR_PROTOCOL_CHECK(simulator_step_atom, TypedIdentifier<SimulatorWorldInfo>);
                    const auto data = BlackBoard::instance().get<SimulatorWorldInfo>(key).value();
                    mQueue.emplace(data.lastUpdate, data.tfGround2Robot);

                    TimePoint current = mQueue.back().first;

                    std::optional<decltype(PostureData::tfGround2Robot)> cur;
                    while(!mQueue.empty() && current - mQueue.front().first > mDelay) {
                        cur = mQueue.front().second;
                        mQueue.pop();
                    }

                    if(!cur.has_value())
                        return;

                    const auto& info = cur.value();

                    PostureData posture;
                    posture.lastUpdate = current;
                    posture.tfGround2Robot = info;
                    if(mLastData.has_value()) {
                        const auto& lastData = mLastData.value();
                        const auto dt = Scalar<UnitType::Time>{ static_cast<double>((current - lastData.lastUpdate).count()) /
                                                                Clock::period::den * Clock::period::num };

                        glm::dvec3 scale, translate, translateOld, skew;
                        glm::dvec4 perspective;
                        glm::dquat quat, quatOld;
                        glm::decompose(posture.tfGround2Robot.invTransformMat(), scale, quat, translate, skew, perspective);
                        glm::decompose(lastData.tfGround2Robot.invTransformMat(), scale, quatOld, translateOld, skew,
                                       perspective);

                        Point<UnitType::Distance, FrameOfRef::Ground> pos{ translate };
                        Point<UnitType::Distance, FrameOfRef::Ground> posOld{ translateOld };

                        Point<UnitType::Angle, FrameOfRef::Ground> angle{ glm::eulerAngles(quat) };
                        Point<UnitType::Angle, FrameOfRef::Ground> angleOld{ glm::eulerAngles(quatOld) };

                        posture.linearVelocityOfRobot = (pos - posOld) / dt;
                    } else {
                        posture.linearVelocityOfRobot = decltype(PostureData::linearVelocityOfRobot){ glm::zero<glm::dvec3>() };
                    }
                    mLastData = posture;

                    // TODO: add noise

                    sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, posture));
                },
                 [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    }
};

HUB_REGISTER_CLASS(FakeIMU);
