#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "Timer.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "SuppressWarningEnd.hpp"

class FixedIMU final : public HubHelper<caf::event_based_actor, void, update_posture_atom> {
    Identifier mKey;

public:
    FixedIMU(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [&](timer_atom) {
                    ACTOR_PROTOCOL_CHECK(timer_atom);
                    PostureData posture;
                    posture.lastUpdate = SynchronizedClock::instance().now();
                    posture.tfGround2Robot = Transform<FrameOfRef::Ground, FrameOfRef::Robot>{ glm::identity<glm::dmat4>() };
                    posture.linearVelocityOfRobot =
                        Vector<UnitType::LinearVelocity, FrameOfRef::Ground>{ glm::zero<glm::dvec3>() };

                    sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, posture));
                },
                 [this](start_atom) {
                     ACTOR_PROTOCOL_CHECK(start_atom);
                     Timer::instance().addTimer(address(), 100ms);
                 } };
    }
};

HUB_REGISTER_CLASS(FixedIMU);
