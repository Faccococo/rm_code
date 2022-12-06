#include "BlackBoard.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <Timer.hpp>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_system.hpp>
#include <caf/event_based_actor.hpp>
#include <fmt/format.h>

#include "SuppressWarningEnd.hpp"

class ChaosMonkey final : public HubHelper<caf::event_based_actor, void> {
    std::vector<caf::actor_addr> mActors;

    void flushActors() {
        mActors.clear();
        for(auto& [name, actor] : system().registry().named_actors())
            mActors.push_back(caf::actor_cast<caf::actor_addr>(actor));
    }

public:
    ChaosMonkey(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {
        flushActors();
        Timer::instance().addTimer(address(), 50ms);
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](timer_atom) {
                     ACTOR_PROTOCOL_CHECK(timer_atom);
                     // TODO: random kill
                 } };
    }
};

HUB_REGISTER_CLASS(ChaosMonkey);
