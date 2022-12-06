#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <Timer.hpp>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_system.hpp>
#include <caf/event_based_actor.hpp>
#include <fmt/format.h>

#include "SuppressWarningEnd.hpp"

class DaemonActor final : public caf::event_based_actor {
    std::unordered_map<caf::actor_addr, std::string> mActors;
    bool mStarted = false;

public:
    DaemonActor(caf::actor_config& base, const std::vector<std::pair<std::string, caf::actor>>& actors)
        : event_based_actor{ base } {
        for(auto&& [name, actor] : actors) {
            this->monitor(actor);
            mActors.emplace(actor.address(), name);
        }
        this->set_down_handler([&](const caf::down_msg& msg) {
            if(globalStatus != RunStatus::running)
                return;

            const auto name = mActors[msg.source];
            logInfo(fmt::format("Actor {} down: {}", name, caf::to_string(msg.reason)));
            // TODO: resume actors
        });
        Timer::instance().addTimer(address(), 500ms);
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    mStarted = true;
                },
                 [this](timer_atom) {
                     ACTOR_PROTOCOL_CHECK(timer_atom);
                     // TODO: send requests
                     if(!mStarted)
                         return;
                 },
                 [this](monitor_response_atom) { ACTOR_PROTOCOL_CHECK(monitor_response_atom); } };
    }
};

caf::actor createDaemonActor(caf::actor_system& sys, const std::vector<std::pair<std::string, caf::actor>>& actors) {
    return sys.spawn<DaemonActor>(actors);
}
