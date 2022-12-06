#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <glm/gtc/random.hpp>

#include "SuppressWarningEnd.hpp"

class SimpleStrategy final : public HubHelper<caf::event_based_actor, void, set_target_atom> {
    Identifier mKey;

public:
    SimpleStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [&](detect_available_atom, GroupMask, Identifier key) {
                    ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                    const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                    SelectedTarget selected;
                    selected.lastUpdate = data.lastUpdate;
                    auto minDistance = std::numeric_limits<double>::max();
                    for(auto& target : data.targets) {
                        if(const auto distance = glm::length(target.center.mVal); minDistance > distance) {
                            selected.selected = target;
                            minDistance = distance;
                        }
                    }

                    sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                },
                 [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    }
};

HUB_REGISTER_CLASS(SimpleStrategy);
