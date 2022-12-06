#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "SelectedTarget.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <glm/glm.hpp>

#include "SuppressWarningEnd.hpp"

struct HeroStrategySettings final {};

template <class Inspector>
bool inspect(Inspector& f, HeroStrategySettings& x) {
    return f.object(x).fields();
}

class HeroStrategy final : public HubHelper<caf::event_based_actor, HeroStrategySettings, set_target_atom> {
    Identifier mKey;

public:
    HeroStrategy(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](detect_available_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                     const auto data = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                     SelectedTarget selected;
                     selected.lastUpdate = data.lastUpdate;

                     auto minDistance = std::numeric_limits<double>::max();
                     for(auto& target : data.targets) {
                         const auto vec = target.center.mVal;
                         const auto distance = vec.x * vec.x + vec.y * vec.y;
                         if(distance < minDistance) {
                             selected.selected = target;
                             minDistance = distance;
                         }
                     }
                     if(!selected.selected.has_value())
                         return;
                     sendAll(set_target_atom_v, BlackBoard::instance().updateSync<SelectedTarget>(mKey, selected));
                 } };
    }
};

HUB_REGISTER_CLASS(HeroStrategy);
