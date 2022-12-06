#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>

#include "SuppressWarningEnd.hpp"

struct IMUFilterSettings final {};

template <class Inspector>
bool inspect(Inspector& f, IMUFilterSettings& x) {
    return f.object(x).fields();
}

class IMUFilter final : public HubHelper<caf::event_based_actor, IMUFilterSettings, update_posture_atom> {
    Identifier mKey;

public:
    IMUFilter(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](update_posture_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(update_posture_atom, TypedIdentifier<PostureData>);
                     auto res = BlackBoard::instance().get<PostureData>(key).value();
                     // Implement here

                     sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(IMUFilter);
