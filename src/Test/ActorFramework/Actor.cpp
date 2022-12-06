#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>

#include "SuppressWarningEnd.hpp"

class Input final : public HubHelper<caf::event_based_actor, void, payload_atom> {
public:
    Input(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
            ACTOR_PROTOCOL_CHECK(start_atom);
            for(int32_t i = 0; i < 10; ++i)
                for(int32_t j = 0; j < 10; ++j)
                    sendAll(payload_atom_v, i, j);
            terminateSystem(*this, true);
        } };
    }
};

HUB_REGISTER_CLASS(Input);

class Output final : public HubHelper<caf::event_based_actor, void> {
public:
    Output(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](payload_atom, const int32_t a, const int32_t b) {
                     ACTOR_PROTOCOL_CHECK(payload_atom, int32_t, int32_t);
                     const auto res = a + b;
                     caf::aout(this) << a << " + " << b << " = " << res << std::endl;
                 } };
    }
};

HUB_REGISTER_CLASS(Output);
