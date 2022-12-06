#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>

#include "SuppressWarningEnd.hpp"

class FakeSerialPort final : public HubHelper<caf::event_based_actor, void, ore_instructions_atom> {
    Identifier mKey;

public:
    FakeSerialPort(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    logInfo("Testing ore detection. ");
                    sendAll(ore_instructions_atom_v, true);
                },
                 [&](ore_detect_available_atom, double angle) {
                     ACTOR_PROTOCOL_CHECK(ore_detect_available_atom, double);
                     logInfo(fmt::format("{}", angle));
                 } };
    }
};
HUB_REGISTER_CLASS(FakeSerialPort);
