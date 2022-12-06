#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <csignal>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>

#include "SuppressWarningEnd.hpp"

class BlockedActor final : public HubHelper<caf::event_based_actor, void, payload_atom> {
    inline static bool mFailed = false;

public:
    BlockedActor(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](payload_atom, int32_t, int32_t) {
                     ACTOR_PROTOCOL_CHECK(payload_atom, int32_t, int32_t);
                     if(mFailed)
                         sendAll(payload_atom_v, 0, 0);
                     else {
                         mFailed = true;
                         while(true)
                             ;
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(BlockedActor);

class ExceptionKilled final : public HubHelper<caf::event_based_actor, void, payload_atom> {
    inline static bool mFailed = false;

public:
    ExceptionKilled(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](payload_atom, int32_t, int32_t) {
                     if(mFailed)
                         sendAll(payload_atom_v, 0, 0);
                     else {
                         mFailed = true;
                         throw std::runtime_error("emulated exception");
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(ExceptionKilled);

class SignalKilled final : public HubHelper<caf::event_based_actor, void, payload_atom> {
    inline static bool mFailed = false;

public:
    SignalKilled(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](payload_atom, int32_t, int32_t) {
                     if(mFailed)
                         sendAll(payload_atom_v, 0, 0);
                     else {
                         mFailed = true;
                         raise(SIGSEGV);
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(SignalKilled);

class DaemonTester final : public HubHelper<caf::event_based_actor, void, payload_atom> {
public:
    DaemonTester(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    sendAll(payload_atom_v, 0, 0);
                },
                 [&](payload_atom, int32_t, int32_t) {
                     ACTOR_PROTOCOL_CHECK(payload_atom, int32_t, int32_t);
                     terminateSystem(*this, true);
                 } };
    }
};

HUB_REGISTER_CLASS(DaemonTester);
