#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Utility.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>

#include "SuppressWarningEnd.hpp"

struct AvailabilityTestSettings final {
    uint32_t testCount;
    double threshold;
};

template <class Inspector>
bool inspect(Inspector& f, AvailabilityTestSettings& x) {
    return f.object(x).fields(f.field("testCount", x.testCount), f.field("threshold", x.threshold));
}

class AvailabilityTester final : public HubHelper<caf::event_based_actor, AvailabilityTestSettings, payload_atom> {
    uint32_t mSentCount = 0;
    uint32_t mSuccessCount = 0;

public:
    AvailabilityTester(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {
        Timer::instance().addTimer(address(), 10ms);
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](timer_atom) {
                     ACTOR_PROTOCOL_CHECK(timer_atom);
                     if(++mSentCount <= mConfig.testCount) {
                         sendAll(payload_atom_v, 0, 0);
                     } else {
                         appendTestResult(fmt::format("Availability {:.2f}% ({}/{}) Require {:.2f}%",
                                                      mSuccessCount / static_cast<double>(mConfig.testCount) * 100.0,
                                                      mSuccessCount, mConfig.testCount, mConfig.threshold * 100.0));
                         terminateSystem(*this, mSuccessCount >= mConfig.testCount * mConfig.threshold);
                     }
                 },
                 [&](payload_atom, int32_t, int32_t) { ++mSuccessCount; } };
    }
};

HUB_REGISTER_CLASS(AvailabilityTester);

class MessageForwarder final : public HubHelper<caf::event_based_actor, void, payload_atom> {
public:
    MessageForwarder(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](payload_atom, int32_t, int32_t) { sendAll(payload_atom_v, 0, 0); } };
    }
};

HUB_REGISTER_CLASS(MessageForwarder);
