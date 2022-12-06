#pragma once
#include "Constants.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/allowed_unsafe_message_type.hpp>
#include <caf/is_error_code_enum.hpp>
#include <caf/type_id.hpp>

#include "SuppressWarningEnd.hpp"

#include <cstdint>

using Clock = std::chrono::steady_clock;
static_assert(std::is_same_v<Clock::period, std::nano>);

enum class Color { Red, Blue };

struct GlobalSettings final {
    double gForce;
    double dragCoefficient;
    bool bullet42mm;

    Color selfColor = Color::Red;
    double bulletSpeed = 15.00;
    bool started = false;

    [[nodiscard]] double bulletRadius() const noexcept {
        return bullet42mm ? radiusOf42mm : radiusOf17mm;
    }

    [[nodiscard]] double bulletMass() const noexcept {
        return bullet42mm ? massOf42mm : massOf17mm;
    }

    static GlobalSettings& get() {
        static GlobalSettings settings;
        return settings;
    }
};

template <class Inspector>
bool inspect(Inspector& f, GlobalSettings& x) {
    return f.object(x).fields(f.field("gForce", x.gForce), f.field("dragCoefficient", x.dragCoefficient),
                              f.field("bullet42mm", x.bullet42mm));
}

struct Identifier {
    uint64_t val;
};

template <typename T>
struct TypedIdentifier final : Identifier {
    using Payload = T;
};

CAF_BEGIN_TYPE_ID_BLOCK(ArtinxHub, caf::first_custom_type_id);

CAF_ADD_ATOM(ArtinxHub, start_atom);
CAF_ADD_ATOM(ArtinxHub, detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, set_target_atom);
CAF_ADD_ATOM(ArtinxHub, set_target_info_atom);
CAF_ADD_ATOM(ArtinxHub, update_posture_atom);
CAF_ADD_ATOM(ArtinxHub, update_head_atom);
CAF_ADD_ATOM(ArtinxHub, simulator_step_atom);
CAF_ADD_ATOM(ArtinxHub, timer_atom);
CAF_ADD_ATOM(ArtinxHub, image_frame_atom);
CAF_ADD_ATOM(ArtinxHub, car_detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, armor_detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, armor_nnet_detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, energy_detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, predict_success_atom);
CAF_ADD_ATOM(ArtinxHub, ore_instructions_atom);
CAF_ADD_ATOM(ArtinxHub, ore_detect_available_atom);
CAF_ADD_ATOM(ArtinxHub, radar_locate_succeed_atom);
CAF_ADD_ATOM(ArtinxHub, radar_locate_request_atom);
CAF_ADD_ATOM(ArtinxHub, num_classify_request_atom);
CAF_ADD_ATOM(ArtinxHub, monitor_request_atom);
CAF_ADD_ATOM(ArtinxHub, monitor_response_atom);
CAF_ADD_ATOM(ArtinxHub, payload_atom);
CAF_ADD_ATOM(ArtinxHub, ore_alignment_available_atom);
CAF_ADD_ATOM(ArtinxHub, energy_detector_control_atom);

CAF_ADD_TYPE_ID(ArtinxHub, (Identifier));

CAF_END_TYPE_ID_BLOCK(ArtinxHub);

CAF_ALLOW_UNSAFE_MESSAGE_TYPE(Identifier);

using GroupMask = uint32_t;

template <typename... T>
struct __ImplActorProtocol final {
    static constexpr bool check() noexcept {
        return false;
    }
};

#define ACTOR_PROTOCOL_DEFINE(...)                  \
    template <>                                     \
    struct __ImplActorProtocol<__VA_ARGS__> final { \
        static constexpr bool check() noexcept {    \
            return true;                            \
        }                                           \
    }

template <typename... Args>
constexpr bool __impl_actor_protocol_call() noexcept {
    return __ImplActorProtocol<Args...>::check();
}

#define ACTOR_PROTOCOL_CHECK(...) static_assert(__impl_actor_protocol_call<__VA_ARGS__>(), "Mismatched protocol")

ACTOR_PROTOCOL_DEFINE(start_atom);
ACTOR_PROTOCOL_DEFINE(timer_atom);
ACTOR_PROTOCOL_DEFINE(monitor_response_atom);
ACTOR_PROTOCOL_DEFINE(payload_atom, int32_t, int32_t);
ACTOR_PROTOCOL_DEFINE(ore_detect_available_atom, double);
ACTOR_PROTOCOL_DEFINE(ore_instructions_atom, bool);
ACTOR_PROTOCOL_DEFINE(set_target_info_atom, GroupMask, Clock::rep, double, double, bool);
