#pragma once
#include "DetectedTarget.hpp"
#include "Timer.hpp"
#include <optional>

struct SelectedTarget final {
    TimePoint lastUpdate;
    std::optional<DetectedTarget> selected;
    Vector<UnitType::Distance, FrameOfRef::Robot> position;
};

ACTOR_PROTOCOL_DEFINE(set_target_atom, TypedIdentifier<SelectedTarget>);
ACTOR_PROTOCOL_DEFINE(predict_success_atom, TypedIdentifier<SelectedTarget>);
