#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <vector>

struct SimulatorWorldInfo final {
    TimePoint lastUpdate;

    Transform<FrameOfRef::Ground, FrameOfRef::Robot, true> tfGround2Robot;

    std::vector<Point<UnitType::Distance, FrameOfRef::Ground>> targets;
};

ACTOR_PROTOCOL_DEFINE(simulator_step_atom, TypedIdentifier<SimulatorWorldInfo>);
