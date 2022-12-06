#pragma once
#include "Timer.hpp"
#include "Transform.hpp"

struct PostureData final {
    TimePoint lastUpdate;

    Transform<FrameOfRef::Ground, FrameOfRef::Robot> tfGround2Robot;

    Vector<UnitType::LinearVelocity, FrameOfRef::Ground> linearVelocityOfRobot;
};

ACTOR_PROTOCOL_DEFINE(update_posture_atom, TypedIdentifier<PostureData>);
