#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <vector>

enum class ArmorType { Small, Large };

struct DetectedTarget final {
    Point<UnitType::Distance, FrameOfRef::Gun> center;
    double area;
    int32_t id;
    ArmorType type;
    Vector<UnitType::LinearVelocity, FrameOfRef::Robot> velocity;
};

struct DetectedTargetArray final {
    TimePoint lastUpdate;
    std::vector<DetectedTarget> targets;
};

ACTOR_PROTOCOL_DEFINE(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
