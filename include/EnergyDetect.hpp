#pragma once
#include "CameraFrame.hpp"

struct DetectedEnergyInfo final {
    TimePoint lastUpdate;
    Point<UnitType::Distance, FrameOfRef::Gun> point;
    Point<UnitType::Distance, FrameOfRef::Gun> prePoint;
};

ACTOR_PROTOCOL_DEFINE(energy_detect_available_atom, TypedIdentifier<DetectedEnergyInfo>);
ACTOR_PROTOCOL_DEFINE(energy_detector_control_atom, bool);
