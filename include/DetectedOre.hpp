#pragma once
#include "CameraFrame.hpp"

enum class OreDetectorMode { GOLD_OVER, GOLD_GROUND, SILVER_GROUND, NONE };

struct OrePosition final {
    // begin with 0
    uint64_t totalNum;
    uint64_t flashingIndex;
};

struct OreAlignmentMessage final {
    CameraFrame frame;
    OreDetectorMode detectMode;
    OreDetectorMode lastMode;
    double moveDistance;
};

ACTOR_PROTOCOL_DEFINE(ore_alignment_available_atom, TypedIdentifier<OreAlignmentMessage>);
