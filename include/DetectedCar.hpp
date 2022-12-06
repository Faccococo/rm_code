#pragma once
#include "CameraFrame.hpp"

struct DetectedCarArray final {
    CameraFrame frame;
    std::vector<cv::Rect> cars;
};

ACTOR_PROTOCOL_DEFINE(car_detect_available_atom, TypedIdentifier<DetectedCarArray>);
