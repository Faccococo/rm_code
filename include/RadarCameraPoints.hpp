#pragma once
#include "CameraFrame.hpp"
#include "Timer.hpp"

struct RadarCameraPointsArray final {
    TimePoint lastUpdate;
    CameraInfo cameraInfo;
    Color selfColor;
    std::vector<cv::Point2f> imagePoints;
};

ACTOR_PROTOCOL_DEFINE(radar_locate_request_atom, TypedIdentifier<RadarCameraPointsArray>);
ACTOR_PROTOCOL_DEFINE(radar_locate_succeed_atom, TypedIdentifier<Transform<FrameOfRef::Camera, FrameOfRef::Ground, true>>);
