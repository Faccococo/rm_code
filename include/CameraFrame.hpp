#pragma once
#include "Timer.hpp"
#include "Transform.hpp"
#include <variant>

#include "SuppressWarningBegin.hpp"

#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

struct CameraInfo final {
    std::variant<Transform<FrameOfRef::Gun, FrameOfRef::Camera, true>, Transform<FrameOfRef::Robot, FrameOfRef::Camera, true>>
        transform;
    std::string identifier;
    cv::Mat cameraMatrix;
    cv::Mat distCoefficients;
    uint32_t width;
    uint32_t height;
};

struct CameraFrame final {
    TimePoint lastUpdate;
    CameraInfo info;
    cv::Mat frame;
};

ACTOR_PROTOCOL_DEFINE(image_frame_atom, TypedIdentifier<CameraFrame>);
