#pragma once
#include "CameraFrame.hpp"

#include "SuppressWarningBegin.hpp"

#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

// Origin: left-top corner of the car's ROI
struct PairedLight final {
    cv::RotatedRect r1;
    cv::RotatedRect r2;
};

struct Armor final {
    int id;
    PairedLight pairedLight;
};

struct DetectedArmorArray final {
    CameraFrame frame;
    std::vector<Armor> armors;
};

struct NNetDetectedArmor final {
    cv::Point2f light4Point[4];  // 灯条四点坐标
    cv::Rect_<float> lightRect;  // 灯条四点矩形
    int robotType;  // 机器人类别（0：哨兵，1：英雄，2：工程，3、4、5：步兵，6：前哨站，7：基地）
    int robotColor;  // 颜色分类（0：蓝色，1：红色，2：灰色）
    int rectArea;    //矩形面积
    float prob;    //分类置信度
    std::vector<cv::Point2f> detectedArmors; //探测到的所有装甲板的灯条的四点坐标
};

struct NNetDetectedArmorArray final {
    std::vector<NNetDetectedArmor> armors;
    CameraFrame frame;
};

ACTOR_PROTOCOL_DEFINE(armor_detect_available_atom, TypedIdentifier<DetectedArmorArray>);
ACTOR_PROTOCOL_DEFINE(armor_nnet_detect_available_atom, TypedIdentifier<NNetDetectedArmorArray>);
