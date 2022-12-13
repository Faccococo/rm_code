#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "EnergyDetect.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <utility>

#include "SuppressWarningEnd.hpp"

// struct EnergyDetectorSettings final {
//     float armor_min_lw_ratio;
//     float armor_max_lw_ratio;
//     float min_armor_area;
//     float max_armor_area;
//     float approx_epsilon;
//     float min_armor_vane_ratio;
//     float max_armor_vane_ratio;
//     float vane_min_lw_ratio;
//     float vane_max_lw_ratio;
//     float vane_min_contour_rrect_ratio;
//     float vane_max_contour_rrect_ratio;
//     float min_vane_area;
//     float max_vane_area;
//};
struct EnergyDetectorSettings final {
    int smallPredictMode;
    int bigPredictMode;
    int preFrames;
    float armorMinArea;
    float armorMaxArea;
    float armorMinWHRatio;
    float armorMaxWHRatio;
    float armorMinAreaRatio;
    float stripMinArea;
    float stripMaxArea;
    float stripMinWHRatio;
    float stripMaxWHRatio;
    float stripMaxAreaRatio;
    float noiseArea;
    float predictAngle;
    float radius;
    float offsetPreAngle;
    cv::Point2f offset;
};

template <class Inspector>
bool inspect(Inspector& f, EnergyDetectorSettings& x) {
    return f.object(x).fields(f.field("smallPredictMode", x.smallPredictMode), f.field("bigPredictMode", x.bigPredictMode),
                              f.field("preFrames", x.preFrames).fallback(12), f.field("armorMinArea", x.armorMinArea),
                              f.field("armorMaxArea", x.armorMaxArea), f.field("armorMinWHRatio", x.armorMinWHRatio),
                              f.field("armorMaxWHRatio", x.armorMaxWHRatio), f.field("armorMinAreaRatio", x.armorMinAreaRatio),
                              f.field("stripMinArea", x.stripMinArea), f.field("stripMaxArea", x.stripMaxArea),
                              f.field("stripMaxWHRatio", x.stripMaxWHRatio), f.field("stripMaxAreaRatio", x.stripMaxAreaRatio),
                              f.field("noiseArea", x.noiseArea), f.field("predictAngle", x.predictAngle),
                              f.field("radius", x.radius), f.field("offsetPreAngle", x.offsetPreAngle),
                              f.field("offsetX", x.offset.x), f.field("offsetY", x.offset.y));
}

class EnergyDetector final
    : public HubHelper<caf::event_based_actor, EnergyDetectorSettings, energy_detect_available_atom, image_frame_atom> {

    Identifier mKey;

    bool mEnabled = true;


    void reset() {}

    void debugView(const std::string_view& name, const cv::Mat& src, const std::function<void(cv::Mat&)>& func) {
#ifndef ARTINXHUB_DEBUG
        // return;
#endif

        const auto hash = std::hash<std::string_view>{}(name);
        const Identifier newKey{ mKey.val ^ hash };

        cv::Mat res;
        src.copyTo(res);
        if(res.depth() == CV_8U)
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 255 });
        else
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 0, 255, 0 });

        func(res);

        CameraFrame frame;
        frame.frame = std::move(res);

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(newKey, std::move(frame)));
    }

    static void setBinary(const cv::Mat& src, cv::Mat& binary) {
        std::vector<cv::Mat> imgChannels;
        cv::split(src, imgChannels);

        cv::Mat gray, grayBin, colorBin;
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

        constexpr auto threshold = 90;
        if(GlobalSettings::get().selfColor == Color::Red) {
            cv::threshold(gray, grayBin, threshold, 255, cv::THRESH_BINARY);
            const auto energyRed = imgChannels[2] - imgChannels[0];
            cv::threshold(energyRed, colorBin, threshold, 255, cv::THRESH_BINARY);
        } else {
            cv::threshold(gray, grayBin, threshold, 255, cv::THRESH_BINARY);
            const auto energyBlue = imgChannels[0] - imgChannels[2];
            cv::threshold(energyBlue, colorBin, threshold, 255, cv::THRESH_BINARY);
        }

        binary = grayBin & colorBin;

        // only for test
        // constexpr auto threshold = 200;
        // binary = imgChannels[0] > threshold & imgChannels[1] > threshold & imgChannels[2] > threshold;
    }

    /*
     * 本项目中，旋转矩形的顶点顺序如下
     * rect[0]:矩形顶点中最左侧的点
     * rect[1]:矩形顶点中最上方的点
     * rect[2]:矩形顶点中最右侧的点
     * rect[3]:矩形顶点中最下方的点
     *
     * 在本方法中，经过变换之后的矩形width,height,angle定义如下
     * width:矩形的短边
     * height:矩形的长边
     * angle:x轴顺时针旋转到与短边重合经过的角度
     * */
    static void regularRotated(cv::RotatedRect& r) {
        if(r.size.width > r.size.height) {
            std::swap<float>(r.size.width, r.size.height);
            r.angle = r.angle >= 0.0f ? r.angle - 90.0f : r.angle + 90.f;
        }

        if(r.angle < 0)
            r.angle += 180.0f;
    }

    struct EnergyVane final {
        std::vector<cv::Point> contour;
        cv::RotatedRect rrect;
        std::vector<cv::Point> hull;
        int hullNum;
        float cArea;
    };

    struct EnergyArmor final {
        EnergyVane vane;
        std::vector<cv::Point> contour;
        cv::RotatedRect rrect;
        float cArea;
        float areaRatio;
        cv::Point direction;
        float angel;
        cv::Point circleCenter;
    };

    bool detectArmor(const cv::Mat& src, std::pair<cv::Point2f, cv::Point2f>& res) {
        ACTOR_EXCEPTION_PROBE();
        std::vector<EnergyArmor> candidateArmors;
        cv::Mat imgOut;
        src.copyTo(imgOut);
        auto binary = src.clone();
        setBinary(src, binary);

        auto element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
        dilate(binary, binary, element);

        //        debugView("binary", binary, [](cv::Mat&) {});

        std::vector<std::vector<cv::Point>> armorContours;
        std::vector<cv::Vec4i> armorHierarchy;
        findContours(binary, armorContours, armorHierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        const auto armorContoursSize = armorContours.size();
        debugView("contours", src, [&](cv::Mat& img) { cv::drawContours(img, armorContours, -1, cv::Scalar{ 0, 255, 0 }); });

        for(int i = 0; i < static_cast<int>(armorContoursSize); i++) {
            double sonArea = cv::contourArea(armorContours[i]);
            if(sonArea < 20)
                continue;
            cv::RotatedRect sonRect = minAreaRect(armorContours[i]);
            regularRotated(sonRect);

            float length = sonRect.size.height;
            float width = sonRect.size.width;

            if(length / width < 1.2 || length / width > 2.5 || sonRect.size.area() < 200 || sonRect.size.area() > 1700)
                continue;

            std::vector<cv::Point> sonHull;
            cv::approxPolyDP(armorContours[i], sonHull, 1.0 * 4.0, true);
            if(sonHull.size() < 4 || sonHull.size() > 10)
                continue;

            // 父轮廓遍历
            // 面积比、长宽比、角点数、凸包面积比
            for(int j = armorHierarchy[i][3]; j < static_cast<int >(armorContours.size() )&& j > -1; j = armorHierarchy[j][3]) {
                float fatherArea = static_cast<float>(contourArea(armorContours[j]));
                if(sonArea / fatherArea < 0.15 || sonArea / fatherArea > 0.55)
                    continue;

                std::vector<cv::Point> fatherHull;
                cv::approxPolyDP(armorContours[j], fatherHull, 1.0, true);
                if(fatherHull.size() < 6)
                    continue;

                EnergyVane vane;
                vane.contour.assign(armorContours[j].begin(), armorContours[j].end());
                vane.rrect = cv::minAreaRect(vane.contour);
                regularRotated(vane.rrect);
                cv::approxPolyDP(vane.contour, vane.hull, 1.0, true);
                vane.hullNum = static_cast<int>(vane.hull.size());
                vane.cArea = static_cast<float >(cv::contourArea(vane.contour));

                EnergyArmor armor;
                armor.vane = vane;
                armor.contour.assign(armorContours[i].begin(), armorContours[i].end());
                armor.rrect = cv::minAreaRect(armor.contour);
                armor.cArea = static_cast<float >(cv::contourArea(armor.contour));
                armor.areaRatio = armor.cArea / armor.vane.cArea;
                armor.direction = armor.rrect.center - armor.vane.rrect.center;
                armor.angel = static_cast<float >(atan2(armor.direction.y, armor.direction.x));
                armor.circleCenter = armor.vane.rrect.center * 3.3 - 2.3 * armor.rrect.center;
                candidateArmors.push_back(armor);
                break;
            }
        }

        if(candidateArmors.empty()) {
            logInfo("Energy detect failed");
            return false;
        }

        EnergyArmor target;
        if(!chooseTarget(candidateArmors, target)) {
            logInfo("Target choose failed");
            return false;
        }

        std::vector<std::vector<cv::Point>> targets;
        targets.push_back(target.contour);

        cv::RotatedRect armorRect = cv::minAreaRect(target.contour);
        cv::Point2f armorVertices[4];
        armorRect.points(armorVertices);
        drawRotatedRect(imgOut, armorRect, cv::Scalar{ 0, 255, 255 });

        if(!calCenter(target, binary)) {
            logInfo("Center detect failed");
            return false;
        }

        res = {target.rrect.center, target.circleCenter};

        cv::line(imgOut, target.circleCenter, armorVertices[3], cv::Scalar{ 0, 255, 255 });
//        cv::line(imgOut, target.circleCenter, armorVertices[1], cv::Scalar{ 0, 255, 255 });

        debugView("target", imgOut, [](cv::Mat&) {});

        return true;
    }

    static bool chooseTarget(std::vector<EnergyArmor> candidate, EnergyArmor& target) {
        auto cmp = [](const EnergyArmor& a, const EnergyArmor& b) -> bool {
            if(a.vane.hullNum >= b.vane.hullNum) {
                return true;
            } else if(a.vane.hullNum < b.vane.hullNum) {
                return false;
            } else {
                return a.areaRatio > b.areaRatio;
            }
        };
        std::sort(candidate.begin(), candidate.end(), cmp);
        for(const auto& armor : candidate) {
            if(!isTarget(armor)) {
                logInfo("Target choose failed");
                continue;
            }
            if(!isActivated(armor)) {
                target = armor;
                return true;
            }
        }
        logInfo("All is activated");
        return false;
    }

    static bool isTarget(const EnergyArmor& armor) {
        if(armor.areaRatio > 0.55 || armor.areaRatio < 0.15 || armor.vane.hullNum < 5 || armor.vane.hullNum > 80 ||
           armor.vane.cArea / armor.vane.rrect.size.area() < 0.3 || armor.vane.cArea / armor.vane.rrect.size.area() > 1.7)
            return false;
        return true;
    }

    static bool isActivated(const EnergyArmor& armor) {
        if(armor.areaRatio < 0.15 || armor.areaRatio > 0.35 || armor.vane.cArea / armor.vane.rrect.size.area() < 0.7 ||
           armor.vane.cArea / armor.vane.rrect.size.area() > 0.9 || armor.vane.hullNum < 6 || armor.vane.hullNum > 30)
            return false;
        return true;
    }

    bool calCenter(EnergyArmor& armor, const cv::Mat& binary) {
        auto circleCenter = cv::Point2f(armor.circleCenter);
        float roi = armor.rrect.size.width > armor.rrect.size.height ? armor.rrect.size.width : armor.rrect.size.height;
        roi /= 1.7;
        cv::Rect2f roiRect = cv::Rect2f(circleCenter.x - roi, circleCenter.y - roi, 2 * roi, 2 * roi);
        roiRect &= cv::Rect2f(cv::Point2f(0, 0), cv::Point2f(static_cast<float >(binary.cols), static_cast<float >(binary.rows)));

        if(roiRect.width <= 10 || roiRect.height <= 10)
            return false;

        cv::Mat binROI = binary(roiRect);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        findContours(binROI, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point2f(roiRect.x, roiRect.y));
        if(contours.empty())
            return false;
        debugView("center", binROI, [&](cv::Mat& img) { cv::drawContours(img, contours, -1, cv::Scalar{ 255, 255, 255 }); });

        int iMax = -1;
        double maxArea = 0;
        for(int i = 0; i < static_cast<int >(contours.size()); i++) {
            double area = contourArea(contours[i]);
            double rectArea = cv::minAreaRect(contours[i]).size.area();
            if(area / armor.cArea < 0.1)
                continue;
            if(area / rectArea < 0.7)
                continue;
            if(area > maxArea) {
                maxArea = area;
                iMax = i;
            }
        }
        if(iMax < 0)
            return false;
        cv::RotatedRect rect = minAreaRect(contours[iMax]);
        //        if(calDistance(armor.circleCenter, rect.center) / roi > 0.3)
        //            return false;

        armor.circleCenter = rect.center;

        return true;
    }

    static float calDistance(cv::Point2f pt1, cv::Point2f pt2) {
        cv::Point2f dis = pt1 - pt2;
        return static_cast<float>(sqrt(pow(dis.x, 2) + pow(dis.y, 2)));
    }

public:
    EnergyDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    reset();
                },
                 [&](energy_detector_control_atom, const bool enable) {
                     ACTOR_PROTOCOL_CHECK(energy_detector_control_atom, bool);
                     if(mEnabled != enable)
                         reset();
                     mEnabled = enable;
                 },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                     ACTOR_EXCEPTION_PROBE();

                     if(!mEnabled)
                         return;
                     auto [lastUpdate, info, frame] = BlackBoard::instance().get<CameraFrame>(key).value();

                     DetectedEnergyInfo res;
                     res.lastUpdate = lastUpdate;

                     std::pair<cv::Point2f, cv::Point2f> center;
                     if(!detectArmor(frame, center))
                         return;

//                     //TODO : PNP solve
//                     res.armorPoint = Point<UnitType::Distance, FrameOfRef::Robot>{0, 0, 0};
//                     res.rPoint = Point<UnitType::Distance, FrameOfRef::Robot>{0, 0, 0};


                     sendAll(energy_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, res));
                 } };
    }
};

HUB_REGISTER_CLASS(EnergyDetector);
