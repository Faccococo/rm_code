#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/calib3d.hpp>

#include "SuppressWarningEnd.hpp"

struct ArmorLocatorSettings final {
    float ratioThreshold;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorSettings& x) {
    return f.object(x).fields(
        f.field("ratioThreshold",x.ratioThreshold));
}

class ArmorLocator final
    : public HubHelper<caf::event_based_actor, ArmorLocatorSettings, detect_available_atom, image_frame_atom> {
    Identifier mKey, mHeadKey{};
    const std::vector<cv::Point3d> mObjectPointsSmall = {
        { -widthOfSmallArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
        { -widthOfSmallArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfSmallArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfSmallArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
    };
    const std::vector<cv::Point3d> mObjectPointsLarge = {
        { -widthOfLargeArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
        { -widthOfLargeArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfLargeArmor / 2, -heightOfArmorLightBar / 2, 0.0 },
        { +widthOfLargeArmor / 2, +heightOfArmorLightBar / 2, 0.0 },
    };
    std::vector<cv::Point2f> mImagePoint{ 4 };

    static double evalArea(cv::Point2d p1, cv::Point2d p2, cv::Point2d p3) {
        const auto v1 = p2 - p1;
        const auto v2 = p3 - p1;
        return v1.x * v2.y - v1.y * v2.x;
    }

    static double evalArea(cv::Point2d p1, cv::Point2d p2, cv::Point2d p3, cv::Point2d p4) {
        return -evalArea(p1, p2, p3) - evalArea(p1, p3, p4);
    }

    std::pair<Point<UnitType::Distance, FrameOfRef::Camera>, ArmorType>
    solve([[maybe_unused]] cv::Mat& debugView, const cv::Mat& cameraMatrix, const PairedLight& armor) {
        boxRect(mImagePoint, armor.r1);
        const auto area1 = armor.r1.size.area();

        const cv::Point2d lt = 0.5 * (mImagePoint[1] + mImagePoint[2]);
        const cv::Point2d lb = 0.5 * (mImagePoint[0] + mImagePoint[3]);

        boxRect(mImagePoint, armor.r2);
        const auto area2 = armor.r2.size.area();

        const cv::Point2d rt = 0.5 * (mImagePoint[1] + mImagePoint[2]);
        const cv::Point2d rb = 0.5 * (mImagePoint[0] + mImagePoint[3]);

        const auto area = evalArea(lt, lb, rb, rt);

        const cv::Mat_<double> distCoeff;
        cv::Mat rvec, tvec;

        /*
        const auto left = 0.5 * (lt + lb);
        const auto right = 0.5 * (rt + rb);

        const auto distHorizontal = std::hypot(left.x - right.x, left.y - right.y);
        const auto distVertical = 0.5 * (std::hypot(lb.x - lt.x, lb.y - lt.y) + std::hypot(rb.x - rt.x, rb.y - rt.y));

        const auto ratio = distHorizontal / distVertical;
        constexpr auto ratioThreshold = 0.5 * (widthOfLargeArmor + widthOfSmallArmor) / heightOfArmorLightBar;
         */
        const auto ratio = area / std::fmax(0.001, area1 + area2);
        HubLogger::watch("armor ratio", ratio);

        mImagePoint = { lt, lb, rb, rt };
        [[maybe_unused]] const auto res =
            cv::solvePnP(ratio > mConfig.ratioThreshold ? mObjectPointsLarge : mObjectPointsSmall, mImagePoint, cameraMatrix, distCoeff,
                         rvec, tvec, false, cv::SOLVEPNP_IPPE);
        glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        if(p0.z > 0.0)
            p0 = -p0;

#ifdef ARTINXHUB_DEBUG
        cv::drawFrameAxes(debugView, cameraMatrix, distCoeff, rvec, tvec,
                          static_cast<float>(ratio > mConfig.ratioThreshold ? widthOfLargeArmor : widthOfSmallArmor) * 0.5f);
#endif

        return { Point<UnitType::Distance, FrameOfRef::Camera>{ p0 },
                 ratio > mConfig.ratioThreshold ? ArmorType::Large : ArmorType::Small };
    }

public:
    ArmorLocator(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](armor_detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(armor_detect_available_atom, TypedIdentifier<DetectedArmorArray>);
                     ACTOR_EXCEPTION_PROBE();

                     auto data = BlackBoard::instance().get<DetectedArmorArray>(key).value();
                     // logInfo(data.armors[0].armors.size());
                     DetectedTargetArray res;
                     res.lastUpdate = data.frame.lastUpdate;
                     const auto& cameraInfo = data.frame.info;

                     Transform<FrameOfRef::Gun, FrameOfRef::Camera, true> transform;
                     if(cameraInfo.transform.index() == 0) {
                         transform = std::get<0>(cameraInfo.transform);
                     } else {
                         const auto& tfRobot2Camera = std::get<1>(cameraInfo.transform);
                         const auto headInfo = BlackBoard::instance().get<HeadInfo>(mHeadKey).value();
                         transform = combine(headInfo.tfRobot2Gun.invTransformObj(), tfRobot2Camera);
                     }


                     auto debugView = data.frame.frame.clone();

                     for(const auto& [id, armor] : data.armors) {
                         auto armorLight = armor;

                         const auto [point, type] = solve(debugView, cameraInfo.cameraMatrix, armorLight);

                         // TODO: projected area
                         res.targets.push_back(
                             { transform.invTransform(point), 0.0, id, type, decltype(DetectedTarget::velocity){ glm::zero<glm::dvec3>() } });
                     }

#ifdef ARTINXHUB_DEBUG
                     std::swap(debugView, data.frame.frame);
                     sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, std::move(data.frame)));
#endif

                     sendAll(detect_available_atom_v, mGroupMask, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 },
                 [&](update_head_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
                     mHeadKey = key;
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorLocator);
