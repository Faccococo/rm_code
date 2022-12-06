#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "RadarCameraPoints.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/calib3d.hpp>

#include "SuppressWarningEnd.hpp"

struct RadarLocatorSetting final {};

template <class Inspector>
bool inspect(Inspector& f, RadarLocatorSetting& x) {
    return f.object(x).fields();
}

class RadarLocator final : public HubHelper<caf::event_based_actor, RadarLocatorSetting, radar_locate_succeed_atom> {
    Identifier mKey;
    std::vector<cv::Point3f> mObjectPoints = {
        cv::Point3f(1.51f, 7.5f, 1.12f),  // from rival's base,clockwise
        cv::Point3f(12.897f, 1.867f, 0.6f), cv::Point3f(19.195f, 8.612f, 0.615f),  cv::Point3f(19.195f, 9.272f, 0.615f),
        cv::Point3f(12.03f, 10.500f, 0.6f), cv::Point3f(10.931f, 12.546f, 1.228f),  // guardStation's height unknown, can't find
                                                                                    // in manual
        /*could add two additional points but may be too many points
         *cv::Point3f(11.446,11.653,0.000),
         *cv::Point3f(11.446,13.44,0.000),
         */
    };

    std::optional<glm::dmat4> locatePosition(const cv::Mat& cameraMatrix, const std::vector<cv::Point2f>& imagePoints,
                                             const Color selfColor) {
        const cv::Mat_<double> distCoeff;
        cv::Mat rvec, tvec;
        if(cv::solvePnP(mObjectPoints, imagePoints, cameraMatrix, distCoeff, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
            cv::Mat rotateMat;
            cv::Rodrigues(rvec, rotateMat);
            glm::dmat3 rotate{};
            memcpy(glm::value_ptr(rotate), rotateMat.ptr(), sizeof(double) * 3 * 3);
            glm::dmat4 trans = { rotate };

            if(selfColor == Color::Blue) {
                trans[3][0] = tvec.at<double>(0, 0);
                trans[3][1] = tvec.at<double>(1, 0);
                trans[3][2] = tvec.at<double>(2, 0);
            } else {
                trans[3][0] = 28 - tvec.at<double>(0, 0);
                trans[3][1] = 15 - tvec.at<double>(1, 0);
                trans[3][2] = tvec.at<double>(2, 0);
            }
            return trans;
        }
        return std::nullopt;
    }

public:
    RadarLocator(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](radar_locate_request_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(radar_locate_request_atom, TypedIdentifier<RadarCameraPointsArray>);
                     ACTOR_EXCEPTION_PROBE();

                     const auto data = BlackBoard::instance().get<RadarCameraPointsArray>(key).value();
                     const auto& info = data.cameraInfo;
                     if(const auto radarTransform = locatePosition(info.cameraMatrix, data.imagePoints, data.selfColor)) {
                         const Transform<FrameOfRef::Camera, FrameOfRef::Ground, true> transform{ glm::inverse(
                             radarTransform.value()) };
                         sendAll(radar_locate_succeed_atom_v, BlackBoard::instance().updateSync(mKey, transform));
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(RadarLocator);
