#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "Timer.hpp"
#include "Utility.hpp"
#include <cstdint>
#include <queue>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>

#include "SuppressWarningEnd.hpp"

static constexpr double zNear = 0.5;
static constexpr double zFar = 50.0;

struct ArmorLocatorTesterSettings final {
    uint32_t count;
    double fov;
    uint32_t imageWidth, imageHeight;
    double length, width, height;
    double noiseStd;
    double maxError;  // distance(expected,error) / distance(expected,origin) or angle
    bool judgeAngle;
};
template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorTesterSettings& x) {
    return f.object(x).fields(f.field("count", x.count), f.field("fov", x.fov), f.field("imageWidth", x.imageWidth),
                              f.field("imageHeight", x.imageHeight), f.field("length", x.length), f.field("width", x.width),
                              f.field("height", x.height), f.field("noiseStd", x.noiseStd), f.field("maxError", x.maxError),
                              f.field("judgeAngle", x.judgeAngle));
}

// NOTICE: ArmorLocator Only
class ArmorLocatorTester final
    : public HubHelper<caf::event_based_actor, ArmorLocatorTesterSettings, armor_detect_available_atom> {
    Identifier mKey;
    glm::dmat4 mMat;
    std::queue<Point<UnitType::Distance, FrameOfRef::Gun>> mExpected{};
    uint32_t mCount = 0;
    double mMeanError = 0.0;

    void next() {
        const auto center = glm::linearRand(glm::dvec3{ -mConfig.width, -mConfig.height, -mConfig.length },
                                            glm::dvec3{ mConfig.width, mConfig.height, -zNear });
        const auto yaw = glm::linearRand(0.1, 0.9) * glm::pi<double>();
        // const auto pitch = glm::linearRand(-0.1, 0.1) * glm::pi<double>();
        constexpr auto pitch = 0.0;

        const auto forward = glm::dvec3{ std::cos(yaw) * std::cos(pitch), std::sin(pitch), std::sin(yaw) * std::cos(pitch) };
        // const auto up = glm::normalize(glm::dvec3{ glm::linearRand(-0.2, 0.2), 1.0, glm::linearRand(-0.2, 0.2) });
        constexpr auto up = glm::dvec3{ 0.0, 1.0, 0.0 };
        const auto horizontal = glm::cross(forward, up);
        const auto vertical = glm::cross(horizontal, forward);

        const auto generateNoise = [&] {
            return glm::clamp(glm::gaussRand(0.0, mConfig.noiseStd), -mConfig.noiseStd * 3.0, mConfig.noiseStd * 3.0);
        };

        const auto generateRotatedRect = [&](const glm::dvec3& vecX) {
            const auto off1 = vecX * (widthOfSmallArmor * 0.5);
            const auto off2 = vecX * (widthOfSmallArmor * 0.5 - widthOfArmorLightBar);
            const auto off3 = vertical * (heightOfArmorLightBar * 0.5);
            const auto off4 = vertical * (heightOfArmorLightBar * -0.5);

            const glm::dvec3 corners[4] = { center + off1 + off3, center + off2 + off3, center + off1 + off4,
                                            center + off2 + off4 };

            std::vector<cv::Point2f> pts;
            pts.reserve(4);

            for(auto& pos : corners) {
                const auto projected = mMat * glm::dvec4{ pos, 1.0 };
                const auto posX = (projected.x / projected.w / 2 + 0.5) * mConfig.imageWidth + generateNoise();
                const auto posY = (0.5 - projected.y / projected.w / 2) * mConfig.imageHeight + generateNoise();
                pts.emplace_back(static_cast<float>(posX), static_cast<float>(posY));
            }

            return cv::minAreaRect(pts);
        };

        mExpected.push(decltype(mExpected)::value_type{ center });

        Armor armor{ 0, PairedLight{ generateRotatedRect(horizontal), generateRotatedRect(-horizontal) } };

        DetectedArmorArray res;

        const cv::Mat cameraMatrix =
            (cv::Mat_<double>(3, 3) << mConfig.imageWidth / 2 / tan(glm::radians(mConfig.fov) / 2), 0, mConfig.imageWidth / 2, 0,
             mConfig.imageHeight / 2 / tan(glm::radians(mConfig.fov) / 2), mConfig.imageHeight / 2, 0, 0, 1);
        const cv::Mat distCoefficients;

        res.frame = CameraFrame{ SynchronizedClock::instance().now(),
                                 { { Transform<FrameOfRef::Gun, FrameOfRef::Camera, true>{ glm::identity<glm::dmat4>() } },
                                   "ArmorLocatorTester",
                                   cameraMatrix,
                                   distCoefficients,
                                   mConfig.imageWidth,
                                   mConfig.imageHeight },
                                 cv::Mat{} };
        res.armors.push_back(std::move(armor));

        sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
    }

public:
    ArmorLocatorTester(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) }, mMat{
              glm::perspectiveFovRH(glm::radians(mConfig.fov), static_cast<double>(mConfig.imageWidth),
                                    static_cast<double>(mConfig.imageHeight), zNear, zFar) *
              glm::lookAtRH(glm::dvec3{ 0.0 }, glm::dvec3{ 0.0, 0.0, -1.0 }, glm::dvec3{ 0.0, 1.0, 0.0 })
          } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    next();
                },
                 [&](detect_available_atom, GroupMask, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(detect_available_atom, GroupMask, TypedIdentifier<DetectedTargetArray>);
                     const auto solved = BlackBoard::instance().get<DetectedTargetArray>(key).value().targets.front().center;

                     const auto expected = mExpected.front();
                     mExpected.pop();

                     const auto expectedRaw = expected.mVal;
                     const auto solvedRaw = solved.mVal;

                     double error;
                     std::string message;
                     bool passAbsolute = false;
                     if(mConfig.judgeAngle) {
                         const auto expectedDir = glm::normalize(expectedRaw);
                         const auto solvedDir = glm::normalize(solvedRaw);
                         const auto angle = std::acos(glm::dot(expectedDir, solvedDir));
                         error = angle / glm::pi<double>();
                         message = fmt::format("Error: {:.2f}% (degree {:.3f})", error * 100.0, glm::degrees(angle));
                     } else {
                         const auto dist = distance(expected, solved).mVal;
                         const auto scale = glm::length(expected.mVal);
                         error = dist / scale;  // relative error
                         message = fmt::format(
                             "Error: {:.2f}% ({:.3f}/{:.3f}) Expected {:.3f} {:.3f} {:.3f} Solved {:.3f} {:.3f} {:.3f}",
                             error * 100.0, dist, scale, expectedRaw.x, expectedRaw.y, expectedRaw.z, solvedRaw.x, solvedRaw.y,
                             solvedRaw.z);
                         passAbsolute = dist < 0.03;  // 3cm
                     }

                     if(error < mConfig.maxError * 2.0 || passAbsolute)
                         logInfo(message);
                     else
                         logError(message);

                     if(error < 1.0) {
                         mMeanError += error;
                         ++mCount;
                     }

                     if(mCount >= mConfig.count) {
                         mMeanError /= mCount;
                         logInfo(fmt::format("Mean error {:.2f}%", mMeanError * 100.0));
                         if(mMeanError < mConfig.maxError)
                             logInfo("Test passed");
                         else
                             logError("Test failed");
                         appendTestResult(fmt::format("Mean error {:.2f}% (Require {:.2f}%) {}", mMeanError * 100.0,
                                                      mConfig.maxError * 100.0, mConfig.judgeAngle ? "Angle" : "Distance"));
                         terminateSystem(*this, mMeanError < mConfig.maxError);
                     } else
                         next();
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorLocatorTester);
