#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <opencv2/barcode.hpp>

#include "SuppressWarningEnd.hpp"

struct OreExchangeRectifierSettings final {
    std::string superResProtoTxt;
    std::string superResCaffeModel;
    int validDetectionRequired;
    double permissibleAngleRangeOfError;
    double deltaTime;
    double movingRate;
};

template <class Inspector>
bool inspect(Inspector& f, OreExchangeRectifierSettings& x) {
    return f.object(x).fields(
        f.field("superResProtoTxt", x.superResProtoTxt).invariant([](const std::string& superResProtoTxt) {
            return fs::exists(superResProtoTxt) && fs::is_character_file(superResProtoTxt);
        }),
        f.field("superResCaffeModel", x.superResCaffeModel).invariant([](const std::string& superResCaffeModel) {
            return fs::exists(superResCaffeModel) && fs::is_regular_file(superResCaffeModel);
        }),
        f.field("validDetectionRequired", x.validDetectionRequired).invariant([](const int& validDetectionRequired) {
            return validDetectionRequired > 0;
        }),
        f.field("permissibleAngleRangeOfError", x.permissibleAngleRangeOfError)
            .invariant([](const double& permissibleAngleRangeOfError) { return permissibleAngleRangeOfError > 0; }),
        f.field("deltaTime", x.deltaTime).invariant([](const double& deltaTime) { return deltaTime > 0; }),
        f.field("movingRate", x.movingRate).invariant([](const double& movingRate) { return movingRate > 0; }));
}
class OreExchangeRectifier final
    : public HubHelper<caf::event_based_actor, OreExchangeRectifierSettings, ore_detect_available_atom> {
    Identifier mKey;
    enum class AutomataStates { Off, Finding, Learning, BackFinding, Rectifying };
    AutomataStates mAutomataState{ AutomataStates::Off };
    cv::barcode::BarcodeDetector mBarcodeDetector{ mConfig.superResProtoTxt, mConfig.superResCaffeModel };
    cv::Rect oreBackSurfaceDetect(const cv::Mat& image) const {
        cv::Mat corners;
        if(!mBarcodeDetector.detect(image, corners))
            return {};
        CAF_ASSERT(corners.cols == 4 && corners.channels() == 2);
        std::vector<cv::Rect> result(corners.rows);
        for(int i = 0; i < corners.rows; ++i) {
            std::vector<cv::Point2f> points(4);
            for(int j = 0; j < 4; ++j) {
                points[j] = { corners.at<cv::Vec2f>(i, j)[0], corners.at<cv::Vec2f>(i, j)[1] };
            }
            result[i] = cv::minAreaRect(points).boundingRect();
        }
        return *std::max_element(result.begin(), result.end(),
                                 [](const cv::Rect& rect1, const cv::Rect& rect2) { return rect1.area() < rect2.area(); });
    }

    void off() {
        mValidDetectionCount = 0;
        mMaxHeight = std::numeric_limits<int32_t>::min();
        mAngularVelocity = -mConfig.movingRate;
        mPrevTheta = 0;  // reset to initial value.
    }

    int mValidDetectionCount;  // 0
    void finds(const cv::Rect& rect) {
        send(+mConfig.movingRate);
        if(rect.empty()) {
            mValidDetectionCount = 0;
            return;
        }
        mValidDetectionCount++;
        // only when a consecutive image sequence is found to have bar code, do we go to the next state.
        if(mValidDetectionCount >= mConfig.validDetectionRequired) {
            mValidDetectionCount = 0;  // reuse for validMisDetectionCount.
            mAutomataState = AutomataStates::Learning;
        }
    }
    int mMaxHeight;  // std::numeric_limits<int32_t>::min()
    void learns(const cv::Rect& rect) {
        auto& validMisDetectionCount = mValidDetectionCount;
        const auto& validMisDetectionRequired = mConfig.validDetectionRequired;
        send(+mConfig.movingRate);
        if(rect.empty()) {
            validMisDetectionCount++;
            // only when a consecutive image sequence is found not to have bar code, do we go to the next state.
            if(validMisDetectionCount >= validMisDetectionRequired) {
                validMisDetectionCount = 0;  // reuse for mValidDetectionCount.
                mAutomataState = AutomataStates::BackFinding;
            }
        } else {
            validMisDetectionCount = 0;
            mMaxHeight = std::max((int)mMaxHeight, rect.height);
        }
    }
    void backFinds(const cv::Rect& rect) {
        send(-mConfig.movingRate);
        if(rect.empty()) {
            mValidDetectionCount = 0;
            return;
        }
        mValidDetectionCount++;
        // only when a consecutive image sequence is found to have bar code, do we go to the next state.
        if(mValidDetectionCount >= mConfig.validDetectionRequired) {
            mValidDetectionCount = 0;
            mAutomataState = AutomataStates::Rectifying;
        }
    }
    double mAngularVelocity;  //-1
    double mPrevTheta;        // 0
    void rectifies(const cv::Rect& rect) {
        send(mAngularVelocity);
        if(rect.empty()) {
            return;  // If some frames don`t find the bar code, it does not matter.
        } else {
            const double theta = glm::acos(rect.height / mMaxHeight);  // we cannot know its direction.
            const double deltaTheta = theta - mPrevTheta;
            mPrevTheta = theta;
            if(theta < mConfig.permissibleAngleRangeOfError)
                mValidDetectionCount++;
            if(mValidDetectionCount >= mConfig.validDetectionRequired) {
                send(-90);
                mAutomataState = AutomataStates::Off;
                return;
            }
            // if theta is declining, then the sign of velocity is correct.
            // if theta is increasing, then the sign of velocity is wrong.
            mAngularVelocity =
                glm::sign(mAngularVelocity) * (-deltaTheta / mConfig.deltaTime);  // it seems velocity is reasonable.
        }
    }
    // TODO: let ore_detect_available_atom (SerialPort.cpp) handle it.
    void send(double pitch) {
        sendAll(ore_detect_available_atom_v, pitch);
    }

public:
    OreExchangeRectifier(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return {
            [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [&](ore_instructions_atom, bool /*may be useful*/, Identifier) {
                // ACTOR_PROTOCOL_CHECK(ore_instructions_atom, bool, TypedIdentifier<int>);  // TODO: value type of key
                off();  // initialize the values for member variable.
                // situation 1: OFF rectifier is waked up.
                // situation 2: ON or other state, someone wants to stop it when some exceptions may be observed.
                mAutomataState = (mAutomataState == AutomataStates::Off ? AutomataStates::Finding : AutomataStates::Off);
            },
            [&](image_frame_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                ACTOR_EXCEPTION_PROBE();

                if(mAutomataState == AutomataStates::Off) {
                    off();
                    return;
                }
                const auto rect =
                    oreBackSurfaceDetect(BlackBoard::instance().get<CameraFrame>(key).value().frame);  // safe and need not copy
                switch(mAutomataState) {
                    case AutomataStates::Finding: {
                        finds(rect);
                        return;
                    }
                    case AutomataStates::Learning: {
                        learns(rect);
                        return;
                    }
                    case AutomataStates::BackFinding: {
                        backFinds(rect);
                        return;
                    }
                    case AutomataStates::Rectifying: {
                        rectifies(rect);
                        return;
                    }
                    case AutomataStates::Off: {
                        return;
                        // raiseError("Invalid state.");
                    }
                }
            },
        };
    }
};
HUB_REGISTER_CLASS(OreExchangeRectifier);
