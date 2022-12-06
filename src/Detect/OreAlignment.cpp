#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedOre.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <magic_enum.hpp>
#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

struct OreAlignmentSettings final {
    // HSV range
    std::vector<double> goldOreHsvLow;
    std::vector<double> goldOreHsvHigh;
    std::vector<double> silverOreHsvLow;
    std::vector<double> silverOreHsvHigh;
    std::vector<double> lightBarHsvLow;
    std::vector<double> lightBarHsvHigh;
    // area range(pixel,[low value, high value])
    std::vector<uint32_t> overGoldAreaRange;
    std::vector<uint32_t> groundGoldAreaRange;
    std::vector<uint32_t> groundSilverAreaRange;
    std::vector<uint32_t> lightbarAreaRange;

    int32_t altitude;      // between ore and light bar(pixel)
    int32_t widthExpand;   // expand search range on width(pixel)
    int32_t heightExpand;  //...
    int32_t historyFrameCount;
    double flashFrequencyLimit;  // double in [0,1.0]
    double distanceToOre;
    double offset;  // positive when camera is at the right of the car center axis(meter)
    double limitMovementDistance;
};

template <class Inspector>
bool inspect(Inspector& f, OreAlignmentSettings& x) {
    return f.object(x).fields(
        f.field("goldOreHsvLow", x.goldOreHsvLow).invariant([](auto& c) { return c.size() == 3; }),
        f.field("goldOreHsvHigh", x.goldOreHsvHigh).invariant([](auto& c) { return c.size() == 3; }),
        f.field("silverOreHsvLow", x.silverOreHsvLow).invariant([](auto& c) { return c.size() == 3; }),
        f.field("silverOreHsvHigh", x.silverOreHsvHigh).invariant([](auto& c) { return c.size() == 3; }),
        f.field("lightBarHsvLow", x.lightBarHsvLow).invariant([](auto& c) { return c.size() == 3; }),
        f.field("lightBarHsvHigh", x.lightBarHsvHigh).invariant([](auto& c) { return c.size() == 3; }),
        f.field("overGoldAreaRange", x.overGoldAreaRange).invariant([](auto& c) { return c.size() == 2; }),
        f.field("groundGoldAreaRange", x.groundGoldAreaRange).invariant([](auto& c) { return c.size() == 2; }),
        f.field("groundSilverAreaRange", x.groundSilverAreaRange).invariant([](auto& c) { return c.size() == 2; }),
        f.field("lightbarAreaRange", x.lightbarAreaRange).invariant([](auto& c) { return c.size() == 2; }),
        f.field("altitude", x.altitude), f.field("widthExpand", x.widthExpand), f.field("heightExpand", x.heightExpand),
        f.field("historyFrameCount", x.historyFrameCount), f.field("flashFrequencyLimit", x.flashFrequencyLimit),
        f.field("offset", x.offset), f.field("limitMovementDistance", x.limitMovementDistance));
}

class OreAlignment final : public HubHelper<caf::event_based_actor, OreAlignmentSettings, ore_alignment_available_atom> {
    Identifier mKey;

    std::vector<cv::Rect> mOreRectArray;
    cv::Mat mFrameBgr, mFrameHsv, mBinaryOreFrame;
    std::deque<OrePosition> mOrePositionHistory;

    OreAlignmentMessage solveDirection(OreAlignmentMessage& oreMessage, const OreAlignmentSettings& settings) {
        double tempDistance = 0.0;

        switch(oreMessage.detectMode) {
            case OreDetectorMode::GOLD_OVER: {
                detectOres(oreMessage, { settings.goldOreHsvLow, settings.goldOreHsvHigh }, settings.overGoldAreaRange);
                addToHistory(detectLightBar(oreMessage.frame.frame, mOreRectArray, settings), oreMessage.lastMode,
                             settings);  // todo:fix this for the no need of the history strategy
                uint32_t lightOutFrames = 0;
                for(auto& atom : mOrePositionHistory)
                    lightOutFrames += (atom.flashingIndex == atom.totalNum) ? 0 : 1;
                if(static_cast<double>(lightOutFrames) / static_cast<double>(settings.historyFrameCount) >=
                   settings.flashFrequencyLimit) {
                    std::for_each(mOrePositionHistory.rbegin(), mOrePositionHistory.rend(), [&](const OrePosition& atom) {
                        if(atom.flashingIndex != atom.totalNum)
                            tempDistance = transformToRealDistance(mOreRectArray[atom.flashingIndex], oreMessage.frame, settings);
                    });
                } else {
                    tempDistance = std::numeric_limits<double>::infinity();
                }
            } break;

            case OreDetectorMode::GOLD_GROUND: {
                detectOres(oreMessage, { settings.goldOreHsvLow, settings.goldOreHsvHigh }, settings.groundGoldAreaRange);
                tempDistance = transformToRealDistance(mOreRectArray[0], oreMessage.frame, settings);
                for(auto& atom : mOreRectArray) {
                    tempDistance = (transformToRealDistance(atom, oreMessage.frame, settings) < abs(tempDistance)) ?
                        transformToRealDistance(atom, oreMessage.frame, settings) :
                        tempDistance;
                }
            } break;

            case OreDetectorMode::SILVER_GROUND: {
                detectOres(oreMessage, { settings.silverOreHsvLow, settings.silverOreHsvHigh }, settings.groundSilverAreaRange);
                tempDistance = transformToRealDistance(mOreRectArray[0], oreMessage.frame, settings);
                for(auto& atom : mOreRectArray) {
                    tempDistance = (transformToRealDistance(atom, oreMessage.frame, settings) < abs(tempDistance)) ?
                        transformToRealDistance(atom, oreMessage.frame, settings) :
                        tempDistance;
                }
            } break;

            case OreDetectorMode::NONE:
                tempDistance = std::numeric_limits<double>::infinity();
                break;
        }
        return OreAlignmentMessage{ oreMessage.frame, OreDetectorMode::NONE, oreMessage.detectMode, tempDistance };
    }

    void detectOres(const OreAlignmentMessage& message, const std::vector<std::vector<double>>& hsvRange,
                    const std::vector<uint32_t>& areaRange) {
        std::vector<std::vector<cv::Point2i>> orePointsArray;
        switch(message.detectMode) {
            case OreDetectorMode::SILVER_GROUND:
                [[fallthrough]];
            case OreDetectorMode::GOLD_OVER:
                [[fallthrough]];
            case OreDetectorMode::GOLD_GROUND:
                [[fallthrough]];
            case OreDetectorMode::NONE:
                inRange(mFrameHsv, hsvRange[0], hsvRange[1], mBinaryOreFrame);
                break;
        }
        // cv::Mat tempMat = binaryOreFrame.clone();
        // dilate(tempMat, binaryOreFrame, std::vector<int32_t>{ 1, 1, 1, 1, 1, 1, 1, 1, 1 });  // todo:improve ore's shape

        findContours(mBinaryOreFrame, orePointsArray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for(const auto& singleMine : orePointsArray) {

            if(cv::Rect tempRect = boundingRect(singleMine);
               areaRange[0] < static_cast<uint32_t>(tempRect.area()) && areaRange[1] > static_cast<uint32_t>(tempRect.area())) {
                mOreRectArray.push_back(tempRect);
                cv::rectangle(mFrameBgr, tempRect, cv::Scalar(255, 0, 0), 1, cv::LINE_8);
            }
        }
        std::sort(mOreRectArray.begin(), mOreRectArray.end(),
                  [](const cv::Rect& front, const cv::Rect& back) { return front.x < back.x; });
    }

    OrePosition detectLightBar(const cv::Mat& frame, const std::vector<cv::Rect>& oreRectArray,
                               const OreAlignmentSettings& settings) {
        cv::Mat lightBarFrame;
        uint64_t index = 0;
        std::vector<std::vector<cv::Point2i>> lightBarPointsArray;
        for(const auto& oreRect : oreRectArray) {
            cv::Mat cutFrame = frame.rowRange(oreRect.y - settings.heightExpand - settings.altitude,
                                              oreRect.y + oreRect.height + settings.heightExpand - settings.altitude);
            cutFrame = cutFrame.colRange(oreRect.x - settings.widthExpand, oreRect.x + oreRect.width + settings.widthExpand);

            cv::inRange(cutFrame, settings.lightBarHsvLow, settings.lightBarHsvHigh, lightBarFrame);
            cv::findContours(lightBarFrame, lightBarPointsArray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for(const auto& singleLightBar : lightBarPointsArray) {
                if(auto tempRect = cv::boundingRect(singleLightBar);
                   settings.lightbarAreaRange[0] < static_cast<uint32_t>(tempRect.area()) &&
                   static_cast<uint32_t>(tempRect.area()) < settings.lightbarAreaRange[1]) {
                    ++index;
                    // draw the signal light on frame, could be removed
                    tempRect = cv::Rect(cv::Point2i(tempRect.x + oreRect.x - settings.widthExpand,
                                                    tempRect.y + oreRect.y - settings.heightExpand),
                                        tempRect.size());
                    cv::rectangle(mFrameBgr, tempRect, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
                }
            }
        }
        return OrePosition{ oreRectArray.size(), index };
    }

    void addToHistory(const OrePosition& currentPosition, OreDetectorMode lastMode, const OreAlignmentSettings& settings) {
        if(lastMode != OreDetectorMode::GOLD_OVER && !mOrePositionHistory.empty()) {
            mOrePositionHistory.clear();
        } else {
            if(mOrePositionHistory.size() < static_cast<size_t>(settings.historyFrameCount))
                mOrePositionHistory.push_back(currentPosition);
            else {
                mOrePositionHistory.pop_front();
                mOrePositionHistory.push_back(currentPosition);
            }
        }  // todo:add frame filter
    }

    double transformToRealDistance(const cv::Rect& rect, const CameraFrame& frame, const OreAlignmentSettings& settings) {
        // FIXME: use cameraMatrix instead
        return (rect.x + static_cast<double>(rect.width) / 2.0 - frame.info.width) /
            (frame.info.width / 2.0 / std::tan(glm::radians(30.0))) * settings.distanceToOre +
            settings.offset;
        /*const auto res = cv::solvePNP()*/
    }  // TODO : may don't have enough precision!!!

public:
    OreAlignment(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](ore_alignment_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(ore_alignment_available_atom, TypedIdentifier<OreAlignmentMessage>);
                     ACTOR_EXCEPTION_PROBE();

                     auto data = BlackBoard::instance().get<OreAlignmentMessage>(key).value();

                     data.frame.frame.convertTo(mFrameBgr, CV_32FC3, 1.0 / 255.0);
                     cv::cvtColor(mFrameBgr, mFrameHsv, cv::COLOR_BGR2HSV_FULL);
                     sendAll(ore_alignment_available_atom_v,
                             BlackBoard::instance().updateSync(mKey, solveDirection(data, mConfig)));
                 } };
    }
};

HUB_REGISTER_CLASS(OreAlignment);
