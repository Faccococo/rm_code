#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "DetectedCar.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <limits>
#include <vector>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <opencv2/tracking.hpp>

#include "SuppressWarningEnd.hpp"

using MultiTrackers = std::vector<cv::Ptr<cv::TrackerKCF>>;
using VectorRect = std::vector<cv::Rect>;

constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr uint32_t maxNumRobot = 10;
constexpr double iouThreshold = 0.5;

class CarTracker final : public HubHelper<caf::event_based_actor, void, car_detect_available_atom> {
    Identifier mKey;
    MultiTrackers mTrackers = MultiTrackers(maxNumRobot, nullptr);
    VectorRect mTrackedBoxes = VectorRect(maxNumRobot, cv::Rect(0, 0, 0, 0));
    bool mInitialFlag = false;
    int32_t mInitialisedTrackerNum = 0;

    std::vector<double> computeIoUAdjacencyMat(const VectorRect& detectRectRes, const VectorRect& trackRectRes) {
        std::vector<double> res(trackRectRes.size() * detectRectRes.size());
        constexpr auto computeIoU = [](const cv::Rect& rect1, const cv::Rect rect2) {
            const auto distX = abs(rect1.x - rect2.x);
            const auto width = (rect1.x < rect2.x ? rect1.width : rect2.width) - distX;
            const auto distY = abs(rect1.y - rect2.y);
            const auto height = (rect1.y < rect2.y ? rect1.height : rect2.height) - distY;
            auto areaIoU = width * height;
            if(width < 0 && height < 0) {
                areaIoU *= -1;
            }
            return areaIoU;
        };
        for(size_t i = 0; i < trackRectRes.size(); ++i) {
            for(size_t j = 0; j < detectRectRes.size(); ++j) {
                res[i * detectRectRes.size() + j] = computeIoU(trackRectRes[i], detectRectRes[j]) + 1e6;
            }
        }
        return res;
    }

public:
    CarTracker(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        for(uint32_t i = 0; i < maxNumRobot; i++) {
            mTrackers[i] = cv::TrackerKCF::create();
        }
    }

    caf::behavior make_behavior() override {
        return {
            [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [&](image_frame_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                ACTOR_EXCEPTION_PROBE();

                if(!mInitialFlag) {
                    return;
                }
                const auto data = BlackBoard::instance().get<CameraFrame>(key).value();

                const auto t1 = Clock::now();
                DetectedCarArray carTrackedRes;
                carTrackedRes.frame = data;

                for(int i = 0; i < mInitialisedTrackerNum; ++i) {
                    if(mTrackers[i]->update(carTrackedRes.frame.frame, mTrackedBoxes[i])) {
                        carTrackedRes.cars.push_back(mTrackedBoxes[i]);
                    }
                }
                const auto t2 = Clock::now();
                logInfo(fmt::format("image_frame_atom:track time {:.4f}s", static_cast<double>((t2 - t1).count()) / 1e9));
                sendAll(car_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes)));
            },
            [&](car_detect_available_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(car_detect_available_atom, TypedIdentifier<DetectedCarArray>);
                ACTOR_EXCEPTION_PROBE();

                const auto carDetectedRes = BlackBoard::instance().get<DetectedCarArray>(key).value();

                const auto t1 = Clock::now();
                DetectedCarArray carTrackedRes;
                carTrackedRes.cars = carDetectedRes.cars;
                carTrackedRes.frame = carDetectedRes.frame;

                if(!mInitialFlag) {
                    for(size_t i = 0; i < carDetectedRes.cars.size(); ++i) {
                        mTrackedBoxes[i] = carDetectedRes.cars[i];
                        mTrackers[i]->init(carTrackedRes.frame.frame, mTrackedBoxes[i]);
                        mInitialisedTrackerNum++;
                    }
                    mInitialFlag = true;

                    sendAll(car_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes)));
                } else {
                    VectorRect trackRectRes;
                    std::vector<uint32_t> trackRectIndex;
                    for(int i = 0; i < mInitialisedTrackerNum; ++i) {
                        if(mTrackers[i]->update(carTrackedRes.frame.frame, mTrackedBoxes[i])) {
                            trackRectRes.push_back(mTrackedBoxes[i]);
                            trackRectIndex.push_back(i);
                        }
                    }
                    if(trackRectRes.size() >= carDetectedRes.cars.size()) {
                        const auto iouAdjacencyMat = computeIoUAdjacencyMat(carDetectedRes.cars, trackRectRes);
                        const auto matchResult = solveKM(static_cast<uint32_t>(carDetectedRes.cars.size()),
                                                         static_cast<uint32_t>(trackRectRes.size()), iouAdjacencyMat);

                        for(size_t i = 0; i < trackRectRes.size(); ++i) {
                            if(matchResult[i] < carDetectedRes.cars.size()) {
                                if(iouAdjacencyMat[matchResult[i] * trackRectRes.size() + i] >
                                   iouThreshold * trackRectRes[i].area() + 1e6) {
                                    mTrackedBoxes[trackRectIndex[i]] = carDetectedRes.cars[matchResult[i]];
                                    continue;
                                }
                            }
                            carTrackedRes.cars.push_back(trackRectRes[i]);
                        }
                    } else {
                        mTrackers.clear();
                        mTrackedBoxes.clear();
                        mInitialisedTrackerNum = 0;
                        for(const auto& detectedRect : carDetectedRes.cars) {
                            mTrackers.push_back(cv::TrackerKCF::create());
                            mTrackedBoxes.push_back(detectedRect);
                            mTrackers[mInitialisedTrackerNum]->init(carTrackedRes.frame.frame,
                                                                    mTrackedBoxes[mInitialisedTrackerNum]);
                            mInitialisedTrackerNum++;
                        }
                    }

                    const auto t2 = Clock::now();
                    logInfo(fmt::format("carDetect_atom:track time {:.4f}s", static_cast<double>((t2 - t1).count()) / 1e9));
                    sendAll(car_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(carTrackedRes)));
                }
            },
        };
    }
};

HUB_REGISTER_CLASS(CarTracker);
