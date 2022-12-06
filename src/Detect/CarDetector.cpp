#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "DetectedCar.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <inference_engine.hpp>

#include "SuppressWarningEnd.hpp"

namespace IE = InferenceEngine;

struct CarDetectorSettings final {
    double nmsThreshold;
    double boundingBoxThreshold;
    uint32_t inputWidth;
    uint32_t inputHeight;
    std::string xmlPath;
    std::string binPath;
    std::string deviceName;
    uint32_t numClasses;
    uint32_t carId;
};

template <class Inspector>
bool inspect(Inspector& f, CarDetectorSettings& x) {
    return f.object(x).fields(
        f.field("nmsThreshold", x.nmsThreshold).fallback(0.45),
        f.field("boundingBoxThreshold", x.boundingBoxThreshold).fallback(0.3), f.field("inputWidth", x.inputWidth),
        f.field("inputHeight", x.inputHeight), f.field("xmlPath", x.xmlPath), f.field("binPath", x.binPath),
        f.field("deviceName", x.deviceName).fallback("CPU"), f.field("numClasses", x.numClasses), f.field("carId", x.carId));
}

class CarDetector final : public HubHelper<caf::event_based_actor, CarDetectorSettings, car_detect_available_atom> {
    Identifier mKey;
    IE::Core mInferenceEngine;
    IE::CNNNetwork mNetwork;
    IE::ExecutableNetwork mExecutableNetwork;
    std::string mInputBlobName, mOutputBlobName;

    double blobFromImage(const cv::Mat& input, const IE::Blob::Ptr& imgBlob) const {
        const auto scale = std::fmin(mConfig.inputWidth / static_cast<double>(input.cols),
                                     mConfig.inputHeight / static_cast<double>(input.rows));
        const auto newWidth = static_cast<int32_t>(scale * input.cols), newHeight = static_cast<int32_t>(scale * input.rows);
        cv::Mat resized;
        cv::resize(input, resized, cv::Size{ newWidth, newHeight });
        cv::Mat full{ static_cast<int>(mConfig.inputHeight), static_cast<int>(mConfig.inputWidth), CV_8UC3,
                      cv::Scalar{ 114, 114, 114 } };
        resized.copyTo(full(cv::Rect{ 0, 0, resized.cols, resized.rows }));

        const auto memoryBlob = IE::as<IE::MemoryBlob>(imgBlob);
        const auto mapping = memoryBlob->wmap();
        auto ptr = mapping.as<float*>();
        for(uint32_t channelIdx = 0; channelIdx < 3; ++channelIdx) {
            for(uint32_t y = 0; y < mConfig.inputHeight; ++y) {
                for(uint32_t x = 0; x < mConfig.inputWidth; ++x) {
                    *(ptr++) = static_cast<float>(full.at<cv::Vec3b>(y, x)[channelIdx]);
                }
            }
        }
        return scale;
    }

    std::vector<cv::Rect> decodeOutputs(const IE::Blob::Ptr& blob, const double scale, const int32_t width,
                                        const int32_t height) {
        const auto memoryBlob = IE::as<IE::MemoryBlob>(blob);
        const auto mapping = memoryBlob->rmap();
        const auto ptr = mapping.as<float*>();

        std::vector<std::tuple<cv::Rect2f, int32_t, float>> proposals;

        const auto generateProposal = [ptr, this, &proposals](const int32_t anchorIdx, const float g0Base, const float g1Base,
                                                              const float stride) {
            const auto basePos = anchorIdx * (mConfig.numClasses + 5);
            const auto centerX = (ptr[basePos + 0] + g0Base) * stride;
            const auto centerY = (ptr[basePos + 1] + g1Base) * stride;
            const auto w = std::exp(ptr[basePos + 2]) * stride;
            const auto h = std::exp(ptr[basePos + 3]) * stride;

            const auto objectness = ptr[basePos + 4];

            auto maxProb = static_cast<float>(mConfig.boundingBoxThreshold);
            uint32_t selected = std::numeric_limits<uint32_t>::max();

            for(uint32_t classIdx = 0; classIdx < mConfig.numClasses; ++classIdx) {
                const auto classScore = ptr[basePos + 5 + classIdx];
                const auto prob = objectness * classScore;
                if(prob > maxProb) {
                    maxProb = prob;
                    selected = classIdx;
                }
            }

            if(selected == mConfig.carId) {  // just detect cars
                const auto x0 = centerX - w * 0.5f;
                const auto y0 = centerY - h * 0.5f;
                proposals.push_back({ cv::Rect2f{ cv::Point2f{ x0, y0 }, cv::Size2f{ w, h } }, selected, maxProb });
            }
        };

        const auto strides = { 8, 16, 32 };
        int32_t anchorIdx = 0;
        for(auto stride : strides) {
            const auto gridCountW = mConfig.inputWidth / stride;
            const auto gridCountH = mConfig.inputHeight / stride;

            for(uint32_t g1 = 0; g1 < gridCountH; ++g1)
                for(uint32_t g0 = 0; g0 < gridCountW; ++g0) {
                    generateProposal(anchorIdx++, static_cast<float>(g0), static_cast<float>(g1), static_cast<float>(stride));
                }
        }

        std::sort(proposals.begin(), proposals.end(),
                  [](const auto& lhs, const auto& rhs) { return std::get<float>(lhs) > std::get<float>(rhs); });

        // apply NMS
        std::vector<std::tuple<cv::Rect2f, int32_t, float>> picked;
        for(auto& proposal : proposals) {
            bool keepFlag = true;
            const auto& [rectA, classIdxA, probA] = proposal;
            const auto areaA = rectA.area();

            for(const auto& [rectB, classIdxB, probB] : picked) {
                const auto interArea = (rectA & rectB).area();
                const auto outerArea = areaA + rectB.area() - interArea;
                const auto IoU = interArea / outerArea;
                if(IoU > static_cast<float>(mConfig.nmsThreshold)) {
                    keepFlag = false;
                    break;
                }
            }

            if(keepFlag)
                picked.push_back(proposal);
        }

        std::vector<cv::Rect> detected;
        detected.reserve(picked.size());

        for(const auto& [rect, classIdx, prob] : picked) {
            const auto x0 = rect.x / scale;
            const auto y0 = rect.y / scale;
            const auto x1 = (static_cast<double>(rect.x) + rect.width) / scale;
            const auto y1 = (static_cast<double>(rect.y) + rect.height) / scale;

            const auto ix0 = static_cast<int32_t>(std::fmax(std::fmin(x0, static_cast<double>(width) - 1.0), 0.0));
            const auto iy0 = static_cast<int32_t>(std::fmax(std::fmin(y0, static_cast<double>(height) - 1.0), 0.0));
            const auto ix1 = static_cast<int32_t>(std::fmax(std::fmin(x1, static_cast<double>(width) - 1.0), 0.0));
            const auto iy1 = static_cast<int32_t>(std::fmax(std::fmin(y1, static_cast<double>(height) - 1.0), 0.0));

            detected.push_back({ cv::Point{ ix0, iy0 }, cv::Point{ ix1, iy1 } });
        }

        return detected;
    }

public:
    CarDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        ACTOR_EXCEPTION_PROBE();

        mNetwork = mInferenceEngine.ReadNetwork(mConfig.xmlPath, mConfig.binPath);
        auto [outputBlobName, outputBlob] = *mNetwork.getOutputsInfo().begin();
        mOutputBlobName = outputBlobName;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
        outputBlob->setPrecision(IE::Precision::FP16);

        const auto devices = mInferenceEngine.GetAvailableDevices();
        for(const auto& device : devices) {
            logInfo("Available inference engine device: " + device);
        }

        mExecutableNetwork = mInferenceEngine.LoadNetwork(mNetwork, mConfig.deviceName);
        auto [inputBlobName, inputBlob] = *mNetwork.getInputsInfo().begin();
        mInputBlobName = inputBlobName;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                     ACTOR_EXCEPTION_PROBE();

                     DetectedCarArray res;
                     res.frame = BlackBoard::instance().get<CameraFrame>(key).value();

                     auto request = mExecutableNetwork.CreateInferRequest();
                     const auto inputBlob = request.GetBlob(mInputBlobName);
                     const auto scale = blobFromImage(res.frame.frame, inputBlob);

                     const auto t0 = Clock::now();

                     request.Infer();  // TODO: asynchronously?

                     const auto t1 = Clock::now();

                     const auto outputBlob = request.GetBlob(mOutputBlobName);
                     res.cars = decodeOutputs(outputBlob, scale, res.frame.frame.cols, res.frame.frame.rows);
                     const auto t2 = Clock::now();

                     logInfo(fmt::format("infer time {:.4f}s decode time {:.4f}s", static_cast<double>((t1 - t0).count()) / 1e9,
                                         static_cast<double>((t2 - t1).count()) / 1e9));

                     sendAll(car_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(CarDetector);
