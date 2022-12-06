#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "Utility.hpp"

#include <string>
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

#include <Eigen/Core>
#include <inference_engine.hpp>
#include <string.h>

namespace IE = InferenceEngine;

struct NNetArmorDetectorSettings final {
    std::string networkPath;  // network training file path
    int inputWidth;
    int inputHeight;
    int numClasses;        // Number of classes 8
    int numColors;         // Number of color 4
    float bboxConfThresh;  // 0.6
    uint32_t topK;         // TopK
    float nmsThresh;       // 0.3
    float fftConfError;    // 0.15
    float fftMinIou;       // 0.9
};

template <class Inspector>
bool inspect(Inspector& f, NNetArmorDetectorSettings& x) {
    return f.object(x).fields(f.field("networkPath", x.networkPath), f.field("inputWidth", x.inputWidth),
                              f.field("inputHeight", x.inputHeight), f.field("numClasses", x.numClasses),
                              f.field("numColors", x.numColors), f.field("bboxConfThresh", x.bboxConfThresh),
                              f.field("topK", x.topK), f.field("nmsThresh", x.nmsThresh), f.field("fftConfError", x.fftConfError),
                              f.field("fftMinIou", x.fftMinIou));
}

/**
 * @brief 存储任务所需数据的结构体
 *
 */
struct GridAndStride final {
    int grid0;
    int grid1;
    int stride;
};

class NNetArmorDetector final
    : public HubHelper<caf::event_based_actor, NNetArmorDetectorSettings, armor_nnet_detect_available_atom, image_frame_atom> {
    Identifier mKey;
    IE::Core mIe;
    IE::CNNNetwork mNetwork;
    IE::ExecutableNetwork mExeNetwork;
    IE::InferRequest mInferRequest;

    InferenceEngine::MemoryBlob::Ptr mInputMemBlobPtr;
    InferenceEngine::MemoryBlob::CPtr mOutputMemBlobPtr;

    Eigen::Matrix<float, 3, 3> mTransformMatrix;

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

    cv::Mat scaledResize(const cv::Mat& img, Eigen::Matrix<float, 3, 3>& transformMatrix) {
        float r = std::min(mConfig.inputWidth / (img.cols * 1.0), mConfig.inputHeight / (img.rows * 1.0));
        int unpadWidth = r * img.cols;
        int unpadHeight = r * img.rows;

        int dw = mConfig.inputWidth - unpadWidth;
        int dh = mConfig.inputHeight - unpadHeight;

        dw /= 2;
        dh /= 2;

        transformMatrix << 1.0 / r, 0, -dw / r, 0, 1.0 / r, -dh / r, 0, 0, 1;

        cv::Mat re;
        cv::resize(img, re, cv::Size(unpadWidth, unpadHeight));
        cv::Mat out;
        cv::copyMakeBorder(re, out, dh, dh, dw, dw, cv::BorderTypes::BORDER_CONSTANT);

        return out;
    }

    void generateGridsAndStride(int targetWidth, int targetHeight, std::vector<int>& strides,
                                std::vector<GridAndStride>& gridStrides) {
        for(auto stride : strides) {
            int numGridWidth = targetWidth / stride;
            int numGridHeight = targetHeight / stride;

            for(int g1 = 0; g1 < numGridHeight; g1++) {
                for(int g0 = 0; g0 < numGridWidth; g0++) {
                    gridStrides.push_back(GridAndStride{ g0, g1, stride });
                }
            }
        }
    }

    int argmax(const float* ptr, int len) {
        int maxArg = 0;
        for(int i = 1; i < len; i++) {
            if(ptr[i] > ptr[maxArg])
                maxArg = i;
        }
        return maxArg;
    }

    void generateYoloxProposals(const std::vector<GridAndStride>& gridStrides, const float* featPtr,
                                const Eigen::Matrix<float, 3, 3>& transformMatrix, float probThreshold,
                                std::vector<NNetDetectedArmor>& armors) {

        const int numAnchors = gridStrides.size();
        // Travel all the anchors
        for(int anchorIndex = 0; anchorIndex < numAnchors; anchorIndex++) {
            const int grid0 = gridStrides[anchorIndex].grid0;
            const int grid1 = gridStrides[anchorIndex].grid1;
            const int stride = gridStrides[anchorIndex].stride;

            // 9 means(x1, y1, x2, y2, x3, y3, x4, y4, confidence).size()
            const int basicPos = anchorIndex * (9 + mConfig.numColors + mConfig.numClasses);

            // yolox/models/yolo_head.py decode logic
            //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
            //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
            float x1 = (featPtr[basicPos + 0] + grid0) * stride;
            float y1 = (featPtr[basicPos + 1] + grid1) * stride;
            float x2 = (featPtr[basicPos + 2] + grid0) * stride;
            float y2 = (featPtr[basicPos + 3] + grid1) * stride;
            float x3 = (featPtr[basicPos + 4] + grid0) * stride;
            float y3 = (featPtr[basicPos + 5] + grid1) * stride;
            float x4 = (featPtr[basicPos + 6] + grid0) * stride;
            float y4 = (featPtr[basicPos + 7] + grid1) * stride;

            int boxColor = argmax(featPtr + basicPos + 9, mConfig.numColors);
            int boxClass = argmax(featPtr + basicPos + 9 + mConfig.numColors, mConfig.numClasses);

            float boxProb = (featPtr[basicPos + 8]);

            if(boxProb >= probThreshold) {
                NNetDetectedArmor armor;

                Eigen::Matrix<float, 3, 4> light4PointNorm;
                Eigen::Matrix<float, 3, 4> light4PointDst;

                light4PointNorm << x1, x2, x3, x4, y1, y2, y3, y4, 1, 1, 1, 1;

                light4PointDst = transformMatrix * light4PointNorm;

                for(int i = 0; i < 4; i++) {
                    armor.light4Point[i] = cv::Point2f(light4PointDst(0, i), light4PointDst(1, i));
                    armor.detectedArmors.push_back(armor.light4Point[i]);
                }

                std::vector<cv::Point2f> tmp(armor.light4Point, armor.light4Point + 4);
                armor.lightRect = cv::boundingRect(tmp);

                armor.robotType = boxClass;
                armor.robotColor = boxColor;
                armor.prob = boxProb;

                armors.push_back(armor);
            }

        }  // point anchor loop
    }

    float intersectionArea(const NNetDetectedArmor& a, const NNetDetectedArmor& b) {
        cv::Rect2f inter = a.lightRect & b.lightRect;
        return inter.area();
    }

    void qsortDescentInplace(std::vector<NNetDetectedArmor>& faceObjects, int left, int right) {
        int i = left;
        int j = right;
        float p = faceObjects[(left + right) / 2].prob;

        while(i <= j) {
            while(faceObjects[i].prob > p)
                i++;

            while(faceObjects[j].prob < p)
                j--;

            if(i <= j) {
                // swap
                std::swap(faceObjects[i], faceObjects[j]);

                i++;
                j--;
            }
        }
    }

    void qsortDescentInplace(std::vector<NNetDetectedArmor>& objects) {
        if(objects.empty())
            return;
        qsortDescentInplace(objects, 0, objects.size() - 1);
    }

    void nmsSortedBboxes(std::vector<NNetDetectedArmor>& faceObjects, std::vector<int>& picked, float nmsThreshold) {
        picked.clear();

        const int n = faceObjects.size();

        std::vector<float> areas(n);
        for(int i = 0; i < n; i++) {
            areas[i] = faceObjects[i].lightRect.area();
        }

        for(int i = 0; i < n; i++) {
            NNetDetectedArmor& a = faceObjects[i];

            int keep = 1;
            for(int j = 0; j < (int)picked.size(); j++) {
                NNetDetectedArmor& b = faceObjects[picked[j]];

                // intersection over union
                float interArea = intersectionArea(a, b);
                float unionArea = areas[i] + areas[picked[j]] - interArea;
                float iou = interArea / unionArea;
                if(iou > nmsThreshold) {
                    keep = 0;
                    // Stored for FFT
                    if(iou > mConfig.fftMinIou && abs(a.prob - b.prob) < mConfig.fftConfError && a.robotType == b.robotType &&
                       a.robotColor == b.robotColor) {
                        for(int k = 0; k < 4; k++) {
                            b.detectedArmors.push_back(a.light4Point[k]);
                        }
                    }
                }
            }

            if(keep)
                picked.push_back(i);
        }
    }

    void decodeOutputs(const float* prob, std::vector<NNetDetectedArmor>& armors,
                       const Eigen::Matrix<float, 3, 3>& transformMatrix, int imgwidth, int imgHeight) {
        std::vector<NNetDetectedArmor> proposals;
        std::vector<int> strides = { 8, 16, 32 };
        std::vector<GridAndStride> gridStrides;

        generateGridsAndStride(mConfig.inputWidth, mConfig.inputHeight, strides, gridStrides);
        generateYoloxProposals(gridStrides, prob, transformMatrix, mConfig.bboxConfThresh, proposals);
        qsortDescentInplace(proposals);

        if(proposals.size() >= mConfig.topK)
            proposals.resize(mConfig.topK);
        std::vector<int> picked;
        nmsSortedBboxes(proposals, picked, mConfig.nmsThresh);
        int count = picked.size();
        armors.resize(count);

        for(int i = 0; i < count; i++) {
            armors[i] = proposals[picked[i]];
        }
    }

    /**
     * @brief 海伦公式计算三角形面积
     *
     * @param pts 三角形顶点
     * @return float 面积
     */
    static float calcTriangleArea(cv::Point2f pts[3]) {
        auto a = sqrt(pow((pts[0] - pts[1]).x, 2) + pow((pts[0] - pts[1]).y, 2));
        auto b = sqrt(pow((pts[1] - pts[2]).x, 2) + pow((pts[1] - pts[2]).y, 2));
        auto c = sqrt(pow((pts[2] - pts[0]).x, 2) + pow((pts[2] - pts[0]).y, 2));

        auto p = (a + b + c) / 2.f;

        return sqrt(p * (p - a) * (p - b) * (p - c));
    }

    /**
     * @brief 计算四边形面积
     *
     * @param pts 四边形顶点
     * @return float 面积
     */
    static float calcTetragonArea(cv::Point2f pts[4]) {
        return calcTriangleArea(&pts[0]) + calcTriangleArea(&pts[1]);
    }

    bool blobImg(const cv::Mat& imageFromCamera, float* blobDataPtr) {
        if(imageFromCamera.empty()) {
            logWarning(fmt::format("NNet Armor Detector receive  empty img!"));
            return false;
        }
        cv::Mat resizedImg = scaledResize(imageFromCamera, mTransformMatrix);
        cv::Mat pre;
        cv::Mat preSplit[3];
        resizedImg.convertTo(pre, CV_32F);
        cv::split(pre, preSplit);

        auto imgOffset = mConfig.inputHeight * mConfig.inputWidth;
        // 将img拷贝进blob
        for(int c = 0; c < 3; c++) {
            // memcpy(blobDataPtr, preSplit[c].data, imgOffset * sizeof(float));
            std::copy(reinterpret_cast<float*>(preSplit[c].data),
                      reinterpret_cast<float*>(preSplit[c].data) + imgOffset * sizeof(uchar), blobDataPtr);
            blobDataPtr += imgOffset;
        }

        return true;
    }

    void postProcess(std::vector<NNetDetectedArmor>& armors) {
        for(auto& armor : armors) {
            // 对候选框预测角点进行平均,降低误差
            if(armor.detectedArmors.size() >= 8) {
                auto N = armor.detectedArmors.size();
                cv::Point2f detectedArmorsFinal[4];

                for(uint32_t i = 0; i < N; i++) {
                    detectedArmorsFinal[i % 4] += armor.detectedArmors[i];
                }

                for(int i = 0; i < 4; i++) {
                    detectedArmorsFinal[i].x = detectedArmorsFinal[i].x / (N / 4);
                    detectedArmorsFinal[i].y = detectedArmorsFinal[i].y / (N / 4);
                }

                armor.light4Point[0] = detectedArmorsFinal[0];
                armor.light4Point[1] = detectedArmorsFinal[1];
                armor.light4Point[2] = detectedArmorsFinal[2];
                armor.light4Point[3] = detectedArmorsFinal[3];
            }
            armor.rectArea = static_cast<int>(calcTetragonArea(armor.light4Point));
        }
    }

public:
    NNetArmorDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        // 1. 读取网络
        mNetwork = mIe.ReadNetwork(mConfig.networkPath);
        if(mNetwork.getOutputsInfo().size() != 1) {
            throw std::logic_error("Sample supports topologies with 1 output only");
        }

        // 2. 配置输入输出blob
        // 输入blob
        auto [inputName, inputInfo] = *mNetwork.getInputsInfo().begin();

        auto [outputName, outputInfo] = *mNetwork.getOutputsInfo().begin();

        // 3. 加载网络
        mExeNetwork = mIe.LoadNetwork(mNetwork, "CPU");

        // 4. 创建推理请求
        mInferRequest = mExeNetwork.CreateInferRequest();

        const auto inputBlob = mInferRequest.GetBlob(inputName);
        mInputMemBlobPtr = IE::as<IE::MemoryBlob>(inputBlob);

        const IE::Blob::Ptr outputBlob = mInferRequest.GetBlob(outputName);
        mOutputMemBlobPtr = IE::as<IE::MemoryBlob>(outputBlob);
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                     ACTOR_EXCEPTION_PROBE();

                     const auto t0 = Clock::now();
                     const auto frame = BlackBoard::instance().get<CameraFrame>(key).value();
                     NNetDetectedArmorArray res;
                     res.frame = frame;

                     auto inputBlobHolder = mInputMemBlobPtr->wmap();
                     float* blobDataPtr = inputBlobHolder.as<float*>();
                     if(!blobImg(res.frame.frame, blobDataPtr)) {
                         return;
                     }

                     mInferRequest.Infer();

                     auto outputHolder = mOutputMemBlobPtr->rmap();
                     const float* netPredict = outputHolder.as<const IE::PrecisionTrait<IE::Precision::FP32>::value_type*>();
                     decodeOutputs(netPredict, res.armors, mTransformMatrix, res.frame.info.width, res.frame.info.height);

                     postProcess(res.armors);

                     const auto t1 = Clock::now();
                     logInfo(
                         fmt::format("NNet armor detector:decode time {:.4f}s", static_cast<double>((t1 - t0).count()) / 1e9));

                     sendAll(armor_nnet_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(NNetArmorDetector);
