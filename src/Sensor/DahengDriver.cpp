#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <GxIAPI.h>
#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>
#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

static void loadCalibration(const bool disableUndistort, const std::string& identifier, const uint32_t width,
                            const uint32_t height, const double fallbackFov, cv::Mat& cameraMatrix, cv::Mat& distCoefficients,
                            bool& undistort) {
    const auto inputFileName = "./data/camera_calibration/" + identifier + ".xml";

    const cv::FileStorage fs(inputFileName, cv::FileStorage::READ);
    if(!disableUndistort && std::filesystem::exists(inputFileName) && fs.isOpened()) {
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoefficients;
        undistort = true;
    } else {
        logWarning(
            fmt::format("Failed to get calibration info for S/N {}. Use fallback fov {} instead.", identifier, fallbackFov));
        cameraMatrix = (cv::Mat_<double>(3, 3) << width / 2.0 / std::tan(glm::radians(fallbackFov) / 2.0), 0,
                        static_cast<double>(width) / 2.0, 0, height / 2.0 / std::tan(glm::radians(fallbackFov) / 2.0),
                        static_cast<double>(height) / 2.0, 0, 0, 1);
        distCoefficients = cv::Mat{};
        undistort = false;
    }
}

struct DahengDriverSettings final {
    std::string openMode;
    std::string identifier;
    double fps;
    double fov;
    double exposureTime;
    bool flip;
    bool disableUndistort;
    glm::dvec3 offset;  // based on gun
};

enum class OpenMode { Index, SerialNumber };

template <class Inspector>
bool inspect(Inspector& f, DahengDriverSettings& x) {
    return f.object(x).fields(
        f.field("openMode", x.openMode).invariant([](const std::string& v) { return v == "Index" || v == "SerialNumber"; }),
        f.field("identifier", x.identifier),
        f.field("fps", x.fps).fallback(30.0).invariant([](const double v) { return v >= 1.0 && v <= 500.0; }),
        f.field("fov", x.fov), f.field("exposureTime", x.exposureTime), f.field("flip", x.flip).fallback(false),
        f.field("disableUndistort", x.disableUndistort).fallback(false), f.field("dx", x.offset.x).fallback(0.0),
        f.field("dy", x.offset.y).fallback(0.0), f.field("dz", x.offset.z).fallback(0.0));
}

static void checkGXStatus(const GX_STATUS status) {
    if(status != GX_STATUS_SUCCESS) {
        const auto error = "GX Error: " + std::to_string(status);
        logError(error.c_str());
    }
}

class DahengLibGuard final : Unmovable {
public:
    DahengLibGuard() {
        checkGXStatus(GXInitLib());
    }

    ~DahengLibGuard() {
        checkGXStatus(GXCloseLib());
    }
};

static void initLib() {
    static DahengLibGuard guard;
}

class DahengDriver final : public HubHelper<caf::event_based_actor, DahengDriverSettings, image_frame_atom> {
    Identifier mKey;
    GX_DEV_HANDLE mDevice;
    bool mStartFlag = false;

    static constexpr auto pixelFormat = GX_PIXEL_FORMAT_BAYER_RG8;
    static constexpr auto pixelCast = cv::COLOR_BayerRG2RGB_EA;
    static constexpr auto pixelStorageFormat = CV_8UC1;

    std::deque<Clock::rep> mLastFrames;

    cv::Mat mCameraMatrix;
    cv::Mat mDistCoefficients;
    std::string mCameraSerialNumber;
    bool mDoUndistort;

    void reportFrameRate(const Clock::time_point timeStamp) {
        const auto current = timeStamp.time_since_epoch().count();
        mLastFrames.push_back(current);

        while(current - mLastFrames.front() > 1'000'000'000)
            mLastFrames.pop_front();

        const auto delta = std::max(static_cast<Clock::rep>(1), current - mLastFrames.front());
        const auto fps = (static_cast<double>(mLastFrames.size()) - 1.0) * 1e9 / static_cast<double>(delta);
        HubLogger::watch("fps", static_cast<uint32_t>(fps));
    }

    void newFrameImpl(Clock::time_point timeStamp, const cv::Mat& frame, uint32_t width, uint32_t height) {
        cv::Mat bgr;
        cv::cvtColor(frame, bgr, pixelCast);

        if(mConfig.flip) {
            cv::Mat flipped;
            cv::flip(bgr, flipped, -1);
            std::swap(bgr, flipped);
        }

        reportFrameRate(timeStamp);

        CameraFrame frameData;
        frameData.lastUpdate = timeStamp;
        frameData.info.cameraMatrix = mCameraMatrix;
        frameData.info.distCoefficients = mDistCoefficients;
        frameData.info.identifier = mCameraSerialNumber;
        frameData.info.width = width;
        frameData.info.height = height;
        frameData.info.transform =
            Transform<FrameOfRef::Gun, FrameOfRef::Camera, true>(glm::translate(glm::identity<glm::dmat4>(), -mConfig.offset));

        // if (mDoUndistort) {
        //     auto src = bgr.clone();
        //     cv::undistort(src, bgr, mCameraMatrix, mDistCoefficients, frameData.info.cameraMatrix);
        // }

        frameData.frame = std::move(bgr);
        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, std::move(frameData)));
    }

#ifdef ARTINX_DAHENG_USB2
    std::thread mCaptureThread;
    bool mRunning = true;

    void acquireOneFrame() {}
#else

    void newFrame(GX_FRAME_CALLBACK_PARAM* pFrameData) {
        if(pFrameData->status != GX_FRAME_STATUS_SUCCESS || !mStartFlag)
            return;

        const auto timeStamp = SynchronizedClock::instance().now();  // TODO: propagation time and internal timer

        // TODO: reduce reallocation
        cv::Mat frame{ cv::Size{ pFrameData->nWidth, pFrameData->nHeight }, pixelStorageFormat };
        memcpy(frame.data, pFrameData->pImgBuf, pFrameData->nImgSize);
        newFrameImpl(timeStamp, frame, pFrameData->nWidth, pFrameData->nHeight);
    }

#endif

public:
    DahengDriver(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        initLib();

        GX_OPEN_PARAM deviceDesc;
        deviceDesc.accessMode = GX_ACCESS_CONTROL;
        deviceDesc.openMode = mConfig.openMode == "Index" ? GX_OPEN_MODE::GX_OPEN_INDEX : GX_OPEN_MODE::GX_OPEN_SN;
        deviceDesc.pszContent = mConfig.identifier.data();

        checkGXStatus(GXOpenDevice(&deviceDesc, &mDevice));
        char strSN[256];
        size_t size = 256;
        checkGXStatus(GXGetString(mDevice, GX_STRING_DEVICE_SERIAL_NUMBER, strSN, &size));
        mCameraSerialNumber = strSN;
        logInfo(fmt::format("open camera {}", mCameraSerialNumber));

#ifdef ARTINX_DAHENG_USB2
        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS));
#else
        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON));
        checkGXStatus(GXSetFloat(mDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, mConfig.fps));
#endif

        // checkGXStatus(GXSetEnum(device, GX_ENUM_FLAT_FIELD_CORRECTION, GX_ENUM_FLAT_FIELD_CORRECTION_ON));
        // checkGXStatus(GXSetEnum(device, GX_ENUM_NOISE_REDUCTION_MODE, GX_NOISE_REDUCTION_MODE_ON));
        // checkGXStatus(GXSetFloat(device, GX_FLOAT_NOISE_REDUCTION, 2.0));

#ifndef ARTINX_DAHENG_USB2
        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED));
#endif
        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF));
        // checkGXStatus(GXSetEnum(device, GX_ENUM_EXPOSURE_TIME_MODE, GX_EXPOSURE_TIME_MODE_ULTRASHORT));

        GX_FLOAT_RANGE range;
        checkGXStatus(GXGetFloatRange(mDevice, GX_FLOAT_EXPOSURE_TIME, &range));

        checkGXStatus(GXSetFloat(mDevice, GX_FLOAT_EXPOSURE_TIME, 1000000.0 * mConfig.exposureTime));

        checkGXStatus(GXSetEnum(mDevice, GX_ENUM_PIXEL_FORMAT, pixelFormat));

        /*
        if(mConfig.decimation) {
            checkGXStatus(GXSetEnum(mDevice, GX_ENUM_BINNING_HORIZONTAL_MODE, GX_BINNING_VERTICAL_MODE_AVERAGE));
            checkGXStatus(GXSetEnum(mDevice, GX_ENUM_BINNING_VERTICAL_MODE, GX_BINNING_VERTICAL_MODE_AVERAGE));
            checkGXStatus(GXSetInt(mDevice, GX_INT_BINNING_HORIZONTAL, mConfig.decimation));
            checkGXStatus(GXSetInt(mDevice, GX_INT_BINNING_VERTICAL, mConfig.decimation));
            //checkGXStatus(GXSetInt(mDevice, GX_INT_DECIMATION_HORIZONTAL, mConfig.decimation));
            //checkGXStatus(GXSetInt(mDevice, GX_INT_DECIMATION_VERTICAL, mConfig.decimation));
        }
        */

        int64_t width, height;
        checkGXStatus(GXGetInt(mDevice, GX_INT_WIDTH_MAX, &width));
        checkGXStatus(GXGetInt(mDevice, GX_INT_HEIGHT_MAX, &height));

        logInfo(fmt::format("Resolution for {}: {} x {}", mCameraSerialNumber, width, height));

        checkGXStatus(GXSetInt(mDevice, GX_INT_WIDTH, width));
        checkGXStatus(GXSetInt(mDevice, GX_INT_HEIGHT, height));
        checkGXStatus(GXSetInt(mDevice, GX_INT_OFFSET_X, 0));
        checkGXStatus(GXSetInt(mDevice, GX_INT_OFFSET_Y, 0));

        loadCalibration(mConfig.disableUndistort, mCameraSerialNumber, static_cast<uint32_t>(width),
                        static_cast<uint32_t>(height), mConfig.fov, mCameraMatrix, mDistCoefficients, mDoUndistort);

#ifdef ARTINXHUB_WINDOWS
        auto bImplementPacketSize = false;
        checkGXStatus(GXIsImplemented(mDevice, GX_INT_GEV_PACKETSIZE, &bImplementPacketSize));
        if(bImplementPacketSize) {
            uint32_t unPacketSize = 0;
            checkGXStatus(GXGetOptimalPacketSize(mDevice, &unPacketSize));
            checkGXStatus(GXSetInt(mDevice, GX_INT_GEV_PACKETSIZE, unPacketSize));
        }
#endif

#ifndef ARTINX_DAHENG_USB2
        checkGXStatus(GXRegisterCaptureCallback(mDevice, this, [](GX_FRAME_CALLBACK_PARAM* pFrameData) {
            static_cast<DahengDriver*>(pFrameData->pUserParam)->newFrame(pFrameData);
        }));
#endif

        checkGXStatus(GXSendCommand(mDevice, GX_COMMAND_ACQUISITION_START));

#ifdef ARTINX_DAHENG_USB2
        mCaptureThread = std::thread([this] {
            auto current = Clock::now();
            const auto increment = static_cast<Clock::duration>(static_cast<int64_t>(1'000'000'000 / mConfig.fps));

            GX_FRAME_DATA data;

            int64_t payloadSize;
            checkGXStatus(GXGetInt(mDevice, GX_INT_PAYLOAD_SIZE, &payloadSize));
            std::vector<uint8_t> payload(payloadSize);
            data.pImgBuf = payload.data();

            while(mRunning) {
                std::this_thread::sleep_until(current);
                GXGetImage(mDevice, &data, 100);
                if(data.nStatus == GX_FRAME_STATUS_SUCCESS && mStartFlag) {
                    const auto timeStamp = SynchronizedClock::instance().now();  // TODO: propagation time and internal timer

                    // TODO: reduce reallocation
                    cv::Mat frame{ cv::Size{ data.nWidth, data.nHeight }, pixelStorageFormat };
                    memcpy(frame.data, data.pImgBuf, data.nImgSize);
                    newFrameImpl(timeStamp, frame, data.nWidth, data.nHeight);
                    current += increment;
                }
            }
        });
#endif
    }

    ~DahengDriver() override {
#ifdef ARTINX_DAHENG_USB2
        mRunning = false;
        mCaptureThread.join();
#endif
        checkGXStatus(GXSendCommand(mDevice, GX_COMMAND_ACQUISITION_STOP));
#ifndef ARTINX_DAHENG_USB2
        checkGXStatus(GXUnregisterCaptureCallback(mDevice));
#endif
        checkGXStatus(GXCloseDevice(mDevice));
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) {
            ACTOR_PROTOCOL_CHECK(start_atom);
            mStartFlag = true;
        } };
    }
};

HUB_REGISTER_CLASS(DahengDriver);
