#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <cstdint>
#include <filesystem>
#include <memory>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>
#include <opencv2/videoio.hpp>

#include "SuppressWarningEnd.hpp"

namespace fs = std::filesystem;

struct VideoRecorderSettings final {
    std::string base;
    double segmentLength;
    double fps;
};

template <class Inspector>
bool inspect(Inspector& f, VideoRecorderSettings& x) {
    return f.object(x).fields(
        f.field("base", x.base),
        f.field("segmentLength", x.segmentLength).fallback(60.0).invariant([](double v) { return v >= 10.0; }),
        f.field("fps", x.fps).fallback(30.0).invariant([](double v) { return v >= 1.0 && v <= 120.0; }));
}

class VideoRecorder final : public HubHelper<caf::event_based_actor, VideoRecorderSettings> {
    bool mStartFlag = false;
    int mFormat = 0;
    cv::Size mSize = { 0, 0 };
    uint32_t mFrameCount = 0, mTotal = 0;
    std::unique_ptr<cv::VideoWriter> mWriter;
    Clock::time_point mStart = Clock::now();

    const int mFourCc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

public:
    VideoRecorder(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {}
    ~VideoRecorder() override {
        mWriter.reset();
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    mStartFlag = true;
                },
                 [this](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);

                     if(!mStartFlag)
                         return;

                     const auto current = Clock::now();
                     const auto frameCount =
                         static_cast<uint32_t>(static_cast<double>((current - mStart).count()) / (1e9 / mConfig.fps));
                     if(mTotal >= frameCount)
                         return;
                     ++mTotal;

                     const auto frameData = BlackBoard::instance().get<CameraFrame>(key).value();
                     // TODO: record camera info

                     if(mWriter &&
                        (frameData.frame.type() != mFormat || frameData.frame.size() != mSize ||
                         mFrameCount >= static_cast<uint32_t>(mConfig.fps * mConfig.segmentLength))) {
                         mWriter.reset();
                     }
                     if(!mWriter) {
                         if(!fs::exists(mConfig.base) && !fs::create_directories(mConfig.base)) {
                             const auto error = "Failed to create directory " + mConfig.base;
                             logError(error.c_str());
                         }
                         mWriter = std::make_unique<cv::VideoWriter>(
                             mConfig.base + "/" + std::to_string(Clock::now().time_since_epoch().count()%1000000000) + ".mp4", mFourCc,
                             mConfig.fps, frameData.frame.size());
                         mFormat = frameData.frame.type();
                         mSize = frameData.frame.size();
                         mFrameCount = 0;
                     }
                     mWriter->write(frameData.frame);
                     ++mFrameCount;
                 } };
    }
};

HUB_REGISTER_CLASS(VideoRecorder);
