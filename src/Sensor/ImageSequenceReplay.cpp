#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Timer.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <exception>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>

#include "SuppressWarningEnd.hpp"

struct ImageSequenceReplaySettings final {
    std::string path;
    std::string extension;  // jpg, png, etc.
    double fps;
    double fov;
    uint32_t width;
    uint32_t height;
};
template <class Inspector>
bool inspect(Inspector& f, ImageSequenceReplaySettings& x) {
    return f.object(x).fields(
        f.field("path", x.path).invariant([](const std::string& path) { return fs::exists(path) && fs::is_directory(path); }),
        f.field("extension", x.extension),
        f.field("fps", x.fps).fallback(30.0).invariant([](const double v) { return v >= 1.0 && v <= 120.0; }),
        f.field("fov", x.fov), f.field("width", x.width), f.field("height", x.height));
}

class ImageSequenceReplay final : public HubHelper<caf::event_based_actor, ImageSequenceReplaySettings, image_frame_atom> {
    Identifier mKey;
    std::vector<std::string> mImages;
    uint32_t mCount = 0;

    cv::Mat resize(const cv::Mat& frame) const {
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size{ static_cast<int32_t>(mConfig.width), static_cast<int32_t>(mConfig.height) });
        return resized;
    }

    void next() {
        const auto path = mImages[mCount % mImages.size()];

        auto img = cv::imread(path);

        CameraFrame res;
        res.frame = (mConfig.width == static_cast<uint32_t>(img.cols) && mConfig.height == static_cast<uint32_t>(img.rows)) ?
            std::move(img) :
            resize(img);
        res.info.width = mConfig.width;
        res.info.height = mConfig.height;
        res.info.identifier = "ImageSequenceReplay";
        res.info.cameraMatrix =
            (cv::Mat_<double>(3, 3) << mConfig.width / 2 / tan(glm::radians(mConfig.fov) / 2), 0, mConfig.width / 2, 0,
             mConfig.height / 2 / tan(glm::radians(mConfig.fov) / 2), mConfig.height / 2, 0, 0, 1);
        res.info.distCoefficients = cv::Mat_<double>{};
        res.info.transform = Transform<FrameOfRef::Gun, FrameOfRef::Camera, true>(glm::identity<glm::dmat4>());
        res.lastUpdate = SynchronizedClock::instance().now();

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));

        ++mCount;
    }

public:
    ImageSequenceReplay(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        mImages.reserve(1000);
        for(auto const& dir_entry : std::filesystem::directory_iterator{ mConfig.path }) {
            const auto filePath = dir_entry.path().string();
            if(filePath.find(mConfig.extension) == std::string::npos)
                continue;
            mImages.push_back(dir_entry.path().string());

            if(mImages.size() == 0) {
                const auto errInfo =
                    fmt::format("Path:{} do not exsit picture with extension {}", mConfig.path, mConfig.extension);
                logError(errInfo);
                throw std::invalid_argument::exception();
            }
        }
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    Timer::instance().addTimer(address(),
                                               std::chrono::microseconds{ static_cast<int64_t>(1'000'000 / mConfig.fps) });
                },
                 [this](timer_atom) {
                     ACTOR_PROTOCOL_CHECK(timer_atom);
                     next();
                 } };
    }
};

HUB_REGISTER_CLASS(ImageSequenceReplay);
