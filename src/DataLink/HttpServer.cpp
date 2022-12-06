#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "RadarCameraPoints.hpp"
#include "Utility.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/blocking_actor.hpp>
#include <caf/event_based_actor.hpp>
#define CPPHTTPLIB_SEND_FLAGS 0x4000
#include <fmt/format.h>
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#ifdef ARTINXHUB_WINDOWS
#define NOMINMAX
#include <Windows.h>
#endif

#ifdef ARTINXHUB_LINUX
#include <net/if.h>
#include <sys/ioctl.h>
#endif

#include "SuppressWarningEnd.hpp"

struct ImageWithFilter {
    cv::Mat image;
    bool isEnable = true;
};

class HttpServer final : public HubHelper<caf::event_based_actor, void, radar_locate_request_atom> {
    httplib::Server mServer;
    std::unordered_map<uint64_t, ImageWithFilter> mImage;
    std::mutex mMutex;
    std::thread mListener;

    std::string mhostIpAddress;
    std::streambuf* mClogBuffer;
    std::stringstream mLogStream;

    Identifier mKey;

#if defined(ARTINXHUB_LINUX)
#define ETH_NAME "wlp0s20f3"
    std::string getHostIpAddress() {
        int sockFd;
        struct sockaddr_in sockIn;
        struct ifreq ifReq;

        sockFd = socket(AF_INET, SOCK_DGRAM, 0);
        if(sockFd != -1) {
            strncpy(ifReq.ifr_name, ETH_NAME, IFNAMSIZ);   // Interface name
            if(ioctl(sockFd, SIOCGIFADDR, &ifReq) == 0) {  // SIOCGIFADDR obtain interface address
                memcpy(&sockIn, &ifReq.ifr_addr, sizeof(ifReq.ifr_addr));
                return inet_ntoa(sockIn.sin_addr);
            }
        }
        return "127.0.0.1";
    }
#endif

    std::optional<std::vector<uchar>> generateImageData(const std::string& path) {
        if(path.empty())
            return std::nullopt;

        uint64_t id;
        try {
            id = std::stoull(path);
        } catch(std::exception&) {
            return std::nullopt;
        }

        std::unique_lock guard{ mMutex };
        if(!mImage.count(id) || !mImage[id].isEnable)
            return std::nullopt;
        auto img = mImage[id].image;
        guard.unlock();

        std::vector<uchar> data;
        if(!cv::imencode(".jpg", img, data))
            return std::nullopt;
        return data;
    }
    std::string generateFilterJson() {
        nlohmann::json result = nlohmann::json::array();
        std::lock_guard<std::mutex> guard{ mMutex };
        for(const auto& v : mImage) {
            result.push_back({ std::to_string(v.first), v.second.isEnable });
        }
        return result.dump();
    }

public:
    HttpServer(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mClogBuffer{ std::clog.rdbuf() }, mKey{ generateKey(this) } {
#if defined(ARTINXHUB_WINDOWS)
        mhostIpAddress = "127.0.0.1";
#elif defined(ARTINXHUB_LINUX)
        mhostIpAddress = getHostIpAddress();
#endif
        // std::clog.rdbuf(mLogStream.rdbuf());

        mServer.set_mount_point("/pages", "./pages");

        mServer.Get("/status", [](const httplib::Request&, httplib::Response& res) {
            res.set_content("hello world!    clock " + std::to_string(::clock()), "text/plain");
        });
        //        mServer.Get("/profile", [this](const httplib::Request&, httplib::Response& res) {
        //            res.set_content("hello world!    clock " + std::to_string(::clock());, "text/plain");
        //        });
        //        mServer.Get("/parameters", [this](const httplib::Request& req, httplib::Response& res) {
        //            res.set_content("hello world!    clock " + std::to_string(::clock()), "text/plain");
        //        });
        mServer.Get(R"(/img/(\d+).*)", [this](const httplib::Request& req, httplib::Response& res) {
            auto path = req.matches[1];
            std::string p = path;
            res.set_content_provider(
                "multipart/x-mixed-replace;boundary=MJP",
                [this, p](size_t, httplib::DataSink& sink) {
                    if(const auto img = generateImageData(p)) {
                        auto vec = img.value();
                        sink.os << "--MJP\r\n"
                                   "Content-Type: image/jpeg\r\n"
                                   "Content-Length: "
                                << vec.size() << "\r\n\r\n";
                        sink.os.write(reinterpret_cast<const char*>(vec.data()), static_cast<long>(vec.size()));
                    }
                    return true;
                },
                [](bool) {});
        });
        mServer.Get("/log", [this](const httplib::Request&, httplib::Response& res) {
            res.set_content(mLogStream.str(), "text/plain");
            mLogStream.str("");
        });
        mServer.Get("/watch", [](const httplib::Request&, httplib::Response& res) {
            res.set_content(nlohmann::json(HubLogger::watches).dump(), "application/json");
        });
        mServer.Post("/filter", [this](const httplib::Request& req, httplib::Response& res) {
            if(req.body.empty()) {
                res.set_content(generateFilterJson(), "text/plain");
                return;
            }
            auto j = nlohmann::json::parse(req.body);
            std::lock_guard guard{ mMutex };
            for(auto& [k, v] : j.items()) {
                mImage[std::stoull(k)].isEnable = v;
            }
            res.set_content("", "text/plain");
        });
        mServer.Post("/radar", [this](const httplib::Request& req, httplib::Response&) {
            auto j = nlohmann::json::parse(req.body);
            const float x = j[0], y = j[1];
            RadarCameraPointsArray data;
            // TODO
            data.imagePoints.emplace_back(x, y);
            sendAll(radar_locate_request_atom_v, BlackBoard::instance().updateSync(mKey, data));
        });
        mServer.Get("/exit", [this](const httplib::Request&, httplib::Response&) {
            mServer.stop();
            terminateSystem(*this, true);
        });
        mListener = std::thread{ [this] { mServer.listen(mhostIpAddress.c_str(), 5630); } };
    }
    ~HttpServer() override {
        std::clog.rdbuf(mClogBuffer);
        mListener.detach();
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    [[maybe_unused]] const auto res =
#if defined(ARTINXHUB_WINDOWS)
                        ShellExecuteA(nullptr, "open", "http://localhost:5630/pages/index.html", nullptr, nullptr, SW_SHOWNORMAL);
#elif defined(ARTINXHUB_LINUX)
                        ::system(fmt::format("xdg-open http://{}:5630/pages/index.html", mhostIpAddress).c_str());
#else
                        0;
#endif
                },
                 [this](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                     std::lock_guard<std::mutex> guard{ mMutex };
                     mImage[key.val].image = BlackBoard::instance().get<CameraFrame>(key).value().frame;
                 } };
    }
};

HUB_REGISTER_CLASS(HttpServer);
