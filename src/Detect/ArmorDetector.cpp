#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedCar.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <cstdint>

#include <string>
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

struct ArmorDetectorSettings final {
    float globalScale;
    std::vector<int32_t> thresholdForBlue;  // minBlue, maxGreen, maxRed
    std::vector<int32_t> thresholdForRed;   // minRed, maxBlue,maxGreen
    int bgrSubtractForBlue;                 // bgr subtract threshold for blue
    int bgrSubtractForRed;                  // bgr subtract threshold for red
    float minLightRectRatio;                // width/height
    float maxLightRectRatio;                // width/height
    float maxLightAngle;                    // angle(degree)
    float maxAreaRatio;                     // ellipseArea/contourArea
    float minArmorRectRatio;                // height/width
    float maxArmorRectRatio;                // height/width
    float maxArmorAngle;                    // angle(degree)
    float maxLightBaseAngle;                // abs(lightAngle-armorAngle)(degree)
    float maxParallelAngle;                 // abs(lightAngle1-lightAngle2)(degree)
    float minLightHeightRatio;              // lightHeight/armorHeight
};

template <class Inspector>
bool inspect(Inspector& f, ArmorDetectorSettings& x) {
    return f.object(x).fields(f.field("globalScale", x.globalScale),
                              f.field("thresholdForBlue", x.thresholdForBlue).invariant([](auto& c) { return c.size() == 3; }),
                              f.field("thresholdForRed", x.thresholdForRed).invariant([](auto& c) { return c.size() == 3; }),
                              f.field("bgrSubtractForBlue", x.bgrSubtractForBlue).fallback(60),
                              f.field("bgrSubtractForRed", x.bgrSubtractForRed).fallback(60),
                              f.field("minLightRectRatio", x.minLightRectRatio),
                              f.field("maxLightRectRatio", x.maxLightRectRatio), f.field("maxLightAngle", x.maxLightAngle),
                              f.field("maxAreaRatio", x.maxAreaRatio), f.field("minArmorRectRatio", x.minArmorRectRatio),
                              f.field("maxArmorRectRatio", x.maxArmorRectRatio), f.field("maxArmorAngle", x.maxArmorAngle),
                              f.field("maxLightBaseAngle", x.maxLightBaseAngle), f.field("maxParallelAngle", x.maxParallelAngle),
                              f.field("minLightHeightRatio", x.minLightHeightRatio));
}

class ArmorDetector final
    : public HubHelper<caf::event_based_actor, ArmorDetectorSettings, armor_detect_available_atom, image_frame_atom> {
    Identifier mKey;

    // TODO: light pairs affinity

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

    cv::Rect2f boundingRect(const PairedLight& armor) {
        const auto b1 = armor.r1.boundingRect2f();
        const auto b2 = armor.r2.boundingRect2f();
        return b1 | b2;
    }

    static bool isWhite(int32_t b, int32_t g, int32_t r) {
        return b + g + r > 500;
    }

    std::vector<PairedLight> solve(const cv::Mat& image) {
        const cv::Mat scaled = image * mConfig.globalScale;
         debugView("scaled",scaled,[](auto&){});
        const auto lightPart = binary(scaled);

        const auto lights = findLights(image, lightPart);
        return matchLights(image, lights);
    }

    cv::Mat binary(const cv::Mat& src) {
        cv::Mat result(src.size(), CV_8U);
        if(GlobalSettings::get().selfColor == Color::Red) {
            const auto minB = mConfig.thresholdForBlue[0];
            const auto maxG = mConfig.thresholdForBlue[1];
            const auto maxR = mConfig.thresholdForBlue[2];

            for(int32_t row = 0; row != src.rows; ++row) {
                const auto* srcPtr = src.ptr(row);
                auto* resPtr = result.ptr(row);
                for(int32_t col = 0; col != src.cols; ++col) {
                    const auto b = srcPtr[0], g = srcPtr[1], r = srcPtr[2];
                    *resPtr = (b > minB && g < maxG && r < maxR && b - r > mConfig.bgrSubtractForBlue) ? 255 : 0;  // binarization
                    srcPtr += 3;
                    ++resPtr;
                }
            }
        } else {
            const auto minR = mConfig.thresholdForRed[0];
            const auto maxB = mConfig.thresholdForRed[1];
            const auto maxG = mConfig.thresholdForRed[2];
            const auto diffRedBlue = mConfig.bgrSubtractForRed;

            for(int32_t row = 0; row != src.rows; ++row) {
                const auto* srcPtr = src.ptr(row);
                auto* resPtr = result.ptr(row);
                for(int32_t col = 0; col != src.cols; ++col) {
                    const auto b = srcPtr[0], g = srcPtr[1], r = srcPtr[2];
                    //*resPtr = (!isWhite(b, g, r) && r > minR && b < maxB && g < maxG && r  > b + g) ? 255 : 0;  // binarization
                    *resPtr = (r > minR && b < maxB && g < maxG && r - b > diffRedBlue) ? 255 : 0;
                    srcPtr += 3;
                    ++resPtr;
                }
            }
        }

        // cv::medianBlur(result, result, 3);
//                debugView("binary", result, [](auto) {});
        return result;
    }

    std::vector<cv::RotatedRect> findLights(const cv::Mat& color, const cv::Mat& binary) {
        // TODO: down sampling
        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::RotatedRect> lights;
        for(auto& lightContour : contours) {
            // if(cv::contourArea(lightContour) < 300.0)
            //     fixContour(color, binary, lightContour);

            cv::RotatedRect lightRect;
            if(lightContour.size() >= 6) {
                // fitEllipse 返回的旋转矩形角度定义为水平线顺时针旋转到与短轴平行处所转过的角度,范围在[0, 180]
                lightRect = cv::fitEllipse(lightContour);  // produce as ellipse
            } else {
                lightRect = cv::minAreaRect(lightContour);
                if(lightRect.size.height < lightRect.size.width) {
                    std::swap(lightRect.size.width, lightRect.size.height);
                    lightRect.angle += 90;
                }
                continue;
            }

            //灯条矩形的长边不符合要求
            if(lightRect.size.height < 6.0f || lightRect.size.height > 160.f)
                continue;
            //灯条矩形的短边太长了
            if(lightRect.size.width > 20.0f)
                continue;
            //灯条矩形的比率不符合要求
            //            const auto ratio = lightRect.size.width / lightRect.size.height;
            //            if(ratio < mConfig.minLightRectRatio || ratio > mConfig.maxLightRectRatio)
            //                continue;

            //灯条倾斜角度太平了(水平线顺时针旋转到短边的角度太大), 角度范围在 [maxLightAngle, pi - maxLightAngle]内不符合要求
            if(std::sin(glm::radians(lightRect.angle)) > std::sin(glm::radians(mConfig.maxLightAngle))) {
                continue;
            }

            //外接矩形对应的椭圆的面积比外接轮廓的面积大太多了
            //            if(static_cast<double>(lightRect.size.width) * static_cast<double>(lightRect.size.height) *
            //                   glm::quarter_pi<double>() >
            //               mConfig.maxAreaRatio * cv::contourArea(lightContour))
            //                continue;

            lights.emplace_back(lightRect);
        }

                debugView("contour", color, [&](cv::Mat& src) {
                    for(auto& light : lights) {
                        cv::Point2f rotateVertices[4];
                        light.points(rotateVertices);
                        for(int i = 0; i < 4; ++i) {
                            cv::line(src, rotateVertices[i], rotateVertices[(i + 1) % 4], cv::Scalar(0, 255, 255), 1);
                        }
                        // cv::rectangle(src, light.boundingRect(), cv::Scalar{ 0, 255, 255 }, 1);
                        cv::putText(src, fmt::format("({:.2f}", light.angle),
                                    { static_cast<int32_t>(light.center.x), static_cast<int32_t>(light.center.y) },
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{ 255 });
                    }
                });

        std::sort(lights.begin(), lights.end(), [](const auto& lhs, const auto& rhs) { return lhs.center.x < rhs.center.x; });
        return lights;
    }

    std::vector<PairedLight> matchLights([[maybe_unused]] const cv::Mat& src, const std::vector<cv::RotatedRect>& lights) {
        std::vector<std::tuple<uint32_t, uint32_t, float>> pairs;
        for(uint32_t i = 0; i < lights.size(); ++i) {
            for(uint32_t j = i + 1; j < lights.size(); ++j) {
                const auto& lhs = lights[i];
                const auto& rhs = lights[j];

                std::vector<cv::Point2f> pts(8);
                lhs.points(pts.data());
                rhs.points(pts.data() + 4);

                // minAreaRect 返回的旋转矩形定义如下：
                // 角度为水平线顺时针旋转碰的第一条(该边为宽)所转过的角度
                auto rect = cv::minAreaRect(pts);

                //长边为宽，短边为高
                if(rect.size.width < rect.size.height) {
                    std::swap(rect.size.width, rect.size.height);
                }
                //装甲板矩形长度太长了
                if(rect.size.width > 300.f)
                    continue;

                //装甲板矩形高度太小了
                if(rect.size.height < 3.0f)
                    continue;

                //装甲板矩形高和宽的比不符合比率范围
                const auto ratio = rect.size.width / rect.size.height;
                if(ratio > mConfig.maxArmorRectRatio || ratio < mConfig.minArmorRectRatio)
                    continue;

                constexpr auto largeRatio = widthOfLargeArmor / heightOfLargeArmor;
                constexpr auto smallRatio = widthOfSmallArmor / heightOfSmallArmor;

                auto diff = static_cast<float>(std::fabs((ratio - smallRatio) / smallRatio));
                const auto diffLarge = static_cast<float>(std ::fabs(ratio - largeRatio) / largeRatio);
                bool largeArmor = false;
                if(diffLarge < diff) {
                    diff = diffLarge;
                    largeArmor = true;
                }

                const auto area1 = lhs.size.area();
                const auto area2 = rhs.size.area();
                auto par = std::fmin(area1, area2) / std::fmax(area1, area2);
                //两边灯条的面积差太大了
                if(par < 0.2f)
                    continue;

                //两边灯条的面积占比矩形面积太大了
                if(area1 + area2 > 0.6f * rect.size.area())
                    continue;

                //小的那个高度比外接矩形的高度低太多了
                if(std::fmin(lhs.size.height, rhs.size.height) < mConfig.minLightHeightRatio * rect.size.height)
                    continue;

                //装甲板矩形的倾斜角度太大了
                const auto tanRectAngle =
                    std::fabs((lhs.center.y - rhs.center.y) / (lhs.center.x - rhs.center.x + 1e-6));  //避免除0
                const auto rectAngele = std::atan(tanRectAngle);
                if(rectAngele > glm::radians(mConfig.maxArmorAngle))
                    continue;

                //灯条矩形和装甲板矩形的角度差太大了
                auto sinDegree = [](auto degree) { return std::sin(glm::radians(degree)); };
                if(std::fabs(sinDegree(rectAngele) - sinDegree(lhs.angle)) > sinDegree(mConfig.maxLightBaseAngle))
                    continue;
                if(std::fabs(sinDegree(rectAngele) - sinDegree(lhs.angle)) > sinDegree(mConfig.maxLightBaseAngle))
                    continue;

                //两个灯条的角度差太大了
                par = std::fabs(sinDegree(lhs.angle - rhs.angle));
                if(par > sinDegree(mConfig.maxParallelAngle))
                    continue;

                //两根灯条拼成的矩形中不会出现其他灯条
                bool isInteraction = false;
                for(uint32_t k = i + 1; k < j; ++k) {
                    const auto& minRect = rect.boundingRect();
                    if(lights[k].center.y > minRect.tl().y && lights[k].center.y < minRect.br().y) {
                        isInteraction = true;
                        break;
                    }
                }
                if(isInteraction)
                    continue;
                // logInfo(fmt::format("diff:{}, large_diff:{}", smalldiff, diffLarge));
                pairs.emplace_back(i, j, diff + par + (largeArmor ? 1e3f : 0.0f));
            }
        }

        //        debugView("potential", src, [&](cv::Mat& frame) {
        //            uint32_t idx = 0;
        //            for(auto& light : lights) {
        //                cv::ellipse(frame, light, cv::Scalar{ 255, 255, 0 }, 1);
        //                cv::Point2f offset{ -std::sin(glm::radians(light.angle)) * light.size.height,
        //                                    std::cos(glm::radians(light.angle)) * light.size.height };
        //                const auto p0 = light.center + offset;
        //                const auto p1 = light.center - offset;
        //                cv::line(frame, p0, p1, cv::Scalar{ 255, 0, 255 });
        //                cv::putText(frame, std::to_string(idx++),
        //                            { static_cast<int32_t>(light.center.x), static_cast<int32_t>(light.center.y) },
        //                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 0, 0, 255 });
        //            }
        //            for(auto& [i, j, s] : pairs) {
        //                const auto& lhs = lights[i];
        //                const auto& rhs = lights[j];
        //
        //                std::vector<cv::Point2f> pts(8);
        //                lhs.points(pts.data());
        //                rhs.points(pts.data() + 4);
        //
        //                auto rect = cv::minAreaRect(pts);
        //                if(rect.size.width < rect.size.height) {
        //                    std::swap(rect.size.width, rect.size.height);
        //                    rect.angle += 90.0;
        //                }
        //
        //                drawRotatedRect(frame, rect, cv::Scalar{ 0, 0, 255 });
        //                cv::putText(frame, fmt::format("{:.3f}", s),
        //                            { static_cast<int32_t>(rect.center.x), static_cast<int32_t>(rect.center.y) },
        //                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{ 255, 255, 0 });
        //                logInfo(fmt::format("{:.3f}", s));
        //            }
        //        });

        std::sort(pairs.begin(), pairs.end(),
                  [](const auto& lhs, const auto& rhs) { return std::get<float>(lhs) < std::get<float>(rhs); });

        std::vector<PairedLight> res;
        std::vector<bool> used(lights.size(), false);

        for(auto& [i, j, s] : pairs) {
            // logInfo(fmt::format("diff:{}", s));
            if(used[i] || used[j])
                continue;
            used[i] = used[j] = true;
            res.push_back(PairedLight{ lights[i], lights[j] });
        }

        return removeReflected(std::move(res));
    }

    std::vector<PairedLight> removeReflected(std::vector<PairedLight> armors) {
        std::sort(armors.begin(), armors.end(), [](const PairedLight& lhs, const PairedLight& rhs) {
            return lhs.r1.center.y + lhs.r2.center.y < rhs.r1.center.y + rhs.r2.center.y;
        });

        std::vector<PairedLight> res;
        res.reserve(armors.size());
        std::vector<cv::Rect2f> exceptBounds;
        exceptBounds.reserve(armors.size());
        for(auto& armor : armors) {
            auto bound = boundingRect(armor);

            bool flag = true;
            for(auto& exceptBound : exceptBounds) {
                if((bound & exceptBound) == bound) {
                    flag = false;
                    break;
                }
            }
            if(!flag)
                continue;

            bound.x -= bound.width * 0.25f;
            bound.width *= 1.5f;
            bound.y -= bound.height * 0.15f;
            bound.height *= 5.0f;
            exceptBounds.push_back(bound);
            res.push_back(armor);
        }
        return res;
    }

public:
    ArmorDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                     ACTOR_EXCEPTION_PROBE();

                     const auto frame = BlackBoard::instance().get<CameraFrame>(key).value();

                     DetectedArmorArray res;
                     res.frame = frame;

                     auto pairedLightVec = solve(frame.frame);
                     res.armors.reserve(pairedLightVec.size());
                     for(auto& pairedLight : pairedLightVec) {
                         res.armors.push_back({ 0, pairedLight });  // TODO id
                     }

                     sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)));
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorDetector);
