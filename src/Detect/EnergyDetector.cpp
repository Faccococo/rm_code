#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "EnergyDetect.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

struct EnergyDetectorSettings final {
    int smallPredictMode;
    int bigPredictMode;
    int preFrames;
    float armorMinArea;
    float armorMaxArea;
    float armorMinWHRatio;
    float armorMaxWHRatio;
    float armorMinAreaRatio;
    float stripMinArea;
    float stripMaxArea;
    float stripMinWHRatio;
    float stripMaxWHRatio;
    float stripMaxAreaRatio;
    float noiseArea;
    float predictAngle;
    float radius;
    float offsetPreAngle;
    cv::Point2f offset;
};

template <class Inspector>
bool inspect(Inspector& f, EnergyDetectorSettings& x) {
    return f.object(x).fields(f.field("smallPredictMode", x.smallPredictMode), f.field("bigPredictMode", x.bigPredictMode),
                              f.field("preFrames", x.preFrames).fallback(12), f.field("armorMinArea", x.armorMinArea),
                              f.field("armorMaxArea", x.armorMaxArea), f.field("armorMinWHRatio", x.armorMinWHRatio),
                              f.field("armorMaxWHRatio", x.armorMaxWHRatio), f.field("armorMinAreaRatio", x.armorMinAreaRatio),
                              f.field("stripMinArea", x.stripMinArea), f.field("stripMaxArea", x.stripMaxArea),
                              f.field("stripMaxWHRatio", x.stripMaxWHRatio), f.field("stripMaxAreaRatio", x.stripMaxAreaRatio),
                              f.field("noiseArea", x.noiseArea), f.field("predictAngle", x.predictAngle),
                              f.field("radius", x.radius), f.field("offsetPreAngle", x.offsetPreAngle),
                              f.field("offsetX", x.offset.x), f.field("offsetY", x.offset.y));
}

class EnergyDetector final
    : public HubHelper<caf::event_based_actor, EnergyDetectorSettings, energy_detect_available_atom, image_frame_atom> {

    Identifier mKey;

    bool mEnabled = true;

    void reset() {}

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

    static void setBinary(const cv::Mat& src, cv::Mat& binary) {
        std::vector<cv::Mat> imgChannels;
        cv::split(src, imgChannels);

        cv::Mat gray, grayBin, colorBin;
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

        constexpr auto threshold = 90;
        if(GlobalSettings::get().selfColor == Color::Red) {
            cv::threshold(gray, grayBin, threshold, 255, cv::THRESH_BINARY);
            const auto energyRed = imgChannels[2] - imgChannels[0];
            cv::threshold(energyRed, colorBin, threshold, 255, cv::THRESH_BINARY);
        } else {
            cv::threshold(gray, grayBin, threshold, 255, cv::THRESH_BINARY);
            const auto energyBlue = imgChannels[0] - imgChannels[2];
            cv::threshold(energyBlue, colorBin, threshold, 255, cv::THRESH_BINARY);
        }

        binary = grayBin & colorBin;

        // only for test
        // constexpr auto threshold = 200;
        // binary = imgChannels[0] > threshold & imgChannels[1] > threshold & imgChannels[2] > threshold;
    }

    bool stripJudge(const std::vector<cv::Point>& contour, const cv::RotatedRect& rotatedRect) const {
        cv::Point2f rectPoints[4];
        rotatedRect.points(rectPoints);
        const auto height = std::fmin(rotatedRect.size.height, rotatedRect.size.width);
        const auto width = std::fmax(rotatedRect.size.height, rotatedRect.size.width);
        const auto area = static_cast<float>(contourArea(contour));

        if(height * width > mConfig.stripMinArea && height * width < mConfig.stripMaxArea &&
           width / height < mConfig.stripMaxWHRatio && width / height > mConfig.stripMinWHRatio &&
           area / rotatedRect.size.area() < mConfig.stripMaxAreaRatio)
            return true;
        return false;
    }

    bool detectArmor(const cv::Mat& src, cv::RotatedRect& res, double& angle) {
        ACTOR_EXCEPTION_PROBE();
        auto binary = src.clone();
        setBinary(src, binary);

        auto element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));
        dilate(binary, binary, element);
        // element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));
        // erode(binary, binary, element);

        debugView("binary", binary, [](cv::Mat&) {});

        std::vector<std::vector<cv::Point> > armorContours;
        std::vector<cv::Vec4i> armorHierarchy;
        findContours(binary, armorContours, armorHierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        const auto armorContoursSize = armorContours.size();

        if(armorContoursSize == 0) {
            logInfo("Energy detect failed");
            return false;
        }

        debugView("contours", src, [&](cv::Mat& img) { cv::drawContours(img, armorContours, -1, cv::Scalar{ 255, 0, 0 }); });

        std::vector<uint32_t> conIndices;
        for(uint32_t i = 0; i < armorContoursSize; ++i) {
            if(contourArea(armorContours[i]) > mConfig.noiseArea) {
                if(stripJudge(armorContours[i], minAreaRect(armorContours[i]))) {
                    conIndices.push_back(i);
                }
            }
        }

        if(conIndices.empty()) {
            logInfo("Strip detect failed: no strip");
            for(uint32_t i = 0; i < armorContoursSize; ++i) {
                const auto rect = minAreaRect(armorContours[i]);
                cv::Point2f P[4];
                rect.points(P);
                // for(int j = 0; j < 4; ++j) {
                //     line(src, P[j], P[(j + 1) % 4], cv::Scalar(0, 255, 0), 2, 8);
                // }
                logInfo(fmt::format("contourArea: {:.2f}", contourArea(armorContours[i])));
                logInfo(fmt::format("rectArea: {:.2f}", rect.size.area()));
                logInfo(fmt::format("ratio: {: .2f}",
                                    std::max(rect.size.height, rect.size.width) / std::min(rect.size.height, rect.size.width)));
            }
            return false;
        }


        debugView("filtered_contours", src, [&](cv::Mat& img) {
            for(auto idx : conIndices)
                cv::drawContours(img, armorContours, static_cast<int>(idx), cv::Scalar{ 255, 0, 0 });
        });

        auto index = std::numeric_limits<uint32_t>::max();
        auto minScore = 0.15f;

        for(const auto conIndex : conIndices) {
            // const auto finalLength = arcLength(armorContours[conIndex], true);
            // const auto finalArea = contourArea(armorContours[conIndex]);

            const auto ratio =
                static_cast<float>(contourArea(armorContours[conIndex])) / cv::minAreaRect(armorContours[conIndex]).size.area();

            if(const auto score = std::fabs(ratio - 0.42f); score < minScore) {
                minScore = score;
                index = conIndex;
            }
        }
        if(index == std::numeric_limits<uint32_t>::max()) {
            logInfo("Strip detect failed: no strip contour \n");
            return false;
        }

        debugView("selected_contour", src, [&](cv::Mat& img) {
            cv::drawContours(img, armorContours, static_cast<int>(index), cv::Scalar{ 255, 0, 0 });
        });

        const auto finalRect = cv::boundingRect(armorContours[index]);
        const auto finalROI = binary(finalRect);
        const auto moments = cv::moments(finalROI, true);
        const auto centerX = moments.m10 / moments.m00;
        const auto centerY = moments.m01 / moments.m00;

        const auto a = moments.m20 / moments.m00 - centerX * centerX;
        const auto b = moments.m11 / moments.m00 - centerX * centerY;
        const auto c = moments.m02 / moments.m00 - centerY * centerY;

        auto rect = cv::minAreaRect(armorContours[index]);
        if(rect.size.width < rect.size.height) {
            rect.angle += 90.0f;
            std::swap(rect.size.width, rect.size.height);
        }
        auto theta = 0.5 * std::atan2(2 * b, a - c);
        // auto theta = glm::radians(rect.angle);
        const auto dx = (static_cast<double>(rect.center.x) - static_cast<double>(finalRect.x)) - centerX,
                   dy = (static_cast<double>(rect.center.y) - static_cast<double>(finalRect.y)) - centerY;
        if(dx * std::cos(theta) + dy * std::sin(theta) > 0.0)
            theta += glm::pi<double>();
        const auto cos = std::cos(theta), sin = std::sin(theta);
        const auto offset = static_cast<double>(rect.size.width) * 0.27;
        angle = theta;

        debugView("finalROI", finalROI, [&](cv::Mat& img) {
            cv::circle(img, { static_cast<int>(centerX), static_cast<int>(centerY) }, 5, cv::Scalar{ 0 });
            cv::line(img, { static_cast<int>(centerX), static_cast<int>(centerY) },
                     { static_cast<int>(centerX + offset * cos), static_cast<int>(centerY + offset * sin) }, cv::Scalar{ 255 });
        });

        const cv::Point2f center = { static_cast<float>(finalRect.x + centerX + offset * cos),
                                     static_cast<float>(finalRect.y + centerY + offset * sin) };
        res = cv::RotatedRect{ center, { rect.size.width * 0.2f, rect.size.height * 0.8f }, rect.angle };

        if(res.size.width >= res.size.height) {
            std::swap(res.size.width, res.size.height);
            res.angle += 90.0f;
        }

        debugView("detected", src, [&](cv::Mat& img) { drawRotatedRect(img, res, cv::Scalar{ 255, 0, 0 }, 4); });

        return true;
    }

    const std::vector<cv::Point3d> mObjectPointsLarge = {
        { -widthOfLargeArmor / 2, +heightOfLargeArmor / 2, 0.0 },
        { -widthOfLargeArmor / 2, -heightOfLargeArmor / 2, 0.0 },
        { +widthOfLargeArmor / 2, -heightOfLargeArmor / 2, 0.0 },
        { +widthOfLargeArmor / 2, +heightOfLargeArmor / 2, 0.0 },
    };
    std::vector<cv::Point2f> mImagePoint{ 4 };

    Point<UnitType::Distance, FrameOfRef::Camera> solve(const cv::Mat& cameraMatrix, const cv::RotatedRect& armor) {
        // armor.points(mImagePoint.data());
        boxRect(mImagePoint, armor);
        if(std::hypot(mImagePoint[0].x - mImagePoint[1].x, mImagePoint[0].y, mImagePoint[1].y) >
           std::hypot(mImagePoint[2].x - mImagePoint[1].x, mImagePoint[2].y, mImagePoint[1].y))
            std::rotate(mImagePoint.begin(), mImagePoint.begin() + 1, mImagePoint.end());

        const cv::Mat_<double> distCoeff;
        cv::Mat rvec, tvec;

        [[maybe_unused]] const auto res =
            cv::solvePnP(mObjectPointsLarge, mImagePoint, cameraMatrix, distCoeff, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        glm::dvec3 p0 = { tvec.at<double>(0, 0), -tvec.at<double>(1, 0), -tvec.at<double>(2, 0) };

        if(p0.z > 0.0)
            p0 = -p0;

        return Point<UnitType::Distance, FrameOfRef::Camera>{ p0 };
    }

    cv::Mat circleLeastFit(const cv::Mat& armorPoints, glm::dvec3& energyCenter, double& radius) {
        ACTOR_EXCEPTION_PROBE();
        const auto num = armorPoints.rows;
        // const auto dim = armorPoints.cols;
        const auto L1 = cv::Mat::ones(num, 1, CV_32F);
        cv::Mat Inv;
        cv::Mat a1 = armorPoints.t() * armorPoints;
        // std::cout<<"armorPoints"<<armorPoints<<std::endl;
        // std::cout<<"a1"<<a<<std::endl;
        cv::invert(a1, Inv);
        cv::Mat A = Inv * armorPoints.t() * L1;
        cv::Mat B = cv::Mat::zeros(static_cast<int>((num - 1) * num / 2), 3, CV_32F);
        int count = 0;
        //        logInfo("O");
        for(int i = 0; i < num - 1; ++i) {
            //            logInfo(fmt::format("I {:d}", i));
            for(int j = i + 1; j < num; ++j) {
                //                logInfo(fmt::format("J {:d}", j));
                cv::Mat a = armorPoints.row(j) - armorPoints.row(i);
                a.copyTo(B.row(count));
                count++;
            }
        }
        //        logInfo("A");
        cv::Mat L2 = cv::Mat::zeros((num - 1) * num / 2, 1, CV_32F);
        count = 0;
        for(int i = 0; i < num - 1; ++i) {
            for(int j = i + 1; j < num; ++j) {
                count++;
                const auto a = std::hypot(armorPoints.at<float>(j, 0), armorPoints.at<float>(j, 1), armorPoints.at<float>(j, 2));
                const auto b = std::hypot(armorPoints.at<float>(i, 0), armorPoints.at<float>(i, 1), armorPoints.at<float>(i, 2));
                L2.at<float>(count, 0) = (a * a - b * b) / 2;
            }
        }
        //        logInfo("B");
        cv::Mat D = cv::Mat::zeros(4, 4, CV_32F);
        cv::Mat E = B.t() * B;
        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                if(i < 3 && j < 3) {
                    D.at<float>(i, j) = E.at<float>(i, j);
                } else if(i < 3 || j < 3) {
                    D.at<float>(i, j) = A.at<float>(std::min(i, j), 0);
                }
            }
        }
        //        logInfo("C");
        cv::Mat L3 = cv::Mat::ones(4, 1, CV_32F);
        cv::Mat F = B.t() * L2;
        L3.at<float>(0, 0) = F.at<float>(0, 0);
        L3.at<float>(1, 0) = F.at<float>(1, 0);
        L3.at<float>(2, 0) = F.at<float>(2, 0);
        cv::Mat Dt;
        cv::invert(D.t(), Dt, 0);
        cv::Mat C = Dt * L3;
        energyCenter = glm::dvec3{ C.at<float>(0, 0), C.at<float>(1, 0), C.at<float>(2, 0) };
        //        std::cout<< "energy center"<<energyCenter.x<<" "<<energyCenter.y<<" "<<energyCenter.z;
        const cv::Mat C1 = (cv::Mat_<float>(1, 3) << C.at<float>(0, 0), C.at<float>(1, 0), C.at<float>(2, 0));
        radius = 0;
        cv::Mat C2 = armorPoints.clone();
        C1.copyTo(C2.row(0));
        // logInfo("D");
        for(int i = 0; i < num; ++i) {
            //            logInfo(fmt::format("I {:d}",i));
            cv::Mat a = armorPoints.row(i) - C2.row(0);
            //            logInfo(fmt::format("I {:d}",i));
            auto tem = cv::Mat(a);
            //            logInfo(fmt::format("I {:d}",i));
            radius += static_cast<double>(std::hypot(tem.at<float>(0, 0), tem.at<float>(0, 1), tem.at<float>(0, 2)));
        }
        radius /= num;
        //        logInfo("E");
        // std::cout << "A" << A.at<float>(0, 0) << " " << A.at<float>(1, 0) << " " << A.at<float>(2, 0);`

        return A;
    }

    float LineFitLeastSquares(const std::vector<float>& data_y) {
        float A = 0.0;
        float B = 0.0;
        float C = 0.0;
        float D = 0.0;

        int data_n = mConfig.preFrames - 1;
        for(int i = 0; i < data_n; i++) {
            A += static_cast<float>(i * i);
            B += static_cast<float>(i);
            C += static_cast<float>(i) * data_y[i];
            D += data_y[i];
        }

        float a, b;
        if(const auto temp = (static_cast<float>(data_n) * A - B * B); std::fabs(temp) > 1e-6f) {
            a = (static_cast<float>(data_n) * C - B * D) / temp;
            b = (A * D - B * C) / temp;
        } else {
            a = 0;
            b = 0;
        }

        return a * static_cast<float>(data_n + 1) + b;
    }

    Point<UnitType::Distance, FrameOfRef::Gun> predict(const cv::Mat& armorPoints, const glm::dvec3 data, const int direction,
                                                       const float angle) {
        ACTOR_EXCEPTION_PROBE();
        glm::dvec3 energyCenter;
        double radius;
        const auto normalVector = circleLeastFit(armorPoints, energyCenter, radius);
        // int ro = normalVector.rows;
        // int co = normalVector.cols;
        // logInfo(fmt::format("type {:d}",normalVector.type()));
        // logInfo(fmt::format("R {:d}",ro));
        // logInfo(fmt::format("C {:d}",co));
        float preAngle;
        if(direction == 0) {
            preAngle = angle;
        } else {
            preAngle = -angle;
        }
        const auto x = data.x - energyCenter.x;
        const auto y = data.y - energyCenter.y;
        const auto z = data.z - energyCenter.z;

        //        logInfo("F");
        const cv::Mat r = (cv::Mat_<float>(3, 1) << x, y, z);
        //        logInfo("G");
        const auto orthoVector = normalVector.cross(r);
        //        logInfo("H");
        const glm::dvec3 preCenter =
            glm::dvec3{ r.at<float>(0, 0) * cos(preAngle) + orthoVector.at<float>(0, 0) * sin(preAngle) + energyCenter.x,
                        r.at<float>(1, 0) * cos(preAngle) + orthoVector.at<float>(1, 0) * sin(preAngle) + energyCenter.y,
                        r.at<float>(2, 0) * cos(preAngle) + orthoVector.at<float>(2, 0) * sin(preAngle) + energyCenter.z };

        return Point<UnitType::Distance, FrameOfRef::Gun>{ preCenter };
    }

    bool getDirection(std::vector<double> angles, int& direction) {
        int positive = 0;
        int negative = 0;
        for(int j = 1; j < 3; ++j) {
            for(int i = 0; i < mConfig.preFrames - j; ++i) {
                if((angles[i] - angles[i + j]) > 0 || (angles[i] - angles[i + j]) < -300) {
                    positive++;
                } else if((angles[i] - angles[i + j]) < 0 || (angles[i] - angles[i + j]) > 300) {
                    negative++;
                }
            }
        }
        if(positive > negative) {
            direction = 1;  // anti-clockwise
        } else if(positive < negative) {
            direction = 0;  // clockwise
        }
        logInfo(fmt::format("direction {:d}", direction));
        return true;
    }

    bool predictAngle(std::vector<double> angles, float& preAngle) {
        std::vector<float> delAngles(mConfig.preFrames - 1);
        for(int i = 1; i < mConfig.preFrames; i++) {
            delAngles[i] = (float)fabs(angles[i] - angles[i - 1]);
        }
        auto Angle = LineFitLeastSquares(delAngles);
        //   logInfo(fmt::format("Angle {:.4f}", Angle));
        preAngle = Angle * mConfig.offsetPreAngle + mConfig.predictAngle;
        delAngles.clear();
        return true;
    }

    int mCount = 0;
    std::vector<double> mAngles;
    cv::Mat mArmorPoints = cv::Mat::zeros(mConfig.preFrames, 3, CV_32F);

public:
    EnergyDetector(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    ACTOR_PROTOCOL_CHECK(start_atom);
                    reset();
                },
                 [&](energy_detector_control_atom, const bool enable) {
                     ACTOR_PROTOCOL_CHECK(energy_detector_control_atom, bool);
                     if(mEnabled != enable)
                         reset();
                     mEnabled = enable;
                 },
                 [&](image_frame_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame>);
                     ACTOR_EXCEPTION_PROBE();

                     if(!mEnabled)
                         return;
                     auto [lastUpdate, info, frame] = BlackBoard::instance().get<CameraFrame>(key).value();

                     cv::RotatedRect armor;
                     double angle;
                     if(!detectArmor(frame, armor, angle))
                         return;

                     const auto& cameraInfo = info;

                     const auto point = solve(cameraInfo.cameraMatrix, armor);

                     const auto& transform = std::get<0>(cameraInfo.transform);

                     DetectedEnergyInfo res;
                     res.lastUpdate = lastUpdate;
                     res.point = transform.invTransform(point);

                     const auto raw = res.point.mVal;
                     // std::cout << "point" << raw.x << " " << raw.y << " " << raw.z << std::endl;

                     if(mCount < mConfig.preFrames) {
                         mAngles.push_back(angle);
                         mArmorPoints.at<float>(mCount, 0) = static_cast<float>(raw.x);
                         mArmorPoints.at<float>(mCount, 1) = static_cast<float>(raw.y);
                         mArmorPoints.at<float>(mCount, 2) = static_cast<float>(raw.z);
                         mCount++;
                     } else {
                         int direction = 0;
                         float preAngle;
                         getDirection(mAngles, direction);
                         predictAngle(mAngles, preAngle);
                         // logInfo(fmt::format("pre angle {:.4f}", preAngle));
                         res.prePoint = predict(mArmorPoints, raw, direction, preAngle);
                         mAngles.clear();
                         mCount = 0;
                     }
                     // const auto raw1 = res.prePoint.raw();
                     // std::cout << "pre point" << raw1.x << " " << raw1.y << " " << raw1.z << std::endl;
                     sendAll(energy_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, res));
                 } };
    }
};

HUB_REGISTER_CLASS(EnergyDetector);
