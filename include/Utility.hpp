#pragma once
#include "SuppressWarningBegin.hpp"

#include <opencv2/opencv.hpp>

#include "SuppressWarningEnd.hpp"

#include <utility>
#include <vector>

struct PIDParameters final {
    double kp, ki, kd;
};

class PIDSimulator final {
    PIDParameters mParameters;
    double mCurrent = 0.0;
    double mLastError = 0.0;
    double mSumError = 0.0;

public:
    explicit PIDSimulator(const PIDParameters& params) : mParameters{ params } {}
    void reset(const double current) {
        mCurrent = current;
    }
    // period = -1.0: disabled
    // period > 0.0: domain is [0,period)
    // return (position,velocity)
    [[nodiscard]] std::pair<double, double> step(double dt, double target, double maxV, double period = -1.0) noexcept;
};

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness = 1);

enum class RunStatus { running, normalExit, failureExit };

extern RunStatus globalStatus;

namespace caf {
    class local_actor;
}

void terminateSystem(caf::local_actor& actor, bool success);

extern std::string globalConfigName;
void appendTestResult(const std::string& message);
std::vector<uint32_t> solveKM(uint32_t n, uint32_t m, const std::vector<double>& w);

// width < height
// angle = 0
// 1 width 2
// height  height
// 0 width 3
// angle = 90
// 0 height 1
// width    width
// 3 height 2

void boxRect(std::vector<cv::Point2f>& res, const cv::RotatedRect& rect);
