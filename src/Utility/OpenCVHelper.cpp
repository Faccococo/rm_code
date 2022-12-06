#include "Utility.hpp"

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness) {
    cv::Point2f pts[5];
    rect.points(pts);
    pts[4] = pts[0];

    for(int i = 0; i < 4; ++i)
        cv::line(img, cv::Point{ pts[i] }, cv::Point{ pts[i + 1] }, color, thickness);
}

void boxRect(std::vector<cv::Point2f>& res, const cv::RotatedRect& rect) {
    // assert(rect.size.width <= rect.size.height);
    rect.points(res.data());

    uint32_t selectedIdx = 0;
    double minX = 1e5;

    for(uint32_t idx = 0; idx < 4; ++idx)
        if(res[idx].x < minX && res[idx].y > res[(idx + 2) % 4].y) {
            selectedIdx = idx;
            minX = res[idx].x;
        }

    if(selectedIdx)
        std::rotate(res.begin(), res.begin() + selectedIdx, res.begin() + 4);
}
