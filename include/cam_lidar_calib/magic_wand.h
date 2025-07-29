#pragma once
#include <opencv2/opencv.hpp>
class MagicWand {
public:
    static std::vector<cv::Point> findRegion(const cv::Mat& image, cv::Point seed, int color_thresh);
    static cv::Point2f computeCenter(const std::vector<cv::Point>& region);
};
