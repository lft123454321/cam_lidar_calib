#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
class PnPSolver {
public:
    static cv::Mat solvePnP(const std::vector<cv::Point3f>& lidar_pts, const std::vector<cv::Point2f>& img_pts, const cv::Mat& K, const cv::Mat& distCoeffs);
};
