#include "cam_lidar_calib/magic_wand.h"
#include <opencv2/opencv.hpp>
#include <iostream> // Include iostream for debug output
std::vector<cv::Point> MagicWand::findRegion(const cv::Mat& image, cv::Point seed, int color_thresh) {
    // 修正：mask 尺寸需比 image 大2，且初始化为0
    cv::Mat mask = cv::Mat::zeros(image.rows + 2, image.cols + 2, CV_8UC1);
    cv::Scalar newVal(255,255,255);
    int loDiff = color_thresh, upDiff = color_thresh;
    cv::floodFill(image, mask, seed, newVal, 0, cv::Scalar(loDiff,loDiff,loDiff), cv::Scalar(upDiff,upDiff,upDiff), 4 | cv::FLOODFILL_MASK_ONLY | (255 << 8));
    std::vector<cv::Point> region;
    cv::Mat overlay;
    image.copyTo(overlay);
    // 只遍历有效区域，坐标需减1
    for (int y = 1; y < mask.rows - 1; ++y) {
        for (int x = 1; x < mask.cols - 1; ++x) {
            if (mask.at<uchar>(y,x) != 0) {
                region.push_back(cv::Point(x-1, y-1));
                // 半透明红色覆盖
                cv::Vec3b& pix = overlay.at<cv::Vec3b>(y-1, x-1);
                pix = cv::Vec3b(
                    static_cast<uchar>(0.5 * pix[0] + 0.5 * 0),   // B
                    static_cast<uchar>(0.5 * pix[1] + 0.5 * 0),   // G
                    static_cast<uchar>(0.5 * pix[2] + 0.5 * 255)  // R
                );
            }
        }
    }
    cv::imwrite("/tmp/find_region.png", overlay);
    std::cout << "[MagicWand::findRegion] mask size: (" << mask.rows << ", " << mask.cols << "), region size: " << region.size() << ", seed: (" << seed.x << ", " << seed.y << ")" << std::endl;
    return region;
}
cv::Point2f MagicWand::computeCenter(const std::vector<cv::Point>& region) {
    cv::Point2f center(0,0);
    for (const auto& p : region) center.x += p.x, center.y += p.y;
    if (!region.empty()) center *= (1.0f/region.size());
    std::cout << "[MagicWand::computeCenter] region size: " << region.size() << ", center: (" << center.x << ", " << center.y << ")" << std::endl;
    return center;
}
