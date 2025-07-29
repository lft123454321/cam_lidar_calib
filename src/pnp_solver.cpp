#include "cam_lidar_calib/pnp_solver.h"
#include <opencv2/calib3d.hpp>
#include <vector>
cv::Mat PnPSolver::solvePnP(const std::vector<cv::Point3f>& lidar_pts, const std::vector<cv::Point2f>& img_pts, const cv::Mat& K, const cv::Mat& distCoeffs) {
    cv::Mat rvec, tvec;
    std::cout << "[PnPSolver::solvePnP] Starting PnP with " << lidar_pts.size() << " points and " << img_pts.size() << " image points." << std::endl;
    cv::solvePnPRansac(lidar_pts, img_pts, K, distCoeffs, rvec, tvec);
    cv::Mat Rt;
    cv::Rodrigues(rvec, Rt);
    cv::Mat T = cv::Mat::eye(4,4,CV_64F);
    Rt.copyTo(T(cv::Rect(0,0,3,3)));
    T.at<double>(0,3) = tvec.at<double>(0);
    T.at<double>(1,3) = tvec.at<double>(1);
    T.at<double>(2,3) = tvec.at<double>(2);
    return T;
}
