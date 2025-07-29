#include "cam_lidar_calib/calib_node.h"
#include "cam_lidar_calib/magic_wand.h"
#include "cam_lidar_calib/pnp_solver.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <queue>
#include <set>

CalibNode::CalibNode(ros::NodeHandle& nh) {
    // 读取参数
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("image_topic", image_topic_, "/camera/image_raw");
    pnh.param<std::string>("cloud_topic", cloud_topic_, "/velodyne_points");
    pnh.param<std::string>("clicked_point_topic", clicked_point_topic_, "/clicked_point");
    pnh.param<std::string>("camera_info_yaml", camera_info_yaml_, "../config/camera_info.yaml");
    pnh.param<int>("max_pairs", max_pairs_, 20);


    image_transport::ImageTransport it(nh);
    image_sub_ = it.subscribe(image_topic_, 1, &CalibNode::imageCallback, this);
    cloud_sub_ = nh.subscribe(cloud_topic_, 1, &CalibNode::cloudCallback, this);
    clicked_point_sub_ = nh.subscribe(clicked_point_topic_, 1, &CalibNode::clickedPointCallback, this);
    selected_pixel_sub_ = nh.subscribe("/selected_pixel", 1, &CalibNode::selectedPixelCallback, this);
    camera_info_loaded_ = loadCameraInfo(camera_info_yaml_);
    // 新增点云发布器
    cloud_region_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/clicked_cloud_region", 1);
}

bool CalibNode::loadCameraInfo(const std::string& yaml_path) {
    // 读取相机内参
    YAML::Node config = YAML::LoadFile(yaml_path);
    auto mat = config["camera_matrix"]["data"];
    auto dist = config["distortion_coefficients"]["data"];
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    dist_coeffs_ = cv::Mat(1, 5, CV_64F);
    for (int i = 0; i < 9; ++i) camera_matrix_.at<double>(i/3, i%3) = mat[i].as<double>();
    for (int i = 0; i < 5; ++i) dist_coeffs_.at<double>(0, i) = dist[i].as<double>();
    return true;
}

void CalibNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image_ = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void CalibNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    current_cloud_ = cloud;
}

void CalibNode::clickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg) {
    clicked_point_ = msg->point;
    // 查找点云邻域
    std::vector<pcl::PointXYZI> region = findNearbyPoints(current_cloud_, clicked_point_, 0.5, 10.0); // intensity阈值可调
    Eigen::Vector3f cloud_center = computeCloudCenter(region);
    cloud_points_.push_back(cv::Point3f(cloud_center.x(), cloud_center.y(), cloud_center.z()));
    ROS_INFO("Collected cloud point: (%.2f, %.2f, %.2f)", cloud_center.x(), cloud_center.y(), cloud_center.z());
    ROS_INFO("total image points: %zu, total cloud points: %zu", image_points_.size(), cloud_points_.size());
    // 发布邻域点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr region_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    region_cloud->header = current_cloud_->header;
    region_cloud->points.insert(region_cloud->points.end(), region.begin(), region.end());
    cloud_region_pub_.publish(region_cloud);
    tryPnP();
}

std::vector<pcl::PointXYZI> CalibNode::findNearbyPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const geometry_msgs::Point& pt, double radius, double intensity_thresh) {
    std::vector<pcl::PointXYZI> result;
    if (!cloud || cloud->points.empty()) return result;
    // 找到种子点（距离最近且在radius范围内）
    int seed_idx = -1;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        double dist = std::sqrt(std::pow(cloud->points[i].x - pt.x, 2) + std::pow(cloud->points[i].y - pt.y, 2) + std::pow(cloud->points[i].z - pt.z, 2));
        if (dist < radius && dist < min_dist) {
            min_dist = dist;
            seed_idx = i;
        }
    }
    if (seed_idx == -1) return result;
    double seed_intensity = cloud->points[seed_idx].intensity;
    std::queue<int> q;
    std::set<int> visited;
    q.push(seed_idx);
    visited.insert(seed_idx);
    while (!q.empty()) {
        int idx = q.front(); q.pop();
        const auto& p = cloud->points[idx];
        result.push_back(p);
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            if (visited.count(i)) continue;
            double dist = std::sqrt(std::pow(cloud->points[i].x - p.x, 2) + std::pow(cloud->points[i].y - p.y, 2) + std::pow(cloud->points[i].z - p.z, 2));
            double intensity_diff = std::abs(cloud->points[i].intensity - seed_intensity);
            if (dist < radius && intensity_diff < intensity_thresh) {
                q.push(i);
                visited.insert(i);
            }
        }
    }
    return result;
}

Eigen::Vector3f CalibNode::computeCloudCenter(const std::vector<pcl::PointXYZI>& region) {
    Eigen::Vector3f center(0,0,0);
    for (const auto& p : region) center += Eigen::Vector3f(p.x, p.y, p.z);
    if (!region.empty()) center /= region.size();
    return center;
}

// 订阅rqt像素点，执行魔术棒算法，收集image点
void CalibNode::selectedPixelCallback(const geometry_msgs::PointConstPtr& msg) {
    ROS_INFO("Received selected pixel: (%.1f, %.1f)", msg->x, msg->y);
    if (current_image_.empty()) return;
    cv::Point seed(msg->x, msg->y);
    int color_thresh = 5; // 可调参数
    std::vector<cv::Point> region = MagicWand::findRegion(current_image_, seed, color_thresh);
    cv::Point2f center = MagicWand::computeCenter(region);
    image_points_.push_back(center);
    ROS_INFO("Collected image point: (%.1f, %.1f)", center.x, center.y);
    ROS_INFO("total image points: %zu, total cloud points: %zu", image_points_.size(), cloud_points_.size());
    tryPnP();
}

// 点对收集齐后自动PnP计算
void CalibNode::tryPnP() {
    if (image_points_.size() >= max_pairs_ && cloud_points_.size() >= max_pairs_) {
        ROS_INFO("Enough pairs collected, starting extrinsic calculation!");
        cv::Mat T = PnPSolver::solvePnP(cloud_points_, image_points_, camera_matrix_, dist_coeffs_);
        ROS_INFO_STREAM("Extrinsic matrix:\n" << T);
        // 保存为yaml格式
        std::ofstream fout("/tmp/extrinsic_result.yaml");
        fout << "extrinsic_matrix:" << std::endl;
        fout << "  rows: 4" << std::endl;
        fout << "  cols: 4" << std::endl;
        fout << "  data: [";
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                fout << T.at<double>(i, j);
                if (!(i == 3 && j == 3)) fout << ", ";
            }
        }
        fout << "]" << std::endl;
        fout.close();
        image_points_.clear();
        cloud_points_.clear();
    }
}
