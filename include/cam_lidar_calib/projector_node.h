#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <string>

class ProjectorNode {
public:
    ProjectorNode(ros::NodeHandle& nh);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    bool loadCameraParams(const std::string& yaml_path);
    bool loadExtrinsic(const std::string& extrinsic_path);
private:
    ros::Subscriber cloud_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat extrinsic_; // 4x4
    cv::Mat current_image_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    bool camera_ready_ = false;
    bool extrinsic_ready_ = false;
};
