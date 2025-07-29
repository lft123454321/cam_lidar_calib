#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <image_transport/image_transport.h>

class CalibNode {
public:
    CalibNode(ros::NodeHandle& nh);
    bool loadCameraInfo(const std::string& yaml_path);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void clickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg);
    void selectedPixelCallback(const geometry_msgs::PointConstPtr& msg);
    std::vector<pcl::PointXYZI> findNearbyPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const geometry_msgs::Point& pt, double radius, double intensity_thresh);
    Eigen::Vector3f computeCloudCenter(const std::vector<pcl::PointXYZI>& region);
    void tryPnP();
private:
    image_transport::Subscriber image_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber clicked_point_sub_;
    ros::Subscriber selected_pixel_sub_;
    cv::Mat current_image_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;
    geometry_msgs::Point clicked_point_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_info_loaded_ = false;
    std::vector<cv::Point2f> image_points_;
    std::vector<cv::Point3f> cloud_points_;
    int max_pairs_;
    std::string image_topic_;
    std::string cloud_topic_;
    std::string clicked_point_topic_;
    std::string camera_info_yaml_;
    ros::Publisher cloud_region_pub_;
};
