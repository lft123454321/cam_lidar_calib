#include "cam_lidar_calib/projector_node.h"
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

ProjectorNode::ProjectorNode(ros::NodeHandle& nh) {
    ros::NodeHandle pnh("~");
    std::string image_topic, cloud_topic, camera_info_yaml, extrinsic_path, output_topic;
    pnh.param<std::string>("image_topic", image_topic, "/camera/image_raw");
    pnh.param<std::string>("cloud_topic", cloud_topic, "/velodyne_points");
    pnh.param<std::string>("camera_info_yaml", camera_info_yaml, "../config/camera_info.yaml");
    pnh.param<std::string>("extrinsic_path", extrinsic_path, "extrinsic_result.yaml");
    pnh.param<std::string>("output_topic", output_topic, "/projected_image");

    image_transport::ImageTransport it(nh);
    image_sub_ = it.subscribe(image_topic, 1, &ProjectorNode::imageCallback, this);
    cloud_sub_ = nh.subscribe(cloud_topic, 1, &ProjectorNode::cloudCallback, this);
    image_pub_ = it.advertise(output_topic, 1);

    camera_ready_ = loadCameraParams(camera_info_yaml);
    extrinsic_ready_ = loadExtrinsic(extrinsic_path);
    current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

bool ProjectorNode::loadCameraParams(const std::string& yaml_path) {
    YAML::Node config = YAML::LoadFile(yaml_path);
    auto mat = config["camera_matrix"]["data"];
    auto dist = config["distortion_coefficients"]["data"];
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    dist_coeffs_ = cv::Mat(1, 5, CV_64F);
    for (int i = 0; i < 9; ++i) camera_matrix_.at<double>(i/3, i%3) = mat[i].as<double>();
    for (int i = 0; i < 5; ++i) dist_coeffs_.at<double>(0, i) = dist[i].as<double>();
    return true;
}

bool ProjectorNode::loadExtrinsic(const std::string& extrinsic_path) {
    YAML::Node config = YAML::LoadFile(extrinsic_path);
    auto mat = config["extrinsic_matrix"];
    int rows = mat["rows"].as<int>();
    int cols = mat["cols"].as<int>();
    auto data = mat["data"];
    std::vector<double> vals;
    for (size_t i = 0; i < data.size(); ++i) {
        vals.push_back(data[i].as<double>());
    }
    if (rows != 4 || cols != 4 || vals.size() != 16) return false;
    extrinsic_ = cv::Mat(4, 4, CV_64F);
    for (int i = 0; i < 16; ++i) extrinsic_.at<double>(i/4, i%4) = vals[i];
    return true;
}

void ProjectorNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image_ = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // 尝试投影
    if (camera_ready_ && extrinsic_ready_ && current_cloud_ && !current_image_.empty()) {
        cv::Mat img = current_image_.clone();
        for (const auto& pt : current_cloud_->points) {
            cv::Mat pt3d = (cv::Mat_<double>(4,1) << pt.x, pt.y, pt.z, 1.0);
            cv::Mat pt_cam = extrinsic_ * pt3d;
            double X = pt_cam.at<double>(0,0);
            double Y = pt_cam.at<double>(1,0);
            double Z = pt_cam.at<double>(2,0);
            double dist = std::sqrt(pt.x*pt.x + pt.y*pt.y);
            if (Z <= 0.1) continue;
            std::vector<cv::Point3f> obj_pts = {cv::Point3f(X, Y, Z)};
            std::vector<cv::Point2f> img_pts;
            cv::projectPoints(obj_pts, cv::Vec3d(0,0,0), cv::Vec3d(0,0,0), camera_matrix_, dist_coeffs_, img_pts);
            int u = static_cast<int>(img_pts[0].x);
            int v = static_cast<int>(img_pts[0].y);
            if (u >= 0 && v >= 0 && u < img.cols && v < img.rows) {
                // 距离范围归一化到[0,1]
                double norm = std::min(std::max((dist - 3.0) / (50.0 - 3.0), 0.0), 1.0);
                // HSV色彩空间，Hue从0（红）到160（紫）
                int hue = static_cast<int>(norm * 160); // 0-160
                cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
                cv::Mat rgb;
                cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);
                cv::Vec3b color = rgb.at<cv::Vec3b>(0,0);
                cv::circle(img, cv::Point(u, v), 4, cv::Scalar(color[0], color[1], color[2]), -1);
            }
        }
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
        image_pub_.publish(out_msg);
    }
}

void ProjectorNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::fromROSMsg(*msg, *current_cloud_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "projector_node");
    ros::NodeHandle nh;
    ProjectorNode node(nh);
    ROS_INFO("projector_node started.");
    ros::spin();
    return 0;
}
