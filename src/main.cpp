#include "cam_lidar_calib/calib_node.h"
int main(int argc, char** argv) {
    ros::init(argc, argv, "cam_lidar_calib_node");
    ros::NodeHandle nh;
    CalibNode node(nh);
    ros::spin();
    return 0;
}
