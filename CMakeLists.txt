cmake_minimum_required(VERSION 3.0.2)
project(cam_lidar_calib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  pcl_ros
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cam_lidar_calib
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport pcl_ros std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(calib_node src/main.cpp src/calib_node.cpp src/magic_wand.cpp src/pnp_solver.cpp)
target_link_libraries(calib_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_executable(projector_node src/projector_node.cpp)
target_link_libraries(projector_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)
add_dependencies(projector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
