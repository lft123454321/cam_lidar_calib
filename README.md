# Camera-LiDAR Joint Calibration Package

This package provides a ROS1 (Noetic) solution for joint calibration between a single camera and a single LiDAR. It includes:
- A calibration node for collecting corresponding points between image and point cloud
- An rqt GUI plugin for pixel selection in images
- PnP/PnP RANSAC calculation for extrinsic parameters

## Features
- Select a pixel in the image via GUI, use "magic wand" to find similar color region and compute its center
- Select a LiDAR point in RViz, find nearby points and compute their center
- Collect enough pairs, then calculate the transformation between camera and LiDAR

## Usage
### 1. Build
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Run Calibration Node
```
roslaunch cam_lidar_calib calib_node.launch
```

### 3. Run RViz
```
rviz -d rviz/calib.rviz
```

### 4. Select Points
- In the rqt window, click on the image to select a pixel. The plugin will publish `/selected_pixel`. You can open `/tmp/find_region.png` to view the magic wand selection range in real-time (if the selection range is incorrect, try changing the threshold in the code or use a calibration object with more contrast to the environment)
- In RViz, click a point in the point cloud (topic `/clicked_point`).
- The calibration node will process both, find regions, and collect pairs.

### 5. Calibration
- After collecting enough pairs, the node will automatically run PnP/PnP RANSAC and output the extrinsic parameters.

## Calibration Workflow

1. **Start Calibration Node**
   - Use the launch file to start `calib_node`.
   - Configure image, point cloud, and camera info YAML path parameters as needed.

2. **Collect Correspondences**
   - Select image pixels via RQT or other GUI (`/selected_pixel` topic).
   - Click point cloud locations in RViz (`/clicked_point` topic).
   - The node automatically collects image and cloud points, and displays collection status in the console.
   - The selected cloud region is published to `/clicked_cloud_region` for visualization in RViz.

3. **Automatic Extrinsic Calculation**
   - When enough pairs are collected (default: 20), the node runs PnP and saves the extrinsic matrix to `/tmp/extrinsic_result.yaml` in YAML format.

## Verification Workflow

1. **Start Projector Node**
   - Use the launch file to start `projector_node`.
   - Set image, cloud, camera info, and extrinsic YAML paths as parameters.

2. **Visualization**
   - The node subscribes to image and point cloud topics, projects the cloud onto the image using the extrinsic and intrinsic parameters.
   - The projected image is published to `/projected_image`.
   - Projected points are color-coded by distance (rainbow HSV mapping) and visualized as large dots.

## Main Topics
- `/selected_pixel`: Image pixel selection (geometry_msgs/Point)
- `/clicked_point`: Point cloud selection (geometry_msgs/PointStamped)
- `/clicked_cloud_region`: Selected cloud region for visualization (pcl::PointCloud<PointXYZI>)
- `/projected_image`: Image with projected cloud points

## Main Parameters
- `image_topic`, `cloud_topic`, `clicked_point_topic`, `camera_info_yaml`, `extrinsic_path`, `output_topic`, `max_pairs`

## File Outputs
- `/tmp/extrinsic_result.yaml`: Extrinsic calibration result (YAML)

## Visualization
- Use RViz to visualize `/clicked_cloud_region` and `/projected_image` for calibration and verification.

## Dependencies
- ROS Noetic
- OpenCV
- PCL
- cv_bridge
- image_transport
- rqt_gui, python_qt_binding

## Notes
- Make sure your camera and LiDAR topics are correctly set in the launch/config files.
- You can adjust the magic wand and point cloud region parameters in the source code.

## License
MIT
