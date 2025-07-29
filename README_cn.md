# 相机-激光雷达联合标定包

本工具包为单目相机与单线激光雷达的联合标定提供 ROS1 (Noetic) 解决方案。主要功能包括：
- 标定节点：采集图像与点云的对应点
- rqt GUI 插件：用于图像像素选取
- PnP/PnP RANSAC 外参计算

## 主要特性
- 通过 GUI 在图像中选取像素，使用“魔术棒”算法选取相似颜色区域并计算中心
- 在 RViz 中选取激光点，查找邻域点并计算中心
- 收集足够点对后，自动计算相机与激光雷达之间的变换关系

## 使用方法
### 1. 编译
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 启动标定节点
```
roslaunch cam_lidar_calib calib_node.launch
```

### 3. 启动 RViz
```
rviz -d rviz/calib.rviz
```

### 4. 选取点对
- 在 rqt 窗口点击图像选取像素，插件会发布 `/selected_pixel`。可打开 `/tmp/find_region.png` 实时查看魔术棒选区（如选区不理想，可调整阈值或使用对比度更高的标定物）。
- 在 RViz 中点击点云（话题 `/clicked_point`）。
- 标定节点会自动处理并收集点对。

### 5. 标定
- 收集到足够点对后，节点自动运行 PnP/PnP RANSAC，输出外参。

## 标定操作流程

1. **启动标定节点**
   - 使用 launch 文件启动 `calib_node`。
   - 根据实际情况配置图像、点云、内参 YAML 路径等参数。

2. **采集点对**
   - 通过 RQT 或其他界面选择图像像素（`/selected_pixel` 话题）。
   - 在 RViz 中点击点云位置（`/clicked_point` 话题）。
   - 节点会自动收集图像点和点云点，并在终端显示收集状态。
   - 当前选取的点云邻域会发布到 `/clicked_cloud_region`，可在 RViz 可视化。

3. **自动外参计算**
   - 收集到足够点对（默认 20 对）后，节点自动运行 PnP，结果保存为 `/tmp/extrinsic_result.yaml`（YAML 格式）。

## 验证操作流程

1. **启动投影验证节点**
   - 使用 launch 文件启动 `projector_node`。
   - 设置图像、点云、内参、外参 YAML 路径等参数。

2. **可视化验证**
   - 节点订阅图像和点云话题，利用内外参将点云投影到图像。
   - 投影结果图像发布到 `/projected_image`。
   - 投影点按距离进行彩虹色（HSV）编码，显示为大圆点。

## 主要话题
- `/selected_pixel`：图像像素选取（geometry_msgs/Point）
- `/clicked_point`：点云选取（geometry_msgs/PointStamped）
- `/clicked_cloud_region`：选取的点云邻域（pcl::PointCloud<PointXYZI>）
- `/projected_image`：投影结果图像

## 主要参数
- `image_topic`、`cloud_topic`、`clicked_point_topic`、`camera_info_yaml`、`extrinsic_path`、`output_topic`、`max_pairs`

## 文件输出
- `/tmp/extrinsic_result.yaml`：外参标定结果（YAML）

## 可视化
- 使用 RViz 可视化 `/clicked_cloud_region` 和 `/projected_image`，辅助标定与验证。

## 依赖
- ROS Noetic
- OpenCV
- PCL
- cv_bridge
- image_transport
- rqt_gui, python_qt_binding

## 注意事项
- 请确保 launch/config 文件中的相机与激光雷达话题设置正确。
- 可在源码中调整魔术棒和点云邻域参数。

## 许可证
MIT
