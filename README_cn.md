# 相机-激光雷达标定工具包

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
