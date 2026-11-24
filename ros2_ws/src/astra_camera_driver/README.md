# astra_camera_driver

Orbbec Astra Pro Plus 的 ROS 2 C++ 相机驱动。节点通过 V4L2 读取 RGB 图像，通过 OpenNI2 读取深度图，并发布 `sensor_msgs/msg/Image` 和 `sensor_msgs/msg/CameraInfo`。设备支持时会启用 OpenNI 深度到彩色图配准。

## 环境要求

- ROS 2 Humble（或包含下列依赖包的其他 ROS 2 发行版）
- 已运行 `camera/install_astra_openni.sh` 安装 OpenNI2
- OpenCV、`cv_bridge` 和 `camera_info_manager`
- 当前用户有权访问 `/dev/video0` 和 Orbbec USB 设备

在 Ubuntu 上安装 ROS 依赖：

```bash
sudo apt update
sudo apt install -y ros-humble-cv-bridge ros-humble-camera-info-manager libopencv-dev
```

## 构建和启动

```bash
cd ~/project/SENSE
bash camera/install_astra_openni.sh
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select astra_camera_driver
source install/setup.bash
ros2 launch astra_camera_driver astra_camera.launch.py
```

相机安装脚本会把 SDK 缓存在 `camera/.sdk_cache/`。缓存压缩包通过 ZIP 校验后，重复运行时会跳过下载。安装脚本也会编译一个单帧采集辅助程序；ROS 节点会自行持续读取 RGB 和深度流，不会逐帧启动该辅助程序。

## 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB 图像，编码为 `bgr8` |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB 相机标定信息 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 16 位毫米深度图，编码为 `16UC1`；0 表示无效深度 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 深度标定信息；配准后使用 RGB 相机内参 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 仅在启用深度到彩色图配准时发布 |
| `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | 配准深度图对应的 RGB 相机内参 |

设备支持配准且深度图分辨率与 RGB 相同时，`/camera/depth/image_raw` 和 `/camera/aligned_depth_to_color/image_raw` 发布相同的配准深度帧，其坐标系为 `camera_color_optical_frame`。否则只发布原始坐标系下的深度图，坐标系为 `camera_depth_optical_frame`。

## 相机标定

在 `config/astra_camera.yaml` 中，将 `color_camera_info_url` 和 `depth_camera_info_url` 设置为 ROS 相机标定 YAML 文件的 URL，例如：

```yaml
color_camera_info_url: "file:///home/ucas/camera_calibration/color.yaml"
depth_camera_info_url: "file:///home/ucas/camera_calibration/depth.yaml"
```

请使用相机厂商提供的标定参数或实际标定结果，不要估算内参。未配置标定 URL 时，节点仍会发布 `CameraInfo`，但 `K[0] = 0`，表示内参尚未标定。配准深度使用 RGB 相机标定参数；未配准深度使用深度/红外相机标定参数。

## 参数和同步方式

`config/astra_camera.yaml` 可配置视频设备、图像宽高、帧率、RGB 像素格式、深度配准开关、深度镜像开关、同步时间差上限、坐标系名称、话题名称和标定文件 URL。

节点依据计算板接收帧的时间配对深度帧和 RGB 帧。如果两帧接收时间差超过 `sync_tolerance_ms`，该帧对会被丢弃。这是近似同步，不是相机硬件触发同步。
