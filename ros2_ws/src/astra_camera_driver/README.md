# astra_camera_driver

ROS 2 C++ driver for the Orbbec Astra Pro Plus. It reads RGB from V4L2 and depth from OpenNI2, publishes `sensor_msgs/Image` and `sensor_msgs/CameraInfo`, and enables OpenNI depth-to-color registration when the camera reports support.

## Prerequisites

- ROS 2 Humble (or another ROS 2 release with the listed packages)
- OpenNI2 installed by `camera/install_astra_openni.sh`
- OpenCV, `cv_bridge`, and `camera_info_manager`
- The user must be allowed to access `/dev/video0` and the Orbbec USB device

Install ROS dependencies on Ubuntu:

```bash
sudo apt update
sudo apt install -y ros-humble-cv-bridge ros-humble-camera-info-manager libopencv-dev
```

## Build and run

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

The camera installer caches the SDK under `camera/.sdk_cache/`; it does not download the SDK again when the cached archives pass ZIP validation. It also installs a standalone one-frame capture helper. The ROS node opens persistent RGB and depth streams itself and does not launch that helper per frame.

## Topics

| Topic | Type | Description |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | BGR color image (`bgr8`) |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Color camera calibration |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 16-bit depth in millimeters (`16UC1`); zero means invalid |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth calibration, or color calibration when registered |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | Published only when depth-to-color registration is active |
| `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | Color intrinsics for registered depth |

When registration is supported and the output dimensions match RGB, `/camera/depth/image_raw` and `/camera/aligned_depth_to_color/image_raw` contain the same registered frame, with `camera_color_optical_frame`. Otherwise only the native-coordinate depth topic is published with `camera_depth_optical_frame`.

## Calibration

Set `color_camera_info_url` and `depth_camera_info_url` in `config/astra_camera.yaml` to calibrated ROS camera YAML files, for example `file:///home/ucas/camera_calibration/color.yaml`. Do not use guessed intrinsics. If no calibration URL is set, the node still publishes `CameraInfo`, but with `K[0] = 0` to mark it uncalibrated. Registered depth uses the color calibration; native depth uses the depth/IR calibration.

## Parameters

`video_device`, `width`, `height`, `fps`, `rgb_format`, `depth_to_color`, `depth_mirror`, `sync_tolerance_ms`, frame IDs, topic names, and calibration URLs are configurable in `config/astra_camera.yaml`. The node pairs each depth frame with the next RGB frame using host receive time and drops pairs whose receive-time gap exceeds `sync_tolerance_ms`; this is approximate synchronization, not a hardware trigger.
