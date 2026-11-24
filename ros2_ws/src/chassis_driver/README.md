# chassis_driver

读取 STM32 底盘反馈并发布 ROS 2 传感器话题，同时订阅 `/cmd_vel` 并通过同一串口发送速度控制帧。反馈和控制由同一个节点独占串口，避免多个进程争用设备。

## 构建和启动

在已安装 ROS 2 Humble 的计算板上，从工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select chassis_driver
source install/setup.bash
ros2 launch chassis_driver chassis_driver.launch.py
```

如果设备路径与配置不同，可通过 ROS 参数覆盖：

```bash
ros2 launch chassis_driver chassis_driver.launch.py port:=/dev/ttyUSB0
```

确保运行用户有串口设备访问权限。串口监视脚本运行时应先停止它，避免两个进程争用同一串口。

## 发布话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | STM32 反馈的平面速度及其积分位姿；节点不广播 TF |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 三轴加速度与角速度；反馈帧未提供姿态，orientation 标记为不可用 |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 电池电压；协议未提供的电量和电流字段标记为未知 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 串口连接、帧率、校验错误、帧超时、电机标志、命令超时和限幅统计 |

反馈帧没有 STM32 时间戳，消息时间戳使用计算板 ROS 时钟，速度积分间隔使用本地单调时钟。超过 `max_integration_dt_sec` 的间隔不用于位姿积分，并计入诊断信息。

## `/cmd_vel` 控制

订阅类型为 `geometry_msgs/msg/Twist`。支持 `linear.x`（m/s）和 `angular.z`（rad/s）；L150 不支持 Y 轴平移，`linear.y` 及其他未支持分量会被忽略并记入诊断。控制帧以 20 Hz 定期发送，超过 `cmd_vel_timeout_sec` 没有新命令时自动发送零速度。默认速度限制为 `linear.x` ±0.2 m/s、`angular.z` ±0.5 rad/s，超过后会限幅；可在 YAML 中修改。

串口读取超时和写入超时分开配置：默认 `serial_timeout_sec=0.01`、`serial_write_timeout_sec=0.1`。若写控制帧超时，节点会关闭并尝试重连串口；重连期间无法发出速度指令，底盘端应配置串口失联停车保护。

控制帧依据《L150-C30D SE 避障巡线雷达小车用户手册（2026.10.29）》：11 字节，帧头 `0x7B`，两个预留字节置零，X/Y/Z 三轴目标速度为大端有符号 16 位数，BCC 为前 9 字节异或，帧尾 `0x7D`。线速度和角速度分别按 0.001 m/s、0.001 rad/s 编码。手册定义 Z 轴正方向为逆时针，与 ROS `angular.z` 一致。L150 不支持 Y 轴平移，因此 Y 速度发零。

ROS 控制模式需在小车处于 APP 或 PS2 遥控模式时使用；控制串口为 STM32 的 UART3，主控板经 CH9102 USB 串口连接计算板。反馈帧仍为 24 字节、20 Hz，BCC 对帧头及前 21 个字段异或。陀螺仪角速度换算系数按新版手册为 3755。若串口断开，节点无法发送停止帧；底盘固件侧仍需有串口失联停车保护。初次上电控制前请先架空车轮验证方向和限速。

可用 teleop 节点或如下命令持续发布速度，停止发布后节点会在超时后归零：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

当前默认假设反馈速度按 X 前、Y 横向、Z 旋转表达，`vx/vy` 是编码器反馈速度，电机标志 `0` 表示使能、`1` 表示失能。新版手册定义反馈 Z 轴正方向为逆时针，驱动按该方向发布 `/odom` 并积分 yaw。IMU 轴向仍应结合主控板安装方向和 `imu_link` 外参核对。里程计协方差是初始配置值，需根据实测调参。
