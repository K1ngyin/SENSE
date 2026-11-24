# chassis_driver

只接收 STM32 底盘反馈串口帧并发布 ROS 2 传感器话题。当前版本不会打开任何控制发送路径，也不会订阅 `/cmd_vel`。

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
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 串口连接、帧率、校验错误、帧超时、电机标志和队列统计 |

反馈帧没有 STM32 时间戳，消息时间戳使用计算板 ROS 时钟，速度积分间隔使用本地单调时钟。超过 `max_integration_dt_sec` 的间隔不用于位姿积分，并计入诊断信息。

当前默认假设底盘速度按 REP-103 约定表达（X 前、Y 左、Z 上），`vx/vy` 是编码器反馈的底盘速度，电机标志 `0` 表示使能、`1` 表示失能。请在实车联调时确认轴向、符号和反馈频率。里程计协方差是初始配置值，需根据实测调参。

## 后续控制接口

订阅 `/cmd_vel` 并发送 STM32 控制帧尚未实现，因为现有反馈脚本没有定义控制帧格式。确认控制帧布局、单位、校验和超时停车机制后，再扩展该串口驱动；同一串口应始终由一个进程独占。
