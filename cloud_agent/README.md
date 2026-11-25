# SENSE 云端 Agent 与三人联调

课程把 **12 月 16 日的检查点**定为“云端任务请求与机器人端的基础交互”。建议先展示一句“查询小车状态”取得带时间的实机回执，再展示“探索会议室，找到会议桌并到桌旁”的多步任务。验收记录要写明回执来自香橙派、软件模拟器，还是开发电脑。项目方案另列完整语义导航、安全降级等五个实施阶段。[第一周概要](../doc/课程文档/第一周.md) · [项目方案](../doc/plan.md) · [硬件接入步骤](../doc/流程记录.md)

## 项目从接线到验收

课程文档保留了三名成员的姓名，端侧两人的具体职责待组内确认。下表按底盘/定位、视觉/语义、云端三块安排交接。

| 检查点 | 怎么做 | 这一关交出什么 |
|---|---|---|
| 11/25，环境 | 板端负责人核对 Orange Pi 5 Pro 16GB 镜像及供电，准备 Ubuntu 22.04、Tailscale、SSH 和 ROS 2 Humble。Windows 与板子加入同一 tailnet，核对设备名、登录用户和 SSH 主机指纹。 | 板端登录结果、系统和 ROS 版本、一次带日期的检查记录。 |
| 11/25，底盘 | STM32 接板，先核验反馈帧、单位及轴向，再由底盘负责人架空车轮验证超时停车、急停与运动方向。一个驱动节点独占串口。 | 一段含 /odom、/imu/data_raw、/battery_state、/diagnostics 的录包、时间来源与异常说明。 |
| 11/25，相机 | Astra Pro Plus 接 USB，运行相机驱动，核对彩色与深度分辨率、内参、无效深度、配准状态及时间差。 | 同一时段 RGB、深度、CameraInfo 样本及实际 frame_id。 |
| 12/2，SLAM | 对齐 RGB-D、IMU、里程计时间和 TF；在室内走一圈，回放并检查占据地图、位姿和定位丢失反馈。实现导航前还需端侧验证避障、急停和目标可达性。 | 地图版本、map → odom → base_link 的责任节点与坐标约定、有效/失定位两个机器人状态样例。 |
| 12/9，YOLO | 在板端识别物体，结合有效深度、标定和定位投影到地图；多帧合并同一物体，再赋稳定 object_id。 | 类别/区域词表、同类多目标样本、空结果样本、目标消失或旧地图样本。 |
| 12/16，云端基础交互 | 云端启动 Agent，板端 Gateway 主动领取高层请求，调用本地能力并回传状态。先验状态查询。运动工具须等端侧实现了对应 Action、安全监督和物理急停，并经现场验证后再接入。 | 一条真实 request_id 的请求、板端回执、云端任务结果；故障案例留存实际返回。 |
| 完整系统联调 | 语义查询 → 必要时 Frontier 探索 → 查询目标 → 端侧生成可达目标引用 → 本地导航 → 回报结果。分开验证多目标、地图变更、网络中断、定位丢失和导航阻塞。 | 对照日志、地图版本、最终任务状态和车端停止/降级记录。 |

仓库已有 [底盘驱动](../ros2_ws/src/chassis_driver/README.md) 和 [相机驱动](../ros2_ws/src/astra_camera_driver/README.md)。底盘驱动的 /odom 是积分里程计，驱动不广播 TF，电池话题已知量为电压。相机深度为 16UC1、单位毫米、0 为无效；实际配准情况决定 frame_id。这些区别会直接影响是否能把“会议桌”定位到正确地图。仓库内的驱动定义不能替代当天的实机测量。

## 你向两位队友要什么

**底盘、SLAM 与本地导航负责人**请提供板子在线时间、Tailscale 地址、SSH 用户和独立核验的主机指纹；一次已脱敏的状态回包；坐标系、定位有效与安全就绪的判断依据；Gateway 对应的 ROS 服务/Action 名称、类型、任务编号、取消与停止语义。运动联调前还需提供不可达、阻塞、失定位、断网及急停的实际反馈，并说明谁在车旁负责急停。电量百分比未知时传 null。

**相机、YOLO 与语义地图负责人**请提供相机标定与配准记录、坐标和单位说明、支持的类别和区域别名，以及带 object_id、category、region、confidence、frame、observed_at、map_version 的语义查询样本。至少包括无匹配、单个、多个、移动或消失四种情况。请约定 generate_navigation_goal 由哪个端侧节点为当前对象生成可达位置，并在执行前复核。云端日常对接使用摘要，调试图像或录包按需另交。

**你交回给队友**的是两个 HTTP 路径、请求/结果字段、12 个工具约定和验收命令。请组内指定一人负责板端 Gateway 的 ROS 适配。日常调试可通过 Tailscale SSH；正式通信由设备向云端发起 HTTPS 请求。

## 在本机运行你的部分

从仓库根目录执行。Python 3.10 及以上版本即可；本模块仅使用标准库。

```powershell
python -m unittest discover -s cloud_agent -p "test_*.py" -v
python -m cloud_agent --example
```

示例用显式意图驱动内存中的会议室地图。输出 mode=simulation、按顺序调用的工具、参数及最终状态。要测试自然语言理解，将服务商支持工具调用的 Chat Completions **完整地址**设为 SENSE_MODEL_URL，设置 SENSE_MODEL 为模型名、SENSE_MODEL_KEY 为服务商密钥。可从已有环境变量注入密钥；日志只需保存任务、参数和结果。

```powershell
$env:SENSE_MODEL_URL = "https://你的服务商域名/v1/chat/completions"
$env:SENSE_MODEL = "你的模型名"
$env:SENSE_MODEL_KEY = $env:TEAM_MODEL_KEY
python -m cloud_agent "探索会议室，找到会议桌并到桌旁"
```

有模型时上述命令测试“真实模型 + 模拟 Gateway”；--example 完全离线。模型只返回 select_task 的受限字段，确定性编排负责查状态、选择允许的工具、检查回执和报告结果。可用 --task state 等显式意图独立测试 Gateway。默认 Gateway 为模拟器，接真实服务须加 --gateway。

## 在香橙派上验证一次真实状态查询

先通过 Tailscale 确认板子在线并核验 SSH 主机指纹，使用本人获准的 SSH 登录方式。以下探针在 SSH 会话中用板上已有的 python3 从标准输入执行，退出即结束。它只读取主机名、架构、内核、采样时间及 ROS 环境文件是否存在。定位和安全监督返回 null。此步骤能验证基础请求往返，ROS 话题状态和运动能力要由正式 Gateway 证明。

1. 在 Windows PowerShell 的仓库根目录运行下面三行。第二行打开继承临时令牌的窗口 B，当前窗口 A 留给 Agent。

   ```powershell
   $env:SENSE_GATEWAY_TOKEN = python -c "import secrets; print(secrets.token_urlsafe(32))"
   Start-Process powershell -WorkingDirectory (Get-Location).Path
   python -m cloud_agent --gateway
   ```

2. 在窗口 B 运行以下命令，把参数换成组内核验过的 SSH 目标。SSH 将板端回环端口 18787 转发到电脑回环端口 8787。两端口均可用 CLI 参数调整。

   ```powershell
   python -m cloud_agent.readonly_link 用户名@板子的Tailscale地址
   ```

3. 在窗口 A 的提示符输入 /state。如果配置了模型，也可以输入“查看小车现在的状态”。确认 mode=gateway、status=SUCCESS、source=readonly_host_probe，核对 hostname 与 observed_at。探针处理一次请求就退出。若超时，请依次核对 board 在线状态、SSH 主机指纹和登录结果、反向端口转发权限、两窗口令牌，以及 A 是否在 B 启动前开始监听。

## 完整 Gateway 怎么对接

在有证书的云主机运行下列命令。环境变量配置和本机一致。板端验证云端证书，使用同一个单设备令牌。另一个部署办法是让 Agent 监听回环，由同机 HTTPS 反向代理转发这两个路径。

```bash
python3 -m cloud_agent --gateway --host 0.0.0.0 --port 8787 --certfile /path/fullchain.pem --keyfile /path/privkey.pem --wait-seconds 180
```

1. 板端发送 POST /v1/poll，JSON 为 {}。两个接口都带 Content-Type: application/json 和 Authorization: Bearer 加令牌。空闲时收到 {"command":null}，间隔一段时间再轮询。
2. 有请求时，command 包含 request_id、tool、arguments、expires_at（含时区）。板端核验白名单、参数、有效期、处理过的 ID、当前地图和本地安全状态。运动前由 Safety Supervisor 最终准入。
3. 板端向 POST /v1/result 提交同一 request_id、status 和 data 对象。同步查询直接返回 SUCCESS。长任务接受后返回 RUNNING 和 data.task_id；下次进度查询以 task_id 对应同一个本地 Action。
4. HTTP 200 加 accepted=true 表示回执收到。相同回执可重发；冲突回执为 409，已过期的在保留窗口内为 410。一次请求只交付一次，已领取而回执丢失时云端报告 TIMEOUT 和 execution_unknown=true，由板端核实执行状态。服务内存保存单设备进行中的请求与最近回执；重启后的任务恢复依赖板端任务记录。

| 工具 | 云端参数 | 板端 SUCCESS 或 RUNNING 的关键数据 |
|---|---|---|
| get_robot_state | {} | 实测状态、safety_ready、localization_valid；只有两者均为 true 才接受云端运动流程 |
| get_map_summary | {} | map_version、覆盖摘要 |
| query_semantic_map | category、region 可选 | objects 列表和 map_version；每个对象含唯一 object_id |
| detect_objects | region 可选 | 当前观测的目标摘要 |
| request_reobservation | region | 重观测结果或实际状态 |
| start_exploration | region 可选 | 完成返回 SUCCESS；进行中返回 RUNNING 与 task_id |
| get_exploration_status | task_id 可选 | 公开查询可给总体进度；指定任务时回显同一 task_id |
| pause_exploration | {} | 端侧实际暂停结果 |
| generate_navigation_goal | object_id、map_version | goal_id、frame、带时区 timestamp、map_version、reachable；本地保管位姿 |
| navigate_to_pose | 上述 goal_id、frame、timestamp、map_version | 本地复核地图、有效期和可达性后执行，返回结果或 task_id |
| get_navigation_status | task_id | 同一 task_id 的 RUNNING、SUCCESS 或失败原因 |
| stop_robot | {} | 停车结果；云端停止请求优先，本地急停始终由现场独立处理 |

多目标时在交互窗口输入 /choose 加候选 object_id。长任务可设置 --wait-seconds 等待并自动续查，超出预算后用 /status explore 任务编号 或 /status navigate 任务编号检查原动作。RUNNING 表示已受理。定位丢失、地图过期、目标不可达分别返回 LOCALIZATION_LOST、STALE_MAP、UNREACHABLE，由云端显示恢复建议。已经发出的运动请求超时后不会自动重发。

## 提交与演示顺序

先保存离线测试输出，再保存真实模型对几条指令的解析结果，最后保存板端真实状态往返。组内具备本地导航后再记录完整任务、停止、断网与过期地图案例。开发改动都在 cloud_agent/，通过功能分支和合并请求交付；板端负责人按组内计划部署正式 Gateway。截图、答辩与报告中注明运行模式、设备、代码提交及测试时间。
