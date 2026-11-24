# SENSE

## Astra Pro Plus 单帧采集

此项目通过 V4L2 直接采集 RGB，通过 Orbbec OpenNI SDK 采集 Astra Pro Plus 深度。OpenNI SDK 官方设备列表包含 Astra Pro Plus，并提供 Linux ARM64 支持；本相机不使用 `pyorbbecsdk`。

### 一键安装环境和 SDK

在连接相机的 Ubuntu/Linux ARM64 计算板上，从项目根目录运行：

```bash
bash camera/install_astra_openni.sh
```

脚本会下载 Orbbec OpenNI 发布包及其中的 ARM64 SDK 包，安装系统依赖和 SDK，创建 `.venv` 并安装 `requirements.txt`，最后编译深度采集辅助程序到 `~/.local/bin/openni_depth_capture`。脚本需要网络连接和 sudo 权限，请以普通用户运行，不要在整个命令前加 `sudo`。SDK 官方安装说明见 [OpenNI Linux 安装文档](https://github.com/orbbec/OpenNI_SDK/blob/main/Doc/English/0100_Installation.md)。

### 采集一帧 RGB 和深度图

如果安装脚本提示当前用户刚加入 `video` 组，先注销并重新登录。然后在项目根目录运行：

```bash
source .venv/bin/activate
python camera/capture_one_frame.py
```

脚本将图像保存到 `camera/test_img/`：`rgb.png` 是 RGB 图像，`depth.png` 是 16 位原始深度图，`depth_color.png` 是便于查看的彩色深度预览。默认 RGB 设备为 `/dev/video0`，分辨率为 640×480；可通过 `--rgb-device`、`--width`、`--height` 和 `--timeout` 调整。

如果访问 `/dev/video0` 报权限错误，将当前用户加入 `video` 组并重新登录：

```bash
sudo usermod -aG video "$USER"
```

完成后退出虚拟环境：

```bash
deactivate
```
