# SENSE

## Orbbec 深度相机单帧采集

在连接相机的 Ubuntu/Linux 计算板上，从项目根目录创建并启用 Python 虚拟环境：

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
cd ~/project/SENSE
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

采集一帧 RGB 和深度图：

```bash
python camera/capture_one_frame.py
```

脚本将图像保存到 `camera/test_img/`：`rgb.png` 是 RGB 图像，`depth.png` 是保留相机原始数值的 16 位深度图，`depth_color.png` 是便于查看的彩色深度预览。默认等待 30 秒，可通过 `--timeout` 设置等待时间，例如 `python camera/capture_one_frame.py --timeout 60`。

完成后退出虚拟环境：

```bash
deactivate
```

`pyorbbecsdk` 的可用安装包取决于计算板的 CPU 架构和 Python 版本。如果 `pip install -r requirements.txt` 无法安装该包，请按 [Orbbec pyorbbecsdk 官方仓库](https://github.com/orbbec/pyorbbecsdk) 的说明安装适用于本机环境的 SDK；完成后重新运行采集命令。
