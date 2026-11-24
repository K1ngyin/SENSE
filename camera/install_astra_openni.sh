#!/usr/bin/env bash
set -Eeuo pipefail

# One-shot setup for Astra Pro Plus on Ubuntu/Debian ARM64.
# Downloads the official release ZIP, extracts its nested ARM64 SDK ZIP,
# installs OpenNI, creates the project venv, and builds the depth helper.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SDK_URL='https://dl.orbbec3d.com/dist/openni2/v2.3.0.86-beta6/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release.zip'
ARM64_ZIP_NAME='OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d_arm64.zip'
PYPI_INDEX_URL='https://pypi.tuna.tsinghua.edu.cn/simple'

die() {
    echo "错误：$*" >&2
    exit 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<'EOF'
一键配置 Astra Pro Plus（Ubuntu/Debian ARM64）：下载并安装 Orbbec OpenNI，
创建项目 Python 虚拟环境、安装 requirements.txt，并编译深度采集辅助程序。

用法：
  bash camera/install_astra_openni.sh

需要网络连接和 sudo 权限。不要在整个脚本前加 sudo；脚本会在需要时单独调用 sudo。
EOF
    exit 0
fi

[[ $# -eq 0 ]] || die "此脚本不接受参数；SDK 将从脚本内置的 Orbbec 下载链接获取。"
[[ "$EUID" -ne 0 ]] || die "请以普通用户运行脚本，不要使用 sudo bash。脚本会自行调用 sudo。"
case "$(uname -m)" in
    aarch64|arm64) ;;
    *) die "当前架构为 $(uname -m)，此脚本要求 Linux ARM64/aarch64。" ;;
esac
command -v apt-get >/dev/null 2>&1 || die "此脚本需要 Ubuntu/Debian 系统的 apt-get。"
command -v sudo >/dev/null 2>&1 || die "请先安装 sudo 并确保当前用户具有 sudo 权限。"

echo "安装系统依赖..."
sudo apt-get update
sudo apt-get install -y \
    build-essential g++ binutils curl unzip \
    python3-venv python3-pip \
    libusb-1.0-0 libusb-1.0-0-dev libudev1 libudev-dev

VENV_DIR="$PROJECT_DIR/.venv"
if [[ ! -x "$VENV_DIR/bin/python" ]]; then
    echo "创建 Python 虚拟环境：$VENV_DIR"
    python3 -m venv "$VENV_DIR"
fi
echo "安装 Python 依赖..."
"$VENV_DIR/bin/python" -m pip config --site set global.index-url "$PYPI_INDEX_URL"
"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install -r "$PROJECT_DIR/requirements.txt"

SDK_CACHE="$SCRIPT_DIR/.sdk_cache"
SDK_ZIP="$SDK_CACHE/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release.zip"
mkdir -p "$SDK_CACHE"
if [[ ! -s "$SDK_ZIP" ]] || ! unzip -tq "$SDK_ZIP" >/dev/null 2>&1; then
    echo "下载 Orbbec OpenNI SDK..."
    DOWNLOAD_TMP="$SDK_CACHE/.openni-download.zip"
    rm -f "$DOWNLOAD_TMP"
    curl --fail --location --retry 3 --connect-timeout 30 \
        --output "$DOWNLOAD_TMP" "$SDK_URL" \
        || die "下载失败：$SDK_URL"
    unzip -tq "$DOWNLOAD_TMP" >/dev/null \
        || die "下载文件不是有效 ZIP：$DOWNLOAD_TMP"
    mv -f "$DOWNLOAD_TMP" "$SDK_ZIP"
fi

WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT
RELEASE_DIR="$WORK_DIR/release"
SDK_DIR="$WORK_DIR/arm64_sdk"
mkdir -p "$RELEASE_DIR" "$SDK_DIR"

echo "解压外层 SDK 发布包..."
unzip -q "$SDK_ZIP" -d "$RELEASE_DIR"
ARM64_ZIP="$(find "$RELEASE_DIR" -type f -name "$ARM64_ZIP_NAME" -print -quit)"
[[ -n "$ARM64_ZIP" ]] || die \
    "外层 ZIP 中未找到预期的 ARM64 包：$ARM64_ZIP_NAME"

echo "解压 ARM64 SDK：$ARM64_ZIP_NAME"
unzip -q "$ARM64_ZIP" -d "$SDK_DIR"
INSTALL_SCRIPT="$(find "$SDK_DIR" -type f -name install.sh -print -quit)"
[[ -n "$INSTALL_SCRIPT" ]] || die \
    "ARM64 SDK 压缩包内未找到 install.sh，请检查下载包是否完整。"

INSTALL_DIR="$(dirname -- "$INSTALL_SCRIPT")"
echo "安装 OpenNI SDK..."
chmod +x "$INSTALL_SCRIPT"
(
    cd "$INSTALL_DIR"
    sudo ./install.sh
)
sudo ldconfig
sudo udevadm control --reload-rules || true
sudo udevadm trigger || true

HEADER="$(find /usr/include "$SDK_DIR" -type f -name OpenNI.h -print -quit 2>/dev/null || true)"
[[ -n "$HEADER" ]] || die "SDK 安装后找不到 OpenNI.h。"
INCLUDE_DIR="$(dirname -- "$HEADER")"

LIBRARY=""
while IFS= read -r -d '' candidate; do
    MACHINE="$(readelf -h "$candidate" 2>/dev/null \
        | awk -F: '/Machine:/ { sub(/^[[:space:]]+/, "", $2); print $2; exit }' || true)"
    if [[ "$MACHINE" == *AArch64* ]]; then
        LIBRARY="$candidate"
        break
    fi
done < <(find /usr/lib /usr/local/lib "$SDK_DIR" \
    \( -type f -o -type l \) -name 'libOpenNI2.so*' -print0 2>/dev/null)
[[ -n "$LIBRARY" ]] || die \
    "未找到 ARM64 版 libOpenNI2.so；请确认 ARM64 SDK 的 install.sh 安装成功。"
LIB_DIR="$(dirname -- "$(realpath -- "$LIBRARY")")"

DRIVER_DIR="$(find /usr/lib /usr/local/lib -type d -path '*/OpenNI2/Drivers' \
    -print -quit 2>/dev/null || true)"
[[ -n "$DRIVER_DIR" ]] || die \
    "SDK 安装后找不到 OpenNI2/Drivers 插件目录。"
OPENNI2_REDIST="$(dirname -- "$(dirname -- "$DRIVER_DIR")")"

BIN_DIR="$HOME/.local/bin"
mkdir -p "$BIN_DIR"
echo "编译 OpenNI 深度采集辅助程序..."
g++ -std=c++11 -O2 -Wall -Wextra \
    -I"$INCLUDE_DIR" \
    -DOPENNI2_REDIST_DEFAULT="\"$OPENNI2_REDIST\"" \
    "$SCRIPT_DIR/openni_depth_capture.cpp" \
    -Wl,-rpath,"$LIB_DIR" "$LIBRARY" \
    -o "$BIN_DIR/openni_depth_capture"
chmod +x "$BIN_DIR/openni_depth_capture"

if ! id -nG "$USER" | tr ' ' '\n' | grep -qx video; then
    sudo usermod -aG video "$USER"
    echo "已将 $USER 加入 video 组；请注销并重新登录后才能访问 /dev/video0。"
fi

echo
echo "安装完成。重新登录（若刚加入 video 组）后运行："
echo "  cd \"$PROJECT_DIR\""
echo "  source .venv/bin/activate"
echo "  python camera/capture_one_frame.py"
