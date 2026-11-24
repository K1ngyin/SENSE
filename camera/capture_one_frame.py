#!/usr/bin/env python3
"""Capture one Astra Pro Plus RGB frame (V4L2) and depth frame (OpenNI)."""

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def load_dependencies():
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError(
            "NumPy/OpenCV are missing. Activate the project virtual environment "
            "and run: python -m pip install -r requirements.txt"
        ) from exc
    return cv2, np


def find_depth_helper():
    from_path = shutil.which("openni_depth_capture")
    if from_path:
        return Path(from_path)
    user_install = Path.home() / ".local" / "bin" / "openni_depth_capture"
    if user_install.is_file():
        return user_install
    raise RuntimeError(
        "OpenNI depth helper is missing. First run camera/install_astra_openni.sh "
        "with the official Linux ARM64 OpenNI SDK archive."
    )


def capture_rgb(cv2, device, width, height):
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(
            "Cannot open RGB camera {}. Check /dev/video permissions and device path."
            .format(device)
        )
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    try:
        for _ in range(30):
            ok, image = capture.read()
            if ok and image is not None:
                return image
        raise RuntimeError("The RGB camera opened but did not return an image")
    finally:
        capture.release()


def capture_depth(helper, timeout, np, output_dir):
    with tempfile.NamedTemporaryFile(
        prefix="astra_depth_", suffix=".raw", dir=str(output_dir), delete=False
    ) as raw_file:
        raw_path = Path(raw_file.name)

    try:
        result = subprocess.run(
            [str(helper), str(raw_path), str(timeout)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            timeout=timeout + 5,
            check=False,
        )
        if result.returncode != 0:
            details = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(
                "OpenNI could not capture a depth frame. {}".format(details)
            )

        frame_line = next(
            (line for line in result.stdout.splitlines() if line.startswith("FRAME ")),
            None,
        )
        if frame_line is None:
            raise RuntimeError(
                "OpenNI helper returned no frame dimensions. Output: {}".format(
                    result.stdout.strip()
                )
            )
        _, width_text, height_text = frame_line.split()
        width, height = int(width_text), int(height_text)
        depth = np.fromfile(str(raw_path), dtype="<u2")
        if depth.size != width * height:
            raise RuntimeError(
                "Depth data size mismatch: expected {} pixels, received {}".format(
                    width * height, depth.size
                )
            )
        return depth.reshape((height, width)).copy()
    finally:
        raw_path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(
        description="Capture one RGB and depth frame from an Orbbec Astra Pro Plus."
    )
    parser.add_argument("--rgb-device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")

    try:
        cv2, np = load_dependencies()
        helper = find_depth_helper()
        output_dir = Path(__file__).resolve().parent / "test_img"
        output_dir.mkdir(parents=True, exist_ok=True)

        rgb = capture_rgb(cv2, args.rgb_device, args.width, args.height)
        depth = capture_depth(helper, args.timeout, np, output_dir)

        rgb_path = output_dir / "rgb.png"
        depth_path = output_dir / "depth.png"
        preview_path = output_dir / "depth_color.png"
        if not cv2.imwrite(str(rgb_path), rgb):
            raise RuntimeError("Failed to write {}".format(rgb_path))
        if not cv2.imwrite(str(depth_path), depth):
            raise RuntimeError("Failed to write {}".format(depth_path))

        valid = depth[depth > 0]
        normalized = np.zeros(depth.shape, dtype=np.uint8)
        if valid.size:
            near, far = int(valid.min()), int(valid.max())
            if far > near:
                normalized = np.clip(
                    (depth.astype(np.float32) - near) * (255.0 / (far - near)),
                    0,
                    255,
                ).astype(np.uint8)
            else:
                normalized[depth > 0] = 255
        depth_color = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        depth_color[depth == 0] = 0
        if not cv2.imwrite(str(preview_path), depth_color):
            raise RuntimeError("Failed to write {}".format(preview_path))

        print("RGB image:          {} ({}x{})".format(
            rgb_path, rgb.shape[1], rgb.shape[0]
        ))
        print("Depth image (uint16): {} ({}x{})".format(
            depth_path, depth.shape[1], depth.shape[0]
        ))
        print("Depth preview:      {}".format(preview_path))
        if valid.size:
            print("Valid depth values: {} to {} (OpenNI depth units)".format(
                int(valid.min()), int(valid.max())
            ))
        else:
            print("Warning: depth frame contains no non-zero samples.")
        return 0
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print("Camera capture failed: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
