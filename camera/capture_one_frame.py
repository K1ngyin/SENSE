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
        fields = frame_line.split()
        if len(fields) != 5:
            raise RuntimeError(
                "Unexpected OpenNI helper metadata: {}".format(frame_line)
            )
        _, width_text, height_text, units, aligned_text = fields
        width, height = int(width_text), int(height_text)
        aligned = aligned_text == "1"
        depth = np.fromfile(str(raw_path), dtype="<u2")
        if depth.size != width * height:
            raise RuntimeError(
                "Depth data size mismatch: expected {} pixels, received {}".format(
                    width * height, depth.size
                )
            )
        return depth.reshape((height, width)).copy(), units, aligned
    finally:
        raw_path.unlink(missing_ok=True)


def colorize_depth_with_bar(cv2, np, depth, units):
    valid = depth[depth > 0]
    normalized = np.zeros(depth.shape, dtype=np.uint8)
    if valid.size:
        # Percentiles keep a few extreme samples from flattening the preview.
        near = float(np.percentile(valid, 2))
        far = float(np.percentile(valid, 98))
        if far <= near:
            far = near + 1.0
        normalized = np.clip(
            (depth.astype(np.float32) - near) * (255.0 / (far - near)),
            0,
            255,
        ).astype(np.uint8)
    else:
        near, far = 0.0, 1.0

    color = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    color[depth == 0] = 0

    height, width = depth.shape
    bar_top = 36
    bar_bottom = max(bar_top + 1, height - 18)
    bar_x = width + 12
    bar_width = 24
    canvas = np.zeros((height, width + 150, 3), dtype=np.uint8)
    canvas[:, :width] = color

    for y in range(bar_top, bar_bottom + 1):
        fraction = 1.0 - (y - bar_top) / float(bar_bottom - bar_top)
        index = int(round(255.0 * fraction))
        sample = np.array([[index]], dtype=np.uint8)
        bgr = cv2.applyColorMap(sample, cv2.COLORMAP_JET)[0, 0]
        canvas[y, bar_x : bar_x + bar_width] = bgr

    if units == "MM":
        scale_to_meters = 0.001
        unit_label = "Depth (m)"
        format_value = lambda value: "{:.2f}".format(value * scale_to_meters)
    elif units == "100UM":
        scale_to_meters = 0.0001
        unit_label = "Depth (m)"
        format_value = lambda value: "{:.2f}".format(value * scale_to_meters)
    else:
        unit_label = "Depth raw"
        format_value = lambda value: "{:.0f}".format(value)

    cv2.putText(
        canvas, unit_label, (bar_x - 3, 22), cv2.FONT_HERSHEY_SIMPLEX,
        0.42, (255, 255, 255), 1, cv2.LINE_AA
    )
    for tick in range(5):
        fraction = tick / 4.0
        y = int(round(bar_top + fraction * (bar_bottom - bar_top)))
        value = far - fraction * (far - near)
        cv2.line(canvas, (bar_x + bar_width, y), (bar_x + bar_width + 5, y),
                 (255, 255, 255), 1)
        cv2.putText(
            canvas, format_value(value), (bar_x + bar_width + 9, y + 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA
        )
    cv2.rectangle(canvas, (bar_x, bar_top), (bar_x + bar_width, bar_bottom),
                  (230, 230, 230), 1)
    return canvas, valid


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
        depth, units, aligned = capture_depth(helper, args.timeout, np, output_dir)

        rgb_path = output_dir / "rgb.png"
        depth_path = output_dir / "depth.png"
        preview_path = output_dir / "depth_color.png"
        if not cv2.imwrite(str(rgb_path), rgb):
            raise RuntimeError("Failed to write {}".format(rgb_path))
        if not cv2.imwrite(str(depth_path), depth):
            raise RuntimeError("Failed to write {}".format(depth_path))

        depth_color, valid = colorize_depth_with_bar(cv2, np, depth, units)
        if not cv2.imwrite(str(preview_path), depth_color):
            raise RuntimeError("Failed to write {}".format(preview_path))

        print("RGB image:          {} ({}x{})".format(
            rgb_path, rgb.shape[1], rgb.shape[0]
        ))
        print("Depth image (uint16): {} ({}x{})".format(
            depth_path, depth.shape[1], depth.shape[0]
        ))
        print("Depth preview:      {}".format(preview_path))
        if aligned:
            if depth.shape[:2] == rgb.shape[:2]:
                print("Depth registration: depth-to-color enabled")
            else:
                print(
                    "Warning: depth-to-color registration is enabled, but frame "
                    "sizes differ (RGB {}x{}, depth {}x{}).".format(
                        rgb.shape[1], rgb.shape[0], depth.shape[1], depth.shape[0]
                    )
                )
        else:
            print(
                "Warning: OpenNI reports depth-to-color registration unsupported; "
                "depth remains in native coordinates."
            )
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
