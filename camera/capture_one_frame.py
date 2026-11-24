#!/usr/bin/env python3
"""Capture one RGB and depth frame from an Orbbec camera.

Requires the Orbbec pyorbbecsdk Python module, NumPy, and OpenCV on the
computer board. Output is written to camera/test_img next to this script.
"""

import argparse
import sys
import time
from pathlib import Path


def load_dependencies():
    try:
        import cv2
        import numpy as np
        from pyorbbecsdk import Config, OBSensorType, Pipeline
    except ImportError as exc:
        raise RuntimeError(
            "Missing camera dependencies. Install the Orbbec pyorbbecsdk, "
            "NumPy, and OpenCV for the Python environment used on the board. "
            "See https://github.com/orbbec/pyorbbecsdk for SDK setup."
        ) from exc
    return cv2, np, Config, OBSensorType, Pipeline


def color_frame_to_bgr(frame, cv2, np):
    width = frame.get_width()
    height = frame.get_height()
    fmt = str(frame.get_format()).upper().split(".")[-1]
    data = np.frombuffer(frame.get_data(), dtype=np.uint8)

    if fmt in ("MJPG", "MJPEG"):
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("Could not decode the camera's MJPEG color frame")
        return image

    if fmt in ("RGB", "RGB888"):
        return cv2.cvtColor(data.reshape(height, width, 3), cv2.COLOR_RGB2BGR)
    if fmt in ("BGR", "BGR888"):
        return data.reshape(height, width, 3).copy()
    if fmt in ("RGBA", "RGBA8888"):
        return cv2.cvtColor(data.reshape(height, width, 4), cv2.COLOR_RGBA2BGR)
    if fmt in ("BGRA", "BGRA8888"):
        return cv2.cvtColor(data.reshape(height, width, 4), cv2.COLOR_BGRA2BGR)
    if fmt in ("YUYV", "YUY2"):
        return cv2.cvtColor(
            data.reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUY2
        )
    if fmt == "UYVY":
        return cv2.cvtColor(
            data.reshape(height, width, 2), cv2.COLOR_YUV2BGR_UYVY
        )

    raise RuntimeError(
        "Unsupported color frame format: {}. Check the camera profile "
        "selected by the Orbbec SDK.".format(fmt)
    )


def main():
    parser = argparse.ArgumentParser(
        description="Capture one RGB image and one depth map from an Orbbec camera."
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="seconds to wait for both RGB and depth frames (default: 30)",
    )
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")

    try:
        cv2, np, Config, OBSensorType, Pipeline = load_dependencies()
    except RuntimeError as exc:
        print("Error: {}".format(exc), file=sys.stderr)
        return 1

    output_dir = Path(__file__).resolve().parent / "test_img"
    pipeline = None
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        pipeline = Pipeline()
        config = Config()
        try:
            color_profiles = pipeline.get_stream_profile_list(
                OBSensorType.COLOR_SENSOR
            )
            color_profile = color_profiles.get_default_video_stream_profile()
            config.enable_stream(color_profile)
        except Exception as exc:
            raise RuntimeError(
                "Could not configure the camera's RGB stream. Confirm that the "
                "Orbbec camera is connected and supports a color stream. {}".format(exc)
            ) from exc

        try:
            depth_profiles = pipeline.get_stream_profile_list(
                OBSensorType.DEPTH_SENSOR
            )
            depth_profile = depth_profiles.get_default_video_stream_profile()
            config.enable_stream(depth_profile)
        except Exception as exc:
            raise RuntimeError(
                "Could not configure the camera's depth stream. Confirm that the "
                "Orbbec depth sensor is connected. {}".format(exc)
            ) from exc

        pipeline.start(config)
        deadline = time.monotonic() + args.timeout
        color_frame = None
        depth_frame = None
        while time.monotonic() < deadline:
            frames = pipeline.wait_for_frames(100)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is not None and depth_frame is not None:
                break

        if color_frame is None or depth_frame is None:
            raise RuntimeError(
                "Timed out waiting for both RGB and depth frames after {:.1f} s. "
                "Check the USB connection, camera permissions, and stream support."
                .format(args.timeout)
            )

        color_image = color_frame_to_bgr(color_frame, cv2, np)
        depth_width = depth_frame.get_width()
        depth_height = depth_frame.get_height()
        depth = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
            depth_height, depth_width
        ).copy()

        rgb_path = output_dir / "rgb.png"
        depth_path = output_dir / "depth.png"
        depth_preview_path = output_dir / "depth_color.png"
        if not cv2.imwrite(str(rgb_path), color_image):
            raise RuntimeError("Failed to write {}".format(rgb_path))
        if not cv2.imwrite(str(depth_path), depth):
            raise RuntimeError("Failed to write {}".format(depth_path))

        valid_depth = depth[depth > 0]
        preview_input = np.zeros(depth.shape, dtype=np.uint8)
        if valid_depth.size:
            near = float(valid_depth.min())
            far = float(valid_depth.max())
            if far > near:
                preview_input = np.clip(
                    (depth.astype(np.float32) - near) * (255.0 / (far - near)),
                    0,
                    255,
                ).astype(np.uint8)
            else:
                preview_input[depth > 0] = 255
        depth_preview = cv2.applyColorMap(preview_input, cv2.COLORMAP_JET)
        depth_preview[depth == 0] = 0
        if not cv2.imwrite(str(depth_preview_path), depth_preview):
            raise RuntimeError("Failed to write {}".format(depth_preview_path))

        try:
            depth_scale = depth_frame.get_depth_scale()
        except (AttributeError, TypeError):
            depth_scale = None

        print("Captured one frame pair:")
        print("  RGB:          {} ({}x{})".format(
            rgb_path, color_image.shape[1], color_image.shape[0]
        ))
        print("  Depth raw:    {} ({}x{}, uint16)".format(
            depth_path, depth_width, depth_height
        ))
        print("  Depth preview: {}".format(depth_preview_path))
        if depth_scale is not None:
            print("  Depth scale:  {} mm per stored unit".format(depth_scale))
        if valid_depth.size:
            print("  Valid samples: {} to {}".format(
                int(valid_depth.min()), int(valid_depth.max())
            ))
        else:
            print("  Warning: the depth frame contains no non-zero samples.")
        return 0
    except Exception as exc:
        print("Camera capture failed: {}".format(exc), file=sys.stderr)
        return 1
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
