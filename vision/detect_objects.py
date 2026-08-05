#!/usr/bin/env python3
"""P1: RGB-D object detection for grasping.

Runs inside the rk3588s-vision container. Detects colored objects
(sphere / cube / cuboid), estimates their camera-frame 3D position and
physical size, and publishes the result to /student/grasp/objects.

Usage (inside container, after sourcing ROS env):
    python3 /data/vision/detect_objects.py --config /data/vision/color_ranges.json
    python3 /data/vision/detect_objects.py --config ... --once --output-dir /tmp
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from std_msgs.msg import String

DRAW_COLORS = {
    "red": (0, 0, 255),
    "orange": (0, 165, 255),
    "yellow": (0, 255, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "purple": (128, 0, 128),
    "white": (255, 255, 255),
    "black": (0, 0, 0),
}


def image_rgb8(message: Image) -> np.ndarray:
    if message.encoding.lower() not in {"rgb8", "bgr8"}:
        raise RuntimeError(f"unsupported color encoding: {message.encoding}")
    row_width = int(message.step)
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, row_width)
    image = raw[:, : message.width * 3].reshape(message.height, message.width, 3)
    if message.encoding.lower() == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def image_depth16(message: Image) -> np.ndarray:
    if message.encoding.upper() not in {"16UC1", "MONO16"}:
        raise RuntimeError(f"unsupported depth encoding: {message.encoding}")
    byte_order = ">u2" if message.is_bigendian else "<u2"
    values_per_row = int(message.step) // 2
    raw = np.frombuffer(message.data, dtype=np.dtype(byte_order)).reshape(
        message.height, values_per_row
    )
    return raw[:, : message.width].astype(np.uint16, copy=False)


def largest_contour(mask: np.ndarray, min_area: float):
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours:
        return None, 0.0
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    return (contour, area) if area >= min_area else (None, area)


def contour_depth_median(depth: np.ndarray, contour: np.ndarray) -> tuple[float, float]:
    mask = np.zeros(depth.shape, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    total = int(np.count_nonzero(mask))
    valid = depth[(mask > 0) & (depth > 0)]
    if valid.size < 8:
        return None, float(valid.size / total) if total else 0.0
    return float(np.median(valid)), float(valid.size / total)


def classify_shape(
    depth: np.ndarray,
    contour: np.ndarray,
    bbox: tuple[int, int, int, int],
    phys_w: float,
    phys_h: float,
    flat_std_thresh: float,
) -> tuple[str, float]:
    """Classify sphere / cube / cuboid.

    A flat-topped object (box) has near-constant depth along its centre line;
    a sphere curves away, giving a larger depth spread.  The flat/cube-vs-
    cuboid split then uses the physical (back-projected) width/height ratio.
    Returns (shape, surface_std_mm).
    """
    x, y, w, h = bbox
    cy, cx = y + h // 2, x + w // 2
    row = depth[max(0, cy - 2): cy + 3, x: x + w]
    col = depth[y: y + h, max(0, cx - 2): cx + 3]
    row_flat = row[row > 0].astype(np.float64)
    col_flat = col[col > 0].astype(np.float64)
    x_std = float(np.std(row_flat)) if row_flat.size >= 8 else float("nan")
    y_std = float(np.std(col_flat)) if col_flat.size >= 8 else float("nan")
    surface_std = min((s for s in (x_std, y_std) if s == s), default=float("nan"))

    if not (surface_std == surface_std):  # not enough valid depth
        return "unknown", surface_std
    if surface_std > flat_std_thresh:
        return "sphere", surface_std
    if phys_w <= 0 or phys_h <= 0:
        return "cuboid", surface_std
    aspect = phys_w / phys_h
    if 0.72 <= aspect <= 1.38:
        return "cube", surface_std
    return "cuboid", surface_std


def detect(
    color: np.ndarray,
    depth: np.ndarray,
    camera_info: CameraInfo,
    config: dict[str, Any],
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    if color.shape[:2] != depth.shape[:2]:
        raise RuntimeError(f"RGB/depth size mismatch: {color.shape[:2]} vs {depth.shape[:2]}")
    hsv = cv2.cvtColor(cv2.GaussianBlur(color, (5, 5), 0), cv2.COLOR_BGR2HSV)
    kernel_size = max(3, int(config.get("morph_kernel_px", 5)))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    min_area = float(config.get("min_area_px", 350))
    flat_std_thresh = float(config.get("flat_depth_std_mm", 1.8))
    min_depth_mm = float(config.get("min_depth_mm", 250))
    max_depth_mm = float(config.get("max_depth_mm", 1000))
    min_depth_ratio = float(config.get("min_depth_valid_ratio", 0.3))
    fx, fy = float(camera_info.K[0]), float(camera_info.K[4])
    cx, cy = float(camera_info.K[2]), float(camera_info.K[5])
    annotated = color.copy()
    detections: list[dict[str, Any]] = []

    for name, ranges in config["colors"].items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for hsv_range in ranges:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(
                    hsv,
                    np.array(hsv_range["min"], dtype=np.uint8),
                    np.array(hsv_range["max"], dtype=np.uint8),
                ),
            )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contour, area = largest_contour(mask, min_area)
        if contour is None:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue
        u = int(round(moments["m10"] / moments["m00"]))
        v = int(round(moments["m01"] / moments["m00"]))
        x, y, w, h = cv2.boundingRect(contour)
        perimeter = float(cv2.arcLength(contour, True))
        circularity = float(4.0 * np.pi * area / (perimeter * perimeter)) if perimeter else 0.0

        depth_mm, valid_ratio = contour_depth_median(depth, contour)
        camera_point_m = None
        size_m = None
        shape = "unknown"
        surface_std = None
        in_range = (
            depth_mm is not None
            and min_depth_mm <= depth_mm <= max_depth_mm
            and valid_ratio >= min_depth_ratio
        )
        if depth_mm is not None and fx > 0 and fy > 0:
            z_m = depth_mm / 1000.0
            camera_point_m = [(u - cx) * z_m / fx, (v - cy) * z_m / fy, z_m]
            phys_w = w * z_m / fx
            phys_h = h * z_m / fy
            size_m = [round(phys_w, 4), round(phys_h, 4)]
            if in_range:
                shape, surface_std = classify_shape(
                    depth, contour, (x, y, w, h), phys_w, phys_h, flat_std_thresh
                )

        detections.append(
            {
                "color": name,
                "shape": shape,
                "center_px": [u, v],
                "bbox_px": [x, y, w, h],
                "area_px": area,
                "circularity": round(circularity, 3),
                "surface_depth_std_mm": surface_std,
                "center_depth_mm": depth_mm,
                "depth_valid_ratio": round(valid_ratio, 3),
                "size_m": size_m,
                "camera_point_m": camera_point_m,
                "graspable": in_range,
                "frame_id": camera_info.header.frame_id,
            }
        )
        draw = DRAW_COLORS.get(name, (0, 255, 255))
        cv2.drawContours(annotated, [contour], -1, draw, 2)
        cv2.rectangle(annotated, (x, y), (x + w, y + h), draw, 2)
        label = f"{name}/{shape} {depth_mm:.0f}mm" if depth_mm is not None else name
        cv2.putText(
            annotated,
            label,
            (max(0, x + 2), min(annotated.shape[0] - 5, y + 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            draw,
            2,
            cv2.LINE_AA,
        )
        cv2.circle(annotated, (u, v), 4, draw, -1)
    return annotated, detections


def build_result(detections: list[dict[str, Any]], camera_info: CameraInfo) -> str:
    return json.dumps(
        {
            "stamp_unix_s": time.time(),
            "frame_id": camera_info.header.frame_id,
            "count": len(detections),
            "objects": detections,
        },
        ensure_ascii=False,
    )


class DetectNode:
    def __init__(self, config: dict[str, Any]):
        self.config = config
        self.pub_objects = rospy.Publisher(
            "/student/grasp/objects", String, queue_size=1, latch=True
        )
        self.pub_image = rospy.Publisher(
            "/student/grasp/image", CompressedImage, queue_size=1
        )
        self.last_publish = 0.0

    def callback(
        self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo
    ) -> None:
        color = image_rgb8(rgb_msg)
        depth = image_depth16(depth_msg)
        annotated, detections = detect(color, depth, info_msg, self.config)
        result = build_result(detections, info_msg)
        self.pub_objects.publish(result)
        ok, encoded = cv2.imencode(".jpg", annotated)
        if ok:
            comp = CompressedImage()
            comp.header.stamp = rospy.Time.now()
            comp.format = "jpeg"
            comp.data = encoded.tobytes()
            self.pub_image.publish(comp)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--output-dir", default="/tmp")
    parser.add_argument("--color-topic", default="/astra_camera/rgb/image_raw")
    parser.add_argument("--depth-topic", default="/astra_camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/astra_camera/depth/camera_info")
    parser.add_argument("--rate", type=float, default=5.0)
    args = parser.parse_args()

    config = json.loads(Path(args.config).read_text(encoding="utf-8"))
    rospy.init_node("student_grasp_detect", anonymous=True)

    if args.once:
        color_msg = rospy.wait_for_message(args.color_topic, Image, timeout=20)
        depth_msg = rospy.wait_for_message(args.depth_topic, Image, timeout=20)
        info_msg = rospy.wait_for_message(args.camera_info_topic, CameraInfo, timeout=20)
        color = image_rgb8(color_msg)
        depth = image_depth16(depth_msg)
        annotated, detections = detect(color, depth, info_msg, config)
        out = Path(args.output_dir)
        out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out / "latest-grasp-detect.jpg"), annotated)
        print(build_result(detections, info_msg))
        return

    node = DetectNode(config)
    rgb_sub = message_filters.Subscriber(args.color_topic, Image, queue_size=1)
    depth_sub = message_filters.Subscriber(args.depth_topic, Image, queue_size=1)
    info_sub = message_filters.Subscriber(args.camera_info_topic, CameraInfo, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer(
        [rgb_sub, depth_sub, info_sub], 10, 0.05
    )
    ts.registerCallback(node.callback)
    rospy.spin()


if __name__ == "__main__":
    main()
