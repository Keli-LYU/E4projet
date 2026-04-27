#!/usr/bin/env python3
"""
Bridge the AI semantic-segmentation output into the existing Nav2 Yolov7Layer.

Pipeline:
  CARLA RGB image -> /fake_camera/image_raw -> AI_Part/semantic_node.py
  /perception/semantic_mask + CARLA depth -> DetectionBoundsList/DetectionList

The Nav2 plugin only uses DetectionBoundsList for geometry, but it gates its
processing on DetectionList being non-empty. This node publishes one dummy
Detection whenever there is at least one obstacle bound so the plugin processes
the bounds exactly once per frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from perception2d_interfaces.msg import (
    Detection,
    DetectionBounds,
    DetectionBoundsList,
    DetectionList,
)


SUPER_CLASS_COLORS: Dict[int, Tuple[int, int, int]] = {
    0: (0, 0, 0),
    1: (128, 64, 128),
    2: (244, 35, 232),
    3: (157, 234, 50),
    4: (0, 0, 142),
    5: (220, 20, 60),
    6: (250, 170, 30),
    7: (220, 220, 0),
    8: (153, 153, 153),
    9: (70, 70, 70),
    10: (107, 142, 35),
    11: (70, 130, 180),
    12: (110, 190, 160),
}

DEFAULT_LABELS = {
    4: "vehicle",
    5: "pedestrian",
    12: "obstacle",
}


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class LocalPoint:
    forward: float
    y_for_plugin: float


class AiRosBridge(Node):
    def __init__(self) -> None:
        super().__init__("ai_ros_bridge")
        self.bridge = CvBridge()

        self.declare_parameter("carla_rgb_topic", "/carla/ego_vehicle/rgb_cam/image")
        self.declare_parameter("ai_rgb_topic", "/fake_camera/image_raw")
        self.declare_parameter("semantic_mask_topic", "/perception/semantic_mask")
        self.declare_parameter("depth_topic", "/carla/ego_vehicle/depth_cam/image")
        self.declare_parameter("depth_info_topic", "/carla/ego_vehicle/depth_cam/camera_info")
        self.declare_parameter("publish_rgb_to_ai", True)
        self.declare_parameter("min_area_px", 180)
        self.declare_parameter("max_depth_m", 60.0)
        self.declare_parameter("forward_offset_m", 2.0)
        self.declare_parameter("lateral_offset_m", 0.5)
        self.declare_parameter("obstacle_class_ids", [4, 5, 12])
        self.declare_parameter("publish_debug_overlay", True)

        self.carla_rgb_topic = self.get_parameter("carla_rgb_topic").value
        self.ai_rgb_topic = self.get_parameter("ai_rgb_topic").value
        self.semantic_mask_topic = self.get_parameter("semantic_mask_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.depth_info_topic = self.get_parameter("depth_info_topic").value
        self.publish_rgb_to_ai = bool(self.get_parameter("publish_rgb_to_ai").value)
        self.min_area_px = int(self.get_parameter("min_area_px").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.forward_offset_m = float(self.get_parameter("forward_offset_m").value)
        self.lateral_offset_m = float(self.get_parameter("lateral_offset_m").value)
        self.obstacle_class_ids = [int(v) for v in self.get_parameter("obstacle_class_ids").value]
        self.publish_debug_overlay = bool(self.get_parameter("publish_debug_overlay").value)

        self.latest_depth: Optional[np.ndarray] = None
        self.camera_model: Optional[CameraModel] = None

        self.bounds_pub = self.create_publisher(DetectionBoundsList, "/Detection_bounds_list", 10)
        self.coords_pub = self.create_publisher(DetectionList, "/Detection_coordinates_list", 10)
        self.overlay_pub = self.create_publisher(Image, "/ai_ros/debug_detections", 10)

        if self.publish_rgb_to_ai:
            self.rgb_pub = self.create_publisher(Image, self.ai_rgb_topic, 10)
            self.rgb_sub = self.create_subscription(
                Image, self.carla_rgb_topic, self.republish_rgb, 10
            )

        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, self.depth_info_topic, self.camera_info_callback, 10
        )
        self.mask_sub = self.create_subscription(
            Image, self.semantic_mask_topic, self.mask_callback, 10
        )

        self.get_logger().info(
            "ai-ros bridge ready: RGB %s -> %s, mask %s + depth %s -> Nav2 detections"
            % (
                self.carla_rgb_topic,
                self.ai_rgb_topic,
                self.semantic_mask_topic,
                self.depth_topic,
            )
        )

    def republish_rgb(self, msg: Image) -> None:
        self.rgb_pub.publish(msg)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return
        self.camera_model = CameraModel(
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
        )

    def depth_callback(self, msg: Image) -> None:
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = self.decode_depth(raw, msg.encoding)
        except Exception as exc:
            self.get_logger().warn(f"Failed to decode depth image: {exc}")

    def mask_callback(self, msg: Image) -> None:
        if self.latest_depth is None or self.camera_model is None:
            self.publish_empty(msg)
            return

        try:
            rgb_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to decode semantic mask: {exc}")
            self.publish_empty(msg)
            return

        if rgb_mask.shape[:2] != self.latest_depth.shape[:2]:
            depth = cv2.resize(
                self.latest_depth,
                (rgb_mask.shape[1], rgb_mask.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        else:
            depth = self.latest_depth

        bounds = self.extract_bounds(rgb_mask, depth)
        self.publish_detections(msg, bounds)

        if self.publish_debug_overlay:
            self.publish_overlay(msg, rgb_mask, bounds)

    def publish_empty(self, source_msg: Image) -> None:
        bounds_msg = DetectionBoundsList()
        bounds_msg.header = source_msg.header
        coords_msg = DetectionList()
        coords_msg.header = source_msg.header
        self.bounds_pub.publish(bounds_msg)
        self.coords_pub.publish(coords_msg)

    def publish_detections(self, source_msg: Image, bounds: List[DetectionBounds]) -> None:
        bounds_msg = DetectionBoundsList()
        bounds_msg.header = source_msg.header
        bounds_msg.detection_bounds_list = bounds
        self.bounds_pub.publish(bounds_msg)

        coords_msg = DetectionList()
        coords_msg.header = source_msg.header
        if bounds:
            dummy = Detection()
            dummy.x = bounds[0].x_min
            dummy.y = bounds[0].y_min
            coords_msg.detection_list = [dummy]
        self.coords_pub.publish(coords_msg)

    def publish_overlay(
        self, source_msg: Image, rgb_mask: np.ndarray, bounds: List[DetectionBounds]
    ) -> None:
        overlay = rgb_mask.copy()
        label_lines = []
        for bound in bounds:
            label_lines.append(
                f"{bound.label}: x=({bound.x_min:.1f},{bound.x_max:.1f}) "
                f"y=({bound.y_min:.1f},{bound.y_max:.1f})"
            )
        for i, line in enumerate(label_lines[:12]):
            cv2.putText(
                overlay,
                line,
                (10, 24 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        debug_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="rgb8")
        debug_msg.header = source_msg.header
        self.overlay_pub.publish(debug_msg)

    def extract_bounds(self, rgb_mask: np.ndarray, depth: np.ndarray) -> List[DetectionBounds]:
        detections: List[DetectionBounds] = []
        class_map = self.color_mask_to_class_ids(rgb_mask)

        for class_id in self.obstacle_class_ids:
            binary = (class_map == class_id).astype(np.uint8)
            if not np.any(binary):
                continue

            count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
            for component_id in range(1, count):
                area = int(stats[component_id, cv2.CC_STAT_AREA])
                if area < self.min_area_px:
                    continue

                x = int(stats[component_id, cv2.CC_STAT_LEFT])
                y = int(stats[component_id, cv2.CC_STAT_TOP])
                w = int(stats[component_id, cv2.CC_STAT_WIDTH])
                h = int(stats[component_id, cv2.CC_STAT_HEIGHT])
                component = labels == component_id

                bound = self.component_to_bound(component, depth, x, y, w, h, class_id)
                if bound is not None:
                    detections.append(bound)

        return detections

    def component_to_bound(
        self,
        component: np.ndarray,
        depth: np.ndarray,
        x: int,
        y: int,
        w: int,
        h: int,
        class_id: int,
    ) -> Optional[DetectionBounds]:
        row = min(depth.shape[0] - 1, y + max(0, int(h * 0.65)))
        rows = component[max(0, row - 2) : min(component.shape[0], row + 3), x : x + w]
        ys, xs = np.where(rows)
        if xs.size == 0:
            rows = component[y : y + h, x : x + w]
            ys, xs = np.where(rows)
        if xs.size == 0:
            return None

        left_u = x + int(np.min(xs))
        right_u = x + int(np.max(xs))
        sample_v = row if y <= row < y + h else y + h // 2

        left_depth = self.sample_depth(depth, left_u, sample_v, component)
        right_depth = self.sample_depth(depth, right_u, sample_v, component)
        if left_depth is None or right_depth is None:
            return None

        left = self.pixel_to_plugin_point(left_u, sample_v, left_depth)
        right = self.pixel_to_plugin_point(right_u, sample_v, right_depth)
        if left is None or right is None:
            return None

        nearest = self.nearest_component_point(component, depth, x, y, w, h)

        msg = DetectionBounds()
        msg.x_min = left.forward
        msg.y_min = left.y_for_plugin
        msg.x_max = right.forward
        msg.y_max = right.y_for_plugin
        if nearest is not None:
            msg.xx_min = nearest.forward
            msg.yy_min = nearest.y_for_plugin
        else:
            msg.xx_min = 0.0
            msg.yy_min = 0.0
        msg.label = DEFAULT_LABELS.get(class_id, f"class_{class_id}")
        return msg

    def nearest_component_point(
        self, component: np.ndarray, depth: np.ndarray, x: int, y: int, w: int, h: int
    ) -> Optional[LocalPoint]:
        crop_mask = component[y : y + h, x : x + w]
        crop_depth = depth[y : y + h, x : x + w]
        valid = crop_mask & np.isfinite(crop_depth) & (crop_depth > 0.1) & (
            crop_depth < self.max_depth_m
        )
        if not np.any(valid):
            return None

        valid_depths = np.where(valid, crop_depth, np.inf)
        local_index = int(np.argmin(valid_depths))
        local_v, local_u = np.unravel_index(local_index, valid_depths.shape)
        nearest_depth = float(valid_depths[local_v, local_u])
        return self.pixel_to_plugin_point(x + int(local_u), y + int(local_v), nearest_depth)

    def sample_depth(
        self, depth: np.ndarray, u: int, v: int, component: np.ndarray, radius: int = 4
    ) -> Optional[float]:
        y0 = max(0, v - radius)
        y1 = min(depth.shape[0], v + radius + 1)
        x0 = max(0, u - radius)
        x1 = min(depth.shape[1], u + radius + 1)
        depth_crop = depth[y0:y1, x0:x1]
        mask_crop = component[y0:y1, x0:x1]
        values = depth_crop[
            mask_crop
            & np.isfinite(depth_crop)
            & (depth_crop > 0.1)
            & (depth_crop < self.max_depth_m)
        ]
        if values.size == 0:
            return None
        return float(np.median(values))

    def pixel_to_plugin_point(self, u: int, v: int, z: float) -> Optional[LocalPoint]:
        camera = self.camera_model
        if camera is None or not math.isfinite(z) or z <= 0.1:
            return None
        x_cam = ((float(u) - camera.cx) * z) / camera.fx

        forward = z + self.forward_offset_m
        y_for_plugin = x_cam - self.lateral_offset_m
        return LocalPoint(forward=forward, y_for_plugin=y_for_plugin)

    def color_mask_to_class_ids(self, rgb_mask: np.ndarray) -> np.ndarray:
        class_map = np.zeros(rgb_mask.shape[:2], dtype=np.uint8)
        for class_id, color in SUPER_CLASS_COLORS.items():
            class_map[np.all(rgb_mask == np.array(color, dtype=np.uint8), axis=2)] = class_id
        return class_map

    def decode_depth(self, raw: np.ndarray, encoding: str) -> np.ndarray:
        depth = np.asarray(raw)
        if depth.ndim == 2:
            return depth.astype(np.float32)

        encoding = (encoding or "").lower()
        channels = depth.astype(np.float32)
        if channels.shape[2] < 3:
            return channels[:, :, 0].astype(np.float32)

        if "bgra" in encoding or "bgr8" in encoding:
            blue = channels[:, :, 0]
            green = channels[:, :, 1]
            red = channels[:, :, 2]
        else:
            red = channels[:, :, 0]
            green = channels[:, :, 1]
            blue = channels[:, :, 2]

        normalized = (red + green * 256.0 + blue * 65536.0) / (256.0**3 - 1.0)
        return (normalized * 1000.0).astype(np.float32)


def main(args: Optional[Iterable[str]] = None) -> None:
    rclpy.init(args=args)
    node = AiRosBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
