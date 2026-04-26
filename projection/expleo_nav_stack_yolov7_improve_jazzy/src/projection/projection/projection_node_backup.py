#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge


class ProjectionNode(Node):
    def __init__(self):
        super().__init__('projection_node')

        # =========================
        # Parameters
        # =========================
        self.semantic_topic = '/carla/ego_vehicle/semantic_cam/image'
        self.depth_topic = '/carla/ego_vehicle/depth_cam/image'
        self.depth_info_topic = '/carla/ego_vehicle/depth_cam/camera_info'

        self.road_topic = '/semantic/road'
        self.pedestrian_topic = '/semantic/pedestrian'
        self.vehicle_topic = '/semantic/vehicle'
        self.static_topic = '/semantic/static_obstacle'

        self.max_depth = 50.0
        self.sample_step = 4
        self.process_period = 0.5

        # =========================
        # TODO: camera extrinsics
        # Replace with real depth camera -> ego_vehicle transform
        # =========================
        self.cam_offset_x = 0.0
        self.cam_offset_y = 0.0
        self.cam_offset_z = 0.0

        self.bridge = CvBridge()

        # Subscribers
        self.semantic_sub = self.create_subscription(
            Image,
            self.semantic_topic,
            self.semantic_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            self.depth_info_topic,
            self.depth_info_callback,
            10
        )

        # Publishers
        self.road_pub = self.create_publisher(PointCloud2, self.road_topic, 10)
        self.pedestrian_pub = self.create_publisher(PointCloud2, self.pedestrian_topic, 10)
        self.vehicle_pub = self.create_publisher(PointCloud2, self.vehicle_topic, 10)
        self.static_pub = self.create_publisher(PointCloud2, self.static_topic, 10)

        self.timer = self.create_timer(self.process_period, self.process_data)

        # Storage
        self.latest_semantic_msg = None
        self.latest_depth_msg = None
        self.latest_depth_info = None

        self.semantic_logged = False
        self.depth_logged = False
        self.depth_info_logged = False
        self.semantic_debug_logged = False

        self.get_logger().info('Projection node started.')

    # ==================================================
    # Callbacks
    # ==================================================
    def semantic_callback(self, msg: Image):
        self.latest_semantic_msg = msg
        if not self.semantic_logged:
            self.get_logger().info(
                f'Semantic image received: {msg.width}x{msg.height}, encoding={msg.encoding}'
            )
            self.semantic_logged = True

    def depth_callback(self, msg: Image):
        self.latest_depth_msg = msg
        if not self.depth_logged:
            self.get_logger().info(
                f'Depth image received: {msg.width}x{msg.height}, encoding={msg.encoding}'
            )
            self.depth_logged = True

    def depth_info_callback(self, msg: CameraInfo):
        self.latest_depth_info = msg
        if not self.depth_info_logged:
            self.get_logger().info(
                f'Depth camera info received: width={msg.width}, height={msg.height}, k={msg.k}'
            )
            self.depth_info_logged = True



    # ==================================================
    # Debug
    # ==================================================
    def debug_region_semantic_values(self, semantic_img, u0, v0, u1, v1):
        import numpy as np

        region = semantic_img[v0:v1, u0:u1].reshape(-1, 4)
        unique_pixels, counts = np.unique(region, axis=0, return_counts=True)
        sorted_indices = np.argsort(counts)[::-1]

        self.get_logger().info(f'Debug region BGRA values: u=({u0},{u1}), v=({v0},{v1})')

        for i in range(min(10, len(sorted_indices))):
            idx = sorted_indices[i]
            pixel = unique_pixels[idx]
            count = counts[idx]
            self.get_logger().info(
                f'BGRA={pixel.tolist()} count={int(count)}'
            )


    
    # ==================================================
    # Main processing
    # ==================================================
    def process_data(self):
        if (
            self.latest_semantic_msg is None
            or self.latest_depth_msg is None
            or self.latest_depth_info is None
        ):
            return

        try:
            semantic_img = self.bridge.imgmsg_to_cv2(
                self.latest_semantic_msg,
                desired_encoding='bgra8'
            )
            depth_img = self.bridge.imgmsg_to_cv2(
                self.latest_depth_msg,
                desired_encoding='32FC1'
            )
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        height, width = depth_img.shape[:2]

        k = self.latest_depth_info.k
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]

        if fx == 0.0 or fy == 0.0:
            self.get_logger().warn('Invalid camera intrinsics.')
            return

        # =========================
        # TODO: semantic inspection
        # Run once to understand the real BGRA values in the segmentation image
        # =========================
        if not self.semantic_debug_logged:
            self.debug_semantic_values(semantic_img)

            # TODO: manually set a region around the fire truck / bicycle
            self.debug_region_semantic_values(semantic_img, 500, 120, 800, 260)

            self.semantic_debug_logged = True

        road_points = []
        pedestrian_points = []
        vehicle_points = []
        static_points = []

        for v in range(0, height, self.sample_step):
            for u in range(0, width, self.sample_step):
                depth_value = float(depth_img[v, u])

                if not math.isfinite(depth_value):
                    continue
                if depth_value <= 0.0 or depth_value > self.max_depth:
                    continue

                seg_pixel = semantic_img[v, u]
                seg_class = self.classify_semantic(seg_pixel)

                if seg_class == 'ignore':
                    continue

                # Projection in camera frame
                x_cam = (u - cx) * depth_value / fx
                y_cam = (v - cy) * depth_value / fy
                z_cam = depth_value

                # Transform to ego_vehicle frame
                x_ego, y_ego, z_ego = self.camera_to_ego(x_cam, y_cam, z_cam)

                if seg_class == 'road':
                    road_points.append((x_ego, y_ego, z_ego))
                elif seg_class == 'pedestrian':
                    pedestrian_points.append((x_ego, y_ego, z_ego))
                elif seg_class == 'vehicle':
                    vehicle_points.append((x_ego, y_ego, z_ego))
                elif seg_class == 'static':
                    static_points.append((x_ego, y_ego, z_ego))

        # Publish per-class pointclouds
        self.publish_cloud(self.road_pub, road_points)
        self.publish_cloud(self.pedestrian_pub, pedestrian_points)
        self.publish_cloud(self.vehicle_pub, vehicle_points)
        self.publish_cloud(self.static_pub, static_points)

        self.get_logger().info(
            f'Published clouds | road={len(road_points)}, pedestrian={len(pedestrian_points)}, '
            f'vehicle={len(vehicle_points)}, static={len(static_points)}'
        )

    # ==================================================
    # Semantic classification
    # ==================================================
    def classify_semantic(self, seg_pixel):
        b, g, r, a = [int(x) for x in seg_pixel]

        # ignore
        if (b, g, r) in [
            (0, 0, 0),        # background / unknown
            (180, 130, 70),   # sky
        ]:
            return 'ignore'

        # road
        if (b, g, r) in [
            (128, 64, 128),   # road
            (50, 234, 157),   # road line
        ]:
            return 'road'

        # dynamic -> regroup into vehicle
        if (b, g, r) in [
            (142, 0, 0),    # car
            (70, 0, 0),     # truck / large vehicle / likely fire truck
            (0, 0, 255),    # rider
            (230, 0, 0),    # motorcycle
            (60, 20, 220),  # pedestrian
            (32, 11, 119),  # bike
        ]:
            return 'vehicle'
            return 'vehicle'

        # everything else -> static obstacle
        return 'static'

    def debug_semantic_values(self, semantic_img):
        """
        Print the most frequent semantic BGRA values in the current frame.
        Only for debugging / understanding the segmentation encoding.
        """
        import numpy as np

        pixels = semantic_img.reshape(-1, 4)
        unique_pixels, counts = np.unique(pixels, axis=0, return_counts=True)
        sorted_indices = np.argsort(counts)[::-1]

        self.get_logger().info('Top semantic BGRA values in current frame:')

        top_k = min(15, len(sorted_indices))
        for i in range(top_k):
            idx = sorted_indices[i]
            pixel = unique_pixels[idx]
            count = counts[idx]
            self.get_logger().info(
                f'BGRA={pixel.tolist()} count={int(count)}'
            )

    # ==================================================
    # Frame transform
    # ==================================================
    def camera_to_ego(self, x_cam, y_cam, z_cam):
        """
        Temporary transform from camera frame to ego_vehicle frame.

        ==================================================
        # TODO:
        # Replace this with the REAL extrinsic transform.
        # You need to confirm:
        # - camera position relative to ego_vehicle
        # - axis convention of the depth camera
        # - whether a rotation is required
        #
        # Current assumption:
        #   camera forward  -> ego x
        #   camera right    -> -ego y
        #   camera down/up  -> -ego z
        # ==================================================
        """

        x_ego = z_cam + self.cam_offset_x
        y_ego = -x_cam + self.cam_offset_y
        z_ego = -y_cam + self.cam_offset_z

        return x_ego, y_ego, z_ego

    # ==================================================
    # PointCloud2 publisher
    # ==================================================
    def publish_cloud(self, publisher, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'ego_vehicle'

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = self.make_fields()
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True

        if len(points) == 0:
            msg.data = b''
        else:
            msg.data = b''.join([struct.pack('fff', p[0], p[1], p[2]) for p in points])

        publisher.publish(msg)

    def make_fields(self):
        return [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]


def main(args=None):
    rclpy.init(args=args)
    node = ProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
