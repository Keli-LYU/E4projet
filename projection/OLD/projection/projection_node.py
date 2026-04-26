#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ProjectionNode(Node):
    def __init__(self):
        super().__init__('projection_node')

        # ===== Parameters =====
        self.semantic_topic = '/carla/ego_vehicle/semantic_cam/image'
        self.depth_topic = '/carla/ego_vehicle/depth_cam/image'
        self.depth_info_topic = '/carla/ego_vehicle/depth_cam/camera_info'
        self.projected_map_topic = '/projected_map'

        self.map_width_m = 20.0      # lateral width (left-right)
        self.map_height_m = 30.0     # forward range
        self.resolution = 0.2        # meters / cell
        self.max_depth = 50.0        # ignore very far points
        self.sample_step = 4         # skip pixels for speed
        self.process_period = 0.5    # seconds

        # ===== ROS tools =====
        self.bridge = CvBridge()

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

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            self.projected_map_topic,
            map_qos
        )


        self.timer = self.create_timer(self.process_period, self.process_data)

        # ===== Storage =====
        self.latest_semantic_msg = None
        self.latest_depth_msg = None
        self.latest_depth_info = None

        self.semantic_logged = False
        self.depth_logged = False
        self.depth_info_logged = False

        self.get_logger().info('Projection node started.')

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------
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

    # --------------------------------------------------
    # Main processing
    # --------------------------------------------------
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

        grid_msg, grid_data = self.create_empty_grid()

        projected_count = 0

        # Sample pixels to reduce load
        for v in range(0, height, self.sample_step):
            for u in range(0, width, self.sample_step):
                depth_value = float(depth_img[v, u])

                if not math.isfinite(depth_value):
                    continue
                if depth_value <= 0.0 or depth_value > self.max_depth:
                    continue

                seg_pixel = semantic_img[v, u]

                # Placeholder for future semantic filtering
                if not self.is_target_semantic(seg_pixel):
                    continue

                # Camera coordinates
                x_cam = (u - cx) * depth_value / fx
                y_cam = (v - cy) * depth_value / fy
                z_cam = depth_value

                # For a simple 2D local map:
                # forward = z_cam
                # lateral = x_cam
                forward = z_cam
                lateral = x_cam

                cell_index = self.metric_to_grid_index(lateral, forward, grid_msg.info.width, grid_msg.info.height)
                if cell_index is None:
                    continue

                grid_data[cell_index] = 100
                projected_count += 1

        grid_msg.data = grid_data
        self.map_pub.publish(grid_msg)

        self.get_logger().info(
            f'Published projected_map with {projected_count} projected points.'
        )

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------
    def create_empty_grid(self):
        grid = OccupancyGrid()

        width_cells = int(self.map_width_m / self.resolution)
        height_cells = int(self.map_height_m / self.resolution)

        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        grid.info.resolution = self.resolution
        grid.info.width = width_cells
        grid.info.height = height_cells

        origin = Pose()
        origin.position.x = -self.map_width_m / 2.0   # lateral starts at -half width
        origin.position.y = 0.0                       # forward starts at 0
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        grid.info.origin = origin

        data = [0] * (width_cells * height_cells)
        return grid, data

    def metric_to_grid_index(self, lateral, forward, width_cells, height_cells):
        # Ignore points behind vehicle or outside local map
        if forward < 0.0 or forward >= self.map_height_m:
            return None
        if lateral < -self.map_width_m / 2.0 or lateral >= self.map_width_m / 2.0:
            return None

        col = int((lateral + self.map_width_m / 2.0) / self.resolution)
        row = int(forward / self.resolution)

        if col < 0 or col >= width_cells or row < 0 or row >= height_cells:
            return None

        return row * width_cells + col

    def is_target_semantic(self, seg_pixel):
        """
        For now: accept all valid depth pixels.
        Later you can filter by semantic class or color here.
        """
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
