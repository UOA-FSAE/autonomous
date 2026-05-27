"""Camera-LiDAR fusion node (Component B).

This chunk wires up the plumbing only: it subscribes to the camera cone
detections and the LiDAR cloud, sets up a TF listener, projects each camera
seed point from the base_link frame into the LiDAR (velodyne) frame, and logs
the result. Optionally it publishes the projected seeds as RViz markers so the
alignment can be checked visually against the raw point cloud.

The actual sphere crop / refinement / time-synchronised fusion is added in
later chunks; the latest LiDAR cloud is stashed here so those chunks can use it.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs  # noqa: F401  (registers do_transform_point for PointStamped)
from tf2_geometry_msgs import do_transform_point

from fsae_interfaces.msg import Detections


# Camera colour label -> (FusedCone colour code, RGBA marker colour).
# The Detections message groups cones by colour into separate arrays; we keep
# the colour code (matching Cone.msg / FusedCone.msg) alongside each array.
COLOUR_BLUE = 0
COLOUR_ORANGE = 1
COLOUR_YELLOW = 2
COLOUR_OTHER = 3

MARKER_RGBA = {
    COLOUR_BLUE: (0.1, 0.3, 1.0, 0.9),
    COLOUR_ORANGE: (1.0, 0.5, 0.0, 0.9),
    COLOUR_YELLOW: (1.0, 1.0, 0.0, 0.9),
    COLOUR_OTHER: (0.6, 0.6, 0.6, 0.9),
}


class LidarFusionNode(Node):
    """Subscribe to camera + LiDAR, project camera seeds into the LiDAR frame."""

    def __init__(self):
        """Declare parameters, create subscriptions, publishers and TF listener."""
        super().__init__('lidar_fusion')

        # --- parameters (defaults mirror params/lidar_fusion.yaml) ---
        self.camera_topic = self.declare_parameter(
            'camera_topic', 'zed/cone_detection').value
        self.lidar_topic = self.declare_parameter(
            'lidar_topic', '/velodyne_points').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.lidar_frame = self.declare_parameter('lidar_frame', 'velodyne').value
        self.publish_debug_markers = self.declare_parameter(
            'publish_debug_markers', True).value
        self.debug_marker_topic = self.declare_parameter(
            'debug_marker_topic', '/lidar_fusion/seed_markers').value

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- state ---
        self._latest_cloud = None  # most recent PointCloud2 (used by later chunks)

        # --- subscriptions ---
        self.create_subscription(
            Detections, self.camera_topic, self._detections_cb, 10)
        self.create_subscription(
            PointCloud2, self.lidar_topic, self._cloud_cb, 10)

        # --- publishers ---
        self.marker_pub = None
        if self.publish_debug_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray, self.debug_marker_topic, 10)

        self.get_logger().info(
            f"lidar_fusion up: camera='{self.camera_topic}' "
            f"lidar='{self.lidar_topic}' "
            f"transform {self.base_frame} -> {self.lidar_frame}")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _cloud_cb(self, msg: PointCloud2):
        """Stash the most recent LiDAR cloud for use by later chunks."""
        self._latest_cloud = msg

    def _detections_cb(self, msg: Detections):
        """Project every camera seed into the LiDAR frame and log/visualise it."""
        transform = self._lookup_transform()
        if transform is None:
            return

        seeds = self._iter_seeds(msg)
        markers = MarkerArray()
        n = 0
        for idx, (colour, pt) in enumerate(seeds):
            seed_lidar = self._project(pt, transform)
            if seed_lidar is None:
                continue
            n += 1
            self.get_logger().info(
                f"seed[{idx}] colour={colour} "
                f"base_link=({pt.x:.2f},{pt.y:.2f},{pt.z:.2f}) -> "
                f"{self.lidar_frame}=("
                f"{seed_lidar.x:.2f},{seed_lidar.y:.2f},{seed_lidar.z:.2f})")
            if self.marker_pub is not None:
                markers.markers.append(
                    self._make_marker(idx, colour, seed_lidar))

        if self.marker_pub is not None:
            self.marker_pub.publish(markers)

        if n:
            self.get_logger().debug(f"projected {n} camera seeds")

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def _iter_seeds(self, msg: Detections):
        """Yield (colour_code, geometry_msgs/Point) for every cone in Detections."""
        for pt in msg.blue:
            yield COLOUR_BLUE, pt
        for pt in msg.yellow:
            yield COLOUR_YELLOW, pt
        for pt in msg.small_orange:
            yield COLOUR_ORANGE, pt
        for pt in msg.big_orange:
            yield COLOUR_ORANGE, pt

    def _lookup_transform(self):
        """Look up base_frame -> lidar_frame (latest available). None on failure."""
        try:
            # target = lidar_frame, source = base_frame; Time() = latest available.
            return self.tf_buffer.lookup_transform(
                self.lidar_frame, self.base_frame, Time())
        except (LookupException, ConnectivityException,
                ExtrapolationException) as exc:
            self.get_logger().warn(
                f"TF {self.base_frame} -> {self.lidar_frame} unavailable: {exc}")
            return None

    def _project(self, pt: Point, transform) -> Point:
        """Transform a base_link point into the LiDAR frame via TF."""
        stamped = PointStamped()
        stamped.header.frame_id = self.base_frame
        stamped.point = pt
        try:
            out = do_transform_point(stamped, transform)
            return out.point
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"point transform failed: {exc}")
            return None

    def _make_marker(self, idx: int, colour: int, pt: Point) -> Marker:
        """Build a small sphere marker (in the LiDAR frame) for one seed."""
        m = Marker()
        m.header.frame_id = self.lidar_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'lidar_fusion_seeds'
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = pt
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        r, g, b, a = MARKER_RGBA.get(colour, MARKER_RGBA[COLOUR_OTHER])
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        m.lifetime = Duration(seconds=0.5).to_msg()
        return m


def main(args=None):
    """Spin the LiDAR fusion node."""
    rclpy.init(args=args)
    node = LidarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
