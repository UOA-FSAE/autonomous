"""Camera-LiDAR fusion node (Component B).

So far this node subscribes to the camera cone detections and the LiDAR cloud,
sets up a TF listener, and projects each camera seed point from the base_link
frame into the LiDAR (velodyne) frame. As of Chunk 4 it also crops the LiDAR
cloud to a 1 m sphere around each projected seed and logs how many points fall
inside, which is the manual check that the seed is aimed at real cloud data.

The remaining refinement (ground removal, clustering, circle fit) and the
time-synchronised publishing of fused cones are added in later chunks.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs  # noqa: F401  (registers do_transform_point for PointStamped)
from tf2_geometry_msgs import do_transform_point

from fsae_interfaces.msg import Detections

from fsae_lidar_fusion.refinement import crop_sphere


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
        self.r_sphere = self.declare_parameter('r_sphere', 1.0).value
        self.publish_debug_markers = self.declare_parameter(
            'publish_debug_markers', True).value
        self.debug_marker_topic = self.declare_parameter(
            'debug_marker_topic', '/lidar_fusion/seed_markers').value
        self.publish_debug_cloud = self.declare_parameter(
            'publish_debug_cloud', True).value
        self.debug_cloud_topic = self.declare_parameter(
            'debug_cloud_topic', '/lidar_fusion/cropped_points').value

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- state ---
        self._latest_cloud = None    # most recent PointCloud2 message
        self._latest_points = None   # most recent cloud as an (N, 5) numpy array

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
        self.cloud_pub = None
        if self.publish_debug_cloud:
            self.cloud_pub = self.create_publisher(
                PointCloud2, self.debug_cloud_topic, 10)

        self.get_logger().info(
            f"lidar_fusion up: camera='{self.camera_topic}' "
            f"lidar='{self.lidar_topic}' "
            f"transform {self.base_frame} -> {self.lidar_frame}")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _cloud_cb(self, msg: PointCloud2):
        """Stash the latest LiDAR cloud and its parsed (N, 5) numpy array."""
        self._latest_cloud = msg
        self._latest_points = self._parse_cloud(msg)

    def _detections_cb(self, msg: Detections):
        """Project each camera seed, crop the LiDAR sphere, log the point count."""
        transform = self._lookup_transform()
        if transform is None:
            return

        points = self._latest_points
        have_cloud = points is not None and points.shape[0] > 0
        if not have_cloud:
            self.get_logger().warn(
                'no LiDAR cloud yet; projecting seeds without sphere crop')

        seeds = self._iter_seeds(msg)
        markers = MarkerArray()
        crop_accum = []   # cropped point subsets, concatenated for the debug cloud
        n = 0
        for idx, (colour, pt) in enumerate(seeds):
            seed_lidar = self._project(pt, transform)
            if seed_lidar is None:
                continue
            n += 1

            count = -1
            if have_cloud:
                seed_xyz = (seed_lidar.x, seed_lidar.y, seed_lidar.z)
                cropped = crop_sphere(points, seed_xyz, self.r_sphere)
                count = cropped.shape[0]
                if count:
                    crop_accum.append(cropped)

            self.get_logger().info(
                f"seed[{idx}] colour={colour} "
                f"base_link=({pt.x:.2f},{pt.y:.2f},{pt.z:.2f}) -> "
                f"{self.lidar_frame}=("
                f"{seed_lidar.x:.2f},{seed_lidar.y:.2f},{seed_lidar.z:.2f}) "
                f"| {count} pts within {self.r_sphere:.2f} m")
            if self.marker_pub is not None:
                markers.markers.append(
                    self._make_marker(idx, colour, seed_lidar))
                markers.markers.append(
                    self._make_boundary_marker(idx, seed_lidar))

        if self.marker_pub is not None:
            self.marker_pub.publish(markers)

        if self.cloud_pub is not None:
            if crop_accum:
                allpts = np.vstack(crop_accum)
            else:
                allpts = np.empty((0, 5), dtype=float)
            self.cloud_pub.publish(self._make_cloud_msg(allpts))

        if n:
            self.get_logger().debug(f"projected {n} camera seeds")

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def _parse_cloud(self, msg: PointCloud2) -> np.ndarray:
        """Parse a PointCloud2 into an (N, 5) array of (x, y, z, intensity, ring).

        Returns an empty (0, 5) array if the cloud has no points or lacks the
        expected fields (e.g. a sensor that does not publish ``ring``).
        """
        try:
            structured = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z', 'intensity', 'ring'],
                skip_nans=True)
        except (KeyError, AssertionError) as exc:
            self.get_logger().warn(
                f"cloud missing expected fields (need ring): {exc}")
            return np.empty((0, 5), dtype=float)

        n = structured.shape[0]
        if n == 0:
            return np.empty((0, 5), dtype=float)

        out = np.empty((n, 5), dtype=float)
        out[:, 0] = structured['x']
        out[:, 1] = structured['y']
        out[:, 2] = structured['z']
        out[:, 3] = structured['intensity']
        out[:, 4] = structured['ring']
        return out

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
        """Build a small solid sphere marker (LiDAR frame) at one seed."""
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
        # lifetime 0 = persist until overwritten, so a one-shot publish stays.
        m.lifetime = Duration(seconds=0).to_msg()
        return m

    def _make_boundary_marker(self, idx: int, pt: Point) -> Marker:
        """Build a translucent sphere showing the r_sphere crop boundary."""
        m = Marker()
        m.header.frame_id = self.lidar_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'lidar_fusion_sphere'
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = pt
        m.pose.orientation.w = 1.0
        diameter = 2.0 * self.r_sphere
        m.scale.x = m.scale.y = m.scale.z = diameter
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.8, 1.0, 0.15
        m.lifetime = Duration(seconds=0).to_msg()
        return m

    def _make_cloud_msg(self, points: np.ndarray) -> PointCloud2:
        """Build a PointCloud2 (x, y, z, intensity) in the LiDAR frame.

        Used to visualise the cropped, in-sphere points in RViz. An empty
        ``points`` array publishes an empty cloud, which clears the display.
        """
        header = Header()
        header.frame_id = self.lidar_frame
        if self._latest_cloud is not None:
            header.stamp = self._latest_cloud.header.stamp
        else:
            header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        rows = points[:, :4].tolist() if points.shape[0] else []
        return point_cloud2.create_cloud(header, fields, rows)


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
