#!/usr/bin/env python3

"""
RViz Visualization Node for 3D Tracked Objects
Subscribes to tracked objects and publishes visual markers including:
- Velocity arrows (direction and magnitude)
- Track ID labels
- Object positions (spheres/cubes)
- Predicted positions
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
from drive_yolo_msgs.msg import TrackedObjects3D, TrackedObject3D
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA


def rgba(r, g, b, a):
    """ColorRGBA from positional values (ROS 2 messages only take keyword arguments)"""
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class TrackingRVizVisualizer(Node):
    """
    RViz visualization node for tracked objects
    """

    def __init__(self):
        super().__init__('tracking_rviz_visualizer')

        # Load parameters
        self.show_velocity_vectors = self.declare_parameter('show_velocity_vectors', True).value
        self.show_track_ids = self.declare_parameter('show_track_ids', True).value
        self.show_positions = self.declare_parameter('show_positions', True).value
        self.show_predicted_positions = self.declare_parameter('show_predicted_positions', True).value
        self.velocity_scale = self.declare_parameter('velocity_scale', 1.0).value  # Scale factor for arrows
        self.min_velocity_display = self.declare_parameter('min_velocity_display', 0.5).value  # m/s
        self.frame_id = self.declare_parameter('frame_id', 'camera').value

        # Color schemes
        self.use_class_colors = self.declare_parameter('use_class_colors', True).value
        self.default_color = self._parse_color(self.declare_parameter('default_color', '0.0,1.0,0.0,1.0').value)

        # Class-specific colors (RGB + Alpha)
        self.class_colors = {
            'car': rgba(0.0, 0.8, 1.0, 1.0),      # Cyan
            'truck': rgba(1.0, 0.5, 0.0, 1.0),    # Orange
            'person': rgba(1.0, 0.0, 0.0, 1.0),   # Red
            'bicycle': rgba(1.0, 1.0, 0.0, 1.0),  # Yellow
            'motorcycle': rgba(1.0, 0.0, 1.0, 1.0), # Magenta
            'bus': rgba(0.5, 0.0, 1.0, 1.0),      # Purple
        }

        # Subscribe to tracked objects
        self.tracked_objects_sub = self.create_subscription(
            TrackedObjects3D,
            '/tracked_objects_3d',
            self.tracked_objects_callback,
            10
        )

        # Publishers for different marker types
        self.velocity_pub = self.create_publisher(
            MarkerArray, '/tracking/velocity_markers', 10)
        self.position_pub = self.create_publisher(
            MarkerArray, '/tracking/position_markers', 10)
        self.label_pub = self.create_publisher(
            MarkerArray, '/tracking/label_markers', 10)
        self.prediction_pub = self.create_publisher(
            MarkerArray, '/tracking/prediction_markers', 10)

        # Statistics
        self.frame_count = 0
        self.last_stats_time = self.get_clock().now()

        self.get_logger().info("🎨 === RViz Tracking Visualizer Started ===")
        self.get_logger().info(f"  Frame ID: {self.frame_id}")
        self.get_logger().info(f"  Velocity vectors: {'ON' if self.show_velocity_vectors else 'OFF'}")
        self.get_logger().info(f"  Track ID labels: {'ON' if self.show_track_ids else 'OFF'}")
        self.get_logger().info(f"  Position markers: {'ON' if self.show_positions else 'OFF'}")
        self.get_logger().info(f"  Predicted positions: {'ON' if self.show_predicted_positions else 'OFF'}")
        self.get_logger().info(f"  Velocity scale: {self.velocity_scale}x")
        self.get_logger().info(f"  Min velocity for display: {self.min_velocity_display} m/s")
        self.get_logger().info("🎨 =====================================")

    def _parse_color(self, color_string):
        """Parse color string 'r,g,b,a' to ColorRGBA"""
        parts = [float(x) for x in color_string.split(',')]
        return rgba(parts[0], parts[1], parts[2], parts[3])

    def _get_color_for_track(self, track):
        """Get color for a track based on class or use default"""
        if self.use_class_colors and track.class_name.lower() in self.class_colors:
            return self.class_colors[track.class_name.lower()]
        return self.default_color

    def _create_velocity_arrow(self, track, marker_id, timestamp):
        """Create an arrow marker representing velocity"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = timestamp
        marker.ns = "velocity_arrows"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow goes from current position in direction of velocity
        start = Point()
        start.x = track.position_3d.x
        start.y = track.position_3d.y
        start.z = track.position_3d.z

        # Scale velocity by velocity_scale parameter for visibility
        end = Point()
        end.x = start.x + track.velocity_3d.x * self.velocity_scale
        end.y = start.y + track.velocity_3d.y * self.velocity_scale
        end.z = start.z + track.velocity_3d.z * self.velocity_scale

        marker.points = [start, end]

        # Arrow size based on speed
        speed = track.speed_3d
        arrow_width = 0.1 + (speed / 20.0) * 0.3  # 0.1 to 0.4 m
        marker.scale.x = arrow_width  # Shaft diameter
        marker.scale.y = arrow_width * 2  # Head diameter
        marker.scale.z = arrow_width * 2  # Head length

        # Color based on speed (green = slow, red = fast)
        color = self._get_color_for_track(track)

        # Optionally tint color based on speed
        speed_ratio = min(1.0, speed / 25.0)  # Max speed for color scale
        marker.color = rgba(
            color.r + speed_ratio * (1.0 - color.r),  # Redder when faster
            color.g * (1.0 - speed_ratio * 0.5),
            color.b * (1.0 - speed_ratio * 0.5),
            1.0
        )

        marker.lifetime = Duration(seconds=0.2).to_msg()  # Persist for 200ms

        return marker

    def _create_position_marker(self, track, marker_id, timestamp):
        """Create a sphere or cube marker at the object's position"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = timestamp
        marker.ns = "positions"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = track.position_3d
        marker.pose.orientation.w = 1.0

        # Size based on object class or confidence
        base_size = 0.5
        if 'truck' in track.class_name.lower() or 'bus' in track.class_name.lower():
            base_size = 1.2
        elif 'person' in track.class_name.lower():
            base_size = 0.3

        marker.scale.x = base_size
        marker.scale.y = base_size
        marker.scale.z = base_size

        # Color with transparency based on confidence
        color = self._get_color_for_track(track)
        marker.color = rgba(
            color.r, color.g, color.b,
            0.3 + 0.5 * track.confidence  # 0.3 to 0.8 alpha
        )

        marker.lifetime = Duration(seconds=0.2).to_msg()

        return marker

    def _create_label_marker(self, track, marker_id, timestamp):
        """Create a text marker showing track ID and info"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = timestamp
        marker.ns = "labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position above the object
        marker.pose.position.x = track.position_3d.x
        marker.pose.position.y = track.position_3d.y
        marker.pose.position.z = track.position_3d.z + 1.0  # 1m above
        marker.pose.orientation.w = 1.0

        # Text content
        marker.text = f"ID:{track.id}\n{track.class_name}\n{track.speed_3d:.1f}m/s"

        # Text size
        marker.scale.z = 0.4  # Text height

        # White text with good visibility
        marker.color = rgba(1.0, 1.0, 1.0, 1.0)

        marker.lifetime = Duration(seconds=0.2).to_msg()

        return marker

    def _create_prediction_marker(self, track, marker_id, timestamp):
        """Create a marker showing predicted position in 1 second"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = timestamp
        marker.ns = "predictions"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position = track.predicted_position_1s
        marker.pose.orientation.w = 1.0

        # Smaller than position marker
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Semi-transparent version of track color
        color = self._get_color_for_track(track)
        marker.color = rgba(color.r, color.g, color.b, 0.5)

        marker.lifetime = Duration(seconds=0.2).to_msg()

        return marker

    def _create_prediction_line(self, track, marker_id, timestamp):
        """Create a line from current position to predicted position"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = timestamp
        marker.ns = "prediction_lines"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Line from current to predicted
        current = track.position_3d
        predicted = track.predicted_position_1s

        marker.points = [current, predicted]

        marker.scale.x = 0.05  # Line width

        # Dashed appearance through transparency
        color = self._get_color_for_track(track)
        marker.color = rgba(color.r, color.g, color.b, 0.4)

        marker.lifetime = Duration(seconds=0.2).to_msg()

        return marker

    def tracked_objects_callback(self, msg):
        """Main callback for tracked objects"""
        self.frame_count += 1

        # Create marker arrays
        velocity_markers = MarkerArray()
        position_markers = MarkerArray()
        label_markers = MarkerArray()
        prediction_markers = MarkerArray()

        marker_id = 0

        for track in msg.objects:
            # Skip if not moving (for velocity arrows)
            if track.speed_3d < self.min_velocity_display and self.show_velocity_vectors:
                continue

            # Velocity arrow
            if self.show_velocity_vectors and track.is_moving:
                velocity_markers.markers.append(
                    self._create_velocity_arrow(track, marker_id, msg.header.stamp)
                )

            # Position marker
            if self.show_positions:
                position_markers.markers.append(
                    self._create_position_marker(track, marker_id, msg.header.stamp)
                )

            # Track ID label
            if self.show_track_ids:
                label_markers.markers.append(
                    self._create_label_marker(track, marker_id, msg.header.stamp)
                )

            # Predicted position
            if self.show_predicted_positions:
                prediction_markers.markers.append(
                    self._create_prediction_marker(track, marker_id, msg.header.stamp)
                )
                prediction_markers.markers.append(
                    self._create_prediction_line(track, marker_id + 10000, msg.header.stamp)
                )

            marker_id += 1

        # Publish marker arrays
        if self.show_velocity_vectors:
            self.velocity_pub.publish(velocity_markers)
        if self.show_positions:
            self.position_pub.publish(position_markers)
        if self.show_track_ids:
            self.label_pub.publish(label_markers)
        if self.show_predicted_positions:
            self.prediction_pub.publish(prediction_markers)

        # Periodic logging
        if (self.get_clock().now() - self.last_stats_time).nanoseconds / 1e9 > 5.0:
            self.get_logger().info(f"🎨 Visualizing {len(msg.objects)} tracked objects "
                         f"({msg.moving_objects} moving)")
            self.last_stats_time = self.get_clock().now()

    def run(self):
        """Run the visualizer node"""
        self.get_logger().info("🎨 Tracking visualizer running...")
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('tracking_rviz_visualizer')
    visualizer = None
    try:
        visualizer = TrackingRVizVisualizer()
        visualizer.run()
    except KeyboardInterrupt:
        logger.info("🎨 Tracking visualizer shutting down")
    except Exception as e:
        logger.error(f"Failed to start tracking visualizer: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        if visualizer is not None:
            visualizer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
