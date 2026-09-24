#!/usr/bin/env python3
"""
ROS 2 to UDP Bridge Node - FIXED VERSION
Subscribes to drive_yolo 3D tracking results (with velocity) and sends them via UDP
"""

import rclpy
from rclpy.node import Node
import socket
from drive_yolo_msgs.msg import TrackedObjects3D, TrackedObject3D

class ROS1ToUDPBridge(Node):
    def __init__(self):
        super().__init__('ros1_udp_bridge_node')

        # UDP Configuration
        self.udp_host = self.declare_parameter('udp_host', '127.0.0.1').value
        self.udp_port = self.declare_parameter('udp_port', 9999).value

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_address = (self.udp_host, self.udp_port)

        # ROS1 Subscriber - FIXED: Subscribe to tracked objects with velocity
        self.detection_sub = self.create_subscription(
            TrackedObjects3D,           # Changed message type
            '/tracked_objects_3d',      # Changed from /detections_with_distance
            self.tracked_objects_callback,  # Renamed callback
            10
        )

        # Optional: Also subscribe to high-quality tracks if you prefer
        self.hq_tracks_sub = self.create_subscription(
            TrackedObjects3D,
            '/high_quality_tracks_3d',
            self.hq_tracks_callback,
            10
        )

        # Statistics
        self.sent_count = 0
        self.error_count = 0

        self.get_logger().info(f"=== ROS1 to UDP Bridge Started (WITH VELOCITY) ===")
        self.get_logger().info(f"Target: {self.udp_host}:{self.udp_port}")
        self.get_logger().info(f"Subscribing to: /tracked_objects_3d (with velocity info)")
        self.get_logger().info(f"Also subscribing to: /high_quality_tracks_3d")
        self.get_logger().info(f"String format: 'HEADER|detection1|detection2|...'")
        self.get_logger().info(f"================================================")

    def tracked_objects_callback(self, msg):
        """Convert ROS1 TrackedObjects3D to string and send via UDP"""
        try:
            # Convert to simple string format
            udp_string = self.convert_tracked_objects_to_string(msg)

            # Send via UDP
            self.sock.sendto(udp_string.encode('utf-8'), self.target_address)

            self.sent_count += 1

            # Log progress with velocity info
            if self.sent_count % 10 == 0:
                velocity_info = ""
                if msg.objects:
                    max_speed = max(obj.speed_3d for obj in msg.objects)
                    velocity_info = f" (max speed: {max_speed:.1f} m/s)"

                self.get_logger().info(f"Sent {self.sent_count} tracking messages via UDP{velocity_info}")

        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"Error sending UDP message: {e}")

    def hq_tracks_callback(self, msg):
        """Handle high-quality tracks (optional - you can remove this if not needed)"""
        # For now, just log that we got high-quality tracks
        if msg.objects:
            self.get_logger().debug(f"Received {len(msg.objects)} high-quality tracks")

    def convert_tracked_objects_to_string(self, ros1_msg):
        """
        Convert TrackedObjects3D to pipe-separated string format:
        HEADER:timestamp,frame_id|DET:id,score,type,px,py,pz,vx,vy,vz,sx,sy,sz,qx,qy,qz,qw,distance,points|DET:...

        Now with REAL velocity data from the tracking system!
        """
        parts = []

        # Header part
        header_part = f"HEADER:{ros1_msg.header.stamp.sec}.{ros1_msg.header.stamp.nanosec},{ros1_msg.header.frame_id}"
        parts.append(header_part)

        # Tracked object parts - NOW WITH VELOCITY!
        for track in ros1_msg.objects:
            # Tracking ID (unique across time)
            track_id = track.id
            score = track.confidence
            det_type = track.class_id

            # Position from 3D tracking (meters)
            px = track.position_3d.x
            py = track.position_3d.y
            pz = track.position_3d.z

            # VELOCITY - This is what you were missing!
            vx = track.velocity_3d.x
            vy = track.velocity_3d.y
            vz = track.velocity_3d.z

            # Size from 2D bbox (converted to meters - rough estimate)
            sx = track.width * 0.01   # Convert pixels to rough meters
            sy = track.height * 0.01
            sz = 1.0  # Default depth

            # Orientation (default quaternion - no rotation info available)
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

            # Additional tracking info
            distance = (px*px + py*py + pz*pz) ** 0.5  # Calculate distance from position
            lidar_points = track.lidar_points_count

            # Optional: Include tracking quality info in a comment at the end
            speed_3d = track.speed_3d
            track_duration = track.track_duration
            is_moving = track.is_moving

            # Create detection string with REAL velocity data
            det_part = f"DET:{track_id},{score:.3f},{det_type},{px:.3f},{py:.3f},{pz:.3f},{vx:.3f},{vy:.3f},{vz:.3f},{sx:.3f},{sy:.3f},{sz:.3f},{qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f},{distance:.3f},{lidar_points}"

            # Optional: Add tracking metadata as a comment
            # det_part += f"#speed={speed_3d:.2f},duration={track_duration:.1f},moving={is_moving}"

            parts.append(det_part)

        # Join all parts with pipe separator
        result = "|".join(parts)

        # Debug: Log first few characters to verify velocity is included
        if ros1_msg.objects and self.sent_count % 20 == 0:
            self.get_logger().debug(f"Sample UDP string (first 100 chars): {result[:100]}...")

        return result

    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info(f"Shutting down UDP bridge. Sent: {self.sent_count}, Errors: {self.error_count}")
        if self.sock:
            self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    bridge = None
    try:
        bridge = ROS1ToUDPBridge()

        bridge.get_logger().info("UDP Bridge running with VELOCITY support... Press Ctrl+C to stop")
        rclpy.spin(bridge)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('ros1_udp_bridge_node').error(f"Bridge failed: {e}")
    finally:
        # Handle shutdown gracefully
        if bridge is not None:
            bridge.shutdown()
            bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
