#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
import threading
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from http.server import HTTPServer, BaseHTTPRequestHandler
import math
import struct

# OSI imports
try:
    from osi3.osi_sensordata_pb2 import SensorData
except ImportError:
    print("OSI3 not found. Please install with: pip install open-simulation-interface")
    exit(1)


def create_cloud(header, fields, points):
    """
    Build a PointCloud2 from a list of float32 points (sensor_msgs.point_cloud2.create_cloud
    replacement - Foxy has no Python point cloud helpers, sensor_msgs_py came in Galactic)
    """
    point_struct = struct.Struct('<' + 'f' * len(fields))
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = point_struct.size
    cloud.row_step = point_struct.size * len(points)
    cloud.is_dense = False
    cloud.data = b''.join(point_struct.pack(*p) for p in points)
    return cloud


class OSIHTTPHandler(BaseHTTPRequestHandler):
    def __init__(self, node, lidar_publisher, *args, **kwargs):
        self.node = node
        self.lidar_publisher = lidar_publisher
        super().__init__(*args, **kwargs)

    def do_POST(self):
        try:
            # Get content length
            content_length = int(self.headers.get('Content-Length', 0))

            # Read the request body
            post_data = self.rfile.read(content_length)
            #print(time.time())
            # Parse OSI message
            osi_lidar = SensorData()
            if not osi_lidar.ParseFromString(post_data):
                self.node.get_logger().error("Failed to parse OSI Lidar message")
                self.send_error(400, "Invalid OSI data")
                return

            # Convert to PointCloud2 and publish
            self.convert_and_publish_lidar(osi_lidar)

            # Send success response
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Data received')

        except Exception as e:
            self.node.get_logger().error(f"Error processing request: {str(e)}")
            self.send_error(500, str(e))

    def convert_and_publish_lidar(self, osi_lidar):
        """Convert OSI SensorData to ROS PointCloud2 message"""

        # Create header
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = "lidar_frame"

        # Check if we have lidar sensor data
        if not osi_lidar.feature_data.lidar_sensor:
            self.node.get_logger().warning("No lidar sensor data in OSI message")
            return

        lidar_sensor = osi_lidar.feature_data.lidar_sensor[0]

        if not lidar_sensor.detection:
            self.node.get_logger().warning("No detections in lidar sensor data")
            return

        # Convert OSI detections to point cloud points
        points = []
        for detection in lidar_sensor.detection:
            # Extract spherical coordinates
            distance = detection.position.distance
            elevation = detection.position.elevation
            azimuth = detection.position.azimuth
            intensity = detection.intensity

            # Convert spherical to Cartesian coordinates
            x = distance * math.cos(elevation) * math.cos(azimuth)
            y = - distance * math.cos(elevation) * math.sin(azimuth)
            z = distance * math.sin(elevation)

            # Add point with intensity
            points.append([x, y, z, intensity])

        # Define point cloud fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # Create PointCloud2 message
        point_cloud = create_cloud(header, fields, points)

        # Publish the point cloud
        self.lidar_publisher.publish(point_cloud)
        self.node.get_logger().info(f"Published lidar data with {len(points)} points")

    def log_message(self, format, *args):
        """Override to use ROS logging instead of print"""
        self.node.get_logger().debug(format % args)


class LidarOsiHttpReceiver(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__('lidar_osi_http_receiver')

        # Get parameters
        self.port = self.declare_parameter('port', 5002).value
        self.topic_name = self.declare_parameter('topic_name', '/invz_summation_reflection').value

        # Create publisher
        self.lidar_pub = self.create_publisher(PointCloud2, self.topic_name, 10)

        # Create HTTP server
        def handler_factory(*args, **kwargs):
            return OSIHTTPHandler(self, self.lidar_pub, *args, **kwargs)

        self.server = HTTPServer(('', self.port), handler_factory)
        self.get_logger().info(f"Starting HTTP server on port {self.port}")
        self.get_logger().info(f"Publishing lidar data to topic: {self.topic_name}")

        # Start server in separate thread
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down HTTP server...")
        self.server.shutdown()
        self.server_thread.join()


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('lidar_osi_http_receiver')
    receiver = None
    try:
        # Create the receiver
        receiver = LidarOsiHttpReceiver()

        # Keep the node running
        logger.info("Lidar OSI HTTP Receiver is running...")
        rclpy.spin(receiver)

    except KeyboardInterrupt:
        logger.info("Node interrupted")
    except Exception as e:
        logger.error(f"Error: {str(e)}")
    finally:
        logger.info("Node shutting down")
        # Shutdown hook
        if receiver is not None:
            receiver.shutdown()
            receiver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
