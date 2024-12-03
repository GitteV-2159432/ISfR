#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import cbor2
import numpy as np
from std_msgs.msg import String

class PointCloudPublisher(Node):

    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.subscriber_ = self.create_subscription(PointCloud2, '/rs_bpearl/points', self.pointcloud_callback, 10)
        self.publisher_ = self.create_publisher(String, 'encoded_coordinates', 10)
        self.get_logger().info("PointCloud2 subscriber initialized.")

    def pointcloud_callback(self, msg):
        # Check if the required fields are present
        fields = {field.name: field.offset for field in msg.fields}
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            self.get_logger().error("PointCloud2 message does not contain x, y, z fields.")
            return

        # Extract x, y, z coordinates from the PointCloud2 message
        x_coords = []
        y_coords = []
        z_coords = []

        # Use numpy to interpret the data
        data = np.frombuffer(msg.data, dtype=np.float32)
        num_points = len(data) // 3  # Each point has 3 fields (x, y, z)

        # Make sure the number of points is valid
        if len(data) % 3 != 0:
            self.get_logger().error("Data length is not a multiple of 3. Cannot extract coordinates.")
            return

        # Extract coordinates and convert to Python floats
        for i in range(num_points):
            x_coords.append(float(data[i * 3]))      # x coordinate
            y_coords.append(float(data[i * 3 + 1]))  # y coordinate
            z_coords.append(float(data[i * 3 + 2]))  # z coordinate

        # Create a dictionary for the coordinates
        coordinates = {
            'x': x_coords,
            'y': y_coords,
            'z': z_coords
        }
        self.get_logger().info(f'Coordinate - x: {coordinates}')

        # Encode the coordinates to CBOR
        encoded_data = cbor2.dumps(coordinates)

        # Publish the encoded data
        msg_string = String()
        msg_string.data = encoded_data.hex()  # Convert bytes to hex string for publishing
        self.publisher_.publish(msg_string)
        self.get_logger().info('Published encoded coordinates.')

def main(args=None):
    rclpy.init(args=args)

    pointcloud_publisher = PointCloudPublisher()

    rclpy.spin(pointcloud_publisher)

    # Destroy the node explicitly
    pointcloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()