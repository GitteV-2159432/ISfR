#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cbor2

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'encoded_coordinates',  # Ensure this matches the publisher's topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        
        # Decode the CBOR data
        try:
            # Convert hex string back to bytes
            cbor_data = bytes.fromhex(msg.data)
            coordinates = cbor2.loads(cbor_data)
            
            # Print the coordinates
            x_coords = coordinates.get('x', [])
            y_coords = coordinates.get('y', [])
            z_coords = coordinates.get('z', [])
            
            for x, y, z in zip(x_coords, y_coords, z_coords):
                self.get_logger().info(f'Coordinate - x: {x}, y: {y}, z: {z}')
        
        except Exception as e:
            self.get_logger().error(f'Failed to decode CBOR data: {e}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()