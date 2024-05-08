#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs

class CoordinateTransformer(Node):
    def __init__(self):
        super().__init__('coordinate_transformer')
        self.subscription = self.create_subscription(
            PointStamped,
            'joint16_coordinates',
            self.joint16_callback,
            10)
        self.publisher_transformed = self.create_publisher(PointStamped, 'joint16_coordinates_transformed', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def joint16_callback(self, msg):
        try:
            # Get the transform from 'camera_frame' to 'world'
            transform = self.tf_buffer.lookup_transform(
                'world',  # target frame
                'camera_frame',  # source frame
                rclpy.time.Time())  # get the latest available transform
            # Transform the point from 'camera_frame' to 'world' frame
            transformed_point = tf2_geometry_msgs.do_transform_point(msg, transform)

            self.publisher_transformed.publish(transformed_point)
        except Exception as e:
            self.get_logger().error("Failed to transform point: %s" % str(e))

def main(args=None):
    rclpy.init(args=args)
    transformer = CoordinateTransformer()
    rclpy.spin(transformer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
