import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import numpy as np

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_cam_broadcaster')
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

    def publish_transform(self):
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "camera_frame"

        # Define the transformation from the camera's reference frame to the robot's reference frame
        # This could be a transformation matrix
        translation = geometry_msgs.msg.Vector3(x=0.10, y=-0.63, z=0.07)
        rotation = geometry_msgs.msg.Quaternion(x=0.0, y=-0.7071068, z=0.7071068, w=0.0)

        static_transformStamped.transform.translation = translation
        static_transformStamped.transform.rotation = rotation

        self.tf_broadcaster.sendTransform(static_transformStamped)

def main(args=None):
    rclpy.init(args=args)

    static_publisher = StaticTransformPublisher()

    while rclpy.ok():
        static_publisher.publish_transform()
        rclpy.spin_once(static_publisher)

    static_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()