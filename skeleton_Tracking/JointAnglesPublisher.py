#JointAnglesPublisher.py
from std_msgs.msg import Float32MultiArray
import rclpy

class JointAnglesPublisher:
    def __init__(self, node_name='joint_angles_publisher'):
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.publisher = self.node.create_publisher(Float32MultiArray, 'joint_angles_topic', 10)

    def publish_joint_angles(self, joint_angles):
        normalized_angles = []

        for angle_tuple in joint_angles:
            normalized_tuple = tuple(angle % 360.0 for angle in angle_tuple)
            normalized_angles.append(normalized_tuple)

        # Create a Float32MultiArray message
        msg = Float32MultiArray()

        # Flatten the list of tuples to create a 1D list
        flat_normalized_angles = [angle for angle_tuple in normalized_angles for angle in angle_tuple]

        # Assign the normalized angles to 'data'
        msg.data = flat_normalized_angles

        # Publish the message
        self.publisher.publish(msg)
        
    def spin(self):
        rclpy.spin(self.node)

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown() 

