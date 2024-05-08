#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from UR_Kinematics import inverse_kinematic_solution, DH_matrix_UR5e
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np



class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.window = []

    def filter_point(self, point):
        self.window.append(point)
        if len(self.window) > self.window_size:
            self.window = self.window[-self.window_size:]

        filtered_point = PointStamped()
        filtered_point.header = point.header
        filtered_point.point.x = sum(p.point.x for p in self.window) / len(self.window)
        filtered_point.point.y = sum(p.point.y for p in self.window) / len(self.window)
        filtered_point.point.z = sum(p.point.z for p in self.window) / len(self.window)

        return filtered_point


class PointSubscriber(Node):
    def __init__(self):
        super().__init__('point_subscriber')
        self.filter = MovingAverageFilter(window_size =15)
        self.subscription = self.create_subscription(PointStamped, 'joint16_coordinates_transformed', self.callback, 10)
        self.publisher = self.create_publisher(JointTrajectory, 'joint_angles', 10)
        self.prev_solution = None

    def callback(self, msg):
        filtered_point = self.filter.filter_point(msg)
        if filtered_point.point.x > 1.00:
            ee_x = 1.00
        elif filtered_point.point.x < -1.00:
            ee_x = -1.00
        else:
            ee_x = filtered_point.point.x

        if filtered_point.point.y < -5.00:
            ee_y = -1.00
        elif filtered_point.point.y > -2.00:
            ee_y = 0.00
        else:
            ee_y = (filtered_point.point.y + 2.00)/3.00

        if filtered_point.point.z > 1.079:
            ee_z = 1.079
        elif filtered_point.point.z < 0.15:
            ee_z = 0.15
        else:
            ee_z = filtered_point.point.z

        transform = np.matrix([[1.00000000e+00, 0.00000000e+00, 0.00000000e+00, ee_x],
                            [0.00000000e+00, 0.00000000e+00, -1.00000000e+00, ee_y],
                            [0.00000000e+00, 1.00000000e+00, 0.00000000e+00, ee_z],
                            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
                        
        IKS = inverse_kinematic_solution(DH_matrix_UR5e, transform)
    

        if self.prev_solution is None:
            i = 0; 
        else: 
            mse0 = np.mean((IKS[:, 0].flatten() - self.prev_solution) ** 2)
            mse1 = np.mean((IKS[:, 1].flatten() - self.prev_solution) ** 2)

            if mse0 < mse1:
                i = 0
            else:
                i = 1

        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        joint_trajectory_msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",]
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [IKS[j, i] for j in range(6)] 
        joint_trajectory_point.velocities = [0.0, 0.0, 0.0, 0.00, 0.0, 0.0]
        joint_trajectory_point.time_from_start.sec = 0  
        joint_trajectory_point.time_from_start.nanosec = 50000000
        joint_trajectory_msg.points.append(joint_trajectory_point)
        self.publisher.publish(joint_trajectory_msg)

        # Store the current solution for next iteration
        self.prev_solution = IKS[:, i]



def main(args=None):
    rclpy.init(args=args)
    point_subscriber = PointSubscriber()
    rclpy.spin(point_subscriber)
    point_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()