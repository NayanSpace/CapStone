#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class JTCClient(rclpy.node.Node):
    """Small test client for the jtc"""

    def __init__(self):
        super().__init__("jtc_client")
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self._subscriber = self.create_subscription(
            JointTrajectory,
            'joint_angles',
            self.joint_trajectory_callback,
            10
        )
        
        self.i = 0
        self._send_goal_future = None
        self._get_result_future = None
        
    def joint_trajectory_callback(self, msg):
        '''if self.i >= len(self.goals):
            self.get_logger().info("Done with all trajectories")
            return
        self.i += 1'''
        self.execute_trajectory(msg)

    def execute_trajectory(self, msg):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg
        self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Done with result: {self.error_code_to_str(result.error_code)}")
        #if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            #time.sleep(0.2)
            #self.executetrajectory()

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"


def main(args=None):
    rclpy.init(args=args)

    jtc_client = JTCClient()
    rclpy.spin(jtc_client)

    rclpy.shutdown()


if __name__ == "__main__":
    main()