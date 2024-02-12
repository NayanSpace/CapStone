#!/usr/bin/env python3
import time
import sys
import tty
import termios
import threading

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

        self.current_joint = 0  # Index of the currently selected joint
        self.joint_positions = [0.0] * len(self.joints)  # Initial positions of each joint
        self.key_listener_thread = threading.Thread(target=self.listen_to_keyboard_input)
        self.key_listener_thread.start()

    def listen_to_keyboard_input(self):
        # Setup for reading a single character from the terminal without waiting for Enter
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)

            while True:
                char = sys.stdin.read(1)
                self.process_key(char)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def process_key(self, key):
        if key == 'q':  # Quit the program
            rclpy.shutdown()
            sys.exit()
        elif key == 'j':  # Move to the previous joint
            self.current_joint = (self.current_joint - 1) % len(self.joints)
        elif key == 'k':  # Move to the next joint
            self.current_joint = (self.current_joint + 1) % len(self.joints)
        elif key == 'a':  # Decrease position of the current joint
            self.joint_positions[self.current_joint] -= 0.1
        elif key == 'd':  # Increase position of the current joint
            self.joint_positions[self.current_joint] += 0.1

    def send_joint_trajectory(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions

        # Create a Duration object for time_from_start
        duration = Duration()
        duration.sec = 0
        duration.nanosec = 500000000  # 0.5 seconds
        point.time_from_start = duration

        goal.trajectory.points.append(point)

        self.get_logger().info(f"Sending joint positions: {self.joint_positions}")
        self._action_client.send_goal_async(goal)

    def run(self):
        while rclpy.ok():
            self.send_joint_trajectory()
            time.sleep(0.1)  # Adjust the frequency of sending joint positions


def main(args=None):
    rclpy.init(args=args)

    jtc_client = JTCClient()
    jtc_client.run()


if __name__ == "__main__":
    main()

