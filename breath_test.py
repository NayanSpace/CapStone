import math
import time
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

# Define breathing parameters
inhale_duration = 5  # Duration of inhale phase in seconds
exhale_duration = 5  # Duration of exhale phase in seconds
amplitude = 0.3      # Amplitude of breathing motion (adjust as needed)

class JTCClient(rclpy.node.Node):
    def __init__(self):
        super().__init__("jtc_client")

        self.current_trajectory_index = 0
        self.start_time = time.time()

        # Define joint names
        self.joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Create action client
        controller_name = "scaled_joint_trajectory_controller/follow_joint_trajectory"
        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self.execute_next_trajectory()

    def execute_next_trajectory(self):
        traj_name = "breathing_trajectory"  # Assuming only one trajectory for breathing
        self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        self.get_logger().info(f"Executing trajectory {traj_name}")

        # Calculate time-dependent input for the sine function
        current_time = time.time() - self.start_time
        sin_input = current_time / (3) * 2 * math.pi

        # Create trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joints

        # Set joint positions based on the time-varying sine input
        point = JointTrajectoryPoint()
        point.positions = [
            1.57 + 0.75*amplitude * math.sin(sin_input),
            -1.57 + 2*amplitude * math.sin(sin_input),
            0.00 - 2.5*amplitude * math.sin(sin_input),
            -1.57 + 2.5*amplitude * math.sin(sin_input),
            -1.57,
            0.00
        ]
        point.velocities = [0.0] * len(self.joints)
        point.time_from_start = Duration(sec=inhale_duration, nanosec=0)
        goal.trajectory.points.append(point)

        self._send_goal_future = self._action_client.send_goal_async(goal)
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
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            time.sleep(1)  # Add a delay between trajectory executions if needed
            self.execute_next_trajectory()

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

