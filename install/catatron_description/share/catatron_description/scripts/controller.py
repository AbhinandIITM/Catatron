#! /usr/bin/env python3

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class SteeringActionClient(Node):

    def __init__(self):
        super().__init__('controller')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def send_goal(self, angle):
        goal_msg = FollowJointTrajectory.Goal()

        # List of 12 joint names for Catatron
        joint_names = [
            "hip1_fl", "hip1_fr", "hip1_br", "hip1_bl",  # Front and back hips (left and right)
            "hip2_fl", "hip2_fr", "hip2_br", "hip2_bl",  # Same for the other side (left and right)
            "knee_fl", "knee_fr", "knee_br", "knee_bl"   # Knee joints for all legs
        ]

        # Create trajectory points
        points = []
        
        # First point: Initial positions (all zeros, assuming initial position is neutral)
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0] * 12  # 12 joints with zero position
        points.append(point1)

        # Second point: The target positions (all joints set to the same angle)
        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point2.positions = [angle] * 12  # Set all joints to the given angle
        points.append(point2)

        # Set goal time tolerance
        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        # Send goal and handle feedback
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get the result and shutdown after completion
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        # Optional: Log feedback for real-time status
        feedback = feedback_msg.feedback
        # self.get_logger().info('Received feedback: ' + str(feedback))


def main(args=None):
    # Initialize ROS 2
    rclpy.init()

    # Instantiate action client
    action_client = SteeringActionClient()

    # Get angle from command-line arguments
    if len(sys.argv) < 2:
        print("Please provide an angle as a command-line argument.")
        return

    angle = float(sys.argv[1])
    # Send goal to action server
    action_client.send_goal(angle)

    # Keep the node running to wait for the action result
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
