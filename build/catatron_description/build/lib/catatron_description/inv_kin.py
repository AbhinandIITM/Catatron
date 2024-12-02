#! /usr/bin/env python3

import sys
import rclpy
import numpy as np
from urdf_parser_py.urdf import URDF
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
    def calculate_angles(self):
        leg_dimensions   = self.getLegDimensions()
        feet_pos = [-0.1, -0.1,-0.13]
        leg      = 'fl'
        hr, joint_angles = self.inv_kinematics(leg_dimensions,leg,feet_pos)
        if(hr):
            return joint_angles
        else:
            print("Inv kin not possible.")
            return [0,0,0]
        
    def inv_kinematics(self,leg_dimensions,leg,feet_pos):
        L0 = np.abs(np.sum(leg_dimensions[leg][:,1], axis=0))  # Offset of the feet in the y axis
        L1 = np.abs(leg_dimensions[leg][2,2])                  # Length of the femur in the z axis
        L2 = np.abs(leg_dimensions[leg][3,2])                  # Length of the tibia in the z axis
        
        [x, y, z] = feet_pos
        
        z_corr_2 = y**2 + z**2 - L0**2
        D        = float(x**2 + y**2 + z**2 - L0**2 - L1**2 - L2**2)/(2*L1*L2)

        if D > 1 or z_corr_2 < 0:                             # If D > 1, or z_corr_2 < 0, reachability is not possible
            return False, np.array([0,0,0])                   #     return [0,0,0] angles
        
        z_corr   = np.sqrt( z_corr_2 )


        if 'l' in leg:                                         # Left leg 
            a = np.pi - np.arctan2(-z, y) - np.arctan2(z_corr, -L0)     # hip1 angle
        
        else:                                                  # Right leg
            a =  np.arctan2(z, y) + np.arctan2(z_corr, -L0)     # hip1 angle 

            
        #c = np.arctan2( np.sqrt(1 - D**2),  D )                    # knee angle, inverted knee angle
        c = np.arctan2( - np.sqrt(1 - D**2),  D )                   # knee angle

        
        # hip2 angle
        b = -np.pi+ np.arctan2(  x, -z_corr  ) - np.arctan2(  L2*np.sin(c), L1 + L2*np.cos(c)  ) 

        angles = np.array([a,b,c])
        return True, angles
        
    def getLegDimensions(self):
        urdf_model  = r'/home/abhinand/Envisage/catatron_ws/src/catatron_description/urdf/catatron.urdf'

        urdf_str = open(urdf_model, 'r').read()
        robot_urdf = URDF.from_xml_string(urdf_str)

        dimentions = {}
        leg_names   = ['fl', 'fr', 'bl', 'br']
        joint_names = ['hip1', 'hip2', 'knee', 'feet']
        
        for name in leg_names:
            dimentions[name] = {}

        
        for joint in robot_urdf.joints:
            for joint_name in joint_names:
                if joint_name in joint.name:
                    name = joint.name.split('_')[1]
                    xyz = np.array( joint.origin.xyz, dtype = 'float32' )
                    #print(xyz)
                    dimentions[name][joint_name] = xyz


        dimentions_leg = {}
        for key in dimentions:
            dimentions_leg[key] = np.array(  [dimentions[key][j] for j in joint_names] )

        return dimentions_leg


    def send_goal(self, angle1,angle2,angle3):
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
        point2.positions = [angle1,0,0,0,angle2,0,0,0,angle3,0,0,0]  # Set all joints to the given angle
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
    action_client.calculate_angles()
    # action_client.send_goal(angle)

    # Keep the node running to wait for the action result
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
