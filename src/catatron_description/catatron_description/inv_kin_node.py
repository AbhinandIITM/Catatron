#! /usr/bin/env python3
import sys
import ast
import rclpy
import numpy as np
from urdf_parser_py.urdf import URDF
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterType


import logging
from std_msgs.msg import Float64MultiArray
import time

logging.basicConfig(
    level=logging.INFO, 
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(),  # Logs to console
        logging.FileHandler("steering_action_client.log", mode="w"),  # Logs to file
    ],
)
logger = logging.getLogger(__name__)

class SteeringActionClient(Node):

    def __init__(self):
        
        super().__init__('inv_kin_node')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        # self.publisher = self.create_publisher(Float64MultiArray, 'final_positions', 1)    
        self.current_joint_angles = [0.0] * 12
        self.client = self.create_client(GetParameters, '/joint_angles_param/get_parameters')
        self.set_client = self.create_client(SetParameters, '/joint_angles_param/set_parameters')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service to become available...')
        
   
    def get_joint_angles(self):
        request = GetParameters.Request()
        request.names = ['joint_angles'] 
        
        # Wait for the response
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            angles = response.values[0].double_array_value  
            self.get_logger().info(f"{angles} received.")
            return angles
            
        else:
            self.get_logger().error('Failed to fetch joint angles parameter')
            return [0.0] * 12
    from rcl_interfaces.msg import ParameterValue

    def set_angles(self, angles):
    # Create a parameter update request
        param_request = SetParameters.Request()
        
        # Create a ParameterValue to store the joint angles
        # param_value = ParameterValue()
        param_value = Parameter()
        param_value.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        param_value.value.double_array_value = angles  # Set the value as a double array
        param_request.parameters = [param_value]
        # param_request.parameters = [
        #     Parameter(name='joint_angles', value=param_value)
        # ]

        # Call the parameter service
        future = self.set_client    .call_async(param_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Updated joint angles to: {angles}")
        else:
            self.get_logger().error("Failed to set joint angles.")

    
    def calculate_angles(self, legs, feet_positions):
        leg_dimensions = self.getLegDimensions()
        current_angles = self.get_joint_angles()
        self.current_joint_angles = current_angles
        results = {}
        for leg, feet_pos in zip(legs, feet_positions):
            if leg not in leg_dimensions:
                logger.error(leg)
                logger.info(f"Invalid leg identifier: {leg}. Must be one of ['fl', 'fr', 'bl', 'br'].")
                results[leg] = [0, 0, 0]
                continue

            hr, joint_angles = self.inv_kinematics(leg_dimensions, leg, feet_pos)
            if hr:
                # logger.info(f"Calculated joint angles for {leg}: {joint_angles}")
                results[leg] = joint_angles
            else:
                logger.info(f"Inverse kinematics not possible for {leg} with feet position {feet_pos}.")
                results[leg] = [0, 0, 0]

        self.send_goal(results)


    def inv_kinematics(self, leg_dimensions, leg, feet_pos):
        L0 = np.abs(np.sum(leg_dimensions[leg][:, 1], axis=0))  # Offset of the feet in the y axis
        L1 = np.abs(leg_dimensions[leg][2, 2])                  # Length of the femur in the z axis
        L2 = np.abs(leg_dimensions[leg][3, 2])                  # Length of the tibia in the z axis

        [x, y, z] = feet_pos
        z_corr_2 = y**2 + z**2 - L0**2
        D = float(x**2 + y**2 + z**2 - L0**2 - L1**2 - L2**2) / (2 * L1 * L2)

        if D > 1 or z_corr_2 < 0:
            return False, np.array([0, 0, 0])

        z_corr = np.sqrt(z_corr_2)
        if 'l' in leg:
            a = np.pi - np.arctan2(-z, y) - np.arctan2(z_corr, -L0)
        else:
            a = np.arctan2(z, y) + np.arctan2(z_corr, -L0)

        c = np.arctan2(-np.sqrt(1 - D**2), D)
        b = -np.pi + np.arctan2(x, -z_corr) - np.arctan2(L2 * np.sin(c), L1 + L2 * np.cos(c))

        angles = np.array([a, b, c])
        return True, angles

    def getLegDimensions(self):
        urdf_model = r'/home/ajoymathew07/Catatron/Catatron//src/catatron_description/urdf/catatron.urdf'

        urdf_str = open(urdf_model, 'r').read()
        robot_urdf = URDF.from_xml_string(urdf_str)

        dimensions = {}
        leg_names = ['fl', 'fr', 'bl', 'br']
        joint_names = ['hip1', 'hip2', 'knee', 'feet']

        for name in leg_names:
            dimensions[name] = {}

        for joint in robot_urdf.joints:
            for joint_name in joint_names:
                if joint_name in joint.name:
                    name = joint.name.split('_')[1]
                    xyz = np.array(joint.origin.xyz, dtype='float32')
                    dimensions[name][joint_name] = xyz

        dimensions_leg = {}
        for key in dimensions:
            dimensions_leg[key] = np.array([dimensions[key][j] for j in joint_names])

        return dimensions_leg

    def send_goal(self, leg_angles):
        goal_msg = FollowJointTrajectory.Goal()

        joint_names = [
            "hip1_fr", "hip1_fl", "hip2_fr", "hip2_fl",
            "hip2_br", "hip1_bl", "hip2_bl", "knee_bl",
            "knee_fr", "hip1_br", "knee_fl", "knee_br"
        ]

        positions = self.current_joint_angles[:]
        for leg, angles in leg_angles.items():
            hip1_index = joint_names.index("hip1_" + leg)
            hip2_index = joint_names.index("hip2_" + leg)
            knee_index = joint_names.index("knee_" + leg)
            self.current_joint_angles[hip1_index] = positions[hip1_index] = angles[0]
            self.current_joint_angles[hip2_index] = positions[hip2_index] = angles[1]
            self.current_joint_angles[knee_index] = positions[knee_index] = angles[2]
        
        # msg = Float64MultiArray()
        # msg.data = positions
        # self.publisher.publish(msg)
        # logger.info(f"published  {positions}")
        self.set_angles(self.current_joint_angles)
        
        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = positions
        point1.time_from_start = Duration(seconds=5, nanoseconds=0).to_msg()
        points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=10, nanoseconds=0).to_msg()
        point2.positions = positions
        points.append(point2)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

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
        init_time = time.time()   
        self.get_logger().info(f'{time.time()-init_time}')   
        # self.publisher = self.create_publisher(Float64MultiArray, 'final_positions', 1)      
        # while time.time()-init_time < 10.0:
        #     msg = Float64MultiArray()
        #     msg.data = self.current_joint_angles
        #     self.publisher.publish(msg)
        #     time.sleep(0.1)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init()
    action_client = SteeringActionClient()
    logger.info(sys.argv)
    if len(sys.argv) < 9:
        logger.info("Usage: inv_kin.py <leg1> <x1> <y1> <z1> <leg2> <x2> <y2> <z2>")
        return
    leg1 = sys.argv[1]
    feet_pos1 = [float(arg) for arg in sys.argv[2:5]]

    leg2 = sys.argv[5]
    feet_pos2 = [float(arg) for arg in sys.argv[6:9]]
    legs = [leg1, leg2]
    feet_positions = [feet_pos1, feet_pos2]

    
    action_client.calculate_angles(legs, feet_positions)

if __name__ == '__main__':
    main() 