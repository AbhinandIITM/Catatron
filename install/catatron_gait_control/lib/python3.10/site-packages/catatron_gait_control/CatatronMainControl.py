import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from catatron_gait_control.CatatronGaitControl import CatatronGaitControl
from catatron_gait_control.IK import InverseKinematics
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
import threading

USE_IMU = True
RATE = 60  # Hertz

class CatatronMainControl(Node):
    def __init__(self):
        super().__init__("CatatronMainControl")

        self.callback_group1 = MutuallyExclusiveCallbackGroup()
        self.callback_group2 = MutuallyExclusiveCallbackGroup()
        self.callback_group3 = MutuallyExclusiveCallbackGroup()
        self.callback_group4 = MutuallyExclusiveCallbackGroup()
        self.callback_group5 = MutuallyExclusiveCallbackGroup()
        self.callback_group6 = MutuallyExclusiveCallbackGroup()

        body = [0.1908, 0.080]
        legs = [0.0, 0.04, 0.100, 0.094333]
        #tochange
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.client = self.create_client(GetParameters, '/joint_angles_param/get_parameters')
        self.set_client = self.create_client(SetParameters, '/joint_angles_param/set_parameters')
        self.catatron_gait_control = CatatronGaitControl(body, legs, USE_IMU)
        self.inverseKinematics = InverseKinematics(body, legs)
        
        if USE_IMU:
            self.create_subscription(Imu, "/imu_plugin/out", self.catatron_gait_control.imu_orientation, 10)
        self.create_subscription(Joy, "/virtual_joystick", self.catatron_gait_control.arg_command, 10)
        
        # Creating a separate thread for the timer callback
        self.create_timer(0.6, self.control_loop, callback_group =  self.callback_group1)
        
    def get_joint_angles(self):
        request = GetParameters.Request()
        request.names = ['joint_angles'] 
        
        # Wait for the response
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            angles = response.values[0].double_array_value  
            # self.get_logger().info(f"{angles} received.")
            return angles
            
        else:
            self.get_logger().error('Failed to fetch joint angles parameter')
            return [0.0] * 12
    
    def set_angles(self, angles):
        param_request = SetParameters.Request()
        param_value = Parameter()
        param_value.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        param_value.value.double_array_value = angles  
        param_request.parameters = [param_value]

        future = self.set_client.call_async(param_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Updated joint angles to: {angles}")
        else:
            self.get_logger().error("Failed to set joint angles.")

    def control_loop(self):
        self.get_logger().info("control loop running")
        while rclpy.ok():
            try:
                
                leg_positions= self.catatron_gait_control.run()
                self.catatron_gait_control.change_controller()
                
                dx = self.catatron_gait_control.state.body_local_position[0]
                dy = self.catatron_gait_control.state.body_local_position[1]
                dz = self.catatron_gait_control.state.body_local_position[2]

                roll = self.catatron_gait_control.state.body_local_orientation[0]
                pitch = self.catatron_gait_control.state.body_local_orientation[1]
                yaw = self.catatron_gait_control.state.body_local_orientation[2]
                joint_angles = self.inverseKinematics.inverse_kinematics(leg_positions=leg_positions,dx=dx, dy=dy, dz=dz, roll=roll, pitch=pitch, yaw=yaw)

                # self.get_logger().info(f"goal angles {joint_angles}")
                self.send_goal(joint_angles)
               
            
            except Exception as e:
                self.get_logger().error(f"Error in control loop: {e}")
                
            rclpy.spin(self)  # Ensure that we handle the events and callbacks

    def send_goal(self, angles):
        goal_msg = FollowJointTrajectory.Goal()
        # print(angles)
        joint_names = [
            "hip1_fr", "hip1_fl", "hip2_fr", "hip2_fl",
            "hip2_br", "hip1_bl", "hip2_bl", "knee_bl",
            "knee_fr", "hip1_br", "knee_fl", "knee_br"
        ]     
        points = []
        
        point1 = JointTrajectoryPoint()
        point1.positions = self.get_joint_angles()
        points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=3, nanoseconds=0).to_msg()
        point2.positions = angles 
        points.append(point2)
        self.set_angles(angles)
        goal_msg.goal_time_tolerance = Duration(seconds=3, nanoseconds=0).to_msg()
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
        # rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init(args=args)
    node = CatatronMainControl()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        
        # rclpy.spin(node)
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the node is destroyed properly before shutdown
        if 'node' in locals():  # Check if the node was created
            node.destroy_node()

        
        # Properly shutdown the node
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
