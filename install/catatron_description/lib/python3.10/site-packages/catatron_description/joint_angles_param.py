#!/usr/bin/env python3
import rclpy
import logging
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from time import sleep

logging.basicConfig(
    level=logging.INFO,  # Change to DEBUG for detailed logs
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

class JointAnglesParam(Node):

    def __init__(self):
        super().__init__('joint_angles_param')
        
        # Correct service name for GetParameters
        self.cli = self.create_client(GetParameters, '/steering_action_client/get_parameters')  # Adjust service name
        
        # Wait for the service to be available and check periodically
        self.wait_for_service()
        self.request = GetParameters.Request()
        
    def wait_for_service(self):
        # Wait for the service to be available, retrying every 1 second
        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().info('Service not available, retrying...')
            sleep(0.3)  # Wait a bit before retrying

    def send_request(self, param):
        self.request.names = param
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    joint_angles_node = JointAnglesParam()
    
    # Send request to get joint angles (or any other parameter you need)
    response = joint_angles_node.send_request(['joint_angles'])
    
    # Check if response is valid and print out the values
    if response:
        joint_angles_node.get_logger().info(f"Response values: {response.values}")
    else:
        joint_angles_node.get_logger().info("Failed to get parameters.")
    
    # Spin to keep the node alive and process callbacks
    rclpy.spin(joint_angles_node)

if __name__ == '__main__':
    main()
