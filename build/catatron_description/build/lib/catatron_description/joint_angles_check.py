#!/usr/bin/env python3
import rclpy
import logging
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

logging.basicConfig(
    level=logging.INFO,  # Change to DEBUG for detailed logs
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

class JointAnglesCheck(Node):
    def __init__(self):
        super().__init__('joint_angles_check')
        self.subscription()  # Call the subscription method
    
    def callback_func(self, msg):
        # Callback function to process incoming messages
        self.get_logger().info(f"Received message: {msg.data}")
    
    def subscription(self):
        # Correct method name, use self.callback_func as the callback
        self.create_subscription(
            Float64MultiArray,  # Message type
            '/final_positions', # Topic name
            self.callback_func, # Callback function
            10                  # QoS profile (default)
        )

def main(args=None):
    rclpy.init(args=args)
    joint_angles_check = JointAnglesCheck()

    # Spin the node to keep it running
    rclpy.spin(joint_angles_check)

if __name__ == '__main__':
    main()
