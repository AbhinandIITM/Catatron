#!/usr/bin/env python3
import rclpy
import logging
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

logging.basicConfig(
    level=logging.INFO,  
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

class JointAnglesCheck(Node):
    def __init__(self):
        super().__init__('joint_angles_check')
        self.subscription() 
        
    def callback_func(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")

    def subscription(self):
        self.create_subscription(
            Float64MultiArray,  
            '/final_positions', 
            self.callback_func, 
            10                  
        )

def main(args=None):
    rclpy.init(args=args)
    joint_angles_check = JointAnglesCheck()
    rclpy.spin(joint_angles_check)

if __name__ == '__main__':
    main()
