#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import argparse
from sensor_msgs.msg import Joy
import sys

class ArgJoy(Node):
    def __init__(self, rate, axes, buttons):
        super().__init__("Joy")

        self.publisher = self.create_publisher(Joy, "/joy", 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_joy)

        self.joy_msg = Joy()
        self.joy_msg.axes = axes
        self.joy_msg.buttons = buttons

    def publish_joy(self):
        self.publisher.publish(self.joy_msg)
        self.get_logger().info(f"Published Joy Message: Axes={self.joy_msg.axes}, Buttons={self.joy_msg.buttons}")
        print("ArgJoy published successfully")


def main(args=None):
    parser = argparse.ArgumentParser(description="Simulate joystick inputs.")
    parser.add_argument(
        "--rate", type=int, default=10, help="Publishing rate in Hz (default: 10)"
    )
    parser.add_argument(
        "--axes", 
        type=float, 
        nargs="*", 
        default=[0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        help="List of joystick axes values (default: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0])",
    )
    parser.add_argument(
        "--buttons", 
        type=int, 
        nargs="*", 
        default=[0] * 11,
        help="List of joystick button states (default: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])",
    )
    parsed_args = parser.parse_args()

    # Pass a list of arguments (sys.argv) to rclpy.init()
    rclpy.init(args=sys.argv)

    simulated_joystick = ArgJoy(rate=parsed_args.rate, axes=parsed_args.axes, buttons=parsed_args.buttons)

    try:
        rclpy.spin(simulated_joystick)
    except KeyboardInterrupt:
        pass
    simulated_joystick.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
