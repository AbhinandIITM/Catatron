#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pynput import keyboard

class SimulatedJoystick(Node):
    def __init__(self):
        super().__init__("simulated_joystick")
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        
        # Initialize axes and buttons
        self.axes = [0] * 8  # 6 axes with range -125 to 126
        self.buttons = [0] * 8  # 6 digital buttons

        # Axis step size
        self.axis_step = 5

        # Keyboard key mappings
        self.key_mapping = {
            # Axis 0 control
            "l": (0, 1),  # Increase axis[0] yaw
            "j": (0, -1), # Decrease axis[0]
            # Axis 1 control
            "i": (1, 1),        
            ",": (1, -1),
            # Axis 2 control
            "u": (2, 1),
            "m": (2, -1),
            # Axis 3 control
            "d": (3, 1),
            "a": (3, -1),
            # Axis 4 control
            "w": (4, 1),        # eje x
            "x": (4, -1),       
            # Axis 5 control 
            "y": (5, 1),        # eje y
            "h": (5, -1),
            # Axis 6 control 
            "t": (6, 1),        # eje y
            "g": (6, -1),
            # Axis 7 control 
            "r": (7, 2),        # eje y
            "f": (7, -2),
            # Buttons control
            "1": (8, 1),  # Button 0
            "2": (9, 1),  # Button 1
            "3": (10, 1),  # Button 2
            "4": (11, 1),  # Button 3
            "5": (12, 1), # Button 4
            "6": (13, 1)  # Button 5
        }

        # Listener for keyboard inputs
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Timer for publishing Joy messages
        self.timer = self.create_timer(0.1, self.publish_joy)  # 10 Hz

    def on_press(self, key):
        try:
            key_char = key.char  # Get the character of the pressed key
            if key_char in self.key_mapping:
                axis_or_button, direction = self.key_mapping[key_char]
                if axis_or_button < 8:  # Axis control
                    self.axes[axis_or_button] = max(
                        -125, min(126, self.axes[axis_or_button] + direction * self.axis_step)
                    )
                else:  # Button control
                    button_index = axis_or_button - 8
                    self.buttons[button_index] = 1  # Set button as pressed
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            key_char = key.char
            if key_char in self.key_mapping:
                axis_or_button, _ = self.key_mapping[key_char]
                if axis_or_button >= 8:  # Button control
                    button_index = axis_or_button - 8
                    self.buttons[button_index] = 0  # Reset button to unpressed
        except AttributeError:
            pass

    def publish_joy(self):
        # Create and publish Joy message
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [float(value) / 126 for value in self.axes]  # Normalize axes to [-1, 1]
        joy_msg.buttons = self.buttons
        self.joy_pub.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    joystick = SimulatedJoystick()
    try:
        rclpy.spin(joystick)
    except KeyboardInterrupt:
        pass
    joystick.listener.stop()
    joystick.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
