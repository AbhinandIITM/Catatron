#!/usr/bin/env python3
# Author: lcatatronl

import rclpy
from rclpy.node import Node
from math import fabs
from numpy import array_equal

from sensor_msgs.msg import Joy


class PS4Controller(Node):
    def __init__(self, rate):
        super().__init__("joystick_ramped")

        # Suscriptor y publicador
        self.subscription = self.create_subscription(Joy, "joy", self.callback, 10)
        self.publisher = self.create_publisher(Joy, "catatron_joy/joy_ramped", 10)

        # Frecuencia de publicación
        self.timer = self.create_timer(1.0 / rate, self.publish_joy)

        # Inicialización de los mensajes Joy
        self.target_joy = Joy()
        self.target_joy.axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.target_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.last_joy = Joy()
        self.last_joy.axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.last_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.last_send_time = self.get_clock().now()

        self.use_button = True
        self.speed_index = 2
        self.available_speeds = [0.5, 1, 3.0, 4.0]

    def callback(self, msg):
        if self.use_button:
            if msg.buttons[4]:
                self.speed_index -= 1
                if self.speed_index < 0:
                    self.speed_index = len(self.available_speeds) - 1
                self.get_logger().info(f"Joystick speed: {self.available_speeds[self.speed_index]}")
                self.use_button = False
            elif msg.buttons[5]:
                self.speed_index += 1
                if self.speed_index >= len(self.available_speeds):
                    self.speed_index = 0
                self.get_logger().info(f"Joystick speed: {self.available_speeds[self.speed_index]}")
                self.use_button = False

        if not self.use_button:
            if not (msg.buttons[4] or msg.buttons[5]):
                self.use_button = True

        self.target_joy.axes = msg.axes
        self.target_joy.buttons = msg.buttons

    def ramped_vel(self, v_prev, v_target, t_prev, t_now):
        step = (t_now - t_prev).nanoseconds / 1e9  # Convertir nanosegundos a segundos
        sign = self.available_speeds[self.speed_index] if (v_target > v_prev) else -self.available_speeds[self.speed_index]
        error = fabs(v_target - v_prev)

        if error < self.available_speeds[self.speed_index] * step:
            return v_target
        else:
            return v_prev + sign * step

    def publish_joy(self):
        t_now = self.get_clock().now()

        # Determinar cambios en el estado
        buttons_change = array_equal(self.last_joy.buttons, self.target_joy.buttons)
        axes_change = array_equal(self.last_joy.axes, self.target_joy.axes)

        if not (buttons_change and axes_change):
            joy = Joy()

            if not axes_change:
                joy.axes = []
                for i in range(len(self.target_joy.axes)):
                    if self.target_joy.axes[i] == self.last_joy.axes[i]:
                        joy.axes.append(self.last_joy.axes[i])
                    else:
                        joy.axes.append(
                            self.ramped_vel(self.last_joy.axes[i], self.target_joy.axes[i], self.last_send_time, t_now)
                        )
            else:
                joy.axes = self.last_joy.axes

            joy.buttons = self.target_joy.buttons
            self.last_joy = joy
            self.publisher.publish(self.last_joy)

        self.last_send_time = t_now


def main(args=None):
    rclpy.init(args=args)
    joystick = PS4Controller(rate=30)

    try:
        rclpy.spin(joystick)
    except KeyboardInterrupt:
        pass
    finally:
        joystick.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
