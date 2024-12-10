#!/usr/bin/env python3
# Author: lnotspotl

import rclpy
import numpy as np
from rclpy.clock import Clock

class PID():
    def __init__(self, kp, ki, kd, clock=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Use the provided clock or create a new one
        self.clock = clock if clock else Clock()

        # Desired roll and pitch angles
        self.desired_roll_pitch = np.array([0.0, 0.0])

        self.I_term = np.array([0.0, 0.0])
        self.D_term = np.array([0.0, 0.0])

        # TODO: Tune max_I
        self.max_I = 0.2
        self.last_error = np.array([0.0, 0.0])
        self.last_time = self.clock.now()  # Initialize the last_time

    def run(self, roll, pitch):
        # Determine error
        error = self.desired_roll_pitch - np.array([roll, pitch])

        # Determine time step
        t_now = self.clock.now()
        step = (t_now - self.last_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds

        if step <= 0:
            # Handle edge case of no time progression
            return np.array([0.0, 0.0])

        # I term update
        self.I_term += error * step

        # Anti-windup
        self.I_term = np.clip(self.I_term, -self.max_I, self.max_I)

        # Approximate first derivative
        self.D_term = (error - self.last_error) / step

        # Update last values
        self.last_time = t_now
        self.last_error = error

        # Compute return values
        P_ret = self.kp * error
        I_ret = self.I_term * self.ki
        D_ret = self.D_term * self.kd

        return P_ret + I_ret + D_ret

    def reset(self):
        self.last_time = self.clock.now()
        self.I_term = np.array([0.0, 0.0])
        self.D_term = np.array([0.0, 0.0])
        self.last_error = np.array([0.0, 0.0])

    def desired_RP_angles(self, des_roll, des_pitch):
        # Set desired roll and pitch angles
        self.desired_roll_pitch = np.array([des_roll, des_pitch])
