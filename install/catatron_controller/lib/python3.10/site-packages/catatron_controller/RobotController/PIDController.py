#!/usr/bin/env python3
# Author: lcatatronl

import rclpy
from rclpy.time import Time
from rclpy.clock import Clock
import numpy as np

import rclpy.clock
class PID_controller(object):
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # desired roll and pitch angles
        # (we don't really care about yaw)
        self.desired_roll_pitch = np.array([0.0,0.0])

        self.I_term = np.array([0.0,0.0])
        self.D_term = np.array([0.0,0.0])

        # TODO: Tune max_I
        self.max_I = 0.2
        self.last_error = np.array([0.0,0.0])

    def run(self, roll, pitch):
        # determine error
        error = self.desired_roll_pitch - np.array([roll, pitch])
        # determine time step
        clock = Clock()
        t_now = clock.now()
        t_now_sec, t_now_nanosec = t_now.seconds_nanoseconds()
        
        last_time_sec, last_time_nanosec = self.last_time.seconds_nanoseconds()

        sec_diff = t_now_sec - last_time_sec
        nanosec_diff = t_now_nanosec - last_time_nanosec
        if nanosec_diff < 0:
            sec_diff -= 1
            nanosec_diff += 1e9  # Sumamos 1 segundo en nanosegundos

        # Ahora calculamos el paso de tiempo en segundos
        step = sec_diff + (nanosec_diff / 1e9)
        # I term update
        self.I_term = self.I_term + error * step

        # anti-windup
        for i in range(2):
            if(self.I_term[i] < -self.max_I):
                self.I_term[i] = -self.max_I
            elif(self.I_term[i] > self.max_I):
                self.I_term[i] = self.max_I

        # approximate first derivate
        self.D_term = (error - self.last_error) / step

        # update last values 
        self.last_time = t_now
        self.last_error = error

        # compute return values
        P_ret = self.kp * error
        I_ret = self.I_term * self.ki
        D_ret = self.D_term * self.kd

        return P_ret + I_ret + D_ret

    def reset(self):
        clock = Clock()
        self.last_time = clock.now()
        self.I_term = np.array([0.0,0.0])
        self.D_term = np.array([0.0,0.0])
        self.last_error = np.array([0.0,0.0])

    def desired_RP_angles(self,des_roll, des_pitch):
        # set desired roll and pitch angles
        self.desired_roll_pitch = np.array([des_roll, des_pitch])
