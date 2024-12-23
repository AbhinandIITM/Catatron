#!/usr/bin/env python3

import numpy as np
from enum import Enum

class BehaviorState(Enum):
    REST = 0
    TROT = 1
    CRAWL = 2
    STAND = 3

class State():
    def __init__(self,default_height):
        self.velocity = np.array([0., 0.])
        self.yaw_rate = 0.

        #initialized in CatatronGaitControl
        #changed to command.robot_height in specific controller
        self.robot_height = -default_height

        #from specific controller
        self.foot_locations = np.zeros((3,4))

        self.body_local_position = np.array([0., 0., 0.])
        self.body_local_orientation = np.array([0., 0., 0.])
        
        #from CatatronGaitControl
        self.imu_roll = 0.
        self.imu_pitch = 0.
        
        self.ticks = 0

        #from CatatronGaitControl
        self.behavior_state = BehaviorState.REST
    def __str__(self):
        return (f"height ={self.robot_height}, "
                f"foot_locations={self.foot_locations}, "
                f"imu_roll={self.imu_roll}, imu_pitch={self.imu_pitch}, "
                f"behavior_state={self.behavior_state}, ticks={self.ticks}),"
                f"velocity {self.velocity}"
                f"position {self.body_local_position}, local orientation {self.body_local_orientation}")

class Command(object):
    def __init__(self,default_height):

        #from arg_command in CatatronGaitControl
        self.velocity = np.array([0., 0.])
        self.yaw_rate = 0.

        #Initialized in CatatronGaitControl
        self.robot_height = -default_height
        self.trot_event = True
        self.crawl_event = False
        self.rest_event = False
        self.stand_event = False
    def __str__(self):
        return (f"height ={self.robot_height}, "
                f"velocity {self.velocity}"
                f"trot event {self.trot_event} crawl event {self.crawl_event} rest_event {self.rest_event} stand_Event {self.stand_event}"
                f"yaw rate {self.yaw_rate} ")
