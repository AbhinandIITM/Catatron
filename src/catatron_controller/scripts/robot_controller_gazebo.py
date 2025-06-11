#!/usr/bin/env python3
#Author: lcatatronl

import rospy

from sensor_msgs.msg import Joy,Imu
from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64

USE_IMU = False
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.1834, 0.1071]
legs = [0.0, 0.03559, 0.09996, 0.1] 

catatron_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

command_topics = ["/catatron_controller/FR1_joint/command",
                  "/catatron_controller/FR2_joint/command",
                  "/catatron_controller/FR3_joint/command",
                  "/catatron_controller/FL1_joint/command",
                  "/catatron_controller/FL2_joint/command",
                  "/catatron_controller/FL3_joint/command",
                  "/catatron_controller/RR1_joint/command",
                  "/catatron_controller/RR2_joint/command",
                  "/catatron_controller/RR3_joint/command",
                  "/catatron_controller/RL1_joint/command",
                  "/catatron_controller/RL2_joint/command",
                  "/catatron_controller/RL3_joint/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

if USE_IMU:
    rospy.Subscriber("catatron_imu/base_link_orientation",Imu,catatron_robot.imu_orientation)
rospy.Subscriber("catatron_joy/joy_ramped",Joy,catatron_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

while not rospy.is_shutdown():
    leg_positions = catatron_robot.run()
    catatron_robot.change_controller()

    dx = catatron_robot.state.body_local_position[0]
    dy = catatron_robot.state.body_local_position[1]
    dz = catatron_robot.state.body_local_position[2]
    
    roll = catatron_robot.state.body_local_orientation[0]
    pitch = catatron_robot.state.body_local_orientation[1]
    yaw = catatron_robot.state.body_local_orientation[2]

    try:
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                               dx, dy, dz, roll, pitch, yaw)
        # print(joint_angles)
        for i in range(len(joint_angles)):
            
            publishers[i].publish(joint_angles[i])
    except:
        pass

    rate.sleep()
