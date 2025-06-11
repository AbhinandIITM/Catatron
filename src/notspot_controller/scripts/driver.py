#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import Adafruit_PCA9685  # Library to control the PCA9685
from mpu6050 import mpu6050  # Library to interface with MPU6050
import math
import tf.transformations as tf

from RobotController import RestController


class RobotDriverNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('robot_driver_node', anonymous=True)

        # Initialize PCA9685
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)  # Set frequency to 50 Hz for servos

        # # Initialize MPU6050 sensor
        # self.imu = mpu6050(0x68)  # Default I2C address of MPU6050
        # rospy.loginfo("MPU6050 initialized successfully.")

        # Joint angles (12 servos assumed)
        self.joint_angles = [0] * 12  # Initial positions
        self.sign=[-0.5,-0.5,0.5,-0.5,0.5,-0.5,0.5,-0.5,0.5,0.5,0.5,-0.5]
        self.off= [-0.1,0.25,0.15,0.15,0.25,0.15,0,0.25,0.15,0,0.25,0.15]

        # Subscribe to joint command topics
        self.command_topics = [
            "/notspot_controller/FR1_joint/command",
            "/notspot_controller/FR2_joint/command",
            "/notspot_controller/FR3_joint/command",
            "/notspot_controller/FL1_joint/command",
            "/notspot_controller/FL2_joint/command",
            "/notspot_controller/FL3_joint/command",
            "/notspot_controller/RR1_joint/command",
            "/notspot_controller/RR2_joint/command",
            "/notspot_controller/RR3_joint/command",
            "/notspot_controller/RL1_joint/command",
            "/notspot_controller/RL2_joint/command",
            "/notspot_controller/RL3_joint/command",
        ]

        for i, topic in enumerate(self.command_topics):
            rospy.Subscriber(topic, Float64, self.joint_command_callback, callback_args=i)

        # # Publish IMU data
        # self.imu_publisher = rospy.Publisher("notspot_imu/base_link_orientation", Imu, queue_size=10)

        # # Timer to read and publish IMU data
        # rospy.Timer(rospy.Duration(0.04), self.publish_imu_data)  # 25 Hz

        self.rest_controller = RestController.RestController(0.15)
        self.nod_timer = rospy.Timer(rospy.Duration(5), self.nod_callback)


    def joint_command_callback(self, msg, joint_index):
        """Callback for receiving joint angles."""
        self.joint_angles[joint_index] = msg.data
        self.update_servo(joint_index, msg.data)

    def update_servo(self, joint_index, angle):
        """Update the servo position based on joint angle."""
        # print(f"angle received are :{angle}")
        angle_degrees=self.sign[joint_index]*math.degrees(angle+self.off[joint_index])
        # print(angle_degrees)
        #angle_degrees+=90.0;
        min_pulse = 150
        max_pulse = 600
        pulse = int(min_pulse + (angle_degrees + 90) / 180.0 *(max_pulse - min_pulse))
        self.pwm.set_pwm(joint_index, 0, pulse)
    
    def nod_callback(self, event):
        """Perform a nodding motion."""
        if self.rest_controller.Nod:
            head_joint_index = 12  # Replace with the actual joint index for the head nod
            nod_angles = [90,100,80,90]  # Replace with actual angles in degrees for nodding motion

            # Simulate the nodding motion
            # for angle in nod_angles:
            #     self.update_servo(head_joint_index,angle)
            #     rospy.sleep(0.5)  # Delay between each movement

    def calculate_orientation(self, accel, gyro):
        """Calculate roll, pitch, and yaw from accelerometer and gyroscope data."""
        # Calculate roll and pitch from accelerometer data
        accel_x, accel_y, accel_z = accel['x'], accel['y'], accel['z']
        roll = math.atan2(accel_y, accel_z)
        pitch = math.atan2(-accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2))

        # Yaw estimation would require a magnetometer or fusion algorithm
        yaw = 0.0  # Placeholder

        return roll, pitch, yaw

    def publish_imu_data(self, event):
        """Publish IMU data from the sensor."""
        try:
            # Read accelerometer and gyroscope data
            accel_data = self.imu.get_accel_data()
            gyro_data = self.imu.get_gyro_data()

            # Calculate orientation (roll, pitch, yaw)
            roll, pitch, yaw = self.calculate_orientation(accel_data, gyro_data)

            # Convert roll, pitch, yaw to quaternion
            quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

            # Prepare IMU message
            msg = Imu()
            msg.orientation.x = quaternion[0]
            msg.orientation.y = quaternion[1]
            msg.orientation.z = quaternion[2]
            msg.orientation.w = quaternion[3]
            msg.angular_velocity.x = gyro_data['x']
            msg.angular_velocity.y = gyro_data['y']
            msg.angular_velocity.z = gyro_data['z']
            msg.linear_acceleration.x = accel_data['x']
            msg.linear_acceleration.y = accel_data['y']
            msg.linear_acceleration.z = accel_data['z']

            # Log RPY for debugging
            rospy.loginfo(f"IMU Roll: {math.degrees(roll)}, Pitch: {math.degrees(pitch)}, Yaw: {math.degrees(yaw)}")

            # Publish the IMU message
            self.imu_publisher.publish(msg)

        except Exception as e:
            rospy.logerr(f"Failed to read IMU data: {e}")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        driver_node = RobotDriverNode()
        driver_node.run()
    except rospy.ROSInterruptException:
        pass
