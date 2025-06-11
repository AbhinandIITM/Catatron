## Project Catatron
📘 About This Project

This repository was developed as part of Project Catatron, under the Tech Ambience Vertical, Envisage, Centre for Innovation (CFI), IIT Madras.
It contains the complete codebase for simulating and controlling a quadruped robot, including the driver code and tests for hardware integration and deployment on the physical robot.


# Initialize catkin workspace
cd src
catkin_init_workspace

# Build the workspace
cd ..
catkin_make

# Source the workspace setup script
source devel/setup.bash

# Make controller script executable
roscd catatron_controller/scripts
chmod +x robot_controller_gazebo.py

# Copy the RoboticsUtilities package to site-packages
cp -r RoboticsUtilities ~/.local/lib/python3.8/site-packages

# Make joystick script executable
roscd catatron_joystick/scripts
chmod +x ramped_joystick.py

cd ../../../

# Source the setup script again (in case the terminal was restarted)
source devel/setup.bash

# Launch the robot simulation
roslaunch notspot run_robot_gazebo.launch


## Gazebo Simulation <br>

![image](https://github.com/user-attachments/assets/f2f8f649-9c1d-4a15-9fcf-f476449d6451) <br>


## 3D printed robot <br>
<img src="https://github.com/user-attachments/assets/a7ca6a13-2a5d-4a5c-bd24-97a7b8d21124" width="500"/>  <br>


### Trot motion <br>
![](https://github.com/AbhinandIITM/Catatron/blob/main/trot.gif) <br>



