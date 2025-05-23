# Catatron ROS 2 Simulation

This repository contains the ROS 2 packages for simulating and controlling the Catatron quadruped robot in Gazebo. It is compatible with ROS 2 Humble.

---

## 📁 Workspace Setup

```bash
mkdir -p ~/catatron_ws/src
cd ~/catatron_ws/src
git clone https://github.com/morg1207/catatron_ros2.git

git checkout catatron_ros2
```

## 📦 Install Dependencies and Build
```bash
cd ~/catatron_ws
rosdep init
sudo apt update
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install

sudo apt install pip -y
pip install pynput 
```
## 🚀 Run the Simulation
<br>
🖥️ Terminal 1

```bash
cd ~/catatron_ws/
source install/setup.bash
ros2 launch catatron_gazebo catatron_gazebo.launch.py
```
🕹️ Terminal 2

```bash
cd ~/catatron_ws/
source install/setup.bash
ros2 run catatron_joystick keyboard_sim_joy
```

# 📚 Citations and References
---

### 🔹 Notspot Robot Simulation - Python Version
- **Author**: lnotspotl  
- **Repository**: [GitHub - notspot_sim_py](https://github.com/lnotspotl/notspot_sim_py)  
- **Year**: 2021  
- **BibTeX**:
  ```
  @misc{notspot_sim_py,
    author       = {lnotspotl},
    title        = {Notspot Robot Simulation - Python Version},
    year         = {2021},
    publisher    = {GitHub},
    howpublished = {\url{https://github.com/lnotspotl/notspot_sim_py}}
  }

  
- **Author**: Andres
- **Repository**: [GitLab](https://gitlab.com/anflores/pavlov_mini)  
- **Year**: 2021  
- **BibTeX**:
  ```
  @misc{pavlov_mini,
    author       = {Andres},
    title        = {pavlov_mini},
    year         = {2021},
    publisher    = {GitHub},
    howpublished = {\url{https://gitlab.com/anflores/pavlov_mini}}
  }

