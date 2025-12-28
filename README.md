# Final Year Project: Portable Laser Scanner

This repository contains the source code and hardware interfacing for my Final Year Project. It includes a ROS 2 package for laser scanning and Python scripts for Arduino-based motor control.

## Repository Structure
- **portable_laser_scanner**: ROS 2 package containing URDF models, launch files, and scanning nodes.
- **ros_arduino**: Scripts for serial communication and motor control via Arduino.

---

## Installation & Setup

To use these packages in your own ROS 2 workspace, follow these steps:

### 1. Clone the Repository
Navigate to the `src` folder of your ROS 2 workspace (e.g., `ros2_ws`):

```bash
cd ~/ros2_ws/src
git clone [https://github.com/Lucifers1/final_year_project.git](https://github.com/Lucifers1/final_year_project.git)
```

### 2. Install Dependencies
```bash
pip install pyserial
sudo apt update
rosdep install -i --from-path . --rosdistro $ROS_DISTRO -y
```

### 3. Build the Workspace
```bash
cd ~/ros2_ws
colcon build --packages-select portable_laser_scanner ros_arduino
source install/setup.bash
```
