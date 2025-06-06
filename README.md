# `drone_racing_ros2`

## Running a Tello simulation in [Gazebo](http://gazebosim.org/)

`tello_gazebo` consists of several components:
* `TelloPlugin` simulates a drone, handling takeoff, landing, and basic flight dynamics
* `markers` contains Gazebo models for fiducial markers
* `fiducial.world` is a simple world with multiple fiducial markers
* `inject_entity.py` is a script to spawn a model (URDF or SDF) in a running Gazebo instance
* the built-in camera plugin is used to emulate the Gazebo forward-facing camera

---

## Installation

### 1️⃣ Install ROS 2 Humble
Follow the instructions at:
[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)  
Use the `ros-humble-desktop` option for full desktop tools.

---

### 2️⃣ Install dependencies
```bash
sudo apt update
sudo apt install \
  gazebo11 libgazebo11 libgazebo11-dev \
  libasio-dev \
  ros-humble-cv-bridge \
  ros-humble-camera-calibration-parsers \
  libignition-rendering6 \
  python3-transformations



#### Build this package
    mkdir -p ~/drone_racing_ros2_ws/src
    cd ~/drone_racing_ros2_ws/src
    git clone https://github.com/TIERS/drone_racing_ros2.git
    cd ..
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    
#### Run a teleop simulation

   cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    ros2 launch tello_gazebo simple_launch.py
    
You will see a single drone in a blank world.
You can control the drone using the joystick.

If you run into the **No namespace found** error re-set `GAZEBO_MODEL_PATH`:

    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    

#### Control the drone 
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1






