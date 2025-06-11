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
first install gazebo for ros humble
    
    source /opt/ros/humble/setup.bash 
    sudo apt-get install ros-${ROS_DISTRO}-ros-gz

after that
    
    sudo apt update
    sudo apt install \
    libasio-dev \
    ros-humble-cv-bridge \
    ros-humble-camera-calibration-parsers \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-ros* \
    libignition-rendering6 

Make a folder as a workspace, inside that folder, make a virtual environment:

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    python -m venv venv
    source venv/bin/activate

#### Build this package
    
    cd ~/ros2_ws/src
    git clone git@github.com:DanMS98/drone_racing_ros2.git
    cd drone_racing_ros2/
    pip install -r requirements.txt
    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build
    
### 3️⃣ Run a teleop simulation

    cd ~/ros2_ws
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
## Instructions for using Docker
There is also an image prepared for this repo to streamline the process. This image also includes the [Tentone Tello-ROS2 repository](https://github.com/tentone/tello-ros2)
 for working with a real Tello.

### The Image  
To build the image, go to the root of the repo run the Docker file using the following command:

    docker build -t drone_racing_ros2:humble .

Before running Docker, make sure to tell your X11 server to allow local connections from root (the user inside the container).

    xhost +local:root

afterwards

    docker run -it --rm \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    drone_racing_ros2:humble


    










