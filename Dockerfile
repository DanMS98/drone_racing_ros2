FROM ros:humble-ros-base


ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        python3-colcon-common-extensions \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-joy \
        git \
        wget \
        nano \
        build-essential \
        libasio-dev \
        ros-humble-cv-bridge \
        ros-humble-camera-calibration-parsers \
        ros-humble-gazebo-dev \
        ros-humble-gazebo-ros* \
        libignition-rendering6 \
        && rm -rf /var/lib/apt/lists/*


WORKDIR /root/ros2_ws/src

# Clone the drone racing ROS2 package
RUN git clone https://github.com/DanMS98/drone_racing_ros2.git

# Install Python dependencies
COPY requirements.txt /root/ros2_ws/
RUN pip3 install --no-cache-dir -r /root/ros2_ws/requirements.txt


WORKDIR /root/ros2_ws

RUN . /opt/ros/humble/setup.sh && \
    colcon build

RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Default command
CMD ["bash"]