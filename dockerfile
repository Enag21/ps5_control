FROM ros:humble-ros-base-jammy

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set the DISPLAY environment variable
ENV DISPLAY=host.docker.internal:0.0

# Create workspace directory and set it as working directory
WORKDIR /home

# Main dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-dev-tools \
    sudo \
    cmake \
    x11-apps \
    mesa-utils \
    python3-pip \
    libhidapi-dev \
    bluez \
    bluetooth \
    libbluetooth-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pydualsense


# PX4 Autopilot
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    apt-get update && apt-get install -y sudo && \
    ./PX4-Autopilot/Tools/setup/ubuntu.sh && make -C PX4-Autopilot px4_sitl


# Setup Micro XRCE-DDS Agent & Client
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && cmake .. && make && \
    sudo make install && sudo ldconfig /usr/local/lib/

# Create base ROS workspace
RUN mkdir -p /home/ros2_ws/src/ && cd /home/ros2_ws/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/Enag21/ps5_control.git && \
    . /opt/ros/humble/setup.sh && \
    cd .. && colcon build && . install/setup.sh

RUN apt-get update && apt-get install -y dbus

CMD ["/bin/bash"]
