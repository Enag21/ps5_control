# ps5_control

In this project I aim to implement multimodal feedback to see how it can help with drone teleoperation using a PS5 controller. 

## Main resources:

- [ROS2](https://docs.ros.org/en/humble/index.html)
- [PX4](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [ros_gz](https://github.com/gazebosim/ros_gz.git)
- [pydualsense](https://github.com/flok/pydualsense)

## How to setup:

ROS2 and PX4 are needed: [ROS2 and PX4 installation guide](https://docs.px4.io/main/en/ros/ros2_comm.html)

Create a ROS2 workspace:

```
mkdir -p ~/ros_ws/src/
cd ~/ros_ws/src/
```
Clone px4_msgs and this repository to `\src` 
```
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/Enag21/ps5_control.git
```

Source the ROS 2 development environment into the current terminal and compile the workspace using colcon:

```
cd ..
source /opt/ros/humble/setup.bash
colcon build
```
Set an Environment Variable for the PX4 Directory in `.bashrc`:
```
export PX4_AUTODIR=$HOME/path/to/your/PX4-Autopilot
```
Got to the ROS2 workspace and run:
```
source install/setup.bash
```
Copy `custom.sdf`file and paste it inside `/PX4-Autopilot/Tools/simulation/gz/worlds/`.
## How to use:

Run the ROS2 launch:

```
ros2 launch ps5_control offboard_ps5_control.launch.py
```

### Controller settings:

| Key            | Function                              |
|----------------|---------------------------------------|
| ○              | Switch to offboard mode               |
| □              | Switch to landing mode                |
| Left joystick  | Control movement direction (XY Plane) |
| Right joystick | Control heading rotation and altitude |
| R1             | Block\unblock altitude                |
| L1             | Block\unblock heading rotation        |
| R2             | Move                                  |
