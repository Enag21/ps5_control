from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the PX4 Autopilot directory from the environment variable
    px4_autopilot_dir = os.getenv('PX4_AUTODIR')
    if px4_autopilot_dir is None:
        raise EnvironmentError("PX4_AUTODIR environment variable not set")

    # Command to run MicroXRCEAgent in a new tab
    start_microxrceagent = ExecuteProcess(
        cmd=['gnome-terminal', '--tab', '--', 'bash', '-c', 'MicroXRCEAgent udp4 -p 8888; bash'],
        output='screen'
    )

    # Command to start PX4 simulation in a new tab
    start_px4_simulation = ExecuteProcess(
        cmd=['gnome-terminal', '--tab', '--', 'bash', '-c', 'cd ' + px4_autopilot_dir + ' && PX4_GZ_WORLD=custom make px4_sitl gz_x500; bash'],
        output='screen'
    )

    # ROS2 node for drone control
    drone_control_node = Node(
        package='ps5_control',
        namespace='ps5_control',
        executable='drone_control',
        output='screen'
    )

    return LaunchDescription([
        start_microxrceagent,
        start_px4_simulation,
        drone_control_node
    ])
