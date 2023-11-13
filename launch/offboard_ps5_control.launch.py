from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the working directories for your commands
    px4_autopilot_dir = os.getenv('HOME') + '/PX4-Autopilot'
    ros_workspace_dir = os.getenv('HOME') + '/ROS_workspaces/drone_control_ws'

    package_dir = get_package_share_directory('ps5_control')
    # Command to run MicroXRCEAgent
    start_microxrceagent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # Command to start PX4 simulation
    start_px4_simulation = ExecuteProcess(
        cmd=['bash', '-c', 'cd ' + px4_autopilot_dir + ' && PX4_GZ_WORLD=custom make px4_sitl gz_x500'],
        output='screen'
    )

    # Command to source ROS workspace
    source_ros_workspace = ExecuteProcess(
        cmd=['bash', '-c', 'source ' + ros_workspace_dir + '/install/setup.bash'],
        output='screen'
    )

    # ROS2 node to launch GPU LiDAR bridge
    gpu_lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen'
    )

    # ROS2 node for drone control
    drone_control_node = Node(
        package = 'ps5_control',
        namespace = 'ps5_control',
        executable = 'drone_control',
        output = 'screen'
    )

    return LaunchDescription([
        start_microxrceagent,
        start_px4_simulation,
        source_ros_workspace,
        drone_control_node,
        gpu_lidar_bridge
    ])