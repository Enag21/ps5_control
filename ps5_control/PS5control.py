import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from pydualsense import pydualsense, TriggerModes
from ps5_control.scripts.helpers import Vector2D, DeadZone

class OffboardControlWithPS5(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_with_ps5')

        # Create instance of PS5controller
        self.ds = pydualsense()
        self.ds.init()
        self.ds.light.setColorI(0,0,255)
        self.dead_zone = 10
        self.joystick_input : bool = False

        if self.ds.device is None:
            self.get_logger().warn("No DualSense controller found!")
            rclpy.shutdown()

        self.ds.circle_pressed += self.circle_down
        self.ds.square_pressed += self.square_down
        self.ds.triggerL.setMode(TriggerModes.Rigid)
        self.ds.triggerR.setMode(TriggerModes.Rigid)

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create publichers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.is_armed = False
        self.delta_L = 0.0
        self.delta_R = 0.0
        self.direction_vector = Vector2D(0.0, 0.0)

        # Create a timer to publish control commands
        self.create_timer(0.01, self.control_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.ds.light.setColorI(0,255,0)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.ds.light.setColorI(0,0,255)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.ds.light.setColorI(255,0,0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def circle_down(self, state):
        if state:
            if self.is_armed:
                self.disarm()
                self.is_armed = False
            else:
                self.engage_offboard_mode()
                self.arm()
                self.is_armed = True

    def square_down(self, state):
        if state:
            self.land()

    def control_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        # Store altitude commands
        triggerL_value = self.ds.state.L2 / 255
        triggerR_value = self.ds.state.R2 / 255

        self.delta_L = triggerL_value
        self.delta_R = triggerR_value

        delta_altitude = self.delta_R - self.delta_L

        # Store direction commands in a 2D vector and normalize
        self.direction_vector.x, self.joystick_input = DeadZone(self.dead_zone, self.ds.state.RX * -1.0)
        self.direction_vector.y, self.joystick_input  = DeadZone(self.dead_zone, self.ds.state.RY * -1.0)

        direction = self.direction_vector.normalize()


        if delta_altitude != 0.0 or self.joystick_input:
            self.publish_altitude_adjustment(direction, delta_altitude)
    
    def publish_altitude_adjustment(self, direction: Vector2D, delta_altitude: float):

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint( self.vehicle_local_position.x + direction.x,
                                            self.vehicle_local_position.y + direction.y,
                                            self.vehicle_local_position.z + delta_altitude)
"""         else:
            self.get_logger().info(f"NO INPUT")
            self.publish_position_setpoint( self.vehicle_local_position.x,
                                            self.vehicle_local_position.y,
                                            self.vehicle_local_position.z + delta_altitude) """

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlWithPS5()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
