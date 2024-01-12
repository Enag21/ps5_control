import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus, VehicleAttitude
from pydualsense import pydualsense, TriggerModes
from ps5_control.scripts.helpers import Vector2D, apply_deadzone, distance_to_force
import quaternion
import math

class OffboardControlWithPS5(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_with_ps5')

        # Create instance of PS5controller
        self.ds = pydualsense()
        self.ds.init()
        self.ds.light.setColorI(0,0,255)
        self.dead_zone = 10
        self.joystick_input : bool = False
        self.ignore_atittude_commands : bool = False
        self.ignore_rotation_commands : bool = False
        self.speed_multiplier : float = 1.0

        if self.ds.device is None:
            self.get_logger().warn("No DualSense controller found!")
            rclpy.shutdown()

        self.ds.circle_pressed += self.circle_down
        self.ds.square_pressed += self.square_down
        self.ds.l1_changed += self.l1_down
        self.ds.r1_changed += self.r1_down
        self.ds.dpad_up += self.dpad_up
        self.ds.dpad_down += self.dpad_down
        self.ds.dpad_right += self.dpad_right
        self.ds.dpad_left += self.dpad_left
        self.ds.triggerL.setMode(TriggerModes.Rigid)
        self.ds.triggerR.setMode(TriggerModes.Rigid)

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
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
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/vehicle_attitude/out', self.vehicle_attitude_callback, qos_profile)
        
        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.is_armed = False
        self.delta_L = 0.0
        self.delta_R = 0.0
        self.direction_vector = Vector2D(0.0, 0.0)
        self.current_yaw = 0.0
        self.delta_pad_x = 0.0
        self.delta_pad_y = 0.0
        self.poosition_delta = Vector2D(0, 0)
        # Create a timer to publish control commands
        self.create_timer(0.01, self.control_callback)


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position


    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status


    def vehicle_attitude_callback(self, vehicle_attitude):
        q = quaternion.from_float_array(
            vehicle_attitude.q[0],
            vehicle_attitude.q[1],
            vehicle_attitude.q[2],
            vehicle_attitude.q[3]
        )
        euler = quaternion.as_euler_angles(q)
        yaw = euler[2]  # Extract yaw component (assuming roll, pitch, yaw)
        self.current_yaw = yaw

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


    def publish_position_setpoint(self, x: float, y: float, z: float, yaw : float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z, yaw]}")


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


    def l1_down(self, state):
        if state:
            self.ignore_atittude_commands = not self.ignore_atittude_commands
    

    def r1_down(self, state):
        if state:
            self.ignore_rotation_commands = not self.ignore_rotation_commands

    def dpad_up(self, state):
        self.delta_pad_x = 0.5
    
    def dpad_down(self, state):
        self.delta_pad_x = -0.5

    def dpad_right(self, state):
        self.delta_pad_y = 0.5

    def dpad_left(self, state):
        self.delta_pad_y = -0.5

    def handle_XY_movement(self) -> Vector2D:
        # Manage XY movement (Left joystick)
        raw_x = self.ds.state.LY * -1 # range is -127 to 128 Y in joystick corresponds to forward for the drone (X)
        raw_y = self.ds.state.LX * 1 # range is -127 to 128 X in joystick corresponds to Y for the drone

        adjusted_x, adjusted_y = apply_deadzone(raw_x, raw_y, self.dead_zone)
        direction = Vector2D(adjusted_x, adjusted_y).normalize()

        drone_yaw = self.vehicle_local_position.heading

        forward_x = math.cos(drone_yaw) * direction.x - math.sin(drone_yaw) * direction.y
        forward_y = math.sin(drone_yaw) * direction.x + math.cos(drone_yaw) * direction.y
        return Vector2D(forward_x, forward_y)


    def handle_yaw_rotation(self):
        drone_yaw = self.vehicle_local_position.heading
        raw_rx = self.ds.state.RX
        raw_ry = self.ds.state.RY

        adjusted_rx, _ = apply_deadzone(raw_rx, raw_ry, self.dead_zone)
        new_yaw = drone_yaw + (adjusted_rx / 128.0) * -5 

        if self.ignore_rotation_commands:
            new_yaw = drone_yaw

        return new_yaw


    def hanlde_altitude(self):
        MAX_ALTITUDE_RATE = 5  # Maximum altitude change rate
        raw_rx = self.ds.state.RX
        raw_ry = self.ds.state.RY

        _, adjusted_ry = apply_deadzone(raw_rx, raw_ry, self.dead_zone)
        altitude_rate = (adjusted_ry / 128.0) * MAX_ALTITUDE_RATE
        if self.ignore_atittude_commands:
            altitude_rate = 0.0
        
        return altitude_rate


    def block_movement(self):
        self.ds.triggerR.setForce(1, force=255)


    def allow_movement(self):
        self.ds.triggerR.setMode(TriggerModes.Off)


    def handle_feedback(self):
        obstacle_position = np.array([-5, 10])
        drone_position = np.array([self.vehicle_local_position.x, self.vehicle_local_position.y])
        distance = np.linalg.norm(obstacle_position - drone_position) - 0.75
        vector_to_obstacle = obstacle_position - drone_position
        movement_dir = self.handle_XY_movement()
        movement_dir = np.array([movement_dir.x, movement_dir.y])
        dot_product = np.dot(vector_to_obstacle, movement_dir)

        if distance < 3.0:
            self.ds.triggerR.setMode(TriggerModes.Rigid)
            force = distance_to_force(distance, max_distance = 7.5)
            self.get_logger().info(f"Close to obstacle: {force}")
            self.ds.light.setColorI(255, 255, 0)
            self.ds.triggerR.setForce(1, force)
            self.ds.setLeftMotor(force)
            self.ds.setRightMotor(force)
            if distance < 1.0:
                self.block_movement()
                if dot_product > 0:
                    self.position_delta *= 0.0
        else:
            self.ds.setLeftMotor(0)
            self.ds.setRightMotor(0)
            self.ds.light.setColorI(0, 255, 0)
            self.allow_movement()


    def control_callback(self):
        self.publish_offboard_control_heartbeat_signal()

        self.position_delta = self.handle_XY_movement()
        new_yaw = self.handle_yaw_rotation()
        altitude_rate = self.hanlde_altitude()
        self.handle_feedback()

        
        #self.get_logger().info(f"R2: {self.delta_pad_x}")
        if self.ds.state.R2 == 0:
            self.position_delta *= 0.0


        self.publish_position_setpoint(self.vehicle_local_position.x + self.position_delta.x,
                                    self.vehicle_local_position.y + self.position_delta.y,
                                    self.vehicle_local_position.z + altitude_rate,
                                    new_yaw)
        

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlWithPS5()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
