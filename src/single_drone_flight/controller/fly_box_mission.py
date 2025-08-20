#!/usr/bin/env python3

"""
Fly Box Mission

This node extends the single_drone_control.py functionality to autonomously fly a drone in a box pattern
after a successful takeoff.

State Machine:
IDLE -> ARMING -> CLIMBING -> INITIAL_HOVER -> FLYING_BOX -> BOX_COMPLETE -> FINAL_HOVER

Author: Generated for single drone flight demo
"""

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Vector3
import time


class FlyBoxMission(Node):
    """
    ROS2 Node for controlling a single drone to fly a box mission autonomously.
    
    It first takes off to a target altitude and then flies to a series of waypoints
    to form a square pattern.
    """

    def __init__(self):
        super().__init__('fly_box_mission')
        
        # QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
            
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile)

        # Create publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Create timers
        # Main control loop timer (50Hz for smooth control)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop_callback)
        
        # State machine timer (10Hz for state transitions)
        state_timer_period = 0.1  # seconds
        self.state_timer = self.create_timer(state_timer_period, self.state_machine_callback)

        # Initialize state variables
        self.current_state = "IDLE"
        self.last_state = "IDLE"
        self.target_altitude = -5.0  # NED frame: negative Z is up
        self.altitude_tolerance = 0.4  # meters, increased tolerance
        self.climb_velocity = 1.0  # m/s
        
        # Timing variables for state transitions
        self.arming_start_time = 0.0
        self.arming_timeout = 10.0  # seconds to wait for arming
        self.startup_delay = 5.0  # seconds to wait before starting arming process
        self.hover_start_time = 0.0
        self.hover_delay = 5.0  # seconds to hover before starting the box mission
        
        # Vehicle status variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.failsafe = False
        self.flight_check = False
        self.offboard_mode = False
        self.offboard_mode_req_count = 0
        
        # Position and attitude variables
        self.current_altitude = 0.0  # NED frame
        self.current_position = Vector3()
        self.takeoff_position = Vector3()
        self.yaw = 0.0 # Initialized as a float
        
        # Control variables
        self.counter = 0
        self.start_time = time.time()
        
        # Box mission specific variables
        self.box_size = 10.0  # meters
        self.waypoint_index = 0
        self.waypoints = []
        self.waypoint_tolerance = 0.5 # meters
        self.takeoff_yaw = 0.0
        
        self.get_logger().info("Fly Box Mission Node Initialized")
        self.get_logger().info(f"Target altitude: {-self.target_altitude} meters above ground")

    def vehicle_status_callback(self, msg):
        """Callback for vehicle status updates"""
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"Navigation State: {msg.nav_state}")
        
        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"Arming State: {msg.arming_state}")

        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"Failsafe: {msg.failsafe}")
            
        if msg.pre_flight_checks_pass != self.flight_check:
            self.get_logger().info(f"Flight Check: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass

    def attitude_callback(self, msg):
        """Callback for vehicle attitude updates"""
        # Extract yaw from quaternion
        orientation_q = msg.q
        self.yaw = float(-(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                              1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1]))))

    def local_position_callback(self, msg):
        """Callback for vehicle local position updates"""
        self.current_position.x = msg.x
        self.current_position.y = msg.y
        self.current_position.z = msg.z
        self.current_altitude = msg.z  # NED frame: negative is up

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Publish a vehicle command"""
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

    def arm(self):
        """Send arm command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def set_offboard_mode(self):
        """Switch to offboard mode"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.offboard_mode = True
        self.offboard_mode_req_count += 1
        self.get_logger().info(f"Offboard mode command sent (request {self.offboard_mode_req_count})")

    def state_machine_callback(self):
        """State machine for autonomous flight"""
        
        if self.current_state != self.last_state:
            self.get_logger().info(f"State transition: {self.last_state} -> {self.current_state}")
            self.last_state = self.current_state
            
        # If not armed and not in a state that is meant to arm, revert to idle.
        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED and self.current_state not in ["IDLE", "ARMING"]:
             self.get_logger().warn(f"Disarmed! Reverting to IDLE from state: {self.current_state}")
             self.current_state = "IDLE"

        if self.current_state == "IDLE":
            # Wait for flight checks to pass and some startup time
            current_time = time.time()
            if self.flight_check and (current_time - self.start_time) > self.startup_delay:
                self.current_state = "ARMING"
                self.arming_start_time = current_time
                self.get_logger().info("Flight checks passed and startup delay complete, starting arming sequence")

        elif self.current_state == "ARMING":
            current_time = time.time()
            
            # Check for arming timeout
            if (current_time - self.arming_start_time) > self.arming_timeout:
                self.current_state = "IDLE"
                self.get_logger().warn("Arming timeout, returning to IDLE state")
                return
                
            # Check if drone is armed and ready for the next state
            if self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                self.current_state = "CLIMBING"
                # Store takeoff position for reference
                self.takeoff_position = self.current_position
                self.takeoff_yaw = self.yaw
                # Ensure we have a valid takeoff yaw value
                if self.takeoff_yaw is None or not isinstance(self.takeoff_yaw, (int, float)):
                    self.takeoff_yaw = 0.0
                self.get_logger().info(f"Takeoff position: ({self.takeoff_position.x:.2f}, {self.takeoff_position.y:.2f}, {self.takeoff_position.z:.2f})")
                self.get_logger().info(f"Takeoff yaw: {math.degrees(self.takeoff_yaw):.2f} degrees")
                
                self.get_logger().info("Armed successfully, switching to offboard and starting climb")
            
            # Continuously send arm and offboard mode commands until successful.
            # This is critical for reliable transitions.
            self.arm()
            self.set_offboard_mode() 
                
        elif self.current_state == "CLIMBING":
            # Now we use offboard control to command the climb
            if not self.flight_check or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                self.current_state = "IDLE"
                self.get_logger().warn("Safety condition failed during climb")
            elif abs(self.current_altitude - self.target_altitude) < self.altitude_tolerance:
                self.current_state = "INITIAL_HOVER"
                self.hover_start_time = time.time()
                self.get_logger().info(f"Target altitude reached! Current: {-self.current_altitude:.2f}m. Starting hover delay.")
            # Continuously request offboard mode to prevent timeout
            self.set_offboard_mode()

        elif self.current_state == "INITIAL_HOVER":
            # Wait for the drone to settle at the target altitude
            current_time = time.time()
            if (current_time - self.hover_start_time) > self.hover_delay:
                # Define waypoints based on current position and start the box mission.
                # Use the takeoff position as the origin for the box pattern
                self.waypoints = [
                    Vector3(x=self.takeoff_position.x + self.box_size, y=self.takeoff_position.y, z=self.target_altitude),
                    Vector3(x=self.takeoff_position.x + self.box_size, y=self.takeoff_position.y + self.box_size, z=self.target_altitude),
                    Vector3(x=self.takeoff_position.x, y=self.takeoff_position.y + self.box_size, z=self.target_altitude),
                    Vector3(x=self.takeoff_position.x, y=self.takeoff_position.y, z=self.target_altitude),
                ]
                self.get_logger().info("Box waypoints defined, starting box mission.")
                self.current_state = "FLYING_BOX"
                self.waypoint_index = 0

            if not self.flight_check or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                self.current_state = "IDLE"
                self.get_logger().warn("Safety condition failed during initial hover")
            # Continuously request offboard mode to prevent timeout
            self.set_offboard_mode()


        elif self.current_state == "FLYING_BOX":
            # Check if we have reached the current waypoint
            current_waypoint = self.waypoints[self.waypoint_index]
            distance_to_waypoint = np.sqrt(
                (self.current_position.x - current_waypoint.x)**2 + 
                (self.current_position.y - current_waypoint.y)**2 + 
                (self.current_position.z - current_waypoint.z)**2
            )

            if distance_to_waypoint < self.waypoint_tolerance:
                self.get_logger().info(f"Waypoint {self.waypoint_index + 1} reached.")
                self.waypoint_index += 1
                
            # Check if all waypoints have been visited
            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info("Box pattern complete.")
                self.current_state = "BOX_COMPLETE"
            
            if not self.flight_check or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                self.current_state = "IDLE"
                self.get_logger().warn("Safety condition failed during box mission")
            # Continuously request offboard mode to prevent timeout
            self.set_offboard_mode()

        elif self.current_state == "BOX_COMPLETE":
            # Once the box is complete, we enter a final hover state
            self.get_logger().info("Mission complete, entering final hover state.")
            self.current_state = "FINAL_HOVER"
            # Continuously request offboard mode to prevent timeout
            self.set_offboard_mode()

        self.counter += 1

    def control_loop_callback(self):
        """Main control loop for offboard commands"""
        # Publish offboard control mode and trajectory setpoint messages
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # Check if yaw is a number before using it
        if not isinstance(self.yaw, (int, float)):
            self.get_logger().warn("Yaw is not a valid number, using 0.0 for now.")
            self.yaw = 0.0
        
        # Setpoint logic based on current state
        if self.current_state == "CLIMBING" or self.current_state == "INITIAL_HOVER" or self.current_state == "FINAL_HOVER":
            # Position control to reach target altitude and then hover
            trajectory_msg.position[0] = self.takeoff_position.x  # Hold X position
            trajectory_msg.position[1] = self.takeoff_position.y  # Hold Y position
            trajectory_msg.position[2] = self.target_altitude      # Target altitude
            trajectory_msg.yaw = float(self.takeoff_yaw)           # Hold initial yaw
            
            # The yawspeed is set below, so only set yawspeed to nan
            trajectory_msg.yawspeed = float('nan')
            
        elif self.current_state == "FLYING_BOX":
            # Fly to the current waypoint in the list
            target_waypoint = self.waypoints[self.waypoint_index]
            trajectory_msg.position[0] = target_waypoint.x
            trajectory_msg.position[1] = target_waypoint.y
            trajectory_msg.position[2] = target_waypoint.z
            
            # Command yaw to face the next waypoint
            dx = target_waypoint.x - self.current_position.x
            dy = target_waypoint.y - self.current_position.y
            target_yaw = math.atan2(dy, dx)
            
            trajectory_msg.yaw = float(target_yaw)
            trajectory_msg.yawspeed = float('nan') # Ensure yawspeed is not commanded
        
        else:
            # If in IDLE or ARMING state, publish a position setpoint at the current position
            # This is important to transition into offboard mode smoothly
            trajectory_msg.position[0] = self.current_position.x
            trajectory_msg.position[1] = self.current_position.y
            trajectory_msg.position[2] = self.current_position.z
            trajectory_msg.yaw = float(self.yaw)
            trajectory_msg.yawspeed = float('nan')

        # Set unused fields to NaN
        trajectory_msg.velocity[0] = float('nan')
        trajectory_msg.velocity[1] = float('nan')
        trajectory_msg.velocity[2] = float('nan')
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')

        self.publisher_offboard_mode.publish(offboard_msg)
        self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    
    print("=" * 60)
    print("Single Drone Flight - Box Mission")
    print("=" * 60)
    print("This node will autonomously:")
    print("1. Arm the drone")
    print("2. Takeoff and climb to 5 meters altitude")
    print("3. Fly a 10m x 10m box pattern")
    print("4. Hover at the final position")
    print("=" * 60)
    
    fly_box_mission = FlyBoxMission()
    
    try:
        rclpy.spin(fly_box_mission)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        fly_box_mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
