#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
from enum import Enum
import math

class FlightState(Enum):
    IDLE = 0
    ARMING = 1
    TAKEOFF = 2
    CLIMBING = 3
    WAYPOINT_NAVIGATION = 4
    HOVERING = 5

class WaypointMission(Node):
    def __init__(self):
        super().__init__('waypoint_mission')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v2',
            self.vehicle_status_callback, qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos_profile)
        
        # State
        self.state = FlightState.IDLE
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.offboard_setpoint_counter = 0
        self.arm_state = 1
        self.nav_state = 0
        
        # Waypoints
        self.waypoints = [
            {'x': 10.0, 'y': 0.0, 'z': -5.0, 'name': 'North'},
            {'x': 10.0, 'y': 10.0, 'z': -5.0, 'name': 'NorthEast'},
            {'x': 0.0, 'y': 10.0, 'z': -5.0, 'name': 'East'},
            {'x': -5.0, 'y': 5.0, 'z': -7.0, 'name': 'West (higher)'},
            {'x': 0.0, 'y': 0.0, 'z': -5.0, 'name': 'Home'},
        ]
        self.current_waypoint = 0
        self.waypoint_threshold = 2.0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Waypoint Mission Node initialized - 5 waypoints')
        
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.arm_state = msg.arming_state
        self.nav_state = msg.nav_state
        
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        
    def control_loop(self):
        self.offboard_setpoint_counter += 1
        
        # Always publish offboard control mode
        self.publish_offboard_control_mode()
        
        # State machine
        if self.state == FlightState.IDLE:
            if self.offboard_setpoint_counter > 10:
                self.state = FlightState.ARMING
                self.get_logger().info('State: ARMING')
                
        elif self.state == FlightState.ARMING:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            if self.arm_state == 2:
                self.state = FlightState.TAKEOFF
                self.get_logger().info('State: TAKEOFF')
                
        elif self.state == FlightState.TAKEOFF:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=5.0)
            if self.nav_state == 17:
                self.state = FlightState.CLIMBING
                self.get_logger().info('State: CLIMBING - switching to offboard')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                
        elif self.state == FlightState.CLIMBING:
            # Hover at current position while climbing
            self.publish_position_setpoint(0.0, 0.0, -5.0)
            if abs(self.vehicle_local_position.z - (-5.0)) < 0.3:
                self.state = FlightState.WAYPOINT_NAVIGATION
                self.get_logger().info('State: WAYPOINT_NAVIGATION - Starting waypoint mission!')
                
        elif self.state == FlightState.WAYPOINT_NAVIGATION:
            if self.current_waypoint < len(self.waypoints):
                wp = self.waypoints[self.current_waypoint]
                self.publish_position_setpoint(wp['x'], wp['y'], wp['z'])
                
                distance = self.get_distance_to_waypoint(wp)
                if distance < self.waypoint_threshold:
                    self.get_logger().info(f"✅ REACHED WP{self.current_waypoint+1}: {wp['name']}")
                    self.current_waypoint += 1
            else:
                self.state = FlightState.HOVERING
                self.get_logger().info('🎉 Mission Complete - HOVERING')
                
        elif self.state == FlightState.HOVERING:
            wp = self.waypoints[-1]
            self.publish_position_setpoint(wp['x'], wp['y'], wp['z'])
    
    def get_distance_to_waypoint(self, wp):
        dx = self.vehicle_local_position.x - wp['x']
        dy = self.vehicle_local_position.y - wp['y']
        dz = self.vehicle_local_position.z - wp['z']
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
    
    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get('param1', 0.0)
        msg.param2 = kwargs.get('param2', 0.0)
        msg.param7 = kwargs.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    print('='*60)
    print('WAYPOINT NAVIGATION MISSION')
    print('='*60)
    rclpy.init(args=args)
    mission = WaypointMission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
