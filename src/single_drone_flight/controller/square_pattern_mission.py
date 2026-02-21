#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleAttitude, VehicleLocalPosition
import math

class SquarePatternMission(Node):
    def __init__(self):
        super().__init__('square_pattern_mission')
        
        # QoS profile
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
            VehicleStatus,
            '/fmu/out/vehicle_status_v2',
            self.vehicle_status_callback,
            qos_profile)
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            qos_profile)
        
        # State variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.takeoff_height = -5.0
        
        # Square waypoints (NED frame)
        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': -5.0, 'name': 'Start'},
            {'x': 10.0, 'y': 0.0, 'z': -5.0, 'name': 'North'},
            {'x': 10.0, 'y': 10.0, 'z': -5.0, 'name': 'East'},
            {'x': 0.0, 'y': 10.0, 'z': -5.0, 'name': 'South'},
            {'x': 0.0, 'y': 0.0, 'z': -5.0, 'name': 'Home'},
        ]
        self.current_waypoint = 0
        self.waypoint_threshold = 1.5  # meters
        
        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Square Pattern Mission initialized!')
        self.get_logger().info(f'Will fly to {len(self.waypoints)} waypoints')
        
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        
    def timer_callback(self):
        self.offboard_setpoint_counter += 1
        
        # Publish offboard control mode
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(offboard_msg)
        
        # Arm and switch to offboard after some setpoints
        if self.offboard_setpoint_counter == 10:
            self.arm()
            self.get_logger().info('Arming...')
            
        if self.offboard_setpoint_counter == 20:
            self.engage_offboard_mode()
            self.get_logger().info('Offboard mode engaged')
        
        # Navigate waypoints
        if self.offboard_setpoint_counter > 20:
            if self.current_waypoint < len(self.waypoints):
                wp = self.waypoints[self.current_waypoint]
                self.publish_trajectory_setpoint(wp['x'], wp['y'], wp['z'])
                
                # Check if reached waypoint
                distance = self.get_distance_to_waypoint(wp)
                
                if distance < self.waypoint_threshold:
                    self.get_logger().info(f"✓ Reached waypoint {self.current_waypoint + 1}: {wp['name']} (distance: {distance:.2f}m)")
                    self.current_waypoint += 1
                    
                    if self.current_waypoint >= len(self.waypoints):
                        self.get_logger().info('🎉 Mission complete! All waypoints reached.')
            else:
                # Mission complete - just hover at last position
                wp = self.waypoints[-1]
                self.publish_trajectory_setpoint(wp['x'], wp['y'], wp['z'])
    
    def get_distance_to_waypoint(self, waypoint):
        dx = self.vehicle_local_position.x - waypoint['x']
        dy = self.vehicle_local_position.y - waypoint['y']
        dz = self.vehicle_local_position.z - waypoint['z']
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    
    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    
    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get('param1', 0.0)
        msg.param2 = kwargs.get('param2', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    print('='*60)
    print('Square Pattern Mission')
    print('='*60)
    print('Waypoints:')
    print('  1. Start (0, 0) at 5m')
    print('  2. North 10m')
    print('  3. East 10m')
    print('  4. South 10m')
    print('  5. West 10m (Home)')
    print('='*60)
    
    rclpy.init(args=args)
    mission = SquarePatternMission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
