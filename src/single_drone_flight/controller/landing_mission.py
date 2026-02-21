#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
from enum import Enum

class FlightState(Enum):
    IDLE = 0
    ARMING = 1
    TAKEOFF = 2
    HOVERING = 3
    LANDING = 4
    LANDED = 5

class LandingMission(Node):
    def __init__(self):
        super().__init__('landing_mission')
        
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
        self.hover_counter = 0
        self.hover_duration = 100  # 10 seconds at 10Hz
        self.arm_state = 1
        self.nav_state = 0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('🛬 Landing Mission initialized!')
        
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.arm_state = msg.arming_state
        self.nav_state = msg.nav_state
        
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        
    def control_loop(self):
        self.offboard_setpoint_counter += 1
        
        self.publish_offboard_control_mode()
        
        if self.state == FlightState.IDLE:
            if self.offboard_setpoint_counter > 10:
                self.state = FlightState.ARMING
                self.get_logger().info('🔧 State: ARMING')
                
        elif self.state == FlightState.ARMING:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            if self.arm_state == 2:
                self.state = FlightState.TAKEOFF
                self.get_logger().info('✈️  State: TAKEOFF')
                
        elif self.state == FlightState.TAKEOFF:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=5.0)
            if self.nav_state == 17:
                self.state = FlightState.HOVERING
                self.get_logger().info('🚁 State: HOVERING - Will hover for 10 seconds then land')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                
        elif self.state == FlightState.HOVERING:
            self.publish_position_setpoint(0.0, 0.0, -5.0)
            self.hover_counter += 1
            
            if self.hover_counter >= self.hover_duration:
                self.state = FlightState.LANDING
                self.get_logger().info('🛬 State: LANDING - Initiating landing sequence')
                
        elif self.state == FlightState.LANDING:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if self.vehicle_local_position.z > -0.5:  # Near ground
                self.state = FlightState.LANDED
                self.get_logger().info('✅ State: LANDED - Mission complete!')
                
        elif self.state == FlightState.LANDED:
            # Do nothing, mission complete
            pass
    
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
    print('🛬 LANDING MISSION')
    print('='*60)
    print('  1. Takeoff to 5 meters')
    print('  2. Hover for 10 seconds')
    print('  3. Land automatically')
    print('='*60)
    
    rclpy.init(args=args)
    mission = LandingMission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
