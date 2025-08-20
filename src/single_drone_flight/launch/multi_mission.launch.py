#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_mission(context, *args, **kwargs):
    mission = LaunchConfiguration('mission').perform(context)

    if mission == 'hover':
        selected_node = Node(
            package='single_drone_flight',
            namespace='single_drone_flight',
            executable='single_drone_control.py',
            name='single_drone_control',
            output='screen'
        )
    elif mission == 'box':
        selected_node = Node(
            package='single_drone_flight',
            namespace='single_drone_flight',
            executable='fly_box_mission.py',
            name='fly_box_mission',
            output='screen'
        )
    else:
        raise RuntimeError(f"Unknown mission argument: {mission}")

    return [
        TimerAction(
            period=20.0,
            actions=[selected_node]
        )
    ]

def generate_launch_description():
    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='hover',  # Options: 'hover', 'box'
        description='Choose mission to run'
    )

    return LaunchDescription([
        mission_arg,
        Node(
            package='single_drone_flight',
            namespace='single_drone_flight',
            executable='simulation_launcher.py',
            name='simulation_launcher',
            output='screen'
        ),
        OpaqueFunction(function=launch_mission),
    ])
