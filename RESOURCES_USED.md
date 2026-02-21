# External Resources Used for ROS Drone Assignment

## Student Information
- **Name:** [SARTHAK AGRAWAL]
- **Date:** February 21, 2026
- **Branch:** ECE


## Missions Implemented

### 1. Square Pattern Mission ✅
- **File:** `square_pattern_mission.py`
- **Description:** Drone flies in a 10m x 10m square pattern at 5m altitude
- **Waypoints:** 5 corners including start/end position

### 2. Waypoint Navigation Mission ✅
- **File:** `waypoint_mission_v2.py`
- **Description:** Drone visits 5 different waypoints at varying altitudes (5m-7m)
- **Features:** State machine with ARMING → TAKEOFF → CLIMBING → WAYPOINT_NAVIGATION → HOVERING

### 3. Landing Mission ✅
- **File:** `landing_mission.py`
- **Description:** Complete flight cycle - Takeoff → Hover 10 seconds → Automatic landing
- **Features:** Demonstrates safe landing procedure


## System Configuration

### Environment
- **Operating System:** Windows 11 with WSL2 (Ubuntu 22.04.5 LTS)
- **ROS 2 Version:** Humble
- **PX4 Version:** Main branch (latest as of Feb 2026)
- **Gazebo Version:** Classic 11.10.2
- **Python Version:** 3.10.12

### Key Dependencies
- `px4_msgs` - Main branch (compatibility fix from release/1.14)
- `rclpy` - ROS 2 Python client library
- `MicroXRCE-DDS Agent` - Compiled from source

## External Resources

### Official Documentation
1. **ROS 2 Humble Documentation**
   - URL: https://docs.ros.org/en/humble/
   - Used for: Understanding ROS 2 node structure, publishers, subscribers, QoS profiles

2. **PX4 Autopilot Documentation**
   - URL: https://docs.px4.io/
   - Used for: Offboard control mode, vehicle commands, SITL setup

3. **Gazebo Classic Documentation**
   - URL: http://gazebosim.org/
   - Used for: Simulation environment setup

### GitHub Repositories
1. **PX4-Autopilot**
   - URL: https://github.com/PX4/PX4-Autopilot
   - Used: Main branch for SITL simulation

2. **px4_msgs**
   - URL: https://github.com/PX4/px4_msgs
   - Branch: main (switched from release/1.14 due to topic compatibility)
   - Fix: Updated topic names from `/vehicle_status` to `/vehicle_status_v2`

3. **Base Codebase**
   - URL: https://github.com/skyflockuav/swarm_drone
   - Branch: intern_codebase
   - Used: `single_drone_control.py` as reference for mission structure

4. **Micro-XRCE-DDS-Agent**
   - URL: https://github.com/eProsima/Micro-XRCE-DDS-Agent
   - Used: ROS 2 ↔ PX4 communication bridge

### AI Assistance (ChatGPT)
Used extensively for:
- **WSL2 Setup:** Step-by-step installation of Ubuntu on Windows
- **Dependency Installation:** ROS 2, PX4, Gazebo, MicroXRCE-DDS Agent setup
- **Debugging Issues:**
  - px4_msgs version mismatch (release/1.14 vs main branch)
  - Topic name changes (`/vehicle_status` → `/vehicle_status_v2`)
  - MicroXRCE-DDS Agent connection issues
  - Battery health simulation errors
- **Mission Development:**
  - State machine design for waypoint navigation
  - Waypoint distance calculation logic
  - Landing sequence implementation
- **Code Structure:** Understanding ROS 2 node architecture, publishers/subscribers patterns

## Issues Resolved

### 1. px4_msgs Version Incompatibility
- **Problem:** Topic size mismatch errors, topics not publishing
- **Solution:** Switched from `release/1.14` to `main` branch
- **Command:** `git clone https://github.com/PX4/px4_msgs.git -b main`

### 2. Topic Name Changes
- **Problem:** Subscribers not receiving data
- **Solution:** Updated topic names in Python code:
  - `/fmu/out/vehicle_status` → `/fmu/out/vehicle_status_v2`
  - `/fmu/out/vehicle_local_position` → `/fmu/out/vehicle_local_position_v1`

### 3. MicroXRCE-DDS Agent Not Found
- **Problem:** `MicroXRCEAgent: command not found`
- **Solution:** Manually compiled and installed from source

### 4. Simulation Battery Errors
- **Problem:** "Battery unhealthy" preventing arming
- **Solution:** Clean PX4 build: `rm -rf build/px4_sitl_default/tmp/ && make clean`

### 5. "Already higher than takeoff altitude" Warning
- **Problem:** Drone not taking off due to altitude estimation
- **Solution:** Clean restart of PX4 with fresh parameters

## Code Attribution

### Base Structure
- **Source:** `single_drone_control.py` from intern_codebase
- **Modifications:** Extended with waypoint navigation and landing logic

### Mission Logic
- **Square Pattern:** Custom implementation based on waypoint array iteration
- **Waypoint Navigation:** State machine approach adapted from base code structure
- **Landing Mission:** Combined takeoff and land commands with hover state

### Original Concepts
- Waypoint threshold distance checking
- State machine transitions for multi-stage missions
- Timer-based hover duration for landing sequence

## Tools & Software Used

- **Development:** Visual Studio Code (optional), nano text editor
- **Version Control:** Git
- **Build System:** colcon (ROS 2 build tool)
- **Simulation:** Gazebo Classic, PX4 SITL
- **Testing:** ROS 2 CLI tools (`ros2 run`, `ros2 topic`)

## Acknowledgments

- **APTCODER** for the internship opportunity and base codebase
- **PX4 & ROS 2 communities** for excellent documentation
- **ChatGPT (OpenAI)** for development assistance and debugging support

*All external sources have been properly documented and credited.*
