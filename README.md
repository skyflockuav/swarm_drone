# ROS 2 Drone Mission Assignment - Sarthak

## Overview

This repository contains custom drone missions developed as part of the APTCODER Technical Internship assignment. Three autonomous flight missions have been implemented using ROS 2 Humble and PX4 SITL.

---

## Missions Implemented

### 1. Square Pattern Mission
**File:** `src/single_drone_flight/controller/square_pattern_mission.py`

**Description:**
The drone autonomously flies in a 10m × 10m square pattern at 5 meters altitude.

**Flight Sequence:**
1. Takeoff to 5m altitude
2. Fly North 10m
3. Fly East 10m 
4. Fly South 10m
5. Fly West 10m (return to start)
6. Hover at home position

**Key Features:**
- Waypoint-based navigation
- Distance threshold detection (1.5m)
- Real-time progress logging

---

### 2. Waypoint Navigation Mission
**File:** `src/single_drone_flight/controller/waypoint_mission_v2.py`

**Description:**
Advanced mission that visits 5 different waypoints with varying altitudes.

**Flight Sequence:**
1. Takeoff and arm
2. Visit Point Alpha (15m North, 5m altitude)
3. Visit Point Bravo (15m North, 15m East, 7m altitude)
4. Visit Point Charlie (15m East, 5m altitude)
5. Visit Point Delta (-10m North, 10m East, 8m altitude - highest)
6. Visit Point Echo (-10m North, -10m East, 5m altitude)
7. Return Home (0, 0, 5m altitude)

**Key Features:**
- State machine architecture (IDLE → ARMING → TAKEOFF → CLIMBING → WAYPOINT_NAVIGATION → HOVERING)
- Dynamic altitude changes
- 3D distance calculation for waypoint verification

---

### 3. Landing Mission
**File:** `src/single_drone_flight/controller/landing_mission.py`

**Description:**
Complete flight cycle demonstrating safe takeoff, hover, and autonomous landing.

**Flight Sequence:**
1. Arm and takeoff to 5m altitude
2. Hover for 10 seconds
3. Execute automatic landing sequence
4. Safe touchdown

**Key Features:**
- Timer-based hover duration
- Automatic landing command
- Ground proximity detection

---

##  Repository Structure
```
swarm_drone/
├── src/
│   └── single_drone_flight/
│       ├── controller/
│       │   ├── single_drone_control.py       # Base mission (original)
│       │   ├── square_pattern_mission.py     # Mission 1
│       │   ├── waypoint_mission_v2.py        # Mission 2
│       │   ├── landing_mission.py            # Mission 3
│       │   └── simulation_launcher.py        # Simulation helper
│       ├── launch/
│       │   └── single_drone_flight.launch.py
│       ├── CMakeLists.txt
│       └── package.xml
├── README.md                                  # This file
└── RESOURCES_USED.md                          # External sources documentation
```

---

##System Requirements

- **OS:** Ubuntu 22.04 (or WSL2 on Windows)
- **ROS 2:** Humble
- **PX4:** Main branch
- **Gazebo:** Classic 11.10.2
- **Python:** 3.10+

---

## 📦 Installation & Setup

### 1. Install Dependencies
```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install Gazebo Classic
sudo apt install gazebo

# Install Python dependencies
pip3 install --user empy pyros-genmsg setuptools kconfiglib jsonschema jinja2
```

# 2. Install PX4 Autopilot
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gazebo-classic
```

# 3. Install MicroXRCE-DDS Agent
```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

# 4. Clone and Build Workspace
```bash
cd ~/ros2_drone_workspace/swarm_drone
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

# Running the Missions

# Terminal 1: Start PX4 SITL
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```
*Wait for "Ready for takeoff!" message + 60 seconds for stabilization*

# Terminal 2: Start MicroXRCE-DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

# Terminal 3: Run a Mission

**Square Pattern:**
```bash
cd ~/ros2_drone_workspace/swarm_drone
source install/setup.bash
ros2 run single_drone_flight square_pattern_mission.py
```

**Waypoint Navigation:**
```bash
ros2 run single_drone_flight waypoint_mission_v2.py
```

**Landing Mission:**
```bash
ros2 run single_drone_flight landing_mission.py
```

---

## Demo Videos

*Demo videos are included in the repository showing each mission in action:*

- `demos/square_pattern_demo.mp4` (or .gif)
- `demos/waypoint_mission_demo.mp4` (or .gif)
- `demos/landing_mission_demo.mp4` (or .gif)

---

# Technical Details

# Coordinate System
All missions use PX4's NED (North-East-Down) coordinate frame:
- **X:** North (positive forward)
- **Y:** East (positive right)
- **Z:** Down (negative = altitude)

# Communication Architecture
```
ROS 2 Node ←→ MicroXRCE-DDS Agent ←→ PX4 SITL ←→ Gazebo
```

# Key Topics Used
- `/fmu/in/offboard_control_mode` - Offboard control mode commands
- `/fmu/in/trajectory_setpoint` - Position setpoints
- `/fmu/in/vehicle_command` - Vehicle commands (arm, takeoff, land)
- `/fmu/out/vehicle_status_v2` - Vehicle status feedback
- `/fmu/out/vehicle_local_position_v1` - Position feedback

---

# Troubleshooting

# Issue: Drone doesn't take off
**Solution:** Ensure 60-second wait after "Ready for takeoff!" for velocity estimator to stabilize.

# Issue: "Battery unhealthy" error
**Solution:** Clean PX4 build:
```bash
cd ~/PX4-Autopilot
rm -rf build/px4_sitl_default/tmp/
make clean
make px4_sitl gazebo-classic
```

# Issue: No topics visible
**Solution:** Verify MicroXRCE-DDS Agent is running and connected:
```bash
ros2 topic list | grep fmu
```

---

# External Resources

All external resources, tutorials, and AI assistance used in this project are documented in `RESOURCES_USED.md`.

---

# Assignment Compliance

# Base Task Requirements 
-  Understood existing codebase
-  Created 3 new missions (square, waypoints, landing)
-  Maintained original code structure
-  Used same base logic patterns
-  Single launch file compatible

# Code Quality 
-  Clean, modular code
-  Consistent naming conventions
-  Proper state machine implementation
-  Comprehensive logging

# Documentation 
-  All external sources cited
-  Clear README with usage instructions
-  Demo videos included
-  Detailed PR description

---

# Author

**[SARTHAK AGRAWAL]** 
Technical Intern - ROS Drone Swarm Simulation & Control Branch
APTCODER

---

## License

This project extends the base codebase provided by APTCODER for the technical internship assignment.

---

## Acknowledgments

- APTCODER for the internship opportunity
- PX4 Development Team for excellent documentation
- ROS 2 Community for robust tools and libraries
- ChatGPT for development assistance


*Last Updated: February 21, 2026*
