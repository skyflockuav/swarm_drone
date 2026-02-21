cmake_minimum_required(VERSION 3.8)
project(single_drone_flight)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python executables
install(PROGRAMS
    controller/single_drone_control.py
    controller/simulation_launcher.py
    controller/square_pattern_mission.py
    controller/waypoint_mission.py
    controller/waypoint_mission_v2.py
    controller/landing_mission.py
    DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
