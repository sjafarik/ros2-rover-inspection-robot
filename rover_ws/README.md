# ROS2 Rover Inspection Robot

This project implements a simulated inspection rover using ROS2 Humble, Python, and Gazebo Sim.

The rover demonstrates core robotics capabilities including:

- ROS2 Nodes
- ROS2 Actions
- ROS2 Services
- Gazebo robot simulation
- Sensor integration (camera, lidar, IMU)
- ROS ↔ Gazebo topic bridging
- Autonomous navigation with obstacle avoidance

## Workspace Structure

rover_ws/
 └── src/
     ├── rover_description
     ├── rover_simulation
     ├── rover_interfaces
     ├── rover_control
     └── rover_bringup

## Running the Simulation

Build the workspace:

colcon build
source install/setup.bash

Launch the rover simulation:

ros2 launch rover_bringup bringup.launch.py


## Technologies Used

- ROS2 Humble
- Gazebo Sim (Ignition)
- Python
- ros_gz_bridge

---

This project was developed as a ROS2 robotics portfolio project.
