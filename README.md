# ROS2 SLAM Exploration

Mapping of Unknown Environments with 2D SLAM using Gazebo, TurtleBot3, slam_toolbox, and RViz.

## Project Overview

This ROS2 package provides a complete setup for autonomous mapping of unknown environments using 2D SLAM (Simultaneous Localization and Mapping). It integrates:

- **Gazebo**: Simulation environment for testing SLAM algorithms
- **TurtleBot3**: Mobile robot platform for navigation
- **slam_toolbox**: 2D SLAM implementation for map building
- **RViz**: Visualization of robot, sensors, and generated maps

## Project Structure

```
ros2_slam_exploration/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package dependencies
├── README.md                   # This file
├── config/                     # Configuration files
│   ├── slam_toolbox_params.yaml    # SLAM parameters
│   └── nav2_params.yaml            # Navigation parameters
├── launch/                     # Launch files
│   ├── gazebo.launch.py            # Gazebo simulation
│   ├── slam.launch.py              # SLAM toolbox
│   ├── rviz.launch.py              # RViz visualization
│   └── slam_exploration.launch.py  # Complete system launch
├── maps/                       # Saved map files
│   └── README.md                   # Map saving instructions
├── rviz/                       # RViz configurations
│   └── slam_view.rviz              # SLAM visualization config
├── src/                        # Custom nodes (if any)
│   └── README.md                   # Source documentation
└── worlds/                     # Gazebo world files
    └── exploration_world.world     # Sample exploration environment
```

## Prerequisites

### ROS2 Installation

This package requires ROS2 (Humble or later recommended). Install ROS2 following the [official documentation](https://docs.ros.org/en/humble/Installation.html).

### Dependencies

Install the required dependencies:

```bash
# TurtleBot3 packages
sudo apt install ros-${ROS_DISTRO}-turtlebot3*

# SLAM Toolbox
sudo apt install ros-${ROS_DISTRO}-slam-toolbox

# Navigation2
sudo apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup

# Gazebo
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
```

### TurtleBot3 Model Setup

Set the TurtleBot3 model environment variable:

```bash
export TURTLEBOT3_MODEL=burger  # or waffle, waffle_pi
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
```

## Installation

1. Create a ROS2 workspace (if not already created):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:

```bash
git clone https://github.com/santoguiia/ros2-slam-exploration.git
```

3. Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_slam_exploration
source install/setup.bash
```

## Usage

### Quick Start - Complete System

Launch the complete SLAM exploration system (Gazebo + SLAM + RViz):

```bash
ros2 launch ros2_slam_exploration slam_exploration.launch.py
```

### Individual Components

#### Launch Gazebo Simulation Only

```bash
ros2 launch ros2_slam_exploration gazebo.launch.py
```

#### Launch SLAM Toolbox Only

```bash
ros2 launch ros2_slam_exploration slam.launch.py
```

#### Launch RViz Only

```bash
ros2 launch ros2_slam_exploration rviz.launch.py
```

### Robot Teleoperation

Control the robot manually for exploration:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Saving Maps

After exploring the environment, save the generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

## Configuration

### SLAM Parameters

Edit `config/slam_toolbox_params.yaml` to adjust SLAM behavior:

- `resolution`: Map resolution (default: 0.05m)
- `max_laser_range`: Maximum laser scan range
- `do_loop_closing`: Enable/disable loop closure
- `minimum_travel_distance`: Minimum distance before processing new scan

### Custom World

To use a custom Gazebo world:

```bash
ros2 launch ros2_slam_exploration gazebo.launch.py world:=/path/to/your/world.world
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Laser scan data |
| `/map` | `nav_msgs/OccupancyGrid` | Generated occupancy grid map |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |

## License

Apache-2.0

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
