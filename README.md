# ROS2 SLAM Exploration

Mapping of Unknown Environments with 2D SLAM using Gazebo, TurtleBot3, slam_toolbox, Navigation2, and explore_lite.

## Project Overview

This ROS2 package provides a complete setup for autonomous mapping of unknown environments using 2D SLAM (Simultaneous Localization and Mapping). It integrates:

- **Gazebo**: Simulation environment for testing SLAM algorithms
- **TurtleBot3**: Mobile robot platform for navigation
- **slam_toolbox**: 2D SLAM implementation for map building
- **Navigation2 (Nav2)**: Path planning and obstacle avoidance
- **explore_lite**: Frontier-based autonomous exploration algorithm
- **RViz**: Visualization of robot, sensors, and generated maps

## Project Structure

```

ros2\_slam\_exploration/
├── CMakeLists.txt              \# Build configuration
├── package.xml                 \# Package dependencies
├── README.md                   \# This file
├── config/                     \# Configuration files
│   ├── slam\_toolbox\_params.yaml    \# SLAM parameters
│   └── nav2\_params.yaml            \# Navigation parameters (Optimized for Burger)
├── launch/                     \# Launch files
│   ├── gazebo.launch.py            \# Gazebo simulation
│   ├── slam.launch.py              \# SLAM toolbox
│   ├── rviz.launch.py              \# RViz visualization
│   └── slam\_exploration.launch.py  \# Complete system launch (Sim + SLAM + RViz)
├── maps/                       \# Saved map files
│   └── README.md                   \# Map saving instructions
├── rviz/                       \# RViz configurations
│   └── slam\_view.rviz              \# SLAM visualization config
├── src/                        \# Custom nodes (if any)
│   └── README.md                   \# Source documentation
└── worlds/                     \# Gazebo world files
└── exploration\_world.world     \# Sample exploration environment

````

## Prerequisites

### ROS2 Installation

This package requires **ROS2 Humble**. Install ROS2 following the [official documentation](https://docs.ros.org/en/humble/Installation.html).

### System Dependencies

Install the required dependencies. **Note:** CycloneDDS is highly recommended for simulation stability, specially on WSL.

```bash
# TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*

# Navigation & SLAM
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox

# Simulation & Middleware
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-rmw-cyclonedds-cpp
````

### Environment Configuration

Add these lines to your `~/.bashrc` to ensure the environment variables are always set:

```bash
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Installation

To use this package, you must create a workspace and clone **both** this repository and the `m-explore-ros2` package (required for autonomous exploration).

1.  **Create a ROS2 workspace:**

<!-- end list -->

```bash
mkdir -p ~/robotica_ws/src
cd ~/robotica_ws/src
```

2.  **Clone the repositories:**

<!-- end list -->

```bash
# Clone this project
git clone [https://github.com/santoguiia/ros2-slam-exploration.git](https://github.com/santoguiia/ros2-slam-exploration.git)

# Clone explore_lite (Frontier Exploration)
git clone [https://github.com/robo-friends/m-explore-ros2.git](https://github.com/robo-friends/m-explore-ros2.git)
```

3.  **Install dependencies and build:**

<!-- end list -->

```bash
cd ~/robotica_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## Usage

To run the full Autonomous Exploration, it is recommended to use **3 separate terminals** to monitor each component.

### Terminal 1: Simulation & SLAM

This launches Gazebo, spawns the robot, starts SLAM Toolbox, and opens RViz.

```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger

# Optional: Use software rendering if Gazebo crashes/freezes on WSL
# export LIBGL_ALWAYS_SOFTWARE=1 

ros2 launch ros2_slam_exploration slam_exploration.launch.py
```

### Terminal 2: Navigation Stack (Nav2)

This starts the navigation system using the custom parameters configured for the Burger robot.

```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=src/ros2-slam-exploration/config/nav2_params.yaml
```

*Wait until you see the message: `[lifecycle_manager_navigation]: Managed nodes are active`*

### Terminal 3: Autonomous Explorer

This starts the frontier-based exploration node. The robot should start moving automatically to explore unknown areas.

```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run explore_lite explore --ros-args \
 -p use_sim_time:=true \
 -p costmap_topic:=/map \
 -p visualize:=true \
 -p min_frontier_size:=0.1 \
 -p planner_frequency:=5.0 \
 -p progress_timeout:=180.0
```

### Manual Control (Optional)

If you want to override the autonomous exploration and drive the robot manually:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Saving Maps

After the exploration is complete, save the generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/robotica_ws/src/ros2-slam-exploration/maps/my_map
```

## Configuration Details

### Nav2 Parameters (`nav2_params.yaml`)

The configuration file in `config/nav2_params.yaml` has been tuned for the TurtleBot3 Burger to avoid common issues:

  - **Plugin Format**: Updated to use `/` instead of `::` (Humble standard).
  - **Robot Radius**: Set to `0.09` (Burger size) to allow navigation in tight spaces.
  - **Controller**: Uses `dwb_core::DWBLocalPlanner`.
  - **Planner**: Uses `nav2_navfn_planner/NavfnPlanner`.
  - **Inflation Radius**: Reduced to allow the robot to approach walls and explore corners.

## License

Apache-2.0