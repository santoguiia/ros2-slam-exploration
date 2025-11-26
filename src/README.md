# Source Directory

This directory is for custom ROS2 nodes specific to this SLAM exploration project.

## Potential Custom Nodes

- **exploration_controller**: Autonomous exploration algorithms
- **map_saver_node**: Automatic map saving during exploration
- **goal_planner**: Custom goal planning for exploration

## Adding Custom Nodes

1. Create your Python or C++ node in this directory
2. Update the `CMakeLists.txt` to include your node
3. Rebuild the package with `colcon build`
