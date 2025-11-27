# Maps Directory

This directory is used to store generated map files from SLAM.

## Saving Maps

To save a map during or after SLAM exploration, use the following command:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

Or with the SLAM Toolbox, you can save the map using the RViz plugin or by calling the service:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "filename: '/path/to/your/map'"
```

## Map Files

Generated maps typically consist of:
- `.pgm` - The grayscale image of the map
- `.yaml` - Configuration file containing map metadata
