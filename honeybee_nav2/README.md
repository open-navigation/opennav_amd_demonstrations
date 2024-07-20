# Honeybee Nav2

This package contains launch files and configurations for navigating the robot using Nav2 with:

- 2D localization for indoor environments
- 3D localization for outdoor environments with 3D lidar
- GPS localization for outdoor environments with GPS

Using `nav2.launch.py`, these can be enabled by setting the launch configuration `localization_type` to `2D`, `3D`, `local`, or `GPS` to obtain each. For 2D/3D, the additional launch configuration of `slam` to `True` or `False` will toggle between localization or mapping for these modes. The launch localization type `local` will disable localization and set the `map->odom` transform to Identity in order to do local odometry-only navigation. See launch files for a full list of launch configurations.

For example:

```
ros2 launch nav2.launch.py localization_type:=2D slam:=True
ros2 launch nav2.launch.py localization_type:=GPS
ros2 launch nav2.launch.py localization_type:=3D slam:=False map:=/path/to/map
ros2 launch nav2.launch.py local_nav:=True
```

This also contains custom behavior trees developed for the demonstrations in `behavior_trees`.
