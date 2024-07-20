# Honeybee Bringup

This contains the core bringup files for the robot system. The robot hardware and/or full simulation can be brought up with the following as the main entrypoint:

```
ros2 launch honeybee_bringup robot.launch.py
```

When the launch argument `use_simulation:=true`, it will launch gazebo simulation rather than hardware.

The `sensors.launch.py` launches the robot's sensors and processing pipelines (Microstrain IMU, Ouster Lidar, Realsense D435) and `base.launch.py` takes care of the clearpath base, state estimation, control, and teleoperation subsystems.
