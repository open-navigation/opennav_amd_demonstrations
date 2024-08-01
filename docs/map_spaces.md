# How to Map and Use Map for Autonomous Navigation

The following is a walkthrough for how to map a space and later use that map for autonomous navigation.

1. Map a space

Using 2D SLAM for example, map a space by first powering on the robot and bringing up its hardware. Its odometry, sensors, and control systems should be online.

Open Rviz, set the global frame to `map`, and visualize the laser scanner and robot model.

Run SLAM Toolbox via the command below and wait for a map to appear in rviz (a couple of moments):

```
ros2 launch honeybee_nav2 slam_toolbox.launch.py
```

Drive the robot around with the joystick until a map is obtained. Optionally: launch `nav2.launch.py slam:=True localization_type:=2D` to be able to use Nav2 to autonomously navigate to positions while mapping.

2. Save map

Use the provided `scripts/save_map` script to save a map to your `~/experiment_files` directory.

3. Launch navigation using map

Finally, this map can be used for navigation via

```
ros2 launch honeybee_nav2 nav2.launch.py map:=/home/administrator/experiment_files/map_XXX.yaml localization_type:=2D slam:=False
```

This should bring up Nav2, AMCL. Set the initial robot estimate in Rviz (or program using `/initialpose` topic) to roughly localize the robot. Then, start navigating!
