# Honeybee Demos 

This package contains a series of real-world applicable demos based on ROS 2 Humble used to capture data for benchmarking the Ryzen AI compute hardware and Honeybee robot's performance. They are also great entry points for robotics applications to show how to setup a robot system, configure Nav2 for a variety of common advanced situations, and design a simple POC autonomy script for research, startups, or prototypers!

![ALT TEXT](./images/opennav_amd_ggb.png)


## Demo 1: High-Speed, Outdoor GPS Navigation

The goal of this demonstration is to show the AMD Ryzen AI compute in action running Nav2 at full speed - 2m/s - outdoors. This is performed on the Presidio main parade lawn in San Francisco, CA because it is a beautiful, generally empty (during weekdays), wide open space in which we can let robots loose at high speeds safely.

TODO image of this experiment

For this demonstration, we use the cheap built-in non-RTK corrected, single antenna GPS to localize the robot to show how to work with Nav2 outdoor with noisy GPS localization. For a refined application, we recommend using an RTK corrected, dual antenna GPS sensor to improve accuracy of localization, positioning tolerances, and allow for persistence of perception data without major jumps. There are many affordable RTK GPS sensors on the market (for example [\[1\]](https://www.digikey.com/en/products/detail/u-blox/ANN-MB-01/9817929?utm_), [\[2\]](https://holybro.com/products/h-rtk-unicore-um982))

For more information, options, and a tutorial on GPS Navigation with Nav2, [see the Nav2 GPS Waypoint Navigation tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html).

### Technical Summary

We set the datum for `robot_localization` to be an arbitrarily selected position on the park in order to ground the localization system near the origin for convenience and such that this application can be repeated using the same waypoints grounded to a consistent coordinate system, as would be necessary for a deployed application. 

We use the `nav2_waypoint_follower` package to follow waypoints in the cartesian frame setup, though can also be done with direct GPS points as well in ROS 2 Iron and newer.

A few important notes on this demonstration's configuration:
- Since we're navigating in non-flat, outdoor 3D spaces, we use a node to segment the ground out from the pointclouds for use in planning/control rather than directly feeding them in with the 3D terrain variations.
- The controller is configured to run at the robot's full speed, 2 m/s. This speed should be used by professionals under supervision if there is any chance of collision with other people or objects. These are dangerous speeds and require attention.
- The planner uses the Smac Planner Hybrid-A* so we can set a conservative maximum turning radius while operating at those speeds so the robot doesn't attempt to flip over due to its own centripetal force. It uses RPP to follow this path closely.
- This demonstration uses a BT that will not replan until its hit its goal or the controller fails to compute trajectories to minimize the impact on localization jumps while the robot is moving and not have 'drunken' behavior
- The global costmap is set to a rolling, static size rather than being set by a map's size or static full field area. This is likely the best configuration for GPS Navigation when not using a map, just ensure that it is sufficiently large to encompass any two sequential viapoints. 
- The positioning tolerances are set comparatively high at 3m due to a noisy non-RTK corrected GPS. Additionally, the perception modules are configured as non-persistent so that the major jumps in localization don't cause series issues in planning for the world model - and we use a 3D lidar with 360 deg coverage. Both of these can be walked back when using RTK for typical uses of Nav2 with GPS localization. 


### Metrics and Data

TODO drone video GIF -- full dataset

TODO metrics / marketing / etc

TODO data set available of full run / gif

An example loop of the main parade ground with noisy GPS data can be seen below. We can still navigate effectively, just need to realistically limit the positional accuracies possible. GPS without RTK can obtain about 3-5m accuracy:

![ALT TEXT](./images/gps.png)

This can be reproduced with the provided rosbag of GPS data

```
ros2 bag play initial_gps_loop_rosbag.db3 --clock 20
ros2 launch honeybee_nav2 gps_localization.launch.py use_sim_time:=True
```

### Notes about operations

Especially with a noisy GPS, its good to let the robot sit with the filter running for a little while before starting up the demo for the filter to converge to its location. I've noticed driving around a little bit to help with that process and converge the orientation from the IMU data.

If reproducing, these are the joystick mappings used on the PS4:
- Estop is 'O'
- Reactivate Estop is Triangle for 5 seconds
- Start demo is Square
- Stop demo is X
- Poweroff backpack is holding PS button for 5 seconds
- Teleop continues to be L1 for slow, R1 for fast deadman with the left stick


## Demo 2: Outdoor, Urban 3D Inspection

TODO Remark on algorithms used and why


A few important notes on the configuration:
- Since we're navigating in non-flat, outdoor 3D spaces, we use a node to segment the ground out from the pointclouds for use in collision avoidance rather than directly feeding them in with the 3D terrain variations.
- Since this is using 3D SLAM, we need to convert the 3D map into a 2D occupancy grid-like thing for sizing the static layer in the global costmap for global planning in the map
- The positioning tolerances & perception modules are set back to normal since we don't have noisy GPS localization and now using 3D lidar SLAM & localization, still outdoors.


## Demo 3: Standard Indoor 

A few important notes on the configuration:
- We move slower in this demonstration - 0.5 m/s - due to being indoors around people for safety
- We use the standard Nav2 localization and SLAM integrations - AMCL and SLAM Toolbox for positioning. We also use largely the standard defaults from Nav2 with the exception of robot specifics like footprint and controller tuning.
- Since we're navigating in a 2D environment, we can use PointCloud to Laserscan to ignore the ground points (which can be noisy on shiny surface). We still use the 3D nature of the lidar by considering a large band from the top of the robot (plus some margin) to a few cm off the ground. This uses the 3D lidar's entire useful data, but reduces computation and noisy points without need for explicit PointCloud filtering or use of the ground segmentation node (less points of potential failure with glass / shiny floors).
- We treat this robot as a large point for global planning as an example with NavFn (oppposed to feasible planners for SE2 footprint checking in other demonstrations). This is a good working example for circular robots or computers with low compute. 
- We use MPPI in this demonstration for highly dynamic behavior in the dynamic human-filled environment
