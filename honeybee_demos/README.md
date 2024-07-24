# Honeybee Demos 

This package contains a series of real-world applicable demos based on ROS 2 Humble used to capture data for benchmarking the Ryzen AI compute hardware and Honeybee robot's performance. They are also great entry points for robotics applications to show how to setup a robot system, configure Nav2 for a variety of common advanced situations, and design a simple POC autonomy script for research, startups, or prototypers!

## Demo 1: High-Speed, Outdoor GPS Navigation

The goal of this demonstration is to show the AMD Ryzen AI compute in action running Nav2 at full speed - 2m/s - outdoors. This is performed on the Presidio main parade lawn in San Francisco, CA because it is a beautiful, generally empty (during weekdays), wide open space in which we can let robots loose at high speeds safely. This demonstration uses GPS to localize the robot while performing a patroling task in a loop and taking some measurements at each point of interest. 

[![ALT TEXT](./images/demo1.JPG)](https://www.youtube.com/watch?v=255o4IS3rHg)

**Note: Click on the image above to see the GPS Patrol demo in action on YouTube!**

For more information, options, and a tutorial on GPS Navigation with Nav2, [see the Nav2 GPS Waypoint Navigation tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html).

#### Technical Summary

We set the datum for `robot_localization` to be an arbitrarily selected position on the park in order to ground the localization system near the origin for convenience and such that this application can be repeated using the same waypoints grounded to a consistent coordinate system, as would be necessary for a deployed application. The experiment was performed over multiple days and this works well.

We use the `nav2_waypoint_follower` package to follow waypoints in the cartesian frame setup, though can also be done with direct GPS points as well in ROS 2 Iron and newer. We pause a few seconds at each waypoint to capture some patrol data.

A few important notes on this demonstration's configuration:
- Since we're navigating in non-flat, outdoor 3D spaces, we use a node to segment the ground out from the pointclouds for use in planning/control rather than directly feeding them in with the 3D terrain variations.
- The controller is configured to run at the robot's full speed, 2 m/s. This speed should be used by professionals under supervision if there is any chance of collision with other people or objects. These are dangerous speeds and require attention.
- The planner uses the Smac Planner Hybrid-A* so we can set a conservative maximum turning radius while operating at those speeds so the robot doesn't attempt to flip over due to its own centripetal force. It uses RPP to follow this path closely.
- The global costmap is set to a rolling, static size rather than being set by a map's size or static full field area. This is likely the best configuration for GPS Navigation when not using a map, just ensure that it is sufficiently large to encompass any two sequential viapoints. 
- This demonstration uses a BT that will not replan until its hit its goal or the controller fails to compute trajectories to minimize the impact of ~1-3m localization jumps on robot motion. This prevents 'drunken' behavior due to non-corrected GPS data's noise.
- The positioning tolerances are set comparatively high at 3m due to the noisy non-RTK corrected GPS. Additionally, the perception modules are configured as non-persistent so that the major jumps in localization don't cause series issues in planning for the world model - we use a 3D lidar, so we have good real-time 360 deg coverage. Both of these could be walked back when using RTK for typical uses of Nav2 with GPS localization. 

#### Metrics

TODO metrics / marketing / etc - 'so much compute left over while navigating at 2m/s, 3d lidar processing, autonomy, etc. Super powerful machine, love the opportunity this presents in a 80W package'. Or... consoliate into the README so its up front for all 3 demos combined concisely?

#### Dataset

The raw data from the robot during an approximately ~5 minute patrol loop including odometry, TF, commands, sensor data, and so forth can be [downloaded in this link.](https://drive.google.com/file/d/110dgsD_lPXHl7Hn6XcIk0RsIgyCl47Ws/view?usp=sharing)

![ALT TEXT](./images/demo1_dataset.gif)

An example loop of the full length of the parade lawn with noisy GPS data can be seen below for illustration purposes. We can still navigate effectively, however we need to be realistic about the limits on positional accuracies possible. GPS without RTK can obtain about 3-5m accuracy, but will jump:

<img src="./images/gps.png" width="480">

This can be reproduced with the [provided rosbag of odometry, GPS data for state estimation](https://drive.google.com/file/d/1sAm1_xIj3lyX5AkacrWUED3nsdiD7rKJ/view?usp=sharing).


```
ros2 bag play initial_gps_loop_rosbag.db3 --clock 20
ros2 launch honeybee_nav2 gps_localization.launch.py use_sim_time:=True
```

#### Notes on GPS

For this demonstration, we use the cheap built-in non-RTK corrected, single antenna GPS to localize the robot to show how to work with Nav2 outdoor with noisy GPS localization. For a refined application, we recommend using an RTK corrected, dual antenna GPS sensor to improve accuracy of localization, positioning tolerances, and allow for persistence of perception data without major jumps. There are many affordable RTK GPS sensors on the market, [for example](https://holybro.com/products/h-rtk-unicore-um982).

With the GPS, its good to let the robot sit with the filter running for a little while before starting up the demo for the filter to converge to its location solidly before starting. I've noticed driving around a little bit to help with that process and converge the orientation from the IMU data.


## Demo 2: Outdoor, Urban 3D Navigation

The goal of this demonstration is to show the AMD Ryzen AI compute in action running Nav2 using 3D SLAM and localization in an urban setting. This is performed on Alameda, an island in the San Francisco bay, because it has a large urban area formerly hosting a Naval Air Station which is perfect for experimentation without much through-traffic. This demonstration maps and localizes the robot within a few city blocks on Alameda and routes the robot along the roadways to go from one building to another representing an urban-navigation use-case.

TODO video
[![ALT TEXT](./images/demo2.JPG)](https://www.youtube.com/watch?v=255o4IS3rHg)

**Note: Click on the image above to see the Urban Navigation demo in action on YouTube!**

This is the experiment's location on Alameda with drone footage showing the area from the sky for scale. The loop is approximately 1km in length and takes the robot about ~10 minutes to navigate around the entire space. On the navigation graph, we can visit a few intersections, a [very nice](https://humblesea.com/) brewery, Alameda's city hall, and a fire training station.

<img src="./images/urban_layout_rotated.jpg" height="270"> <img src="./images/demo2_godview.gif" width="480">


#### Technical Summary

We use (TODO 3D SLAM/LOCALIZATION USED) for 3D localization and SLAM using the Ouster 3D lidar. The OS-0 has a lower range than its OS-1 and OS-2 counterparts (in exchange for a wide vertical FOV), thus this demonstration is performed in an area where buildings and structures can be effectively seen within the limited ~30 meter effective range.

The space can be mapped either autonomously using Nav2 or using a manually controlled data collection run. After the space is mapped, we can use localization to obtain the position of the robot relative to the 3D map generated. This also allows us to annotate the map for navigable space, define points of interest for navigation, or other application-specific needs.

TODO on planner/controller configurations: 2m/s too on roadways (max)

A few important notes on this demonstration's configuration:
- Since we're navigating in non-flat, urban 3D spaces, we use a node to segment the ground out from the pointclouds for use in planning/control rather than directly feeding them in with the 3D terrain variations.
- The global costmap is set by the 3D map's sizing. While the map generated by 3D SLAM is not usually a 2D probablistic occupancy grid, we can obtain an occupancy grid from it to use for long-term planning by taking a band of the 3D map's height ranges and feeding that into the global costmap as the map. If the space is highly non-planar, other methods (likely AI based) should be used to segment ground from non-ground points on the map instead, should freespace planning be required rather than planning on an annotated navigation route graph based on the map.
- The positioning tolerances are set to be 'normal' for indoor 2D applications of 30cm due to the good accuracy of the 3D localization (opposed to uncorrected GPS) to show Nav2 can obtain these accuracies outdoors - its a matter of provided localization accuracy.
- In this demonstration, we use a navigation route graph of the streets and blocks for long-term planning in place of freespace planning on the roadways (i.e. like an AV might). This may be replaced in another application with freespace planning and semantic segmentation to determine navigable spaces live during execution instead and provide more freedom for freespace planning off of roadways.


#### Metrics

TODO metrics / marketing / etc - 'so much compute left over while navigating urban with 3D slam/localization, 3d lidar processing, autonomy, etc. Super powerful machine, love the opportunity this presents in a 80W package'. Or... consoliate into the README so its up front for all 3 demos combined concisely?

#### Dataset

TODO full dataset from actual run + 3D slam hand-driven only dataset
TODO the google maps overlay of the annotated intersections / waypoints (or just 3D map instead, no google?)

#### Notes on 3D Mapping, Planning

For this demonstration, several 3D SLAM and localization solutions were analyzed. Unfortunately, relatively few were robust enough for use and the 3D SLAM selected was the best performing for this robot platform, its sensors, and configuration. While we found 'good enough' success with this method, it is easy to swap in another solution that you prefer as long as it provides the pointcloud map and TF transformations from `map` -> `odom` per REP-105. The OS-0 was particularly challenging to work with outdoors due to its relatively lower range and density.

Planning in Urban spaces can be done 2 primay ways:
- Using freespace algorithms like the NavFn or Smac Planners which allow for planning in all non-occupied spaces constrained by some behavioral constraints like inflation, keepouts, or higher cost regions
- Using route algorithms that take in a pre-defined navigation graph of nodes and edges representing navigable areas (think: google maps routing)

Thus, which way you desire depends on the application behavior in mind, the level of annotation possible, complexity of the environment or 3D map, and compute constraints on the platform to run AI-segmentation methods to determine navigability at run-time, and so on. This demonstration performs only one of these methods but it is worth noting that both are very easily possible with Nav2 and the structure of demonstration. We choose to show the demonstration with navigation graphs to provide additional, different technology demonstrations.

## Demo 3: Standard Indoor 

A few important notes on the configuration:
- We move slower in this demonstration - 0.5 m/s - due to being indoors around people for safety
- We use the standard Nav2 localization and SLAM integrations - AMCL and SLAM Toolbox for positioning. We also use largely the standard defaults from Nav2 with the exception of robot specifics like footprint and controller tuning.
- Since we're navigating in a 2D environment, we can use PointCloud to Laserscan to ignore the ground points (which can be noisy on shiny surface). We still use the 3D nature of the lidar by considering a large band from the top of the robot (plus some margin) to a few cm off the ground. This uses the 3D lidar's entire useful data, but reduces computation and noisy points without need for explicit PointCloud filtering or use of the ground segmentation node (less points of potential failure with glass / shiny floors).
- We treat this robot as a large point for global planning as an example with NavFn (oppposed to feasible planners for SE2 footprint checking in other demonstrations). This is a good working example for circular robots or computers with low compute. 
- We use MPPI in this demonstration for highly dynamic behavior in the dynamic human-filled environment

TODO keepouts