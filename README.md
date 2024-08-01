# Open Navigation - AMD Ryzen AI Demonstrations

This project has demonstrations and analysis using AMD's powerful Ryzen CPU, AI, and acceleration technologies with Nav2, ROS 2 Humble, and the open-source robotics community's technologies. These demononstrations show complete & tuned reference applications to perform **indoor 2D-based**, **urban 3D-based**, and **outdoor GPS-based** navigation. They use AMD's compute technologies and show that they are very well suited to robotics tasks and workloads, with plenty of compute time remaining for AI, business logic, application layers, and other computationally demanding tasks on top of advanced mobility and 3D perception. 

These demonstrations orbit around the Honeybee reference platform, a [Clearpath Robotics Jackal](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) outfitted with:
- AMD Ryzen Zen4 CPU using a [Miniforum UM790 Pro](https://store.minisforum.com/products/minisforum-um790-pro)
- [Ouster OS0-32](https://ouster.com/products/hardware/os0-lidar-sensor)
- [Realsense D435i](https://www.intelrealsense.com/depth-camera-d435i/)
- [Microstrain GX-25](https://www.microstrain.com/inertial-sensors/3dm-gx5-25)

[Demonstration 1: Outdoor GPS Navigation](./honeybee_demos/honeybee_demos/gps_patrol_demo.py) | [Demonstration 2: Urban 3D Navigation](./honeybee_demos/honeybee_demos/urban_navigation_demo.py) 
:-------------------------:|:-------------------------:
[![ALT TEXT](./honeybee_demos/images/demo1_gif.gif)](https://www.youtube.com/watch?v=255o4IS3rHg) |  [![ALT TEXT](./honeybee_demos/images/demo2_gif.gif)](https://www.youtube.com/watch?v=sL2GZdODUcE)
[**Demonstration 3: Long-Duration Indoor Navigation**](./honeybee_demos/honeybee_demos/indoor_long_duration_picking_demo.py) | **Glamour Shot** |
[![ALT TEXT](./honeybee_demos/images/demo3_gif.gif)](https://www.youtube.com/watch?v=evZ-GvswU4o) | <img src="./honeybee_demos/images/opennav_amd_ggb.png" width="500">

**Click on the demo gifs to see the full videos on YouTube!**

This project contains a typical layout for a ROS-based mobile robot:
- `honeybee_description` contains the robot's description, meshes, and frame transformations (URDF)
- `honeybee_gazebo` contains the robot's simulation in modern Gazebo with the full sensor suite
- `honeybee_bringup` contains the bringup scripts to launch the robot's base and sensors on the physical hardware and/or simulation
- `honeybee_watchdogs` contains a set of watchdogs for hardware use, such as checking on the state of lifecycle activations and recording background datasets
- `honeybee_nav2` contains the navigation configurations for the various demonstrations
- `honeybee_demos` contains the demo scripts, launch files, and so forth to perform the applications. These would be notionally replaced by business logic for a refined, deployed application.
- `scripts` contain developer scripts used by Open Navigation to perform the demonstrations which have potential useful value to the community in getting started

Bonus: `docs` contains a number of developer guides for bootstrapping new computers for robots, network setup with ROS 2, setting up field experimental networks, how to visualize data remotely, make software run on startup, and so on.

**[See the `honeybee_demos` package for detailed demonstration descriptions, videos, and datasets](./honeybee_demos)**

<video src="https://github.com/user-attachments/assets/9eef0d12-7b01-4654-be78-96281b261b64" controls autoplay loop></video>

## Launching Robot, Nav2, and Demos

The robot can be launched using `ros2 launch honeybee_bringup robot.launch.py` with the `use_simulation` launch configuration option to specify whether using the physical robot (default) or simulated robot (`use_simulation:=True`). This will bringup the full robot system and/or simulation with sensors.

The navigation system can be launched using `ros2 launch honeyee_nav2 nav2.launch.py` with a number of launch options, such as the localization type to use (3D, 2D, GPS, Local Navigation), simulation status, parameters, SLAM, and so forth.

The demonstrations can be launched using their respective launch files in `honeybee_demos` and utilize Nav2 configured for the particular application, the annotated autonomy scripts developed for the demonstrations, and appropriate watchdogs for data recording and system handling.

See launch files for a full set of launch configurations and options!

## Metrics

TODO metrics / marketing / etc - 'so much compute left over while navigating urban with 3D slam/localization, 3d lidar processing, autonomy, at 2m/s. etc. Super powerful machine, love the opportunity this presents in a 80W package'. 

ANALYSIS HERE on performance (charts/graphs: CPU )

## Build 

This is straight forward to build and work with. Clone this repository into your workspace:

```
mkdir -p amd_ws/src
cd amd_ws/src
git clone git@github.com:open-navigation/opennav_amd_demos.git
```

Then, we need to pull in some dependencies that we cannot obtain from `rosdep`:

```
sudo apt install python3-vcstool  # if don't already have
vcs import . < opennav_AMD_demos/deps.repos
cd ouster-lidar/ouster-ros && git submodule update --init
cd ../../../
```

Next, we need to obtain our dependencies that are available from `rosdep`:

```
rosdep init  # if haven't done
rosdep update
rosdep install -r -y --from-paths src --ignore-src
```

Now, we can build using colcon:

```
colcon build
```


TODO: docs directory for guides


## Details on Robot

The robot has an internal network on the 192.168.131.* range.
- The robot's builtin PC is `192.168.131.1` with username `administrator` & password `clearpath`
- The AMD backpack PC is `192.168.131.10` with username `administrator` & password `clearpath`
- The ouster lidar is `192.168.131.20` 

The Lidar is connected to the builtin PC due to limitations on the number of ethernet ports on the AMD computer (but could be easily remedied by an ethernet switch).

![ALT TEXT](./docs/hardware_design.png)

The controller has the custom layout shown in the diagram below. Various nodes across the system subscribe to the joystick topic to activate these features (i.e. teleop & estop launch with base bringup; poweroff and demo launches with demos).

![ALT TEXT](./docs/PS4_Layout.png)

The daemons that bringup the robot assumes that the workspace is located in `~/amd_ws` for sourcing to launch the resources. This can be easily changed by updating the services for the new workspace location in `honeybee_bringup/systemd` and [following the guide to setup robot bringup](./docs/setup_robot_automatic_bringup.md).

Data from the experiments are recorded and logged by the `nav2_watchdogs` in the `~/experiment_files` directory by default. These have a parameter `filepath` which can be set to use alternative file paths, however the scripts for copying and clearing old data use this filepath as well (but are trivial to update with a new path).

Note: each robot has a `colcon_ws` setup by Clearpath and is a hardcoded path with their auto-generation scripts. It is recommended to not touch this directory to allow for a complete rollback to on-delivery state should issues occur requiring Clearpath's intervention.
