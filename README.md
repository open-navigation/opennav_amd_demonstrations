# Open Navigation - AMD Ryzen AI Demonstrations

This contains demonstrations and analysis using AMD's powerful Ryzen AI CPU and acceleration technologies with Nav2, ROS 2 Humble, and the open-source robotics community's technologies. These demononstrations show complete & tuned reference applications to perform **indoor 2D-based**, **urban 3D-based**, and **outdoor GPS-based navigation** and how AMD's compute technologies are well up to the task with plenty of compute time remaining for advanced AI, business logic and application layers, and other computationally demanding tasks with a low power footprint.

These demonstrations orbit around Honeybee, a [Clearpath Robotics Jackal](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) outfitted with:
- AMD Ryzen Zen4 CPU using a [Miniforum UM790 Pro](https://store.minisforum.com/products/minisforum-um790-pro)
- [Ouster OS0-32](https://ouster.com/products/hardware/os0-lidar-sensor)
- [Realsense D435i](https://www.intelrealsense.com/depth-camera-d435i/)
- [Microstrain GX-25](https://www.microstrain.com/inertial-sensors/3dm-gx5-25)

![ALT TEXT](./honeybee_demos/images/opennav_amd_ggb.png)

This project contains a typical layout for a ROS-based mobile robot:
- `honeybee_description` contains the robot's description, meshes, and frame transformations (URDF)
- `honeybee_gazebo` contains the robot's simulation in modern Gazebo (not Gazebo Classic) with the full sensor suite
- `honeybee_bringup` contains the bringup scripts to launch the robot's base and sensors on the physical hardware and/or simulation
- `honeybee_watchdogs` contains a set of watchdogs for hardware use, such as checking on the state of lifecycle activations and recording background datasets
- `honeybee_nav2` contains the navigation configurations for the various demonstrations
- `honeybee_demos` contains the demo scripts, launch files, and so forth to perform the applications. These would be notionally replaced by business logic for a refined, deployed application.
- `scripts` contain developer scripts used by Open Navigation to perform the demonstrations which have potential useful value to the community in getting started

Bonus: `docs` contains a number of developer guides for bootstrapping new computers for robots, network setup with ROS 2, setting up field experimental networks, how to visualize data remotely, make software run on startup, and so on.

**[See the `honeybee_demos` package for detailed demonstration descriptions, videos, and datasets](./honeybee_demos/README.md)**

TODO Have the marketing gif here
(ground, drone, urban, park, indoor, rviz)

## Launching Robot, Nav2, and Demos

The robot can be launched using `ros2 launch honeybee_bringup robot.launch.py` with the `use_simulation` launch configuration option to specify whether using the physical robot (default) or simulated robot (`use_simulation:=True`). This will bringup the full robot system and/or simulation with sensors.

The navigation system can be launched using `ros2 launch honeyee_nav2 nav2.launch.py` with a number of launch options, such as the localization type to use (3D, 2D, GPS, Local Navigation), simulation status, parameters, SLAM, and so forth.

The demonstrations can be launched using their respective launch files in `honeybee_demos` and utilize Nav2 configured for the particular application, the annotated autonomy scripts developed for the demonstrations, and appropriate watchdogs for data recording and system handling.

See launch files for a full set of launch configurations and options!

---

ANALYSIS HERE on performance (charts/graphs: CPU )

---

## Build 

TODO --> in another page linked

You need to build a few things from source because clearpath depends on unreleased software... Or setup using their repos:

Clearpath specific assets
- https://github.com/clearpathrobotics/clearpath_common
- https://github.com/clearpathrobotics/clearpath_robot/

```
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
sudo apt update
rosdep update
```


## Setup robot Networking, WiFi, Developer Communications

TODO -> another page linked from docs

1. Set a wireless connection with profile named "Jackal" which is a manual IPv4 connection with:

```
address: 192.168.131.100
netmask: 255.255.255.0
Gateway: Leave blank
```

2. Connect robot to your PC via an ethernet cable

3. SSH into the robot (ssh administrator@192.168.131.1) and connect it to your network of choice. Navigate to `/etc/netplan` and modify `60-wireless.yaml` to contain your entry. You can include many network connections.

```
networks:
  wifis:
    wlp2s0:
      optional: true
      access-points:
        <ssid>:
          password: <password>
      dhcp4: yes
      dhcp-overrides:
        send-hostname: true

```

Then, apply with `sudo netplan apply --debug` and ensure no meaningful errors appear. Wait a few seconds and `ping google.com` to make sure the connection is live.

4. Disconnect the to main computer from ethernet. Power cycle the robot. Once back up, ssh into it using its `.local` address, using your serial number

```
ssh administrator@cpr-j100-0842.local
```

If this does not work, you can use nmap to find all the devices connected to your wireless and manually find the right device.

```
nmap -sP 192.168.1.*/24
```

Log back out.

5. Add the IP of the robot to your `/etc/host` so you can SSH into it in the future without `.local`

```
127.0.0.1       localhost
127.0.1.1       reese
192.168.1.64    nova
192.168.1.17    honeybee # new entry
```

6. Enable passwordless SSH by setting up ssh keys between your computer and the robot

```
ssh-keygen -t rsa # hit enter to all prompts without input
ssh-copy-id administrator@honeybee
```

---

Now you can ssh into the robot remotely on this network without password and using a unique robot name!

```
ssh administrator@honeybee
```

## Details on Robot

See MD file

TODO diagram of setup might be nice
Note the following IPs for the internal network:
- Robot base: 192.168.131.1
- AMD Computer: 192.168.131.10
- Ouster Lidar: 192.168.131.20

TODO diagram
Mappings for joystick used by the robot base, demonstration watchdogs, and autonomy scripts:
- Estop is 'O'
- Reactivate Estop is Triangle for 5 seconds
- Start demo is Square and start rosbag record
- Stop demo is X and stop rosbag record
- Poweroff backpack is holding PS button for 5 seconds
- Left joy for teleop with left (L1) rocker for deadman
- Right rocker (R1) for "fast" mode deadman

Assumes the workspace in root `amd_ws` for scripts and systemd daemons, but can be configured with addtl arguments / changed easily. Not tied into any scripts.

Assumes data to be recorded by watchdogs into a `experiment_files` directory, which the copy/delete data scripts use

