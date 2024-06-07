# Open Navigation - AMD Ryzen AI Demonstrations

A project for demonstrations using AMD's Ryzen AI CPU and acceleration technologies with Nav2, ROS 2, and the robotics industry.

These demonstrations orbit around Honeybee, a Clearpath Robotics Jackal outfitted with:
- AMD Ryzen Zen4 CPU
- Ouster OS0-32 lidar
- Realsense D435i depth camera
- Microstrain GX-25

TODO update with sim/hardware
![Honeybee](./honeybee_description/docs/ona01_jackal.png)


Clearpath specific assets
- https://github.com/clearpathrobotics/clearpath_common
- https://github.com/clearpathrobotics/clearpath_robot/

Parent launch file for robot base `ros2 launch honeybee_bringup robot.launch.py` with options:
- `use_simulation` whether to launch on hardware or in simulation with appropriate nodes


## Build 

TODO

You need to build a few things from source because clearpath depends on unreleased software... Or setup using their repos:

```
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
sudo apt update
rosdep update
```


## Setup robot on wifi

1. Connect robot to your PC via an ethernet cable
2. SSH into the robot (ssh admin@IP_ADDRESS) and connect it to your network of choice using `nmcli`
3. Disconnect the to main computer from ethernet and ssh via the `.local` name of the robot PC to make sure it works
4. Add the IP of the robot to your `/etc/host` so you can SSH into it in the future without `.local`


## Setup robot hardware bringup

## Setup robot navigation bringup

## Perform Demonstrations

