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


## Setup robot Networking, WiFi, Developer Communications

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

## Setup robot hardware bringup

See MD file

Note the following IPs for the internal network:
- Robot base: 192.168.131.1
- AMD Computer: 192.168.131.10
- Ouster Lidar: 192.168.131.20

## Setup robot navigation bringup

To run on the hardware standalone without an existing map, use the `slam` option to use the lidar to generate a map live. Else, specifying the path to your `map:=/path/to/map.yaml` if you'd like to initialize in an existing map: 

```
ros2 launch honeybee_nav2 nav2.launch.py slam:=True
```

To run in simulation, you may do the same but with the addition field `use_sim_time` to set simulation time for the servers appropriately:

```
ros2 launch honeybee_nav2 nav2.launch.py use_sim_time:=True slam:=True
```

## Perform Demonstrations

The demonstrations build upon the above and either launch the simulation to perform the task or launch the real robot as shown in the videos! The demonstrations use Nav2 in various configurations, thus it is not brought up with the robot hardware on boot-up like the robot, sensors, and joystick (though it easily could be with the provided `nav2_bringup.service` systemd service for a particular application).



Mappings for joystick:
- Left joy for teleop with left (L1) rocker for deadman
- Right rocker (R1) for "fast" mode deadman
- Circle is estop to override all and stop robot, likely requiring a reboot
- PS button hold 3s to poweroff the AMD backpack computer
