# Open Navigation - AMD Ryzen AI Demonstrations

A project for demonstrations using AMD's Ryzen AI CPU and acceleration technologies with Nav2, ROS 2, and the robotics industry.

These demonstrations orbit around Honeybee, a Clearpath Robotics Jackal outfitted with:
- AMD Ryzen Zen4 CPU
- Ouster OS0-32 lidar
- Realsense D435i depth camera
- Microstrain GX-25

![Honeybee](./honeybee_description/docs/ona01_jackal.png)


Clearpath specific assets
- https://github.com/clearpathrobotics/clearpath_common
- https://github.com/clearpathrobotics/clearpath_robot/


Steve Notes:
- How to use the config/robot.yaml file? Just on the robot or something for offline / sim as well? Why need workspace location / how is this file used on the robot? Auto-bringup of everything included? System info? What if change RMW? Used to generate URDF? https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview
- Generator? https://docs.clearpathrobotics.com/docs/ros/config/generators/ 
- 

AMD

- [ ] simulation

- [ ] DOCUMENT for future reference for myself and others

- [ ] Ouster

- [ ] bringup launch files for robot
- [ ] daemon for bringup
- [ ] nav2 launch files 
- [ ] demo files

- [ ] remote communication & vis for demos
- [ ] 

... demos and such after basic setup





To upgrade to Jazzy
- Replace Ignition with Gazebo (prob own j100 + sensor URDFs)
- Base microros for jazzy reflash
- ...
