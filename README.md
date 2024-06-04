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

Parent launch file for robot base `ros2 launch honeybee_bringup robot.launch.py` with options:
- `use_simulation` whether to launch on hardware or in simulation with appropriate nodes
- `use_sim_time` TODO properly propogated
