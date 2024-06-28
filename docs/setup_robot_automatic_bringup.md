# How to Automatically Startup Your Robot

The following is a short tutorial on how to setup your robot application software to startup systems automatically on power on. It focuses on the Honeybee application, but can be easily modified for your individal applications.

`honeybee_bringup/systemd` contains the services for bringing up the robot hardware and navigation system on power on. The `*.service` files are systemd services that will boot on at startup after the networking stack is up to auto-launch the robot's systems on power on automatically.

Place each of the service files you would like to use into `/etc/systemd/system`. Modify for your desired launch files if needed.

These services can be enabled via `systemctl enable <service>` to launch on startup (or `disable` to stop it) and manually `start`-ed for a single run (and subsiquently `stop`-ed). 

Once running, you can see the output via `journalctl -b -u  <service>` (`-b` for current boot logs only).

Note: this relies on the workspace `amd_ws` in the root directory of the robot containing the necessary work.

You can test this by powering off the computer and powering it back on. After the system is fully initialized, you should be able to see the system started up (i.e. see data with `ros2 topic list`, joystick your robot if you have setup to launch, etc).

## Services provided

- `robot_bringup.service` brings up the robot's hardware
- `nav2_bringup.service` bring up Nav2, which launches after `robot_bringup.service`

