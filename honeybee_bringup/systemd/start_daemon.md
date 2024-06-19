# Start robot bringups

This contains the daemons for bringing up the robot hardware, navigation system and demo softwares in the robot on power on. The `*.service` files are systemd services that will boot on at startup after the networking stack is up to auto-launch the robot's systems on power on automatically.

Place each of the service files into `/etc/systemd/system`

These services can be enabled via `systemctl enable <service>` to launch on startup (or `disable` to stop it) and manually `start`-ed for a single run (and subsiquently `stop`-ed). 

Once running, you can see the output via `journalctl -u  <service>`.

Note: this relies on the workspace `amd_ws` in the root directory of the robot containing the necessary work.

## Services provided

- `robot_bringup.service` brings up the robot's hardware
- `nav2_bringup.service` bring up Nav2

