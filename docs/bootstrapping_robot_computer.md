## How to bootstrap a new computer for a robot

The following is a simple tutorial for setting up a new computer as a robot system for an AMR.

1. Setup the robot computer to boot on power up

Its important that your robot computer turns on when the AMR turns on. The easiest way to do that is to setup your computer to boot up on power on. That way, when power is available, you automatically boot on. Alternatively, most computers come with pins for indicating externally to power on or shut down.

- Hookup your computer to an external monitor / keyboard and when it boots up, enter the BIOS.
- Navigate to the Power tab and set `After Power Failure` or similar to On.
- Exit the BIOS and power on as normal and shut down the computer.
- Test: Unplug and plug the computer back in, see that the lights indicating boot up are now on.

Note: Some computers also have the option to Wake on LAN. If you'd prefer to do this, enable that option in the BIOS and have either another computer or your base power system send a Wake on LAN packet to the computer to power it on via explicit command.

2. Setup the robot computer to automatically log in on startup

It is important that your robot computer is ready to go automatically and not waiting for user input before completing startup. So, we need to set your computer to automatically login.

- In Settings, go to System and select the User that you want to have enter on startup
- Unlock the settings with administrator passwords and turn on the option for `Automatic Login` to `On`.
- Test: Power off the computer, unplug power, and plug it back in. See that the computer boots up and automatically logs into your desired user.

3. Setup Networking

It is necessary to setup your networking to access the computer remotely once installed into the robot platform and interact with sensors.

WiFi:

- While plugged into a monitor, setup wifi using the Network Manager GUI client
- If you are doing this without a monitor, use the `nmcli` tool instead.

Wired:

If you have sensors that require ethernet access, it is wise to setup an internal network (such as `10.2.0.*`) to communicate over rather the exposing it on your wireless network channels. 

- Enter the Network Manager GUI and setup a new wired connection
- Name it "robot internal network" and enter the `IPv4` tab
- Select Manual for the `IPv4 Method`
- Set the IP to be `10.2.0.1` (or whatever you like), `255.255.255.0` as the netmask, and leave the gateway blank
- Setup any sensors or other computers on the internal wired network to have a similar profile with a unique IP in the `10.2.0.*` range.

Now, plug the computer into your Ethernet switch, sensor, etc and see that this profile is used. Test by launching a driver or attempting to `ssh` into another computer on the wired network to verify this is setup properly.


4. Setup ROS and application specific software

You are now ready to setup the software elements of your application. Install your desired ROS version, dependencies, and build/install/etc your application work for use.

---

If you have made it this far, you are now ready to setup your robot to automatically bringup the platform and sensors after the computer is started up. See `setup_robot_automatic_bringup.md` next.
