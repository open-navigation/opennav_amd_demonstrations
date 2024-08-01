## How to bootstrap a new computer for a robot

The following is a simple tutorial for setting up a new computer as a robot system for an AMR.

0. Install some useful tools

Optional, but some useful tools to have around and setup SSH for remote access over wifi or wired connections

```
sudo apt install htop nmap net-tools openssh-client openssh-server tmux
sudo systemctl enable --now ssh
sudo systemctl start ssh
```

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

## WiFi

We won't belabor this point.

- While plugged into a monitor, setup wifi using the Network Manager GUI client
- If you are doing this without a monitor, use the `nmcli` tool instead. For example `sudo nmcli device wifi connect MyWifiNetwork password MyPassword`

## Wired

If you have sensors that require ethernet access or have other computers within the robot that need to network, it is wise to setup an internal network (such as `10.2.0.*`) to communicate over rather the exposing it on your wireless network channels. 

- Enter the Network Manager GUI and setup a new wired connection
- Name it "robot internal network" and enter the `IPv4` tab
- Select Manual for the `IPv4 Method`
- Set the IP to be `10.2.0.1` (or whatever you like), `255.255.255.0` as the netmask, and leave the gateway blank
- Setup any sensors or other computers on the internal wired network to have a similar profile with a unique IP in the `10.2.0.*` range.

Now, plug the computer into your Ethernet switch, sensor, etc and see that this profile is used. Test by launching a driver or attempting to `ssh` into another computer on the wired network to verify this is setup properly.

If you want the robot computers to be able to SSH into each other without passwords, use `ssh-copy-id` from both computers to each other.

### ROS 2

Now that you have the computers able to connect over the internal wired network, now we may need to configure ROS 2 to use this internal network **rather than** the wireless network. This is so that you get high speed wired transport of the data between the computers rather than communicating over the external wireless router. 

This has 2 options:
- Set the wired connection to be higher priority to send data through
- Disable wireless transport altogether for on-board communication only, which may be preferable for security and network performance reasons with many robots.

We may need to configure the DDS vendors to do this. By default, if your wired connection is a higher priority than the wireless, DDS will prefer that route and you're all set. You can find your priorities via `ip route show`, in which the lowest value is of the highest priority. If no priority level is shown for a connection, its priority is `0`.

If you want to disable wireless use, see below:

If using Fast-DDS, save a file containing the following and add `export FASTDDS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml` (TODO or `FASTRTPS_DEFAULT_PROFILES_FILE`?) to your `~/.bashrc` file so that each time a terminal opens, it uses this configuration automatically without your need to set it each time:

``` xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
  <profiles>
    <transport_descriptors>
      <!-- Adding a whitelisted UDPv4 -->
      <transport_descriptor>
        <transport_id>udpv4_transport</transport_id>
        <type>UDPv4</type>

        <!-- For ROS 2 versions Jazzy and later -->
        <!-- Modify fowithr your interface(s) of choice! eno, enx, br0, etc-->
        <!-- <interfaces>
          <allowlist>
            <interface name="wlp59s0"/>
          </allowlist>
        </interfaces> -->

        <!-- For ROS 2 versions prior to Jazzy -->
        <!-- Modify with the wired IP address on this computer! 10.2.0.1, etc -->
        <!-- <interfaceWhiteList>
          <address>10.2.0.1</address>
        </interfaceWhiteList> -->
       
      </transport_descriptor>

      <!-- Adding an SHM transport so you keep the same transport as the default -->
      <transport_descriptor>
        <transport_id>shm_transport</transport_id>
        <type>SHM</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="participant_with_allowlist" is_default_profile="true">
      <rtps>
        <useBuiltinTransports>false</useBuiltinTransports>
        <userTransports>
          <transport_id>udpv4_transport</transport_id>
          <transport_id>shm_transport</transport_id>
        </userTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
```

Thanks to eProsima for their help in configuring and understanding these files! 

If using Cyclone DDS, do the same with the following file's contents and `export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml` instead:

``` xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain>
    <General>
      <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress> <!-- Or enx or ... -->
    </General>
  </Domain>
</CycloneDDS>
```

Note: if you do this, you will no longer be able to see the data on your wireless network from a remote PC. You'll need to either SSH into the robot, communicate via a cloud provider, or install a router that you can connect to on the internal network. This is probably a good thing for most users anyway since exposing your ROS 2 system to the entire WAN is probably not a good option and you'd have to set `ROS_LOCALHOST_ONLY` anyway. This way, the network is effectively trapped on localhost.


4. Setup ROS and application specific software

You are now ready to setup the software elements of your application. Install your desired ROS version, dependencies, and build/install/etc your application work for use.

---

If you have made it this far, you are now ready to setup your robot to automatically bringup the platform and sensors after the computer is started up. See `setup_robot_automatic_bringup.md` next.



------------------

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