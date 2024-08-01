## Setup Clearpath Robot WiFi, Developer PC

1. Set a wireless connection with profile named "Jackal" on your developer PC which is a manual IPv4 connection with:

```
address: 192.168.131.100
netmask: 255.255.255.0
Gateway: Leave blank
```

This can be used for a wired connection to the robot.

2. Connect robot to your PC via an ethernet cable

Make sure to select the "Jackal" profile.

3. SSH into the robot's builtin PC (ssh administrator@192.168.131.1) and connect it to your network of choice. Navigate to `/etc/netplan` and modify `60-wireless.yaml` to contain your entry. You can include many network connections.

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

5. Setup backback computer's Wifi

SSH into the robot over wireless or wired networks. Then enter the backpack computer with IP `ssh administrator@192.168.131.10`. Use `nmcli` to connect to your network. For example:

```
sudo nmcli device wifi connect MyWifiNetwork password MyPassword
```

6. Add the IP of the robot computers to your `/etc/host` so you can SSH into it in the future without `.local`

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

Now you can ssh into the robot remotely on this network without password and using a unique robot name!

```
ssh administrator@honeybee
```
