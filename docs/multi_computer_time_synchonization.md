## How to time synchronize two LAN computers

The following is a simple guide for setting up time sychronization on 2 computers connected over LAN. For this, we use Chrony as the best and standard tool for robotics with smooths out clock jumps.

0. Install Chrony

On both computers, install chrony

```
sudo apt install chrony
```

1. Setup the Chrony Server Computer

Select one of the computers to be the server and the other the client (may have multiple clients). On the server computer, edit `/etc/chrony/chrony.conf` to contain:

```
# Allow NTP client access from local network
# Replace with your LAN address range
allow 192.168.131.0/24

# Enable the server to act as an NTP server
local stratum 10
```

Then, restart chrony `sudo systemctl restart chrony`

2. Setup Chrony Client Computer(s)

On the client computer(s), edit `/etc/chrony/chrony.conf` to contain:

```
# Comment out existing server lines (if any)
# server 0.ubuntu.pool.ntp.org iburst
# server 1.ubuntu.pool.ntp.org iburst

# Add the IP address of the Chrony server
# Replace with the LAN address of your server computer
server 192.168.131.1 iburst
```

Then, restart chrony `sudo systemctl restart chrony`

3. Verify

On the client, run `chronyc tracking` and it should show that it is sychronizing. You should see its source as the server computer using `chronyc sources -v`. You should see the source on the server computer as remote servers.
