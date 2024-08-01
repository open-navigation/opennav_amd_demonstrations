# How to Command, Control, and Visualize Offline

Sometimes your application doesn't have natural internet or wireless communication capabilities. For example, while operating outside without cellular, indoors in a corporate environment where robots cannot be added, or when doing field experiments.

In these cases, it is useful to be able to setup your own temporary network to communicate over. Luckily with the proliferation of cheap mesh routers, this is straight forward to do!

## Setting up field routers

* Purchase a mesh capable router and a large, external battery pack. Set the router up with a custom SSID and password appropriate for the task. I tend to call my networks 'Robot Field Network' with an easy to remember password.

* Connect your developer and robot PCs to this network once before leaving for a field experiment so that the computers automatically connect to the network when away from known networks.

* If the space is too large to be covered by a single router, setup multiple routers with the same mesh network and breadcrumb them around the environment. Make sure you have enough battery packs for each.

* If you require internet, it is possible to setup a router which has LAN (or cellular hotspot) connections for all robot and developer PC traffic to bridge through.

* When the robot and developer's PC are connected, you should be able to SSH into the platforms, visualize data through rviz2, or otherwise interact with the robot as if on any other network

## Notes

- I've had good luck with the [TP-Link Archer AX21](https://www.amazon.com/WiFi-6-Router-Gigabit-Wireless/dp/B08H8ZLKKK) router. While not weather resistent, I don't tend to do experiments in bad weather. This works for a 500ft+ radius from the router in an open field easily.
- Its possible to buy large, commercial antenna that can make wireless connections extend to 2km+ in any direction. If substantial range is required, this may be a good option.
- If setting `ROS_LOCALHOST_ONLY` or limiting DDS discovery to the ethernet interface, you may need to use Foxglove, X-forward Rviz, use NoMachine, or otherwise visualize data from the robot using its internal ROS 2 network and broadcast it back to a remote PC.
