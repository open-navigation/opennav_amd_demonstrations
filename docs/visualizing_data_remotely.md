# How to visualize data on the robot

It is sometimes required to visualize the robot data remotely. While on the same network as your computer, you can simply launch rviz and have ROS 2 communicate topics over your router or wireless network for you to visualize. However with some large topics, this can distort what the robot actually sees or doesn't get successfully delivered over the wireless that the robot's connection properly handles. Additionally, you can set `ROS_LOCALHOST_ONLY` or set the ROS 2 communication network to be only on a local wired network within the robot. 

In all of these cases, it can be necessary to visualize information directly on the robot's computer and transport the desktop to your computer instead.

## SSH

One way to do this is via X forwarding on ssh. For example:

```
ssh username@robot -XC
rviz2
```

You can see an rviz window pop up on your screen!

## NoMachine

Another way is through NoMachine, a remote desktop application available in Linux. You must install it on both the robot and your computer, then set up a connection. 

## Foxglove

Finally, you can set a web bridge for Foxglove, though that will involve transport of the topics over your wireless network to your computer. However, its worth noting that this can be configured to have the foxglove client listen to internal network and broadcast to you.

## Final Notes

Its worth noting that for refined applications, its best to setup a web app, use a robot operations software service, or VPN server to connect and visualize data from your robot. These methods above are good for debugging, early stage prototyping, or research. For products, better, faster, and more robust methods are available not using Rviz or desktop monitoring tools. 

But, conveniently, SSH, NoMachine, and Foxglove can all work without an internet connection if working offline or in the field. All that is required is a shared router but that doesn't need to be connected to the internet. It is for this reason it is often convenient to have a few high power routers around to use for field testing or development to connect robots, developer machines, and other tooling for communication and separated from a larger corporate, university, or home network. Better yet, if mesh routers, you can setup a Robot Field Network over a broad space for development, testing, and prototyping without the need for internet.
