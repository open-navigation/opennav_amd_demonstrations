# How to visualize data or control the robot offline

Sometimes your application doesn't have natural internet or wireless communication capabilities. For example, while operating outside without cellular, indoors in a corporate environment where robots cannot be added, or when doing field experiments.

In these cases, it is useful to be able to setup your own temporary network to communicate over. Luckily with the proliferation of cheap mesh routers, this is straight forward to do!

## Setting up field routers

Purchase routers -- mesh preferable if need more than one 

TP-Link Archer AX21 we've used and like, covers 500ft+ in open fields outdoors and is mesh to extend further if needed

Plug into wall or get battery packs with outlets (laptop battery, backup batteries, etc) and put in central locations

Connect robot + computer to it, can use usual tools now to visualize / remotely command offline

Optional: could connect a router to LAN (or cellular hotspot) to bridge internet over to this, but may break security policies. Can also share connection from computer so robot can access internet through your development manchine.

## Notes

- Weather proof routers
- Larger huge commercial antennas can make this extend to 2km+
- If using ROS LOCALHOST ONLY, see online guide for viz or have data streaming directly
