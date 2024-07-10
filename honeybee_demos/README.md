# Honeybee Demos 

This package contains a series of real-world applicable demos used to capture data for benchmarking the Ryzen AI compute hardware and Honeybee robot's performance. They are also great entry points for robotics applications to show how to setup a robot system, configure Nav2 for a variety of common advanced situations, and design a simple POC autonomy script for research, startups, or prototypers!

## Demo 1: High-Speed, Outdoor GPS Navigation

The goal of this demonstration is to show the AMD Ryzen AI compute in action running Nav2 at full speed - 2m/s. This is performed on the Presidio main parade lawn in San Francisco, CA because it is a beautiful, generally empty (during weekdays), wide open space in which we can let robots loose at high speeds safely.

For this demonstration, we use the built-in non-RTK corrected GPS to localize the robot to show how to work with Nav2 outdoor with noisy GPS localization. For a refined application, we recommend using an RTK corrected GPS sensor to improve accuracy of localization and positioning tolerances.

We set the datem for `robot_localization` to be an arbitrarily selected position on the park in order to (1) ground the localization system near the origin for convenience and (2) such that this application can be repeated using the same waypoints grounded to a consistent coordinate system, as would be necessary for a deployed application.

An example loop of the main parade ground with GPS data can be seen below:

![ALT TEXT](./images/gps.png)



TODO drone video GIF

TODO metrics / marketing / etc

TODO data set available 

## Demo 2: Outdoor 3D Inspection


## Demo 3: Indoor 2D 

