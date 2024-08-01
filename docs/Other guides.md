


- Software deployment and sync (docker)
- VPN for internet communication // operations software (online)
- ROSBagging / sharding / deleting uploaded log / bag files. how to record and play.
- Weak internet access? --> external antennas. Wifi handoffs

- Cloud? Upload, download, visualize, tooling, communication, etc
- Watchdogs: hardware bringup, stop publishing. Servers all: deadlock or crash. System metrics for degraded performance (ex localization or lack of reponsiveness). 
- joystick / buttons for operations (start/stop mapping/save map; joy stick around, estop, run behavior, demo, etc)
- rviz config / foxglove

- Next steps: ansible, ISO, fleet management, updates


ROS 2 issues: subscribing to topics over wifi hoses the network than things stop working on CPU. Bluetooth cuts out 
  --> really need local host only
              large topics like PC2 / images can't record bags with dropping frames or hosing network
              bluetooth stops working reliably.
  --> not really convinced it even works for normal on-cpu scriptions if rosbag causes issues?
              fast-dds?
