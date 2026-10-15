# kmr_iiwa_ros2
ROS2 nodes for communicating with the app in kmr_iiwa_ros2_java repo. Code is taken from https://github.com/MortenMDahl/kmriiwa_ws_devel/tree/foxy/src/kmr_communication

## Note
- Code taken from https://github.com/MortenMDahl/kmriiwa_ws_devel/tree/foxy/src/kmr_communication, separating the package from other higher level stuff and adding more functionalities


### Orignal repo notes
This package handles the communication with the KMR iiwa. 
There is one launch file which will launch all of the seven communication nodes: 
- kmp_commands_node
- kmp_odometry_node
- kmp_laserscan_node
- kmp_statusdata_node
- lbr_statusdata_node
- lbr_commands_node
- lbr_sensordata_node

The connection type (UDP/TCP) can be set in the launch parameters. 
The IP address to your computer should be set in the parameter file called bringup.yaml. 
In this file, you can also change the port number for each of the nodes. 


## 2. Run
To launch all of the communcation nodes, run: 
```
$ ros2 launch kmr_communication sunrise_communication.launch.py 
```
