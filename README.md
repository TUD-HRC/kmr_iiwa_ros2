# kmr_iiwa_ros2

ROS 2 nodes for communicating with the Sunrise app in the [kmr_iiwa_control](https://github.com/TUD-HRC/kmr_iiwa_control) repo.

## Description

This is a ROS 2 package for communicating with the KUKA KMR iiwa mobile manipulator. It allows controlling the mobile base and the arm at the same time.

It has been tested, but not thoroughly so far (see the TODO list below); use it with common sense. The main node is a TCP/UDP server with which the `kmr_iiwa_control` app communicates.

It exposes ROS 2 topics for:

- Arm control (joint positions only for now)
- Mobile base control (x, y and yaw velocities)
- Laser scan data
- Odom data from robot API
- Other robot status data from the KUKA API

Several ROS 2 nodes are started, each communicating over TCP/UDP on a different port. Each node corresponds to a thread of the Sunrise app that handles the communication for a particular robot feature:

- kmp_commands_node
- kmp_odometry_node
- kmp_laserscan_node
- kmp_statusdata_node
- lbr_statusdata_node
- lbr_commands_node
- lbr_sensordata_node

## Requirements

- [kmr_iiwa_msgs](https://github.com/TUD-HRC/kmr_iiwa_msgs)
- [kmr_iiwa_description](https://github.com/TUD-HRC/kmr_iiwa_description) (robot models)
- [kmr_iiwa_control](https://github.com/TUD-HRC/kmr_iiwa_control) - the app that runs on the robot and communicates with this repo. It is not a strict dependency for building the nodes.

## Usage

1. Build this repo and its dependencies with `colcon build`.
2. Check the configuration file `config/nodes_config.yaml`. In particular, set the IP address of the PC running these ROS 2 nodes (`KMR1/ip`, `KMR2/ip`).
3. **Warning**: the arm will move to its home position as soon as the nodes are launched and communication is established. Take care.
4. Launch all communication nodes:

   ```bash
   ros2 launch kmr_iiwa_ros2 sunrise_communication.launch.py
   ```

5. Start the Sunrise app from the robot tablet.

## TODOs / Features to implement

- Odom data has x and y switched. To investigate
- Odom data begins from an origin that should be possible to set somewhere in the robot config. Can not figure where so far. Reset ROS service is present anyway.
- Arm torque control
- Do not auto-home on startup
- Use FRI for the robot arm, which should be a more reliable API
- Check whether it is better (and possible) to run the ROS 2 nodes directly on the robot
- Stabilize the connection; the actual topic frequency is half of what is requested
- Test the UDP connection. Try whether a mixed TCP/UDP connection is better depending on the node function
- Gripper integration
- Integration of other sensors (e.g. cameras)
- Reduce robot fan noise

## Notes

- Code taken from https://github.com/MortenMDahl/kmriiwa_ws_devel/tree/foxy/src/kmr_communication, separating the package from other higher-level code, cleaning it up and adding more functionality.

### Alternatives

#### Arm + base software

- https://github.com/LAKY911/kmriiwa_ws_devel/, also check its fork of the fork. It is built as a KUKA Sunrise application, so Sunrise Workbench is necessary.
- https://github.com/stoic-roboticist/kmriiwa_ros_java and https://github.com/stoic-roboticist/kmriiwa_ros_stack, with the related ROSCon presentation at https://vimeo.com/649644996. The first repo is the Java app, the second one the ROS 1 stack.

#### Mobile base code

- https://github.com/JonahEggenkemper/kuka-kmr-project/tree/main. Uses a custom Java-ROS 1 interface to control the mobile base in ROS 1.

#### KUKA iiwa arm controllers

- First Google result, lots of stars, ROS 2, arm only: https://github.com/ICube-Robotics/iiwa_ros2
- Another one with some stars: https://github.com/idra-lab/kuka_lbr_control, University of Trento
