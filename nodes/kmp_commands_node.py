#!/usr/bin/env python3

# Copyright 2019 Nina Marie Wahl and Charlotte Heggem.
# Copyright 2019 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse

from tcpSocket import TCPSocket
from udpSocket import UDPSocket



def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class KmpCommandsNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('kmp_commands_node')
        self.name = 'kmp_commands_node'
        self.declare_parameter('port', 30002)
        port = int(self.get_parameter('port').value)

        if robot in ('KMR1', 'KMR2'):
            param_name = f'{robot}/ip'
            self.declare_parameter(param_name, '172.31.1.149')
            ip = self.get_parameter(param_name).value
        else:
            ip = None

        if connection_type == 'TCP':
            self.soc = TCPSocket(ip,port,self.name)
        elif connection_type == 'UDP':
            self.soc = UDPSocket(ip,port,self.name)
        else:
            self.soc = None

        # Make a listener for relevant topics
        sub_twist = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        sub_pose = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        sub_shutdown = self.create_subscription(String, 'shutdown', self.shutdown_callback, 10)

        self.get_logger().info("Waiting for socket connection...")
        while rclpy.ok() and not self.soc.isconnected:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Socket connected, node is ready")


    def shutdown_callback(self, data):
        print(data)
        msg = "shutdown"
        self.soc.send(msg)
        #self.udp_soc.isconnected = False

    def twist_callback(self, data):
        msg = 'setTwist ' + str(data.linear.x) + " " + str(data.linear.y) + " " + str(data.angular.z)
        self.soc.send(msg)

    def pose_callback(self, data):
        msg = 'setPose ' + str(data.position.x) + " " + str(data.position.y) + " " + str(data.orientation.z)
        self.soc.send(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    kmp_commands_node = KmpCommandsNode(args.connection,args.robot)

    rclpy.spin(kmp_commands_node)

    try:
        kmp_commands_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
