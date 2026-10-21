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
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse

from tcpSocket import TCPSocket
from udpSocket import UDPSocket


def cl_red(msge): return '\033[31m' + msge + '\033[0m'

class KmpLaserScanNode(Node):
    def __init__(self):
        super().__init__('kmp_laserscan_node')

        self.declare_parameter('robot_name', 'kmr_iiwa')
        robot_name = self.get_parameter('robot_name').value
        self.declare_parameter('ip', '172.31.1.149')
        ip = self.get_parameter('ip').value
        self.declare_parameter('port', 30003)
        port = self.get_parameter('port').value
        self.declare_parameter('connection_type', 'TCP')
        connection_type = self.get_parameter('connection_type').value

        self.get_logger().info(
            f'Starting up node: {self.get_namespace()}/{self.get_name()} '
            f'with {connection_type} socket at: {ip}:{port}'
        )
        
        if connection_type == 'TCP':
            self.soc = TCPSocket(ip, port,self.get_name())
        elif connection_type == 'UDP':
            self.soc = UDPSocket(ip, port,self.get_name())
        else:
            self.get_logger().error("connection_type " + connection_type + " not recognized")
            raise Exception("connection_type " + connection_type + " not recognized")  

        self.last_scan_timestamp = 0
        self.scan_front_right = LaserScan()
        self.scan_back_left = LaserScan()
        # logLaserConfig() method checking LaserConfig from KUKA API gives:
        # startAngle = -2.3561944
        # fov = 4.7123888
        # angularResolution = 0.0087266 (= 0.5deg from datasheet)
        # maximumRange = 29.0 (30.0m from datasheet)
        # accuracy = 0.01
        # typeID = 9 (what does this mean?)
        # remissionMode = 0 (what does this mean?)
        # Datasheet SICK S300 scanning angle 270deg
        # Datasheet resolution options: 0.03m 0.04 0.05 0.06 0.07 0.15
        self.scan_front_right.angle_increment = 0.0087266 #(0.5*math.pi)/180
        self.scan_front_right.angle_min = -2.3561944 #(-135*math.pi)/180
        self.scan_front_right.angle_max = 2.3561944 #(135*math.pi)/180
        self.scan_front_right.range_min = 0.10
        self.scan_front_right.range_max = 29.0
        self.scan_front_right.header.frame_id = f"{robot_name}_scan_front_right_link"
        self.scan_front_right.angle_increment = 0.0087266 #(0.5*math.pi)/180
        self.scan_front_right.angle_min = -2.3561944 #(-135*math.pi)/180
        self.scan_front_right.angle_max = 2.3561944 #(135*math.pi)/180
        self.scan_front_right.range_min = 0.10
        self.scan_front_right.range_max = 29.0
        self.scan_back_left.header.frame_id = f"{robot_name}_scan_back_left_link"

        # Make Publishers for relevant data
        self.pub_scan_front_right = self.create_publisher(LaserScan, 'scan_front_right', qos_profile_sensor_data)
        self.pub_scan_back_left = self.create_publisher(LaserScan, 'scan_back_left', qos_profile_sensor_data)

        self.get_logger().info("Waiting for socket connection...")
        while rclpy.ok() and not self.soc.isconnected:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Socket connected, node is ready")

        while rclpy.ok() and self.soc.isconnected:
            if len(self.soc.laserScanB1):
                self.scan_front_right_callback(self.soc.laserScanB1.pop(0))
            if len(self.soc.laserScanB4):
                self.scan_back_left_callback(self.soc.laserScanB4.pop(0))

    def scan_front_right_callback(self, values):

        if (len(values) == 4):
            kmr_timestamp = values[1]
            self.last_scan_timestamp = kmr_timestamp
            #TODO put real timestap from laser
            self.scan_front_right.header.stamp = self.get_clock().now().to_msg()

            #just for debug, differentiation of the two device is already done in tcpSocket
            #filling self.soc.laserScanB1 or self.soc.laserScanB4
            #assert(values[2] == '1801')

            try:
                self.scan_front_right.ranges = [float(s) for s in values[3].split(',') if len(s)>0]
            except ValueError as e:
                print(values[3].split(','))
                print("Error", e)
            self.pub_scan_front_right.publish(self.scan_front_right)
        else:
            self.get_logger().error("Front Right Scan msg format error: expected lenght 4, received " 
                                    + str(len(values)), throttle_duration_sec = 1.0)

    def scan_back_left_callback(self, values):

        if (len(values) == 4):
            kmr_timestamp = values[1]
            self.last_scan_timestamp = kmr_timestamp
            #TODO put real timestap from laser
            self.scan_back_left.header.stamp = self.get_clock().now().to_msg()

            #just for debug, differentiation of the two device is already done in tcpSocket
            #filling self.soc.laserScanB1 or self.soc.laserScanB4
            #assert(values[2] == '1802')

            try:
                self.scan_back_left.ranges = [float(s) for s in values[3].split(',') if len(s)>0]
            except ValueError as e:
                print(values[3].split(','))
                print("Error", e)
            self.pub_scan_back_left.publish(self.scan_back_left)
        else:
            self.get_logger().error("Back Left Scan msg format error: expected lenght 4, received " 
                                    + str(len(values)), throttle_duration_sec = 1.0)

def main(args=None):
    rclpy.init(args=args)
    laserscan_node = KmpLaserScanNode()

    rclpy.spin(laserscan_node)
    try:
        laserscan_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
