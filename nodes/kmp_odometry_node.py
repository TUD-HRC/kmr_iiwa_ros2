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

import _thread as thread
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse

from tcpSocket import TCPSocket
from udpSocket import UDPSocket


def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class KmpOdometryNode(Node):
    def __init__(self):
        super().__init__('kmp_odometry_node')
        self.name='kmp_odometry_node'

        self.declare_parameter('robot_name', 'kmr_iiwa')
        self.robot_name = self.get_parameter('robot_name').value
        self.declare_parameter('ip', '172.31.1.149')
        ip = self.get_parameter('ip').value
        self.declare_parameter('port', 30004)
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

        self.last_odom_timestamp = 0

        # Make Publisher for odometry
        self.pub_odometry = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)

        # Create tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Waiting for socket connection...")
        while rclpy.ok() and not self.soc.isconnected:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Socket connected, node is ready")

        while rclpy.ok() and self.soc.isconnected:
            self.odom_callback(self.pub_odometry, self.soc.odometry)

    def odom_callback(self, publisher, values):

        # print("ODOM received: " )
        # for val in values:
        #     print(val)

        if (len(values) == 8 and values[1] != self.last_odom_timestamp):
            kmr_timestamp = values[1]
            self.last_odom_timestamp = kmr_timestamp

            x = float(values[2])
            y = float(values[3])
            th = float(values[4])
            vx = float(values[5])
            vy = float(values[6])
            vth = float(values[7])

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"

            point = Point()
            point.x = x
            point.y = y
            point.z = float(0)

            odom_quat = Quaternion()
            quat = self.euler_to_quaternion(0,0,th)
            odom_quat.x = quat[0]
            odom_quat.y = quat[1]
            odom_quat.z = quat[2]
            odom_quat.w = quat[3]

            odom.pose.pose.position = point
            odom.pose.pose.orientation = odom_quat

            odom.child_frame_id = f'{self.robot_name}_base_link'
            linear = Vector3()
            linear.x = vx
            linear.y = vy
            linear.z = float(0)

            angular = Vector3()
            angular.x = float(0)
            angular.y = float(0)
            angular.z = vth

            odom.twist.twist.linear = linear
            odom.twist.twist.angular = angular

            # Create transform
            odom_tf = TransformStamped()
            odom_tf.transform.translation.x = odom.pose.pose.position.x
            odom_tf.transform.translation.y = odom.pose.pose.position.y
            odom_tf.transform.translation.z = odom.pose.pose.position.z
            odom_tf.transform.rotation = odom.pose.pose.orientation

            odom_tf.header.frame_id = odom.header.frame_id
            odom_tf.child_frame_id = odom.child_frame_id
            odom_tf.header.stamp = odom.header.stamp
            self.tf_broadcaster.sendTransform(odom_tf)
            publisher.publish(odom)


    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]


def main(args=None):

    rclpy.init(args=args)
    odometry_node = KmpOdometryNode()

    while rclpy.ok():
        rclpy.spin_once(odometry_node)
    try:
        odometry_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
