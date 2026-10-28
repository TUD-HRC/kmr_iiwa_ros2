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

import math
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Quaternion, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data

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
        self.declare_parameter('init_odom', True)
        self.init_odom = self.get_parameter('init_odom').value
        self.declare_parameter('period_sec', 0.01)
        period_sec = self.get_parameter('period_sec').value

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

        self.last_odom_timestamp = ""
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0
        self.odom_offset_x = 0.0
        self.odom_offset_y = 0.0
        self.odom_offset_th = 0.0

        # Make Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)

        self.reset_odom_srv = self.create_service(Trigger, 'reset_odom', self.reset_odom_callback)
        self.get_odom_offset_srv = self.create_service(Trigger, 'get_odom_offset', self.get_odom_offset_callback)
        # Create tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Waiting for socket connection...")
        while rclpy.ok() and not self.soc.isconnected:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Socket connected, node is ready")

        while rclpy.ok() and self.soc.isconnected:
            self.odom_callback(self.soc.odometry)
            rclpy.spin_once(self, timeout_sec=period_sec)

    def odom_callback(self, values):

        if len(values) == 0:
            return

        if len(values) != 8:
            self.get_logger().error(
                f'Invalid odom message: expected 8 fields, got {len(values)}: {values}'
            )
            return

        if values[1] == self.last_odom_timestamp:
            return
        
        # print("ODOM received: " )
        # for val in values:
        #     print(val)t\

        kmr_timestamp = values[1]
        self.last_odom_timestamp = kmr_timestamp

        self.odom_x = float(values[2])
        self.odom_y = float(values[3])
        self.odom_th = float(values[4])
        vx = float(values[5])
        vy = float(values[6])
        vth = float(values[7])

        if self.init_odom:
            self.odom_offset_x = self.odom_x
            self.odom_offset_y = self.odom_y
            self.odom_offset_th = self.odom_th
            self.init_odom = False

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"

        th_sat = math.atan2(
            math.sin(self.odom_th - self.odom_offset_th),
            math.cos(self.odom_th - self.odom_offset_th)
        )

        odom_quat = Quaternion()
        quat = self.euler_to_quaternion(0,0,th_sat)
        odom_quat.x = quat[0]
        odom_quat.y = quat[1]
        odom_quat.z = quat[2]
        odom_quat.w = quat[3]

        odom.pose.pose.position.x = self.odom_x - self.odom_offset_x
        odom.pose.pose.position.y = self.odom_y - self.odom_offset_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        odom.child_frame_id = f'{self.robot_name}_base_link'

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = vth

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
        self.odom_pub.publish(odom)

    def reset_odom_callback(self, request, response):

        self.odom_offset_x = self.odom_x
        self.odom_offset_y = self.odom_y
        self.odom_offset_th = self.odom_th

        response.success = True
        response.message = (
            f"New Odom offset: "
            f"x={self.odom_offset_x:.3f}, "
            f"y={self.odom_offset_y:.3f}, "
            f"th={self.odom_offset_th:.3f}"
        )

        self.get_logger().info(response.message)

        return response

    def get_odom_offset_callback(self, request, response):
        response.message = (
            f"Odom offset: "
            f"x={self.odom_offset_x:.3f}, "
            f"y={self.odom_offset_y:.3f}, "
            f"th={self.odom_offset_th:.3f}"
        )
        response.success = True
        return response

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
