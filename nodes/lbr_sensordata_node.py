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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from rclpy.qos import qos_profile_sensor_data

from tcpSocket import TCPSocket
from udpSocket import UDPSocket


def cl_red(msge): return '\033[31m' + msge + '\033[0m'

class LbrSensordataNode(Node):
    def __init__(self):
        super().__init__('lbr_sensordata_node')

        self.declare_parameter('robot_name', 'kmr_iiwa')
        robot_name = self.get_parameter('robot_name').value
        self.declare_parameter('ip', '172.31.1.149')
        ip = self.get_parameter('ip').value
        self.declare_parameter('port', 30007)
        port = self.get_parameter('port').value
        self.declare_parameter('connection_type', 'TCP')
        connection_type = self.get_parameter('connection_type').value
        # Publish the newest sample at a bounded rate instead of busy-looping.
        self.declare_parameter('publish_rate', 100.0)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            publish_rate = 100.0

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


        self.last_data_timestamp = 0
        self.prev_time = 0

        self.pub_lbr_sensordata = self.create_publisher(JointState, 'joint_states', 1)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [f"{robot_name}_A{i}" for i in range(1, 8)]

        self.get_logger().info("Waiting for socket connection...")
        while rclpy.ok() and not self.soc.isconnected:
            rclpy.spin_once(self, timeout_sec=0.1)

        # The socket thread only fills a depth-1 deque; publish the latest
        # sample from a timer so this node no longer spins at 100 % CPU.
        self.publish_timer = self.create_timer(
            1.0 / publish_rate, self.publish_latest)

        self.get_logger().info(
            f"Socket connected, node is ready (publish rate "
            f"{publish_rate:.0f} Hz)")

    def publish_latest(self):
        try:
            values = self.soc.lbr_sensordata.popleft()
        except IndexError:
            return
        self.data_callback(values)

    def data_callback(self, values):
        # ">lbr_sensordata,<timestamp>,<7 positions>,<7 torques>"
        parts = values[1].split(',')

        if len(parts) != 16:
            self.get_logger().error(
                f"Invalid lbr_sensordata message, expected 16 fields, got {len(parts)}"
            )
            return

        self.last_data_timestamp = parts[1]

        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        #self.get_logger().info(f"Robot time {parts[1]}, pc time:{self.get_clock().now().nanoseconds}", throttle_duration_sec=1.0)
        
        self.joint_state_msg.position = [float(x) for x in parts[2:9]]
        self.joint_state_msg.effort = [float(x) for x in parts[9:16]]

        self.pub_lbr_sensordata.publish(self.joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    lbr_sensordata_node = LbrSensordataNode()

    rclpy.spin(lbr_sensordata_node)

    try:
        lbr_sensordata_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
