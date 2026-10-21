#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class JointA6Oscillator(Node):

    def __init__(self):
        super().__init__('joint_a6_oscillator')

        self.pub = self.create_publisher(
            JointState,
            '/joint_cmd',
            10
        )

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.names = [
            'joint_a1',
            'joint_a2',
            'joint_a3',
            'joint_a4',
            'joint_a5',
            'joint_a6',
            'joint_a7'
        ]

        # Initial robot position
        self.command_position = [
            -1.5603013815500606,
             0.3438073389507198,
             0.01808761083813114,
            -1.9123970104112364,
            -0.013960183922750686,
             0.8827621411181155,
             0.661942910168283
        ]

        self.a6_pos_1 = 0.4
        self.a6_pos_2 = 1.0

        self.target_a6 = self.a6_pos_1
        self.current_a6 = None

        self.tolerance = 0.02

        self.timer = self.create_timer(
            0.01,  # 10 Hz
            self.publish_command
        )

        self.get_logger().info('Joint A6 oscillator started')

    def joint_state_callback(self, msg):

        try:
            idx = msg.name.index('joint_a6')
            self.current_a6 = msg.position[idx]
        except (ValueError, IndexError):
            return

        if abs(self.current_a6 - self.target_a6) < self.tolerance:

            if self.target_a6 == self.a6_pos_1:
                self.target_a6 = self.a6_pos_2
            else:
                self.target_a6 = self.a6_pos_1

            self.get_logger().info(
                f'Changing target A6 to {self.target_a6:.3f} rad'
            )

    def publish_command(self):

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = self.names

        positions = self.command_position.copy()
        positions[5] = self.target_a6  # joint_a6

        msg.position = positions

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = JointA6Oscillator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()