#!/usr/bin/env python3

"""Smooth multi-joint streaming test for the KMR iiwa.

Publishes a slowly oscillating joint target for *all seven* joints on
``/kmr_iiwa/joint_cmd`` at a fixed 100 Hz, forever.  This bypasses MoveIt
Servo and exercises the same path as ``joint_pos_cmd_test.py``
(``joint_cmd`` -> ``SetJointServoPosition`` -> KUKA SmartServo), but with a
continuously changing target instead of a piecewise-constant one.

Use it to separate these two cases:

Safety notes:

* the oscillation is centred on the pose measured when the script starts, so
  it never jumps to a hard-coded pose;
* the amplitude is small (default 0.06 rad, about 3.4 deg per joint) and is
  ramped in over the first two seconds;
* the default frequency is 0.15 Hz, so joint speeds stay well below
  0.1 rad/s.

Stop with Ctrl-C; the arm then holds the last streamed destination.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    'kmr_iiwa_A1',
    'kmr_iiwa_A2',
    'kmr_iiwa_A3',
    'kmr_iiwa_A4',
    'kmr_iiwa_A5',
    'kmr_iiwa_A6',
    'kmr_iiwa_A7',
]


class JointSineTest(Node):

    def __init__(self, amplitude, frequency, rate, ramp_time):
        super().__init__('joint_sine_cmd_test')

        self.amplitude = amplitude
        self.frequency = frequency
        self.rate = rate
        self.ramp_time = ramp_time
        self.joint_names = list(JOINT_NAMES)

        # Different phase per joint so every joint moves, but the motion stays
        # slow and easy to observe.
        self.phases = [i * 0.7 for i in range(len(self.joint_names))]

        # Filled with the first measured pose; the oscillation is centred here.
        self.center = None
        self.last_state_time = None
        self.last_command = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(
            JointState, '/kmr_iiwa/joint_cmd', qos)

        self.sub = self.create_subscription(
            JointState, '/kmr_iiwa/joint_states', self.joint_state_callback,
            qos)

        self.start_time = time.monotonic()

        self.timer = self.create_timer(1.0 / rate, self.publish_command)

        self.get_logger().info(
            f'Waiting for {"/kmr_iiwa/joint_states"} before streaming...')

    def joint_state_callback(self, msg):
        if self.center is not None:
            self.last_state_time = time.monotonic()
            return

        try:
            positions = [msg.position[msg.name.index(name)]
                         for name in self.joint_names]
        except (ValueError, IndexError):
            self.get_logger().warn(
                'joint_states does not contain all arm joints yet',
                throttle_duration_sec=2.0)
            return

        self.center = positions
        self.last_state_time = time.monotonic()
        self.start_time = time.monotonic()
        self.get_logger().info(
            'Captured start pose, streaming a '
            f'{self.amplitude:.3f} rad / {self.frequency:.3f} Hz sine '
            f'at {self.rate:.0f} Hz')

    def publish_command(self):
        if self.center is None:
            return

        now = time.monotonic()
        elapsed = now - self.start_time

        # Do not keep moving blind if the joint feedback disappears.
        if now - self.last_state_time > 0.5:
            self.get_logger().warn(
                'No joint_states for 0.5 s, holding last command',
                throttle_duration_sec=2.0)
            if self.last_command is not None:
                self.publish_positions(self.last_command)
            return

        # Smooth amplitude ramp-in: no step at the first command.
        if self.ramp_time > 0.0 and elapsed < self.ramp_time:
            u = elapsed / self.ramp_time
            ramp = u * u * (3.0 - 2.0 * u)  # smoothstep
        else:
            ramp = 1.0

        positions = []
        for center, phase in zip(self.center, self.phases):
            angle = (2.0 * math.pi * self.frequency * elapsed) + phase
            positions.append(
                center + ramp * self.amplitude * math.sin(angle))

        self.publish_positions(positions)
        self.last_command = positions

    def publish_positions(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.pub.publish(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--amplitude', type=float, default=0.10,
        help='sine amplitude per joint [rad]')
    parser.add_argument(
        '--frequency', type=float, default=0.15,
        help='sine frequency [Hz]')
    parser.add_argument(
        '--rate', type=float, default=200.0,
        help='command publishing rate [Hz]')
    parser.add_argument(
        '--ramp-time', type=float, default=2.0,
        help='seconds to ramp the amplitude from 0 to full [s]')
    args = parser.parse_args(remove_ros_args(args=argv))

    if args.amplitude <= 0.0:
        parser.error('--amplitude must be positive')
    if args.frequency <= 0.0:
        parser.error('--frequency must be positive')
    if args.rate <= 0.0:
        parser.error('--rate must be positive')

    rclpy.init(args=argv)

    node = JointSineTest(
        args.amplitude, args.frequency, args.rate, args.ramp_time)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
