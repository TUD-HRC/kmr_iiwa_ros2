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

import asyncio
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kmr_iiwa_msgs.msg import LbrStatusdata
from control_msgs.action import FollowJointTrajectory
from tcpSocket import TCPSocket
from udpSocket import UDPSocket
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException



def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class LbrCommandsNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('lbr_commands_node')
        self.name = 'lbr_commands_node'

        self.declare_parameter('port', 30005)
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


        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('robot_name', 'kmr_iiwa')
        robot_name = self.get_parameter('robot_name').value
        self.arm_joint_names = [f'{robot_name}_A{i}' for i in range(1, 8)]
        self.motion_lock = threading.Lock()
        # The FJT path and the servo stream both write to the same socket, so
        # serialize sends.
        self.socket_lock = threading.Lock()

        # Latest servo target.  The subscriber callbacks only store it; a
        # fixed-rate timer actually streams it to the arm.  Sending from the
        # callbacks made the command rate depend on executor wakeups, which
        # dropped to ~10 Hz with a MultiThreadedExecutor and left the
        # SmartServo runtime without updates for hundreds of milliseconds.
        self.latest_positions = None
        self.last_command_time = 0.0
        self.servo_streaming = False
        self.declare_parameter('command_rate', 100.0)
        command_rate = float(self.get_parameter('command_rate').value)
        if command_rate <= 0.0:
            command_rate = 50.0
        self.command_period = 1.0 / command_rate

        # If no new target arrives within this window, stop the servo stream
        # instead of resending a stale destination forever.  MoveIt Servo
        # publishes continuously, so normal teleoperation never trips this.
        self.declare_parameter('command_timeout', 0.3)
        self.command_timeout = float(
            self.get_parameter('command_timeout').value)
        if self.command_timeout <= 0.0:
            self.command_timeout = 0.3

        self.declare_parameter('servo_cmd_topic', 'servo_joint_cmd')
        servo_cmd_topic = self.get_parameter('servo_cmd_topic').value

        # Make a listener for relevant topics

        self.sub_shutdown = self.create_subscription(String, 'shutdown', self.shutdown_callback, 10)
        self.sub_statusdata = self.create_subscription(LbrStatusdata, 'lbr_statusdata', self.status_callback, 10,callback_group=self.callback_group)
        self.sub_joint_command = self.create_subscription(JointState, 'joint_cmd', self.joint_cmd_callback, 1, callback_group=self.callback_group)
        # MoveIt Servo publishes std_msgs/Float64MultiArray (positions only, in
        # the MoveIt group's joint order).
        self.sub_servo_command = self.create_subscription(
            Float64MultiArray, servo_cmd_topic, self.servo_cmd_callback, 1,
            callback_group=self.callback_group)
        self.fjt_action_server = ActionServer(
            self, FollowJointTrajectory,
            'follow_joint_trajectory_controller/follow_joint_trajectory',
            self.fjt_callback, callback_group=self.callback_group)

        self.point_publisher = self.create_publisher(Float64, 'vinkel', 20)

        self.done_moving = False
        self.last_path_variable = False

        self.get_logger().info("Waiting for socket connection...")
        while rclpy.ok() and not self.soc.isconnected:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Start streaming only once the socket is up.  Re-sending the latest
        # target keeps a steady command stream even if no new message arrived.
        self.command_timer = self.create_timer(
            self.command_period, self.send_latest_command)

        self.get_logger().info(
            f"Socket connected, node is ready (command rate "
            f"{1.0 / self.command_period:.0f} Hz)")

    def status_callback(self,data):
        if (self.last_path_variable == False and data.path_finished == True):
            self.done_moving = True
            print("done_moving set to True")
        self.last_path_variable = data.path_finished


    def shutdown_callback(self, data):
        print(data)
        self.get_logger().info('Shutdown requested; sending CancelMotion')
        self.cancel_motion()

    def send_command(self, command, drop_if_busy=True):
        """Send one line to the Sunrise app; returns False if dropped."""
        with self.socket_lock:
            return self.soc.send(command, drop_if_busy=drop_if_busy)

    def send_servo_positions(self, positions):
        """Stream a joint position command to the arm over the TCP socket."""
        if len(positions) != len(self.arm_joint_names):
            self.get_logger().error(
                f'Expected {len(self.arm_joint_names)} joint positions, '
                f'got {len(positions)}',
                throttle_duration_sec=1.0)
            return False
        string_command = "SetJointServoPosition " + " ".join(f"{x:.3f}" for x in positions)
        return self.send_command(string_command)

    def store_servo_positions(self, positions):
        """Remember the newest servo target; the timer does the sending."""
        if len(positions) != len(self.arm_joint_names):
            self.get_logger().error(
                f'Expected {len(self.arm_joint_names)} joint positions, '
                f'got {len(positions)}',
                throttle_duration_sec=1.0)
            return
        self.latest_positions = [float(x) for x in positions]
        self.last_command_time = time.monotonic()
        self.servo_streaming = True

    def send_latest_command(self):
        """Timer callback: stream the latest target at a fixed rate."""
        # A trajectory owns the arm while it runs; don't fight it with a
        # stale servo target.
        if self.motion_lock.locked():
            return
        if not self.servo_streaming or self.latest_positions is None:
            return
        # No new target from ROS for a while: stop instead of driving to the
        # last streamed point (e.g. after the sine test or Servo is stopped).
        if time.monotonic() - self.last_command_time > self.command_timeout:
            self.stop_servo_stream()
            return
        if not self.send_servo_positions(self.latest_positions):
            self.get_logger().warn(
                'Command socket is backlogged; dropping stale target',
                throttle_duration_sec=2.0)

    def stop_servo_stream(self):
        """Stop resending and ask the arm to end the servo motion."""
        if not self.servo_streaming:
            return
        self.get_logger().info(
            f'No new servo target for {self.command_timeout:.2f} s; '
            'sending CancelMotion')
        self.cancel_motion()

    def cancel_motion(self):
        """Stop the servo stream and tell the arm to cancel its motion."""
        self.servo_streaming = False
        self.latest_positions = None
        if not self.send_command('CancelMotion', drop_if_busy=False):
            self.get_logger().error('Could not send CancelMotion to the arm')

    def joint_cmd_callback(self, msg:JointState):
        self.store_servo_positions(msg.position)

    def servo_cmd_callback(self, msg:Float64MultiArray):
        # Float64MultiArray carries no joint names: Servo publishes in the
        # active joint order of its move_group_name ("arm"), kmr_iiwa_A1..A7.
        self.store_servo_positions(msg.data)

    async def fjt_callback(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory

        # The Sunrise controller receives position, velocity and acceleration
        # for all seven arm joints at once. Only one path can be executed at a
        # time, so reject goals that arrive while another one is running.
        if not self.motion_lock.acquire(blocking=False):
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Another trajectory is already being executed'
            goal_handle.abort()
            return result

        try:
            # The trajectory owns the arm now; drop any pending servo target
            # so the fixed-rate timer goes quiet until this goal finishes.
            self.servo_streaming = False
            self.latest_positions = None

            if len(trajectory.points) == 0:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'Trajectory contains no points'
                goal_handle.abort()
                return result

            try:
                trajectory = self.normalize_trajectory(trajectory)
            except ValueError as e:
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = str(e)
                goal_handle.abort()
                return result

            self.get_logger().info(
                f'Received trajectory with {len(trajectory.points)} points')

            # Reset before sending: the arm can report path_finished before
            # path_callback() returns.
            self.done_moving = False
            try:
                self.path_callback(trajectory)
            except Exception as e:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = f'Failed to send trajectory: {e}'
                goal_handle.abort()
                return result

            timeout = self.trajectory_timeout(
                trajectory, goal_handle.request.goal_time_tolerance)
            deadline = time.monotonic() + timeout
            while not self.done_moving:
                if goal_handle.is_cancel_requested:
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = (
                        'Trajectory canceled by client; the arm keeps executing '
                        'the path that was already sent')
                    self.get_logger().warn(result.error_string)
                    goal_handle.canceled()
                    return result
                if time.monotonic() > deadline:
                    result.error_code = (
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED)
                    result.error_string = (
                        f'Timeout after {timeout:.1f} s waiting for path_finished')
                    self.get_logger().error(result.error_string)
                    goal_handle.abort()
                    return result
                await asyncio.sleep(0.01)

            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = ''
            goal_handle.succeed()
            return result
        finally:
            # Drop any servo target received during the trajectory so the
            # timer cannot resume from a stale position.
            self.servo_streaming = False
            self.latest_positions = None
            self.motion_lock.release()

    def normalize_trajectory(self, trajectory):
        """Reorder the trajectory to the joint order expected by the arm."""
        names = list(trajectory.joint_names)
        if sorted(names) != sorted(self.arm_joint_names):
            raise ValueError(
                f'Expected joints {self.arm_joint_names}, got {names}')

        order = [names.index(name) for name in self.arm_joint_names]
        normalized = JointTrajectory()
        normalized.header = trajectory.header
        normalized.joint_names = list(self.arm_joint_names)

        for point in trajectory.points:
            if len(point.positions) != len(names):
                raise ValueError(
                    f'Expected {len(names)} positions per point, '
                    f'got {len(point.positions)}')

            normalized_point = JointTrajectoryPoint()
            normalized_point.positions = [point.positions[i] for i in order]
            if len(point.velocities) == len(names):
                normalized_point.velocities = [point.velocities[i] for i in order]
            else:
                normalized_point.velocities = [0.0] * len(order)
            if len(point.accelerations) == len(names):
                normalized_point.accelerations = [
                    point.accelerations[i] for i in order]
            else:
                normalized_point.accelerations = [0.0] * len(order)
            normalized_point.time_from_start = point.time_from_start
            normalized.points.append(normalized_point)

        return normalized

    def trajectory_timeout(self, trajectory, goal_time_tolerance):
        """Seconds to wait for the arm before giving up on the goal."""
        duration = 0.0
        if trajectory.points:
            last = trajectory.points[-1].time_from_start
            duration = last.sec + last.nanosec * 1e-9
        tolerance = goal_time_tolerance.sec + goal_time_tolerance.nanosec * 1e-9
        return max(duration + tolerance + 10.0, 10.0)

    def path_callback(self, data):
        i=1
        for point in data.points:
            positions = " ".join([str(s) for s in point.positions])
            velocities = " ".join([str(s) for s in point.velocities])
            accelerations = " ".join([str(s) for s in point.accelerations])
            x = Float64()
            x.data = point.positions[6]
            self.point_publisher.publish(x)
            if i == 1:
                type = "StartPoint"
            elif i == len(data.points):
                type = "EndPoint"
            else:
                type = "WayPoint"

            if i == len(data.points):
                msg = 'SetJointTrajectory ' + ">" + type + ">" + positions
            else:
                msg = 'SetJointTrajectory ' + ">" + type + ">" + positions + ">" + velocities + ">" + accelerations


            self.get_logger().info(f'sending command {msg}')

            self.send_command(msg, drop_if_busy=False)
            i+=1


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    lbr_commands_node = LbrCommandsNode(args.connection,args.robot)

    # A single-threaded executor (plain spin) delivers the command topic at
    # the real publish rate.  MultiThreadedExecutor collapsed it to ~10 Hz.
    # fjt_callback is async, so the action server still makes progress while
    # it waits for path_finished.
    try:
        rclpy.spin(lbr_commands_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Do not leave the arm streaming towards a stale target when this
        # node goes away.
        try:
            lbr_commands_node.stop_servo_stream()
            # Give the Sunrise app a moment to read CancelMotion.
            time.sleep(0.1)
        except Exception as e:
            print(cl_red('Error: ') + f"could not cancel motion: {e}")

    try:
        lbr_commands_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
