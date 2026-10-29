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
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
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
from rclpy.executors import MultiThreadedExecutor



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

        # Make a listener for relevant topics

        self.sub_shutdown = self.create_subscription(String, 'shutdown', self.shutdown_callback, 10)
        self.sub_statusdata = self.create_subscription(LbrStatusdata, 'lbr_statusdata', self.status_callback, 10,callback_group=self.callback_group)
        self.sub_joint_command = self.create_subscription(JointState, 'joint_cmd', self.joint_cmd_callback, 10, callback_group=self.callback_group)
        # Name it like a ros2_control controller so MoveIt's
        # moveit_simple_controller_manager (controller `arm_controller`,
        # action_ns `follow_joint_trajectory`) can talk to it unchanged.
        self.fjt_action_server = ActionServer(
            self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory',
            self.fjt_callback, callback_group=self.callback_group)

        self.point_publisher = self.create_publisher(Float64, 'vinkel', 20)

        self.done_moving = False
        self.last_path_variable = False

        self.get_logger().info("Waiting for socket connection...")
        while rclpy.ok() and not self.soc.isconnected:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Socket connected, node is ready")

    def status_callback(self,data):
        if (self.last_path_variable == False and data.path_finished == True):
            self.done_moving = True
            print("done_moving set to True")
        self.last_path_variable = data.path_finished


    def shutdown_callback(self, data):
        print(data)
        msg = "shutdown"
        self.soc.send(msg)
        self.soc.isconnected = False

    def joint_cmd_callback(self, msg:JointState):
        #TODO validate input
        #string_command = "SetJointPosition " + " ".join(f"{x:.3f}" for x in msg.position)
        string_command = "SetJointServoPosition " + " ".join(f"{x:.3f}" for x in msg.position)
        self.soc.send(string_command)

    def fjt_callback(self, goal_handle):
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
                time.sleep(0.01)

            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = ''
            goal_handle.succeed()
            return result
        finally:
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
            msg = 'pathPointLBR ' + ">" + type + ">" + positions + ">" + velocities + ">" + accelerations

            self.get_logger().info(f'sending command {msg}')

            self.soc.send(msg)
            i+=1


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    lbr_commands_node = LbrCommandsNode(args.connection,args.robot)

    executor = MultiThreadedExecutor()
    rclpy.spin(lbr_commands_node, executor)

    try:
        lbr_commands_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
