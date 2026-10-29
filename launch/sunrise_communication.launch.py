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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    connection_type_TCP = 'TCP'
    connection_type_UDP = 'UDP'

    robot = "KMR2"

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('kmr_iiwa_ros2'),
            'config',
            'nodes_config.yaml'))

    # Every node lives under one namespace so that TF, joint states, cmd_vel
    # and the arm's FollowJointTrajectory action all line up with Nav2 and
    # MoveIt (/kmr_iiwa/...).  The nodes use relative topic names, so the
    # namespace is applied once here instead of per node.
    nodes = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),

        launch_ros.actions.Node(
            package="kmr_iiwa_ros2",
            executable="kmp_commands_node.py",
            name="kmp_commands_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir]),

        launch_ros.actions.Node(
            package="kmr_iiwa_ros2",
            executable="kmp_laserscan_node.py",
            name="kmp_laserscan_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir]),

        launch_ros.actions.Node(
            package="kmr_iiwa_ros2",
            executable="kmp_odometry_node.py",
            name="kmp_odometry_node",
            output="screen",
            emulate_tty=True,
            parameters=[param_dir]),

        launch_ros.actions.Node(
            package="kmr_iiwa_ros2",
            executable="kmp_statusdata_node.py",
            name="kmp_statusdata_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir]),

        launch_ros.actions.Node(
            package="kmr_iiwa_ros2",
            executable="lbr_commands_node.py",
            name="lbr_commands_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir]),

        launch_ros.actions.Node(
            package="kmr_iiwa_ros2",
            executable="lbr_statusdata_node.py",
            name="lbr_statusdata_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir]),

        launch_ros.actions.Node(
            package="kmr_iiwa_ros2",
            executable="lbr_sensordata_node.py",
            name="lbr_sensordata_node",
            output="screen",
            emulate_tty=True,
            parameters=[param_dir]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to parameter file to load'),
        DeclareLaunchArgument(
            'namespace',
            default_value='kmr_iiwa',
            description='Namespace for all KMR iiwa nodes'),

        nodes,
    ])
