# Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

import xacro

def generate_launch_description():
    robot_name = "forbal2"
    package_name = "forbal_description"
    rviz_config = os.path.join(get_package_share_directory(
        "forbal_controllers"), "launch", robot_name + ".rviz")
    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", robot_name + ".urdf.xacro")
    robot_description_config = Command(['xacro ',robot_description,' dummy:=',LaunchConfiguration("sim")])

    controller_config = os.path.join(
        get_package_share_directory(
            package_name), "controllers", "forbal2.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false", description="Use simulation mode"),
        ExecuteProcess(
            cmd=['setserial /dev/ttyUSB1 low_latency'],
            shell=True
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config}, controller_config],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config}],
            output="screen",
            remappings=[("/joint_states","/joint_states_fixed")],
        ),

        Node(
            package="forbal_controllers",
            executable="forbal2",
            name="forbal2",
            output="screen",
            parameters=[os.path.join(get_package_share_directory('forbal_controllers'),'config','forbal2.yaml')],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )

    ])
