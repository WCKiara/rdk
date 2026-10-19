"""Show an offline Enlight-LL model driven by play_intermediate4.py."""

import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare("flexiv_description")
    robot_sn = LaunchConfiguration("robot_sn")
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution([package, "urdf", "flexiv.urdf.xacro"]),
                " robot_type:=Enlight-LL robot_sn:=",
                robot_sn,
            ]
        ),
        value_type=str,
    )
    rviz_config = "/lab/rdk/enlight_ll_sim/view_enlight_ll.rviz"
    motion_args = json.loads(os.environ.get("OFFLINE_MOTION_ARGS_JSON", "[]"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_sn", default_value="Enlight-LL-123456"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            "/lab/rdk/enlight_ll_sim/play_intermediate4.py",
                            *motion_args,
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
