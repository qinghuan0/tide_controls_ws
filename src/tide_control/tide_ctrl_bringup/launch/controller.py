'''
Author: qinghuan 1484237245@qq.com
Date: 2025-03-27 11:12:28
FilePath: /tide_controls_ws/src/tide_control/tide_ctrl_bringup/launch/controller.py
Description: 

Copyright (c) 2025 by qinghuan, All Rights Reserved. 
'''
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.actions import OpaqueFunction


joint_state_broadcaster = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_state_broadcaster",
        "--controller-manager",
        "/controller_manager",
    ],
)

gimbal_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "gimbal_controller",
        "--controller-manager",
        "/controller_manager",
    ],
)

left_gimbal_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "left_gimbal_controller",
        "--controller-manager",
        "/controller_manager",
    ],
)

right_gimbal_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "right_gimbal_controller",
        "--controller-manager",
        "/controller_manager",
    ],
)

left_shooter_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "left_shooter_controller",
        "--controller-manager",
        "/controller_manager",
    ],
)

right_shooter_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "right_shooter_controller",
        "--controller-manager",
        "/controller_manager",
    ],
)


def choose_controllers(robot_type):
    sentry_controllers = GroupAction(
        [
            joint_state_broadcaster,
            left_gimbal_controller,
            right_gimbal_controller,
            left_shooter_controller,
            right_shooter_controller,
        ]
    )

    other_controllers = GroupAction(
        [
            joint_state_broadcaster,
            gimbal_controller,
        ]
    )

    def choose_controller_group(context):
        robot_type_value = context.launch_configurations.get("robot_type")
        if robot_type_value == "sentry":
            return [sentry_controllers]
        else:
            return [other_controllers]

    return OpaqueFunction(function=choose_controller_group)
