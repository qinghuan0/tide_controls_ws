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

big_yaw_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "bigyaw_controller",
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
            big_yaw_controller,
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
