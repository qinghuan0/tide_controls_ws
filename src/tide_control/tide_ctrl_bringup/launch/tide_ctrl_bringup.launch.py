import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

sys.path.append(
    os.path.join(get_package_share_directory("tide_ctrl_bringup"), "launch")
)

from controller import choose_controllers


def generate_launch_description():
    ctrl_bringup_pkg_dir = get_package_share_directory("tide_ctrl_bringup")

    robot_type = LaunchConfiguration("robot_type")
    declare_robot_type = DeclareLaunchArgument(
        "robot_type",
        default_value="sentry",
        description="Robot type: sentry, hero, or other",
    )

    sim_mode = LaunchConfiguration("sim_mode")
    declare_sim_mode = DeclareLaunchArgument(
        "sim_mode", default_value="true", description="Use simulation mode if true"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=LaunchConfiguration("sim_mode"),
        description="Use simulation clock if true",
    )

    robot_description_path = PathJoinSubstitution(
        [
            ctrl_bringup_pkg_dir,
            "description",
            robot_type,
        ]
    )

    sim_or_real = PythonExpression(
        ["'_sim' if '", sim_mode, "' == 'true' else '_real'"]
    )

    xacro_file = PathJoinSubstitution(
        [
            robot_description_path,
            PythonExpression(
                ["str('", robot_type, "') + '", sim_or_real, "' + '.xacro'"]
            ),
        ]
    )

    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])

    tide_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("tide_gazebo"),
                        "launch",
                        "gazebo_start.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(sim_mode),
    )

    controller_config = PathJoinSubstitution(
        [
            ctrl_bringup_pkg_dir,
            "config",
            robot_type,
            PythonExpression(
                ["str('", robot_type, "') + '", sim_or_real, "' + '.yaml'"]
            ),
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description}
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            },
            controller_config,
        ],
        output="screen",
        condition=UnlessCondition(sim_mode),
    )

    controllers = choose_controllers(robot_type)

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[
            {
                "port": 8765,
                # "send_buffer_limit": 5000000, # 5MB
                "use_compression": True,
            }
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    return LaunchDescription(
        [
            declare_robot_type,
            declare_sim_mode,
            declare_use_sim_time,
            robot_state_publisher_node,
            control_node,
            controllers,
            tide_gazebo,
            foxglove_node,
        ]
    )
