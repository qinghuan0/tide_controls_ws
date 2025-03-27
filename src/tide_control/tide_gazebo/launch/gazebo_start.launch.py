import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    AppendEnvironmentVariable,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


class WorldType:
    RMUC2024 = "RMUC2024"
    RMUL2024 = "RMUL2024"
    RMUL2025 = "RMUL2025"


def get_world_config(world_type):
    world_configs = {
        WorldType.RMUC2024: {
            "x": "6.35",
            "y": "7.6",
            "z": "0.2",
            "world_path": "RMUC2024_world/RMUC2024_world.world",
        },
        WorldType.RMUL2024: {
            "x": "4.3",
            "y": "3.35",
            "z": "1.1",
            "world_path": "RMUL2024_world/RMUL2024_world.world",
            # 'world_path': 'RMUL2024_world/RMUL2024_world_dynamic_obstacles.world'
        },
        WorldType.RMUL2025: {
            "x": "2.0",
            "y": "7.5",
            "z": "0.7",
            "world_path": "RMUL2025_world/RMUL2025_world.world",
        },
    }
    return world_configs.get(world_type, None)


def generate_launch_description():
    bringup_dir = get_package_share_directory("tide_gazebo")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    use_sim_time = LaunchConfiguration("use_sim_time")

    append_environment_plugin = AppendEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH",
        os.pathsep.join(
            [
                os.path.join(
                    get_package_share_directory("tide_gazebo"),
                    "meshes",
                    "obstacles",
                    "obstacle_plugin",
                    "lib",
                )
            ]
        ),
    )

    append_environment_modules = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        os.pathsep.join(
            [
                os.path.join(
                    get_package_share_directory("tide_robot_description"), "meshes"
                ),
                os.path.join(
                    get_package_share_directory("tide_robot_description"),
                    "meshes",
                    "sentry",
                ),
            ]
        ),
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world", default_value=WorldType.RMUL2025, description="Choose world type"
    )

    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
    )

    def create_gazebo_launch_group(world_type):
        world_config = get_world_config(world_type)
        if world_config is None:
            return None

        return GroupAction(
            condition=LaunchConfigurationEquals("world", world_type),
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-entity",
                        "robot",
                        "-topic",
                        "robot_description",
                        "-x",
                        world_config["x"],
                        "-y",
                        world_config["y"],
                        "-z",
                        world_config["z"],
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
                    ),
                    launch_arguments={
                        "world": os.path.join(
                            bringup_dir, "world", world_config["world_path"]
                        )
                    }.items(),
                ),
            ],
        )

    bringup_RMUC2024_cmd_group = create_gazebo_launch_group(WorldType.RMUC2024)
    bringup_RMUL2024_cmd_group = create_gazebo_launch_group(WorldType.RMUL2024)
    bringup_RMUL2025_cmd_group = create_gazebo_launch_group(WorldType.RMUL2025)

    ld = LaunchDescription()

    ld.add_action(append_environment_plugin)
    ld.add_action(append_environment_modules)

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world_cmd)

    ld.add_action(gazebo_client_launch)
    ld.add_action(bringup_RMUC2024_cmd_group)
    ld.add_action(bringup_RMUL2024_cmd_group)
    ld.add_action(bringup_RMUL2025_cmd_group)

    return ld
