# showcases map_merge with static maps served by map_server

# you can run this with test maps provided in m-explore-extra repo
# https://github.com/hrnr/m-explore-extra

# roslaunch multirobot_map_merge from_map_server.launch map1:=PATH_TO_m-explore-extra/map_merge/gmapping_maps/2011-08-09-12-22-52.yaml map2:=PATH_TO_m-explore-extra/map_merge/gmapping_maps/2012-01-28-11-12-01.yaml rviz:=True

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("multirobot_map_merge"), "config", "params.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    known_init_poses = LaunchConfiguration("known_init_poses")
    rviz = LaunchConfiguration("rviz")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="/",
        description="Namespace for the explore node",
    )
    declare_known_init_poses_argument = DeclareLaunchArgument(
        "known_init_poses",
        default_value="False",
        description="Known initial poses of the robots. If so don't forget to declare them in the params.yaml file",
    )
    declare_rviz_argument = DeclareLaunchArgument(
        "rviz",
        default_value="False",
        description="Run rviz2",
    )

    num_maps = 2
    group_actions = []

    for i in range(num_maps):
        map_argument_name = f"map{i+1}"
        map_yaml_file = LaunchConfiguration(map_argument_name)
        declare_map_argument = DeclareLaunchArgument(
            map_argument_name,
            default_value=f"{map_argument_name}.yaml",
            description="Full path to map yaml file to load",
        )
        map_server = Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            namespace=f"robot{i+1}",
            output="screen",
            parameters=[
                {"yaml_filename": map_yaml_file},
                {"use_sim_time": use_sim_time},
            ],
        )
        map_server_manager = Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            namespace=f"robot{i+1}",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {"node_names": ["map_server"]},
            ],
        )
        group_action = GroupAction(
            [
                PushRosNamespace(namespace=namespace),
                map_server_manager,
                map_server,
                declare_map_argument,
            ]
        )

        group_actions.append(group_action)

    node = Node(
        package="multirobot_map_merge",
        name="map_merge",
        namespace=namespace,
        executable="map_merge",
        parameters=[
            config,
            {"use_sim_time": use_sim_time},
            {"known_init_poses": known_init_poses},
        ],
        output="screen",
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("multirobot_map_merge"), "launch", "map_merge.rviz"
    )
    start_rviz_cmd = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )
    exit_event_handler = RegisterEventHandler(
        condition=IfCondition(rviz),
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason="rviz exited")),
        ),
    )

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_known_init_poses_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_rviz_argument)
    for group_action in group_actions:
        ld.add_action(group_action)
    ld.add_action(node)
    ld.add_action(start_rviz_cmd)
    ld.add_action(exit_event_handler)

    return ld
