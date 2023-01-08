# Copyright (c) 2018 Intel Corporation
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

"""
Example for spawing multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, condition
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Get the launch directory for multirobot_map_merge where we have a modified launch files
    map_merge_dir = get_package_share_directory("multirobot_map_merge")
    launch_dir_map_merge = os.path.join(map_merge_dir, "launch", "tb3_simulation")

    # Names and poses of the robots for known poses demo
    robots_known_poses = [
        {"name": "robot1", "x_pose": 0.0, "y_pose": 0.5, "z_pose": 0.01},
        {"name": "robot2", "x_pose": -3.0, "y_pose": 1.5, "z_pose": 0.01},
    ]
    # Names and poses of the robots for unknown poses demo, the must be very close at beginning
    robots_unknown_poses = [
        {"name": "robot1", "x_pose": -2.0, "y_pose": 0.5, "z_pose": 0.01},
        {"name": "robot2", "x_pose": -3.0, "y_pose": 0.5, "z_pose": 0.01},
    ]

    # Simulation settings
    world = LaunchConfiguration("world")
    simulator = LaunchConfiguration("simulator")

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration("map")

    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    log_settings = LaunchConfiguration("log_settings", default="true")

    known_init_poses = LaunchConfiguration("known_init_poses")
    declare_known_init_poses_cmd = DeclareLaunchArgument(
        "known_init_poses",
        default_value="True",
        description="Known initial poses of the robots. If so don't forget to declare them in the params.yaml file",
    )

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(launch_dir_map_merge, "worlds", "world_only.model"),
        description="Full path to world file to load",
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "simulator",
        default_value="gazebo",
        description="The simulator to use (gazebo or gzserver)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "maps", "turtlebot3_world.yaml"),
        description="Full path to map file to load",
    )

    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        "robot1_params_file",
        default_value=os.path.join(
            launch_dir_map_merge, "config", "nav2_multirobot_params_1.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for robot1 launched nodes",
    )

    declare_robot2_params_file_cmd = DeclareLaunchArgument(
        "robot2_params_file",
        default_value=os.path.join(
            launch_dir_map_merge, "config", "nav2_multirobot_params_2.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for robot2 launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the stacks",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_namespaced_view.rviz"),
        description="Full path to the RVIZ config file to use.",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    slam_toolbox = LaunchConfiguration("slam_toolbox")
    slam_gmapping = LaunchConfiguration("slam_gmapping")
    declare_slam_toolbox_cmd = DeclareLaunchArgument(
        "slam_toolbox", default_value="False", description="Whether run a SLAM toolbox"
    )
    declare_slam_gmapping_cmd = DeclareLaunchArgument(
        "slam_gmapping",
        default_value="False",
        description="Whether run a SLAM gmapping",
    )

    # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            simulator,
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        output="screen",
    )

    robot_sdf = LaunchConfiguration("robot_sdf")
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(bringup_dir, "worlds", "waffle.model"),
        description="Full path to robot sdf file to spawn the robot in gazebo",
    )

    # Define commands for spawing the robots into Gazebo
    spawn_robots_cmds = []
    for robot_known, robot_unknown in zip(robots_known_poses, robots_unknown_poses):
        # after humble release, use spawn_entity.py
        if os.getenv("ROS_DISTRO") == "humble":
            spawn_robots_cmds.append(
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    output="screen",
                    arguments=[
                        "-entity",
                        robot_known["name"],
                        "-file",
                        robot_sdf,
                        "-robot_namespace",
                        TextSubstitution(text=str(robot_known["name"])),
                        "-x",
                        TextSubstitution(text=str(robot_known["x_pose"])),
                        "-y",
                        TextSubstitution(text=str(robot_known["y_pose"])),
                        "-z",
                        TextSubstitution(text=str(robot_known["z_pose"])),
                        "-R",
                        "0.0",
                        "-P",
                        "0.0",
                        "-Y",
                        "0.0",
                    ],
                    condition=IfCondition(known_init_poses),
                )
            )
            spawn_robots_cmds.append(
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    output="screen",
                    arguments=[
                        "-entity",
                        robot_unknown["name"],
                        "-file",
                        robot_sdf,
                        "-robot_namespace",
                        TextSubstitution(text=str(robot_unknown["name"])),
                        "-x",
                        TextSubstitution(text=str(robot_unknown["x_pose"])),
                        "-y",
                        TextSubstitution(text=str(robot_unknown["y_pose"])),
                        "-z",
                        TextSubstitution(text=str(robot_unknown["z_pose"])),
                        "-R",
                        "0.0",
                        "-P",
                        "0.0",
                        "-Y",
                        "0.0",
                    ],
                    condition=UnlessCondition(known_init_poses),
                )
            )
        else:
            spawn_robots_cmds.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, "launch", "spawn_tb3_launch.py")
                    ),
                    launch_arguments={
                        "x_pose": TextSubstitution(text=str(robot_known["x_pose"])),
                        "y_pose": TextSubstitution(text=str(robot_known["y_pose"])),
                        "z_pose": TextSubstitution(text=str(robot_known["z_pose"])),
                        "robot_name": robot_known["name"],
                        "turtlebot_type": TextSubstitution(text="waffle"),
                    }.items(),
                    condition=IfCondition(known_init_poses),
                )
            )
            spawn_robots_cmds.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, "launch", "spawn_tb3_launch.py")
                    ),
                    launch_arguments={
                        "x_pose": TextSubstitution(text=str(robot_unknown["x_pose"])),
                        "y_pose": TextSubstitution(text=str(robot_unknown["y_pose"])),
                        "z_pose": TextSubstitution(text=str(robot_unknown["z_pose"])),
                        "robot_name": robot_unknown["name"],
                        "turtlebot_type": TextSubstitution(text="waffle"),
                    }.items(),
                    condition=UnlessCondition(known_init_poses),
                )
            )

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots_known_poses:
        params_file = LaunchConfiguration(f"{robot['name']}_params_file")

        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, "rviz_launch.py")
                    ),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "namespace": TextSubstitution(text=robot["name"]),
                        "use_namespace": "True",
                        "rviz_config": rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir_map_merge, "tb3_simulation_launch.py")
                    ),
                    launch_arguments={
                        "namespace": robot["name"],
                        "use_namespace": "True",
                        "map": map_yaml_file,
                        "use_sim_time": "True",
                        "params_file": params_file,
                        "autostart": autostart,
                        "use_rviz": "False",
                        "use_simulator": "False",
                        "headless": "False",
                        "slam": "True",
                        "slam_toolbox": slam_toolbox,
                        "slam_gmapping": slam_gmapping,
                        "use_robot_state_pub": use_robot_state_pub,
                    }.items(),
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=["Launching ", robot["name"]],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot["name"], " map yaml: ", map_yaml_file],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot["name"], " params yaml: ", params_file],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot["name"], " rviz config file: ", rviz_config_file],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[
                        robot["name"],
                        " using robot state pub: ",
                        use_robot_state_pub,
                    ],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot["name"], " autostart: ", autostart],
                ),
            ]
        )

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_robot2_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_slam_toolbox_cmd)
    ld.add_action(declare_slam_gmapping_cmd)
    ld.add_action(declare_known_init_poses_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
