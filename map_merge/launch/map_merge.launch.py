import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("multirobot_map_merge"), "config", "params.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    known_init_poses = LaunchConfiguration("known_init_poses")

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
        default_value="True",
        description="Known initial poses of the robots. If so don't forget to declare them in the params.yaml file",
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

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
        remappings=remappings,
    )
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_known_init_poses_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(node)
    return ld
