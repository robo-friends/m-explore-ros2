import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params.yaml"
    )

    node = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        parameters=[config],
        output="screen",
    )
    ld.add_action(node)
    return ld
