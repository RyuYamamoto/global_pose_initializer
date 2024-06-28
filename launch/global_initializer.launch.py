from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    global_pose_initializer_config_path = PathJoinSubstitution(
        [FindPackageShare("global_pose_initializer"), "config", "global_pose_initializer_params.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="global_pose_initializer",
                executable="global_pose_initializer_node",
                name="global_pose_initializer",
                output="screen",
                remappings=[("points_raw", "/velodyne_points")],
                parameters=[global_pose_initializer_config_path],
            ),
        ]
    )
