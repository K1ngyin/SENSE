from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("astra_camera_driver"), "config", "astra_camera.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=default_params,
                description="Astra camera node parameter YAML",
            ),
            Node(
                package="astra_camera_driver",
                executable="astra_camera_node",
                name="astra_camera_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
