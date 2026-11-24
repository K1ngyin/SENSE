from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("chassis_driver"), "config", "chassis_driver.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value=(
                    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C98180921-if00"
                ),
                description="Serial device connected to the STM32 chassis",
            ),
            Node(
                package="chassis_driver",
                executable="chassis_driver",
                name="chassis_driver",
                output="screen",
                parameters=[config_path, {"port": LaunchConfiguration("port")}],
            )
        ]
    )
