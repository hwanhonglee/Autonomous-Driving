from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pc3_ntrip_client.config import DEFAULT_CONFIG_FILE

# HH_260811 - Default to the user's external private config, never the installed template.

DEFAULT_CONFIG = DEFAULT_CONFIG_FILE


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=DEFAULT_CONFIG,
                description=(
                    "External private NTRIP configuration file (regular file, mode 0600)"
                ),
            ),
            Node(
                package="pc3_ntrip_client",
                executable="ntrip_client_node",
                name="pc3_ntrip_client",
                output="screen",
                parameters=[{"config_file": config_file}],
            ),
        ]
    )
