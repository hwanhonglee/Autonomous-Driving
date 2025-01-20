from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyUSB1',
        description='The device file for the GPS receiver'
    )

    container = launch_ros.actions.ComposableNodeContainer(
        name='novatel_gps_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='novatel_gps_driver',
                plugin='novatel_gps_driver::NovatelGpsNode',
                name='novatel_gps',
                parameters=[{
                    'connection_type': 'serial',
                    'device': LaunchConfiguration('device'),
                    'verbose': True,
                    'publish_novatel_positions': True,
                    'publish_novatel_psrdop2': True,
                    'publish_novatel_velocity': True,
                    'publish_novatel_dual_antenna_heading': True,
                    'publish_novatel_heading2': True,
                    'publish_novatel_utm_positions': True,
                    'frame_id': 'gnss_link'
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        device_arg,
        container
    ])
