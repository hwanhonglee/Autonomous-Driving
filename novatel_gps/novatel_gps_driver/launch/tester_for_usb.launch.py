"""Launch an example driver that communicates using USB"""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
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
                    'device': '/dev/ttyUSB0', # HH_231108 for usb port
                    'verbose': True,
                    'publish_novatel_positions': True,
                    'publish_novatel_psrdop2': True,
                    'publish_novatel_velocity': True,
                    'publish_novatel_dual_antenna_heading': True, # HH_230414
                    'publish_novatel_heading2': True, # HH_230414
                    'publish_novatel_utm_positions': True,
                    'frame_id': 'gps'
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
