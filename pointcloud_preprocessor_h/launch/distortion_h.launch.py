from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'time_stamp_field_name',
            default_value='time_stamp',
            description='Name of the time stamp field in the point cloud'
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Whether to use IMU data for correction'
        ),
        DeclareLaunchArgument(
            'input_pointcloud_topic',
            default_value='/sensing/lidar/ouster/crop_box_filter_mirror/mirror_cropped_pointcloud_ex',
            description='Input point cloud topic'
        ),
        DeclareLaunchArgument(
            'output_pointcloud_topic',
            default_value='/sensing/lidar/ouster/rectified/pointcloud_ex',
            description='Output undistorted point cloud topic'
        ),
        DeclareLaunchArgument(
            'input_twist_topic',
            default_value='/sensing/vehicle_velocity_converter/twist_with_covariance',
            description='Input twist topic'
        ),
        DeclareLaunchArgument(
            'input_imu_topic',
            default_value='/sensing/imu/imu_data',
            description='Input IMU topic'
        ),
        Node(
            package='pointcloud_preprocessor',
            executable='distortion_corrector_node',
            name='distortion_corrector',
            namespace='sensing/lidar/ouster',
            output='screen',
            parameters=[{
                'time_stamp_field_name': LaunchConfiguration('time_stamp_field_name'),
                'use_imu': LaunchConfiguration('use_imu')
            }],
            remappings=[
                ('~/input/pointcloud', LaunchConfiguration('input_pointcloud_topic')),
                ('~/output/pointcloud', LaunchConfiguration('output_pointcloud_topic')),
                ('~/input/twist', LaunchConfiguration('input_twist_topic')),
                ('~/input/imu', LaunchConfiguration('input_imu_topic'))
            ]
        )
    ])

