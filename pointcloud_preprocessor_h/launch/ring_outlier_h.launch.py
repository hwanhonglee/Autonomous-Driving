from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'distance_ratio',
            default_value='1.03',
            description='Distance ratio threshold'
        ),
        DeclareLaunchArgument(
            'object_length_threshold',
            default_value='0.1',
            description='Object length threshold'
        ),
        DeclareLaunchArgument(
            'num_points_threshold',
            default_value='4',
            description='Number of points threshold'
        ),
        DeclareLaunchArgument(
            'max_rings_num',
            default_value='128',
            description='Maximum number of rings'
        ),
        DeclareLaunchArgument(
            'max_points_num_per_ring',
            default_value='4000',
            description='Maximum number of points per ring'
        ),
        DeclareLaunchArgument(
            'publish_outlier_pointcloud',
            default_value='false',
            description='Whether to publish outlier point cloud'
        ),
        DeclareLaunchArgument(
            'min_azimuth_deg',
            default_value='0.0',
            description='Minimum azimuth degree'
        ),
        DeclareLaunchArgument(
            'max_azimuth_deg',
            default_value='360.0',
            description='Maximum azimuth degree'
        ),
        DeclareLaunchArgument(
            'max_distance',
            default_value='12.0',
            description='Maximum distance'
        ),
        DeclareLaunchArgument(
            'vertical_bins',
            default_value='128',
            description='Number of vertical bins'
        ),
        DeclareLaunchArgument(
            'horizontal_bins',
            default_value='36',
            description='Number of horizontal bins'
        ),
        DeclareLaunchArgument(
            'noise_threshold',
            default_value='2',
            description='Noise threshold'
        ),
        DeclareLaunchArgument(
            'input_pointcloud_topic',
            default_value='/sensing/lidar/ouster/crop_box_filter_mirror/mirror_cropped_pointcloud_ex',
            description='Input point cloud topic'
        ),
        DeclareLaunchArgument(
            'output_pointcloud_topic',
            default_value='/sensing/lidar/ouster/ring_outlier/pointcloud_ex',
            description='Output point cloud topic'
        ),
        Node(
            package='pointcloud_preprocessor',
            executable='ring_outlier_filter_node',
            name='ring_outlier_filter',
            namespace='sensing/lidar/ouster',
            output='screen',
            parameters=[{
                'distance_ratio': LaunchConfiguration('distance_ratio'),
                'object_length_threshold': LaunchConfiguration('object_length_threshold'),
                'num_points_threshold': LaunchConfiguration('num_points_threshold'),
                'max_rings_num': LaunchConfiguration('max_rings_num'),
                'max_points_num_per_ring': LaunchConfiguration('max_points_num_per_ring'),
                'publish_outlier_pointcloud': LaunchConfiguration('publish_outlier_pointcloud'),
                'min_azimuth_deg': LaunchConfiguration('min_azimuth_deg'),
                'max_azimuth_deg': LaunchConfiguration('max_azimuth_deg'),
                'max_distance': LaunchConfiguration('max_distance'),
                'vertical_bins': LaunchConfiguration('vertical_bins'),
                'horizontal_bins': LaunchConfiguration('horizontal_bins'),
                'noise_threshold': LaunchConfiguration('noise_threshold')
            }],
            remappings=[
                ('/sensing/lidar/ouster/input', LaunchConfiguration('input_pointcloud_topic')),
                ('/sensing/lidar/ouster/output', LaunchConfiguration('output_pointcloud_topic'))
            ],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])

