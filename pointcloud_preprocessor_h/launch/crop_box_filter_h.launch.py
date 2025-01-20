import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def get_vehicle_info(path):
    with open(path, "r") as f:
        gp = yaml.safe_load(f)["/**"]["ros__parameters"]
    p = {}
    p["min_x"] = -gp["rear_overhang"]
    p["max_x"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_y"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_y"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_z"] = 0.0
    p["max_z"] = gp["vehicle_height"]
    return p

def get_mirror_info(path):
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p

def launch_setup(context, *args, **kwargs):
    mirror_param_file = LaunchConfiguration('mirror_param_file').perform(context)
    vehicle_info_param_file = LaunchConfiguration('vehicle_info_param_file').perform(context)

    vehicle_info = get_vehicle_info(vehicle_info_param_file)
    mirror_info = get_mirror_info(mirror_param_file)

    return [
        Node(
            package='pointcloud_preprocessor',  # 패키지 이름 설정
            executable='crop_box_filter_node',
            name='crop_box_filter_self',
            namespace='sensing/lidar/ouster',
            output='screen',
            remappings=[
                ('/sensing/lidar/ouster/input', '/sensing/lidar/ouster/points'),
                ('/sensing/lidar/ouster/output', 'crop_box_filter_self/self_cropped_pointcloud_ex')
            ],
            parameters=[
                {'input_frame': LaunchConfiguration('input_frame').perform(context)},
                {'output_frame': LaunchConfiguration('output_frame').perform(context)},
                {'min_x': vehicle_info['min_x']},
                {'max_x': vehicle_info['max_x']},
                {'min_y': vehicle_info['min_y']},
                {'max_y': vehicle_info['max_y']},
                {'min_z': vehicle_info['min_z']},
                {'max_z': vehicle_info['max_z']},
                {'negative': True}  # 차량 포인트 제거를 위해 negative 설정
            ]
        ),
        Node(
            package='pointcloud_preprocessor',  # 패키지 이름 설정
            executable='crop_box_filter_node',
            name='crop_box_filter_mirror',
            namespace='sensing/lidar/ouster',
            output='screen',
            remappings=[
                ('/sensing/lidar/ouster/input', 'crop_box_filter_self/self_cropped_pointcloud_ex'),
                ('/sensing/lidar/ouster/output', 'crop_box_filter_mirror/mirror_cropped_pointcloud_ex')
            ],
            parameters=[
                {'input_frame': LaunchConfiguration('input_frame').perform(context)},
                {'output_frame': LaunchConfiguration('output_frame').perform(context)},
                {'min_x': mirror_info['min_longitudinal_offset']},
                {'max_x': mirror_info['max_longitudinal_offset']},
                {'min_y': mirror_info['min_lateral_offset']},
                {'max_y': mirror_info['max_lateral_offset']},
                {'min_z': mirror_info['min_height_offset']},
                {'max_z': mirror_info['max_height_offset']},
                {'negative': True}  # 사이드 미러 포인트 제거를 위해 negative 설정
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mirror_param_file',
            default_value='/home/a/autoware/src/vehicle/sample_vehicle_launch/sample_vehicle_description/config/mirror.param.yaml',
            description='Path to the mirror parameter file'
        ),
        DeclareLaunchArgument(
            'vehicle_info_param_file',
            default_value='/home/a/autoware/src/vehicle/sample_vehicle_launch/sample_vehicle_description/config/vehicle_info.param.yaml',
            description='Path to the vehicle info parameter file'
        ),
        DeclareLaunchArgument(
            'input_frame',
            default_value='base_link',  # 기본 값 설정, 필요에 따라 변경
            description='Input frame for the crop box'
        ),
        DeclareLaunchArgument(
            'output_frame',
            default_value='base_link',  # 기본 값 설정, 필요에 따라 변경
            description='Output frame for the crop box'
        ),
        OpaqueFunction(function=launch_setup)
    ])
