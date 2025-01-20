import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml

def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    nodes = []

    # os_sensor node
    os_sensor_node = Node(
        package="ouster_ros",
        executable="os_sensor",
        name="os_sensor",
        parameters=[
            {
                "sensor_hostname": "169.254.78.94",
                "udp_dest": "192.168.1.100",
                "lidar_port": 7502,
                "imu_port": 7503,
                "udp_profile_lidar": "RNG19_RFL8_SIG16_NIR16",
                "lidar_mode": "1024x20",
                "timestamp_mode": "TIME_FROM_ROS_TIME",
                "use_system_default_qos": False
                
            }
        ],
        remappings=[
            ("points", "/sensing/lidar/ouster/points")
        ]       
    )

    # os_cloud node
    os_cloud_node = Node(
        package="ouster_ros",
        executable="os_cloud",
        name="os_cloud",
        parameters=[
            {
                "sensor_frame": "os_lidar_base_link",
                "lidar_frame": "os_lidar",
                "imu_frame": "os_imu",
                "timestamp_mode": "TIME_FROM_ROS_TIME",
                "ptp_utc_tai_offset": -37.0,
                "use_system_default_qos": False,
                "proc_mask": "IMG|PCL|IMU|SCAN",
                "scan_ring": 0,
            }
        ],
        remappings=[
            ("points", "/sensing/lidar/ouster/points")
        ]
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="ouster_node_container",
        namespace="pointcloud_container_name",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="screen"
    )

    # 라이프사이클 관리 명령어 추가
    lifecycle_configure = TimerAction(
        period=5.0,  # 5초 지연 후 실행
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/os_sensor', 'configure'],
            output='screen'
        )]
    )

    lifecycle_activate = TimerAction(
        period=10.0,  # 10초 지연 후 실행
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/os_sensor', 'activate'],
            output='screen'
        )]
    )

    return [os_sensor_node, os_cloud_node, lifecycle_configure, lifecycle_activate]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )
    
    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml")
    add_launch_arg("pointcloud_container_name", "pointcloud_preprocessor", description="")

    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("output_as_sensor_frame", "True", "output final pointcloud in sensor frame")
 
    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
