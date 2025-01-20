# HH_240628
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def get_vehicle_info(context):
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    nodes = []

    nodes.append(
        ComposableNode(
            package="glog_component",
            plugin="GlogComponent",
            name="glog_component",
            namespace=LaunchConfiguration("ouster_ns"),
        )
    )

    nodes.append(
        ComposableNode(
            package="ouster_ros",
            plugin="ouster_ros::OusterDriver",
            name="os_driver",
            namespace=LaunchConfiguration("ouster_ns"),
            parameters=[
                {
                    **create_parameter_dict(
                        "sensor_hostname",
                        "udp_dest",
                        "lidar_port",
                        "imu_port",
                        "udp_profile_lidar",
                        "lidar_mode",
                        "metadata",
                        "sensor_frame",
                        "lidar_frame",
                        "imu_frame",
                        "point_cloud_frame",
                        "timestamp_mode",
                        "ptp_utc_tai_offset",
                        "use_system_default_qos",
                        "proc_mask",
                        "scan_ring",
                        "point_type",
                    ),
                }
            ],
            # remappings=[
            #     ("points", "ouster/points"),
            # ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            namespace=LaunchConfiguration("ouster_ns"),
            remappings=[
                ("input", "points"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    mirror_info = get_vehicle_mirror_info(context)
    cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            namespace=LaunchConfiguration("ouster_ns"),
            remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            namespace=LaunchConfiguration("ouster_ns"),
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    if LaunchConfiguration("output_as_sensor_frame").perform(context):
        ring_outlier_filter_parameters = {"output_frame": LaunchConfiguration("frame_id")}
    else:
        ring_outlier_filter_parameters = {
            "output_frame": ""
        }
    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            namespace=LaunchConfiguration("ouster_ns"),
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "pointcloud_before_sync"),
            ],
            parameters=[ring_outlier_filter_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace=LaunchConfiguration("ouster_ns"),
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="both",
    )

    return [container]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("ouster_ns", "sensing/lidar/ouster", "Override the default namespace of all ouster nodes")
    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("frame_id", "lidar", "frame id")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml")
    
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("lidar_container_name", "ouster_node_container")
    add_launch_arg("output_as_sensor_frame", "True", "output final pointcloud in sensor frame")

    add_launch_arg("sensor_hostname", "os-122330001086.local", "hostname or IP in dotted decimal form of the sensor")
    add_launch_arg("udp_dest", "169.254.78.94", "hostname or IP where the sensor will send data packets")
    add_launch_arg("lidar_port", "7501", "port to which the sensor should send lidar data")
    add_launch_arg("imu_port", "7502", "port to which the sensor should send imu data")
    add_launch_arg("udp_profile_lidar", "", "lidar packet profile")
    add_launch_arg("lidar_mode", "512x20", "resolution and rate")
    add_launch_arg("timestamp_mode", "TIME_FROM_ROS_TIME", "method used to timestamp measurements")
    add_launch_arg("ptp_utc_tai_offset", "0.0", "UTC/TAI offset in seconds")
    add_launch_arg("metadata", "", "path to write metadata file when receiving sensor data")
    add_launch_arg("sensor_frame", "base_link", "sets name of choice for the sensor_frame tf frame")
    add_launch_arg("lidar_frame", "os_lidar", "sets name of choice for the os_lidar tf frame")
    add_launch_arg("imu_frame", "os_imu", "sets name of choice for the os_imu tf frame")
    add_launch_arg("point_cloud_frame", "", "which frame to be used when publishing PointCloud2 or LaserScan messages")
    add_launch_arg("use_system_default_qos", "false", "Use the default system QoS settings")
    add_launch_arg("proc_mask", "IMG|PCL|IMU|SCAN", "use any combination of the 4 flags to enable or disable specific processors")
    add_launch_arg("scan_ring", "0", "use this parameter in conjunction with the SCAN flag")
    add_launch_arg("point_type", "original", "point type for the generated point cloud")

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

