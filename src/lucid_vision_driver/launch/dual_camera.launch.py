# Copyright 2026
# Licensed under the Apache License, Version 2.0.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def load_camera_parameters(filename):
    package_share = get_package_share_directory("lucid_vision_driver")
    parameter_path = os.path.join(package_share, "param", filename)
    with open(parameter_path, "r", encoding="utf-8") as parameter_file:
        return yaml.safe_load(parameter_file)["/**"]["ros__parameters"]


def camera_container(container_name, node_name, parameters):
    # HH_260812 - Arena OpenSystem is process-global, so each physical camera gets
    # its own component_container process rather than two components in one process.
    return ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="lucid_vision_driver",
                plugin="ArenaCameraNode",
                name=node_name,
                parameters=[parameters],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        output="both",
    )


def generate_launch_description():
    windshield_parameters = load_camera_parameters("windshield_cam.yaml")
    loop_top_parameters = load_camera_parameters("loop_top_cam.yaml")

    return LaunchDescription(
        [
            camera_container(
                "windshield_camera_container",
                "arena_camera_node_windshield",
                windshield_parameters,
            ),
            camera_container(
                "loop_top_camera_container",
                "arena_camera_node_loop_top",
                loop_top_parameters,
            ),
        ]
    )
