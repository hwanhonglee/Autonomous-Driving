# camera_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',  # Replace with actual package name
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen',
            parameters=[
                {'camera_device': '/dev/video1'},  # Adjust the camera device path
                {'image_encoding': 'yuv422_yuy2'},  # Adjust the desired image encoding
                # Add other parameters as needed
            ],
        ),
    ])

