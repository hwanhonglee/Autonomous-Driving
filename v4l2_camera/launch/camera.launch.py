# HH_240729
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
                {'image_encoding': 'BGR8'},  # Adjust the desired image encoding to match your needs # yuv422_yuy2
                {'camera_info_url': 'file:///home/a/.ros/camera_info/canlab.yaml'},  # Adjust the path
                {'camera_frame_id': 'camera_link'},  # Adjust camera frame id 
                {'pixel_format': 'BGR8'},  # Adjust  
                {'output_encoding': 'BGR8'},  # Adjust 
                {'image_size': [640, 480]},  # Adjust image size
                # {'image_width': '640'},  # Adjust image size
                # {'image_height': '480'},  # Adjust image size
                # Add other parameters as needed
            ],
        ),
    ])