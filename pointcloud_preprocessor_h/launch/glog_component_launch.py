from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glog_component',
            executable='glog_component_node',  # 실제 실행할 노드 이름으로 변경
            name='glog_component',
            namespace='sensing/logging',
            parameters=[
                # 필요한 파라미터 파일 경로 또는 직접 파라미터 입력
                {'log_level': 'INFO'},
                {'output_file': '/home/a/logs/glog_output.log'}
            ],
            remappings=[
                # 입력/출력 토픽이 있다면 여기에 추가
                # ('input_topic', 'remapped_input_topic'),
                # ('output_topic', 'remapped_output_topic')
            ]
        )
    ])
