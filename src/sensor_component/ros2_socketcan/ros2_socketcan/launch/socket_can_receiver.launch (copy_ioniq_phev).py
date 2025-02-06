# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.


from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # HH_230818
    socket_can_receiver_node_can0 = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver_can0',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': LaunchConfiguration('interface1'),
            'enable_can_fd': LaunchConfiguration('enable_can_fd'),
            'interval_sec': LaunchConfiguration('interval_sec'),
            'filters': LaunchConfiguration('filters'),
            'use_bus_time': LaunchConfiguration('use_bus_time'),
        }],
        output='screen')
    # HH_230818
    socket_can_receiver_configure_event_handler_can0 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_receiver_node_can0,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node_can0),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )
    # HH_230818
    socket_can_receiver_activate_event_handler_can0 = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_receiver_node_can0,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node_can0),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )
    # HH_230818
    socket_can_receiver_node_can1 = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver_can1',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': LaunchConfiguration('interface2'),
            'enable_can_fd': LaunchConfiguration('enable_can_fd'),
            'interval_sec': LaunchConfiguration('interval_sec'),
            'filters': LaunchConfiguration('filters'),
            'use_bus_time': LaunchConfiguration('use_bus_time'),
        }],
        output='screen')
    # HH_230818
    socket_can_receiver_configure_event_handler_can1 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_receiver_node_can1,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node_can1),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )
    # HH_230818
    socket_can_receiver_activate_event_handler_can1 = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_receiver_node_can1,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node_can1),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can0'),
        DeclareLaunchArgument('enable_can_fd', default_value='false'),
        DeclareLaunchArgument('interval_sec', default_value='0.01'),
        DeclareLaunchArgument('use_bus_time', default_value='false'),
        DeclareLaunchArgument('filters', default_value='0:0'),
        DeclareLaunchArgument('auto_configure', default_value='true'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        # HH_230818
        socket_can_receiver_node_can0,
        socket_can_receiver_configure_event_handler_can0,
        socket_can_receiver_activate_event_handler_can0,
        socket_can_receiver_node_can1,  # Add the new node
        socket_can_receiver_configure_event_handler_can1,  # Add the new event handler
        socket_can_receiver_activate_event_handler_can1,  # Add the new event handler
    ])
