# Copyright 2026 Hwanhong Lee
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# HH_260810 - Keep launch strings identical to the typed C++ source-mode contract.
_MODES = {
    'real_only',
    'shadow',
    'hybrid_optional',
    'hybrid_required',
    'virtual_only_required',
}
_CANONICAL_MODES = {'hybrid_optional', 'hybrid_required', 'virtual_only_required'}
_REQUIRED_MODES = {'hybrid_required', 'virtual_only_required'}


# HH_260810 - Reject misspelled Boolean gates instead of relying on string truthiness.
def _parse_bool(name: str, raw_value: str) -> bool:
    normalized = raw_value.strip().lower()
    if normalized in {'true', '1'}:
        return True
    if normalized in {'false', '0'}:
        return False
    raise RuntimeError(f'{name} must be true/false or 1/0, got: {raw_value!r}')


# HH_260810 - Resolve gates before constructing a process so rejection leaves no partial node.
def _launch_setup(context: LaunchContext, *args, **kwargs):
    del args, kwargs
    mode = LaunchConfiguration('mode').perform(context).strip()
    if mode not in _MODES:
        raise RuntimeError(f'unsupported VILS source mode: {mode!r}')

    # HH_260810 - Block canonical selection until Stage 3 and READY/MRM exist.
    if mode in _CANONICAL_MODES:
        raise RuntimeError(
            f'{mode} is intentionally blocked until Stage 3 is accepted and the reviewed '
            'READY/MRM supervision path is implemented'
        )

    # HH_260810 - Preserve the physical pipeline exactly: real_only creates no VILS process.
    if mode == 'real_only':
        return []

    contract_approved = _parse_bool(
        'contract_approved', LaunchConfiguration('contract_approved').perform(context)
    )
    enable_canonical_selection = _parse_bool(
        'enable_canonical_selection',
        LaunchConfiguration('enable_canonical_selection').perform(context),
    )
    enable_required_modes = _parse_bool(
        'enable_required_modes', LaunchConfiguration('enable_required_modes').perform(context)
    )

    # HH_260810 - Canonical selection needs a reviewed contract and an independent launch gate.
    if mode in _CANONICAL_MODES and not contract_approved:
        raise RuntimeError(f'{mode} requires contract_approved:=true')
    if mode in _CANONICAL_MODES and not enable_canonical_selection:
        raise RuntimeError(f'{mode} requires enable_canonical_selection:=true')
    if mode in _REQUIRED_MODES and not enable_required_modes:
        raise RuntimeError(f'{mode} requires enable_required_modes:=true')

    param_file = LaunchConfiguration('param_file').perform(context).strip()
    physical_topic = LaunchConfiguration('physical_objects_topic').perform(context).strip()
    output_topic = LaunchConfiguration('output_objects_topic').perform(context).strip()
    provenance_path = LaunchConfiguration('provenance_log_path').perform(context).strip()
    if not param_file:
        raise RuntimeError('param_file must not be empty outside real_only mode')
    if not physical_topic or not output_topic:
        raise RuntimeError('physical_objects_topic and output_objects_topic must not be empty')
    if contract_approved and not provenance_path:
        raise RuntimeError('an approved contract requires a non-empty provenance_log_path')

    # HH_260810 - Override deployment selectors while retaining the YAML contract values.
    integration_node = Node(
        package='autoware_vils_object_integration',
        executable='vils_object_integration_node',
        name='vils_object_integration',
        output='screen',
        parameters=[
            param_file,
            {
                'mode': mode,
                'contract_approved': contract_approved,
                'enable_canonical_selection': enable_canonical_selection,
                'enable_required_modes': enable_required_modes,
                'topics.physical_objects': physical_topic,
                'topics.output_objects': output_topic,
                'evidence.provenance_log_path': provenance_path,
            },
        ],
    )
    return [integration_node]


# HH_260810 - Expose one interface for the upper PC2 launch and bounded test fixtures.
def generate_launch_description():
    default_param_file = PathJoinSubstitution(
        [
            FindPackageShare('autoware_vils_object_integration'),
            'config',
            'vils_object_integration.param.yaml',
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('mode', default_value='real_only'),
            DeclareLaunchArgument('contract_approved', default_value='false'),
            DeclareLaunchArgument('enable_canonical_selection', default_value='false'),
            DeclareLaunchArgument('enable_required_modes', default_value='false'),
            DeclareLaunchArgument('param_file', default_value=default_param_file),
            DeclareLaunchArgument(
                'physical_objects_topic',
                default_value='/perception/object_recognition/tracking/objects',
            ),
            DeclareLaunchArgument(
                'output_objects_topic',
                default_value='/perception/pc2/vils/candidate_tracked_objects',
            ),
            DeclareLaunchArgument('provenance_log_path', default_value=''),
            OpaqueFunction(function=_launch_setup),
        ]
    )
