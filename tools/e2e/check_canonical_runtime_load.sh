#!/usr/bin/env bash
# HH_260906 - Inspect a fresh canonical runtime without starting ROS nodes or controlling CARLA.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root"
source tools/e2e/workspace_runtime_lock.sh
e2e_acquire_workspace_runtime_lock "read-only canonical runtime load verification"
export AUTOWARE_E2E_SKIP_INSTALL=1
source tools/e2e/env.sh
# HH_260906 - ROS-generated environment hooks are not nounset-compatible.
source autonomy/canonical_rebuild/install/local_setup.bash
python3 - <<'PY'
import os
from pathlib import Path
import subprocess
import sys
root=Path(os.environ['AUTOWARE_E2E_ROOT'])
prefix=root/'autonomy/canonical_rebuild/install'
# HH_260906 - Do not let the root compatibility symlink satisfy this installed-package import.
sys.path=[entry for entry in sys.path if entry and Path(entry).resolve()!=root]
import portable_e2e
from ament_index_python.packages import get_package_prefix
packages=('autoware_e2e_vad_launch','autoware_carla_interface','autoware_tensorrt_vad',
          'autoware_trajectory_follower_node','autoware_map_loader','autoware_mission_planner_universe')
for name in packages:
    observed=Path(get_package_prefix(name))
    if observed!=prefix/name:raise ValueError(f'noncanonical package {name}: {observed}')
    print(f'PACKAGE_PREFIX_OK {name}',flush=True)
for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH'):
    for entry in os.environ.get(key,'').split(':'):
        if entry.startswith(str(root/'install')+'/') or entry.startswith(str(root/'autonomy/install')+'/'):
            raise ValueError(f'legacy active prefix: {key}={entry}')
if Path(portable_e2e.__file__).resolve().parent!=root/'learning/portable_e2e':
    raise ValueError('portable_e2e source differs')
print('CANONICAL_PYTHON_IMPORT_OK',flush=True)
count=0
for name in packages:
    for library in sorted((prefix/name/'lib').glob('*.so')):
        result=subprocess.run(['ldd',str(library)],capture_output=True,text=True,timeout=20,check=True)
        if 'not found' in result.stdout:raise ValueError(f'missing dependencies: {library}\n{result.stdout}')
        count+=1
if count<3:raise ValueError('insufficient shared-library coverage')
print(f'SHARED_LIBRARY_RESOLUTION_OK {count}',flush=True)
PY
# HH_260906 - --show-args parses descriptions only; never remove it in this inspection script.
ros2 launch autoware_e2e_vad_launch portable_e2e_shadow.launch.xml --show-args
ros2 launch autoware_e2e_vad_launch carla_actuation_calibration.launch.xml --show-args
echo CANONICAL_LOAD_CHECK_COMPLETE_NOT_ACTIVATED_NO_VEHICLE_CONTROL
