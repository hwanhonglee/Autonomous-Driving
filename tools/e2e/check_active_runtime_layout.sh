#!/usr/bin/env bash
# HH_260906 - Verify the preserved 282-package installation without launching nodes or CARLA.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source tools/e2e/workspace_runtime_lock.sh
e2e_acquire_workspace_runtime_lock "active runtime layout validation"
source tools/e2e/env.sh
export CUDA_VISIBLE_DEVICES=''
python3 - <<'PY'
import os
from pathlib import Path
import subprocess
import sys
from ament_index_python.packages import get_packages_with_prefixes
root=Path(os.environ['AUTOWARE_E2E_ROOT'])
prefix=root/'autonomy/install'
packages=get_packages_with_prefixes()
installed={p.name for p in prefix.iterdir() if (p/'share/ament_index/resource_index/packages'/p.name).is_file()}
assert len(installed)>=282, len(installed)
for name in installed:
    assert Path(packages[name])==prefix/name, (name,packages.get(name))
print('ACTIVE_PACKAGE_PREFIXES_OK',len(installed),flush=True)
# HH_260906 - Test the installed package, not either source-root search entry.
sys.path=[s for s in sys.path if s and Path(s).resolve() not in (root,root/'learning')]
import portable_e2e
assert Path(portable_e2e.__file__).resolve().parent==root/'learning/portable_e2e'
print('INSTALLED_PORTABLE_IMPORT_OK',flush=True)
from agents.navigation.basic_agent import BasicAgent
print('CARLA_AGENT_IMPORT_OK',flush=True)
for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH'):
    for s in os.environ.get(key,'').split(':'):
        for old in ('install','build','log'):
            assert not (s==str(root/old) or s.startswith(str(root/old)+'/')), (key,s)
count=0
for name in ('autoware_carla_interface','autoware_tensorrt_vad','autoware_trajectory_follower_node',
             'autoware_map_loader','autoware_mission_planner_universe'):
    for library in sorted((prefix/name/'lib').glob('*.so')):
        result=subprocess.run(['ldd',str(library)],text=True,capture_output=True,check=True,timeout=20)
        assert 'not found' not in result.stdout, (library,result.stdout)
        count+=1
assert count>=3
print('ACTIVE_LIBRARY_DEPENDENCIES_OK',count,flush=True)
PY
ros2 launch autoware_e2e_vad_launch portable_e2e_shadow.launch.xml --show-args
ros2 launch autoware_e2e_vad_launch carla_actuation_calibration.launch.xml --show-args
# HH_260906 - Reconfigure only the data/launch package, using existing dependencies with no installers.
colcon --log-base autonomy/log build --build-base autonomy/build --install-base autonomy/install \
  --base-paths autonomy/autoware_e2e_vad_launch --packages-select autoware_e2e_vad_launch \
  --symlink-install --cmake-args -DBUILD_TESTING=ON -Dament_cmake_auto_DIR="$AUTOWARE_E2E_AMENT_CMAKE_AUTO_DIR"
python3 tools/workspace_python.py tools/e2e/mission_planner_build_provenance.py verify
python3 tools/workspace_python.py tools/e2e/vad_object_safety_build_provenance.py verify
echo ACTIVE_RUNTIME_LAYOUT_CHECK_COMPLETE_NO_NODES_STARTED
