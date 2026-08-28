#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"

scripts/e2e/apply_smart_mpc_patches.sh

export AUTOWARE_E2E_SKIP_INSTALL=1
source scripts/e2e/env.sh

colcon build \
  --base-paths src autoware_e2e_vad_launch \
  --symlink-install \
  --executor sequential \
  --packages-select \
    autoware_smart_mpc_trajectory_follower \
    tier4_control_launch \
    autoware_launch \
    autoware_e2e_vad_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

unset AUTOWARE_E2E_SKIP_INSTALL
source scripts/e2e/env.sh

python3 - <<'PY'
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

share = Path(get_package_share_directory("autoware_smart_mpc_trajectory_follower"))
e2e_share = Path(get_package_share_directory("autoware_e2e_vad_launch"))
mpc = yaml.safe_load((share / "param/mpc_param.yaml").read_text(encoding="utf-8"))
trained = yaml.safe_load((share / "param/trained_model_param.yaml").read_text(encoding="utf-8"))
runtime = yaml.safe_load((share / "param/runtime.param.yaml").read_text(encoding="utf-8"))
guard = yaml.safe_load(
    (e2e_share / "config/smart_mpc_goal_stop_guard.param.yaml").read_text(encoding="utf-8")
)
mode = mpc["mpc_parameter"]["system"]["mode"]
use_trained = trained["trained_model_parameter"]["control_application"]["use_trained_model"]
default_guard = runtime["/**"]["ros__parameters"][
    "enable_reference_horizon_goal_stop_guard"
]
opt_in_guard = guard["/**"]["ros__parameters"][
    "enable_reference_horizon_goal_stop_guard"
]
if mode != "ilqr" or use_trained or default_guard or not opt_in_guard:
    raise SystemExit(
        "Refusing Smart MPC configuration: "
        f"mode={mode!r}, use_trained_model={use_trained!r}, "
        f"default_guard={default_guard!r}, opt_in_guard={opt_in_guard!r}"
    )
print(
    "Smart MPC ready: mode=ilqr, use_trained_model=false, "
    "goal_stop_guard_default=false, opt_in_profile=true"
)
PY
