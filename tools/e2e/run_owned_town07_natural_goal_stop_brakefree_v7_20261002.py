"""HH_261002 - Own one fresh zero-normal-terminal-steering teacher; retain all V6 physical and body/source guards."""

from __future__ import annotations

from scripts.e2e import town07_natural_goal_stop_brakefree_v7_20261002 as task_v7
from scripts.e2e import collect_town07_natural_goal_stop_brakefree_v7_20261002 as worker_v7
from scripts.e2e import run_owned_town07_natural_goal_stop_brakefree_v6_20261002 as frozen_v6_owner

task_v7.clone_frozen_provider(frozen_v6_owner,
    task_v7.V6_SOURCES["tools/e2e/run_owned_town07_natural_goal_stop_brakefree_v6_20261002.py"],globals(),
    literals={"v6":"v7",
        "scripts.e2e.run_owned_town07_natural_goal_stop_brakefree_v6_20261002":"scripts.e2e.run_owned_town07_natural_goal_stop_brakefree_v7_20261002"},
    imports={"town07_natural_goal_stop_brakefree_v6_20261002":"town07_natural_goal_stop_brakefree_v7_20261002",
        "collect_town07_natural_goal_stop_brakefree_v6_20261002":"collect_town07_natural_goal_stop_brakefree_v7_20261002"})

if __name__=="__main__":raise SystemExit(main())
