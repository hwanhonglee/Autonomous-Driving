"""HH_261002 - Publish real V7 teacher cameras with unchanged centered V6 rendering and honest whole-interval evidence."""

from __future__ import annotations

from pathlib import Path

from scripts.e2e import publish_town07_natural_goal_stop_teacher_v6_20261002 as frozen_v6
from scripts.e2e import town07_natural_goal_stop_brakefree_v7_20261002 as task_v7


FROZEN_PUBLISHER_SHA256 = "29ccf539330aa04ee89c3042431b44ea35f26ea145e8d99eaa8b4ced82d8eb55"

# HH_261002 - Enumerate identity/text changes only; inverse AST equality keeps pixel, full-membership and timing validation unchanged.
IDENTITIES = {
    "storage/experiments/validation/2026-10-02/town07_natural_goal_stop_teacher_brakefree_v6":
        "storage/experiments/validation/2026-10-02/town07_natural_goal_stop_teacher_brakefree_v7",
    "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/08_actual_goal_stop_teacher_v6":
        "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/14_actual_goal_stop_teacher_v7",
    "carla.actual_goal_stop_teacher_v6_publication.20261002.v1":
        "carla.actual_goal_stop_teacher_v7_publication.20261002.v1",
    "carla.actual_natural_goal_stop_teacher.20261002.v6":
        "carla.actual_natural_goal_stop_teacher.20261002.v7",
    "carla.actual_natural_goal_future64_brakefree_diagnostic.20261002.v6":
        "carla.actual_natural_goal_future64_brakefree_diagnostic.20261002.v7",
    "v6": "v7",
    "future QA must authenticate this exact V6 native declaration and manifest":
        "future QA must authenticate this exact V7 native declaration and manifest",
    "exact reviewed V6 publication caller receipt required": "exact reviewed V7 publication caller receipt required",
    "one actual V6 episode only": "one actual V7 episode only",
    "derived V6 QA summary outside original raw case only": "derived V7 QA summary outside original raw case only",
    "actual V6 declaration/native identity differs": "actual V7 declaration/native identity differs",
    "Town07 V6 ACTUAL TEACHER | ": "Town07 V7 ACTUAL TEACHER | ",
    "fresh fixed V6 publication category required": "fresh fixed V7 publication category required",
    "# 실제 목적지 정차 교사 주행 v6\n\n이 화면은 실제 기록된 6카메라·측정 차량 위치·경로를 보여줍니다. BasicAgent 교사 제어이며 모델 자율주행이나 Autoware/RViz 화면이 아닙니다.\n\n":
        "# 실제 목적지 정차 교사 주행 v7\n\n이 화면은 실제 기록된 6카메라·측정 차량 위치·경로를 보여줍니다. BasicAgent 교사 제어이며 모델 자율주행이나 Autoware/RViz 화면이 아닙니다.\n\n",
}
task_v7.clone_frozen_provider(frozen_v6, FROZEN_PUBLISHER_SHA256, globals(), literals=IDENTITIES)
_identity_load_archive = load_archive


def load_archive(caller_path, caller_sha):
    # HH_261002 - Retain the authenticated renderer provider itself as an input and recheck it through publication.
    require(digest(Path(frozen_v6.__file__).resolve()) == FROZEN_PUBLISHER_SHA256,
            "frozen V6 whole-interval renderer changed")
    bundle = _identity_load_archive(caller_path, caller_sha)
    add_pin(bundle["pins"], Path(frozen_v6.__file__).resolve(), FROZEN_PUBLISHER_SHA256)
    recheck(bundle["pins"])
    return bundle


if __name__ == "__main__":
    raise SystemExit(main())
