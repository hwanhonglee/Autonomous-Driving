"""HH_261002 - Scope a publication-only Town03 identity/wording revision; retain every original archive guard."""
from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path

from . import publish_temporal_route_joint_town03_right_20261002 as previous

SCHEMA = "portable_e2e.actual_joint_town03_known_dev_right_publication.20261002.v2"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
FROZEN_PATH = previous.ROOT / "tools/e2e/publish_temporal_route_joint_town03_right_20261002.py"
FROZEN_SHA = "ab179034c9bd3e73f0776245695e0edcaacb9e316d2c580102786862a73bf7d8"
RUN_OUTPUTS = dict(previous.RUN_OUTPUTS,
    temporal_route_joint_town03_known_dev_right_v3="v3_verified_nn_controlled_green_basic_turn")
_load_archive, _outcome_text, _markdown = previous.load_archive, previous.outcome_text, previous.markdown
_active = False


def publication_sources():
    # HH_261002 - Include this exact wrapper and all original eleven source providers, not just the old entry point.
    return previous.core.audit.source_closure(Path(__file__), previous.ROOT / "tools/workspace_python.py",
        previous.ROOT / "tools/canonical_workspace.py")


def outcome_text(bundle):
    run = bundle["episode"].parent.name
    if run != "temporal_route_joint_town03_known_dev_right_v1" and not bundle["control_census"]["observed_prior_learned_native_effect_count"]:
        return "TEACHER-ONLY / NN INTEGRATION FAILURE / no observed NN effect", "교사만 주행한 NN 통합 실패 — NN 우회전 성능 판정 불가"
    return _outcome_text(bundle)


def markdown(bundle, pngs):
    # HH_261002 - Replace only the zero-NN stage explanation, never any model/owner/body/transport verdict.
    result = _markdown(bundle, pngs)
    if bundle["episode"].parent.name != "temporal_route_joint_town03_known_dev_right_v1" and not bundle["control_census"]["observed_prior_learned_native_effect_count"]:
        before = "이 기록은 실제 NN 제어 효과를 관측하기 전에 끝났습니다. **NN 우회전 성능의 실패나 성공으로 세지 않습니다.** 교사·위험 감지의 제동과 모델 명령을 구분하며, 수신→ACK 처리 시간을 없던 자료로 만들지 않습니다."
        after = "이 기록의 전체 주행은 교사 제어이며 NN 제어 효과는 관측되지 않았습니다. 출발 직후 중단한 v1과 달리, 긴 교사 주행을 했더라도 **NN 우회전 성공으로 세지 않습니다.** NN 통합 실패·원본 owner FAIL·실제 차체 FAIL이 있다면 모두 그대로 유지합니다. 교사가 목적지에 갔다는 사실을 모델 주행이나 신호 준수 성능으로 바꾸지 않습니다."
        previous.require(result.count(before) == 1, "original zero-NN stage wording must occur exactly once")
        result = result.replace(before, after)
    return result + "\n발행 v2는 문구와 정확한 v1/v2/v3 출력 구분만 추가했습니다. 제어·차체·시간·원본 실패 판정을 완화하지 않았으며, 별도 감사 원문은 provenance의 auxiliary_root_audits에만 연결합니다.\n"


def load_archive(caller_path, caller_sha, *, decode=False):
    caller, bundle, pins = _load_archive(caller_path, caller_sha, decode=decode)
    if bundle["episode"].parent.name.endswith("_v3"):
        # HH_261002 - The old v2-only disclosure guard must also apply to the newly enumerated v3 role.
        plan = pins.json(bundle["episode"].parent / "declaration.json")
        previous.require(plan["schema"] == "portable_e2e.owned_route_joint_town03_known_dev_right.20261002.v3",
            "exact prospectively declared v3 owner schema required")
        expected = dict(original_own_world_traffic_actors="ALL", state="Green", frozen=True, ignore_traffic_lights=False,
            all_original_hazard_detectors_and_emergency_returns_unchanged=True, known_v1_hazard_subtype_proven=False,
            closed_course_basic_turn_only_not_traffic_rule_performance=True)
        actual = plan.get("owned_green_signal_fixture", {})
        previous.require(plan.get("actual_manual_first_gear_bootstrap_and_engagement_required") is True
            and all(type(actual.get(k)) is type(v) and actual[k] == v for k, v in expected.items()),
            "v3 must preserve the controlled-green/manual disclosure and all original hazard detectors")
    return caller, bundle, pins


@contextmanager
def activated():
    global _active
    previous.require(not _active and previous.prior.digest(FROZEN_PATH) == FROZEN_SHA,
        "non-nesting exact frozen publisher identity required")
    changes = dict(SCHEMA=SCHEMA, CALLER_SCHEMA=CALLER_SCHEMA, RUN_OUTPUTS=RUN_OUTPUTS,
        publication_sources=publication_sources, outcome_text=outcome_text, markdown=markdown, load_archive=load_archive)
    saved = {name: getattr(previous, name) for name in changes}
    previous.require(saved["load_archive"] is _load_archive and saved["outcome_text"] is _outcome_text and saved["markdown"] is _markdown,
        "unknown publication provider binding forbidden")
    _active = True
    try:
        for name, value in changes.items(): setattr(previous, name, value)
        yield
    finally:
        for name, value in saved.items(): setattr(previous, name, value)
        _active = False


def main(argv=None):
    with activated(): return previous.main(argv)


if __name__ == "__main__": raise SystemExit(main())
