"""HH_261001 - Correct only exact writer camera order in a new audit; preserve captured data and frozen v1 receipts."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import json
from pathlib import Path

from . import audit_temporal_route_recovery_fast45_batch_20261001 as previous

SCHEMA = previous.SCHEMA
AUDIT_REVISION_SCHEMA = "portable_e2e.actual_route_recovery_fast45_writer_order_audit.20261001.v2"
V1_SHA = "f95c0c424287120dbeff8212658497b38892c8a5dab7cb11f57a2e7d0b15bd87"
WRITER_SOURCE = previous.ROOT / "tools/e2e/collect_carla_vad_expert.py"
WRITER_SOURCE_SHA = "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08"
WRITER_CAMERA_ORDER = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
DISPLAY_CAMERA_ORDER = ("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT")
_original_post_ACK = previous.audit_post_ACK_recording
_original_build_report = previous.build_report
_active = False
_order_active = False
require, digest = previous.require, previous.digest


def verify_frozen_sources():
    require(digest(previous.__file__) == V1_SHA, "unknown or modified frozen v1 audit source")
    require(digest(WRITER_SOURCE) == WRITER_SOURCE_SHA, "unknown exact collector writer source")
    declarations = [node for node in ast.parse(WRITER_SOURCE.read_bytes()).body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "MODEL_CAMERA_ORDER" for target in node.targets)]
    require(len(declarations) == 1 and ast.literal_eval(declarations[0].value) == WRITER_CAMERA_ORDER,
        "collector MODEL_CAMERA_ORDER must match the exact frozen writer tuple; no permutation acceptance")


def audit_post_ACK_recording(folder, episode, cameras, timing, pins):
    """HH_261001 - Restore visual order on success or failure; all original lexical, ACK, count, hash and path checks remain."""
    global _order_active
    require(_active and not _order_active, "non-nested explicitly scoped writer-order audit required")
    require(previous.frozen.CAMERAS == DISPLAY_CAMERA_ORDER, "unknown original display camera order cannot be replaced")
    # HH_261001 - Duplicate the unchanged full-audit native camera-order guard at this corrected receipt boundary.
    require(all(tuple(camera.get("camera_order", ())) == WRITER_CAMERA_ORDER for camera in cameras),
        "declared native camera_order differs from exact frozen producer order")
    original = previous.frozen.CAMERAS
    _order_active = True; previous.frozen.CAMERAS = WRITER_CAMERA_ORDER
    try:
        result = _original_post_ACK(folder, episode, cameras, timing, pins)
        result.update(audit_revision_schema=AUDIT_REVISION_SCHEMA, exact_writer_camera_order=list(WRITER_CAMERA_ORDER),
            original_literal_receipts_and_images_unchanged=True, camera_permutation_accepted=False)
        return result
    finally:
        previous.frozen.CAMERAS = original; _order_active = False


@contextmanager
def activated():
    """HH_261001 - Bind one corrected helper in an offline audit only, never mutate source bytes or captured receipts."""
    global _active
    require(not _active and not _order_active, "writer-order audit scope cannot nest")
    verify_frozen_sources()
    require(previous.audit_post_ACK_recording is _original_post_ACK and previous.frozen.CAMERAS == DISPLAY_CAMERA_ORDER,
        "unknown original audit binding cannot be replaced")
    original = previous.audit_post_ACK_recording
    _active = True; previous.audit_post_ACK_recording = audit_post_ACK_recording
    try: yield
    finally: previous.audit_post_ACK_recording = original; _active = False


def build_report(*args):
    pins = {}
    previous.add_pin(pins, Path(__file__))
    previous.add_pin(pins, WRITER_SOURCE, WRITER_SOURCE_SHA)
    with activated(): result = _original_build_report(*args)
    require(result["schema"] == SCHEMA, "original caller/result schema must remain unchanged")
    for path, value in pins.items(): previous.add_pin(result["source_sha256"], path, value)
    result.update(audit_revision_schema=AUDIT_REVISION_SCHEMA, exact_writer_camera_order=list(WRITER_CAMERA_ORDER),
        writer_order_revision_scope="exact collector order instead of display order only; all original safety, body, timing and source gates unchanged",
        captured_source_or_caller_receipts_rewritten=False, frozen_v1_audit_sha256=V1_SHA,
        exact_writer_source_sha256=WRITER_SOURCE_SHA)
    previous.frozen.assert_pins(result["source_sha256"])
    return result


def publish(output, *args):
    output = Path(output)
    require(not output.exists() and not output.is_symlink(), "fresh create-only audit v2 output required")
    result = build_report(*args)
    previous.frozen.assert_pins(result["source_sha256"])
    output.mkdir(parents=True, exist_ok=False)
    for name, value in (("batch_outcome.json", result), ("source_evidence.json", result["source_sha256"])):
        with (output/name).open("x") as stream: json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
    with (output/"README.md").open("x") as stream:
        stream.write(previous.markdown(result) + "\n확인 도구 v2는 기록 카메라의 정확한 순서만 교정했습니다. 원자료·기존 v1 도구·판정 기준은 변경하지 않았습니다.\n")
    previous.frozen.assert_pins(result["source_sha256"])
    return dict(output=str(output), result_sha256=digest(output/"batch_outcome.json"),
        all_trials_joint_passed=result["all_trials_motion_body_policy_AND_wall100ms_passed"],
        audit_revision_schema=AUDIT_REVISION_SCHEMA, simulator_or_GPU_used=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("batch", "caller-receipts", "output"): parser.add_argument("--"+name, type=Path, required=True)
    for name in ("batch-sha256", "caller-receipts-sha256"): parser.add_argument("--"+name, required=True)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    inputs = (args.batch, args.batch_sha256, args.caller_receipts, args.caller_receipts_sha256)
    result = build_report(*inputs) if args.validate_only else publish(args.output, *inputs)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
