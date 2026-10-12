"""HH_261004 - Independently inventory a completed private TRAIN endpoint and its frozen97 numeric bytes."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "storage/experiments/plans/2026-10-04/temporal_route_branch_geometry_pilot_v1"
PYTHON = "/home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python"

REMOTE = r'''
# HH_261004 - Read only a completed fixed endpoint; no optimizer, simulator, package or GPU operation.
import hashlib, json, math, stat
from pathlib import Path
import numpy as np
w = Path('/home/pro6000/personal/hwanhong/portable_e2e')
p = w / 'runs/training/temporal_route_branch_geometry_pilot_20261004_v1'
s = w / 'source_worktrees/temporal_route_branch_geometry_pilot_20261004_v1'
def payload(path):
    assert path.resolve(strict=True) == path and stat.S_ISREG(path.lstat().st_mode)
    raw = path.read_bytes(); assert len(raw) <= 64*1024**2
    return raw
def sha(path): return hashlib.sha256(payload(path)).hexdigest()
def document(path): return json.loads(payload(path))
summary = document(p/'summary.json'); declaration = document(p/'declaration.json')
assert summary['status'] == 'FIXED1000_BRANCH_GEOMETRY_COMPLETE_NOT_PROMOTED'
assert not (p/'failure_receipt.json').exists() and summary['model_promoted'] is False
assert summary['evaluation_only_payloads_opened'] is False
assert summary['fit']['optimizer_updates'] == 1000 and summary['fit']['exposures'] == 5000
assert len(declaration['source_sha256']) == 36 and len(declaration['immutable_input_sha256']) == 35
assert all(sha(s/name) == expected and sha(p/'source'/name) == expected for name,expected in declaration['source_sha256'].items())
assert all(sha(Path(name)) == expected for name,expected in declaration['immutable_input_sha256'].items())
history = [json.loads(line) for line in payload(p/'history.jsonl').splitlines()]
assert len(history) == 1000
assert sha(p/'history.jsonl') == summary['fit']['history_sha256']
assert all(row['optimizer_updates'] == i+1 and row['exposures'] == 5*(i+1)
           and row['sample_ids'] == declaration['fixed_sample_ids'][i]
           and row['frozen97_sha256'] == declaration['frozen97_sha256'] for i,row in enumerate(history))
before = document(p/'baseline_panels.json'); after = document(p/'endpoint_panels.json')
checks=[]; raw_count=decoded_count=legacy_count=0
for domain in range(5):
    old=before[str(domain)]; new=after[str(domain)]; rows=new['rows']
    assert old['denominator'] == new['denominator'] == len(old['rows']) == len(rows) == 32
    assert len(set(row['sample_id'] for row in rows)) == 32
    for a,b in zip(old['rows'],rows):
        assert a['sample_id'] == b['sample_id'] and a['batch_sha256'] == b['batch_sha256']
        assert a['parent'] == b['parent'] and a['parent_output_sha256'] == b['parent_output_sha256']
        assert b['split']=='TRAIN' and b['parity']['raw_speed_byte_equal'] is True
        raw_count += 1
        if domain == 2:
            assert b['parity']['instantaneous_speed_byte_equal'] is None and b['parity']['arc_byte_equal'] is None
            assert b['parity']['decoded_parity_reason'] == 'LEGACY_RAW_ONLY_NO_DECODER'
            legacy_count += 1
        else:
            assert b['parity']['instantaneous_speed_byte_equal'] is True and b['parity']['arc_byte_equal'] is True
            decoded_count += 1
    metrics = ['true_raw_curvature_huber'] if domain==2 else ['true_ADE_m','true_FDE_m','true_raw_curvature_huber']
    terms=[(metric,reduction,1.05) for metric in metrics for reduction in ('mean','peak')] if domain<3 else [('true_ADE_m','mean',.9),('true_FDE_m','mean',1.)]
    for metric,reduction,factor in terms:
        values=[[float(row[arm][metric]) for row in rows] for arm in ('parent','candidate')]
        assert all(math.isfinite(v) and v>=0 for group in values for v in group)
        a,b=[sum(group)/32 if reduction=='mean' else max(group) for group in values]
        limit=a*factor
        checks.append(dict(domain=domain,metric=metric,reduction=reduction,denominator=32,parent=a,candidate=b,maximum=limit,passed=b<=limit))
gates=summary['TRAIN_engineering_gates']
assert len(checks)==18 and checks==gates['checks'] and (raw_count,decoded_count,legacy_count)==(160,128,32)
assert gates['numerical_slack']==0 and gates['model_promoted'] is False
parent=w/'runs/training/temporal_route_joint_20261002_v1/exports/step_006000/model_state.npz'
assert sha(parent)=='b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625'
branch={'route_recovery_branch.0.weight','route_recovery_branch.0.bias','route_recovery_branch.2.weight','route_recovery_branch.2.bias'}
export_receipts=[]
with np.load(parent,allow_pickle=False) as reference:
    assert len(reference.files)==len(set(reference.files))==101
    for update in (500,1000):
        d=p/'checkpoints'/('step_%06d'%update); metadata=document(d/'metadata.json')
        assert metadata['trainable_tensor_count']==4 and metadata['frozen_tensor_count']==97
        assert metadata['inference_loader_authorized'] is False and metadata['simulator_control_authorized'] is False
        assert sha(d/'model_state.npz')==metadata['npz_sha256']
        with np.load(d/'model_state.npz',allow_pickle=False) as arrays:
            assert len(arrays.files)==len(set(arrays.files))==101 and set(arrays.files)==set(reference.files)
            assert all(arrays[k].shape==reference[k].shape and arrays[k].dtype==reference[k].dtype==np.float32
                       and np.isfinite(arrays[k]).all() for k in arrays.files)
            assert all(arrays[k].tobytes()==reference[k].tobytes() for k in arrays.files if k not in branch)
            changed=[k for k in sorted(branch) if arrays[k].tobytes()!=reference[k].tobytes()]
        export_receipts.append(dict(update=update,frozen97_parent_exact=True,changed_branch_names=changed,npz_sha256=sha(d/'model_state.npz'),metadata_sha256=sha(d/'metadata.json')))
assert document(p/'started.json')['images']==summary['final_images']
files={str(q.relative_to(p)):sha(q) for q in sorted(p.rglob('*')) if q.is_file()}
assert not any(q.is_symlink() for q in p.rglob('*'))
print(json.dumps(dict(status='ROOT_COMPLETED_FIXED1000_INDEPENDENT_NUMERIC_RECHECK',
    files_sha256=files,total_bytes=sum((p/name).stat().st_size for name in files),
    source36_and_input35_pins_exact=True,all1000_journal_rows_exact=True,
    original_TRAIN_images_before_after=summary['final_images'],gate_checks=checks,
    engineering_gates_passed=sum(c['passed'] for c in checks),engineering_gates_denominator=18,
    raw_speed_parity_count=raw_count,decoded_speed_arc_parity_count=decoded_count,legacy_unavailable_count=legacy_count,
    exports=export_receipts,fit=summary['fit'],model_promoted=False,feature_approved=False)))
'''


def main():
    # HH_261004 - Root must wait for the actual fit dispatcher to end before this create-only postcheck.
    fit = json.loads((PLAN / "ROOT_remote_GPU0_fixed1000_fit.json").read_text())
    if fit["returncode"] != 0:
        raise RuntimeError("the actual fixed endpoint did not complete; retain failure instead of postchecking approval")
    command = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "pro6000-training",
               shlex.join([PYTHON, "-B", "-c", REMOTE])]
    result = subprocess.run(command, capture_output=True, text=True)
    with (PLAN / "ROOT_remote_endpoint_postcheck_attempt.json").open("x", encoding="utf-8") as stream:
        json.dump(dict(returncode=result.returncode, stdout=result.stdout, stderr=result.stderr,
                       source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()), stream, indent=2)
        stream.write("\n")
    if result.returncode:
        raise RuntimeError("endpoint postcheck failed; complete evidence retained")
    receipt = json.loads(result.stdout)
    with (PLAN / "ROOT_remote_endpoint_audit.json").open("x", encoding="utf-8") as stream:
        json.dump(receipt, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({key:receipt[key] for key in (
        "status","total_bytes","engineering_gates_passed","engineering_gates_denominator",
        "raw_speed_parity_count","decoded_speed_arc_parity_count","legacy_unavailable_count","model_promoted")},sort_keys=True))


if __name__ == "__main__":
    main()
