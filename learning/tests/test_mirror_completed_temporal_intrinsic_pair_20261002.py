"""HH_261002 - Synthetic full fixed-pair archives only; no actual remote/model/GPU inputs."""
import json
import hashlib
import shlex
import shutil
import subprocess
import sys
from types import SimpleNamespace
import pytest
from scripts.e2e import mirror_completed_temporal_intrinsic_pair_20261002 as tool


def put(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))


@pytest.fixture(scope="module")
def original(tmp_path_factory):
    base=tmp_path_factory.mktemp("hh_synthetic_completed_pair");run=base/"run";source=base/"source";run.mkdir();source.mkdir();sources={}
    for i in range(29):
        name="tools/source_%02d.py"%i;p=source/name;p.parent.mkdir(exist_ok=True);p.write_text("# synthetic %d\n"%i);sources[name]=tool.digest(p)
    protocol=dict(sources=sources,fixture_only=True);put(source/"authorization.json",dict(contract=protocol));authsha=tool.digest(source/"authorization.json")
    put(source/"transfer_receipt.json",dict(file_sha256=dict(sources,**{"authorization.json":authsha}),source_file_count=29,authorization_file_count=1))
    declaration=dict(schema=tool.TRAINING,protocol=protocol,source_sha256=sources,authorization_sha256=authsha,
        fixed_sample_ids=[["r0","r1","v4","legacy"] for _ in range(6000)],
        fixed_schedule_sha256=hashlib.sha256(json.dumps([[0,1,2,3] for _ in range(6000)],sort_keys=True,separators=(",",":")).encode()).hexdigest())
    put(run/"declaration.json",declaration);results={}
    for arm in ("control","intrinsic_curve"):
        folder=run/arm;put(folder/"declaration.json",dict(declaration,arm=arm));receipts=[]
        for name in sources:
            target=folder/"source"/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source/name,target)
        for step in range(500,6001,500):
            export=folder/("exports/step_%06d"%step);checkpoint=folder/("checkpoints/step_%06d.pt"%step)
            checkpoint.parent.mkdir(exist_ok=True);checkpoint.write_bytes(b"hashed only synthetic PT - not pickle")
            export.mkdir(parents=True);(export/"model_state.npz").write_bytes(b"hashed only synthetic NPZ - not loaded")
            put(export/"metadata.json",dict(optimizer_updates=step,arm=arm,source_sha256=sources,declaration_sha256=tool.digest(folder/"declaration.json")))
            receipts.append(dict(optimizer_updates=step,checkpoint_sha256=tool.digest(checkpoint),npz_sha256=tool.digest(export/"model_state.npz"),metadata_sha256=tool.digest(export/"metadata.json")))
        rows=[dict(optimizer_updates=i,examples_seen=4*i,sample_ids=declaration["fixed_sample_ids"][i-1],indices=[0,1,2,3],
            legacy_curved=i%2==1,intrinsic_coefficient=.05 if arm=="intrinsic_curve" and i%2 else 0.,batch_sha256="a"*64,loss=.5) for i in range(1,6001)]
        (folder/"history.jsonl").write_text("".join(json.dumps(row)+"\n" for row in rows))
        results[arm]=dict(optimizer_updates=6000,examples_seen=24000,completed_fixed_endpoint=True,checkpoints=receipts);put(folder/"fit_result.json",results[arm])
    screens={arm:dict(diagnostic_screen_count=71,checks=[dict(name="screen_%d"%i,passed=i%2==0) for i in range(71)]) for arm in results}
    put(run/"summary.json",dict(schema=tool.TRAINING,status="BOTH_FIXED6000_COMPLETE_NOT_PROMOTED",actual_batch_bytes_equal=True,
        all71_retained_per_arm=True,model_promoted=False,vehicle_control_approved=False,arms=results,endpoint_diagnostic_screens=screens))
    put(run/"evaluation_opened_after_both_final.json",dict(arms=results,checkpoint_selection=False))
    for arm in ("parent","control","intrinsic_curve"):
        for cohort in ("recovery_train","recovery_holdout","v4","legacy","known599","intrinsic_legacy_TRAIN_only"):
            put(run/(arm+"_"+cohort+".json"),dict(synthetic=True))
    return base


@pytest.fixture
def archive(original,tmp_path):
    shutil.copytree(original,tmp_path/"pair");return tmp_path/"pair/run",tmp_path/"pair/source"


def inspect(run,source):
    return tool.inspect_pair(run,source,tool.digest(run/"summary.json"),tool.digest(run/"declaration.json"),tool.digest(source/"authorization.json"))


def test_full_pair_preserves_failed_screens_and_hashes_pt_only(archive):
    run,source=archive;value=inspect(run,source)
    assert len(value["sources"])==29 and len(value["endpoints"])==2
    assert sum("checkpoints/step_" in name for name in value["files"])==24
    assert value["files"]["control/checkpoints/step_006000.pt"]["sha256"]==tool.digest(run/"control/checkpoints/step_006000.pt")
    assert not (run/"completed_pair_authority.json").exists()


@pytest.mark.parametrize("change",["unfinished","one_final_missing","one_checkpoint_missing","metric_missing","changed_batch","changed_indices","wrong_coefficient","source_changed","archive_changed","eval_early","missing_screen"])
def test_any_missing_or_mismatched_completion_fails(archive,change):
    run,source=archive
    if change=="unfinished":d=tool.read(run/"summary.json");d["status"]="INCOMPLETE_PAIR_NO_EVALUATION_NOT_PROMOTED";put(run/"summary.json",d)
    elif change=="one_final_missing":(run/"intrinsic_curve/exports/step_006000/model_state.npz").unlink()
    elif change=="one_checkpoint_missing":(run/"control/checkpoints/step_000500.pt").unlink()
    elif change=="metric_missing":(run/"parent_known599.json").unlink()
    elif change in ("changed_batch","changed_indices","wrong_coefficient"):
        path=run/"intrinsic_curve/history.jsonl";rows=path.read_text().splitlines();row=json.loads(rows[99]);key={"changed_batch":"batch_sha256","changed_indices":"indices","wrong_coefficient":"intrinsic_coefficient"}[change]
        row[key]={"changed_batch":"b"*64,"changed_indices":[1,0,2,3],"wrong_coefficient":.05}[change]
        rows[99]=json.dumps(row);path.write_text("\n".join(rows)+"\n")
    elif change=="source_changed":(source/"tools/source_00.py").write_text("changed")
    elif change=="archive_changed":(run/"control/source/tools/source_00.py").write_text("changed")
    elif change=="eval_early":d=tool.read(run/"evaluation_opened_after_both_final.json");d["checkpoint_selection"]=True;put(run/"evaluation_opened_after_both_final.json",d)
    else:d=tool.read(run/"summary.json");d["endpoint_diagnostic_screens"]["control"]["checks"].pop();put(run/"summary.json",d)
    with pytest.raises(Exception):inspect(run,source)


def test_symlink_rejected_before_copy(archive):
    run,source=archive;(run/"linked").symlink_to(source,target_is_directory=True)
    with pytest.raises(ValueError,match="symlink"):inspect(run,source)


def test_first_read_pin_rejects_mid_parse_mutation(archive,monkeypatch):
    run,source=archive;old=tool.read
    def changed(path,expected=None):
        if path==run/"summary.json":put(path,dict(old(path),mutation=True))
        return old(path,expected)
    monkeypatch.setattr(tool,"read",changed)
    with pytest.raises(ValueError,match="between inventory and parsing"):inspect(run,source)


def test_remote_program_is_same_stdlib_algorithm_without_models(archive,monkeypatch):
    run,source=archive;monkeypatch.setattr(tool,"RUN",run);monkeypatch.setattr(tool,"SOURCE",source)
    args=SimpleNamespace(expected_summary_sha256=tool.digest(run/"summary.json"),expected_declaration_sha256=tool.digest(run/"declaration.json"),authorization_sha256=tool.digest(source/"authorization.json"))
    command=tool.remote_command(args);program=shlex.split(command[-1])[-1]
    result=subprocess.run([sys.executable,"-I","-B","-c",program],capture_output=True,check=True)
    assert json.loads(result.stdout)==inspect(run,source)
    assert command[:len(tool.SSH)]==tool.SSH and "torch" not in program and "nvidia-smi" not in program


def test_public_scopes_and_lexical_escape_are_fixed():
    assert str(tool.RUN)=="/home/pro6000/personal/hwanhong/portable_e2e/runs/training/temporal_intrinsic_curve_pair_20261002_v2"
    assert str(tool.SOURCE).endswith("source_worktrees/temporal_intrinsic_curve_pair_20261002_v2")
    assert tool.LOCAL.parent==tool.RECEIPT.parent and tool.LOCAL!=tool.RECEIPT
    with pytest.raises(ValueError):tool.canonical(tool.ROOT/"tools/../storage")


def test_duplicate_and_nonfinite_json_rejected(tmp_path):
    for content in ('{"x":1,"x":2}','{"x":NaN}'):
        path=tmp_path/"bad.json";path.write_text(content)
        with pytest.raises(ValueError):tool.read(path)


def test_mirror_is_create_only_and_cannot_self_authorize(archive,tmp_path,monkeypatch):
    run,source=archive;before=inspect(run,source);local=tmp_path/"mirror";receipt=tmp_path/"receipt"
    monkeypatch.setattr(tool,"ROOT",source);monkeypatch.setattr(tool,"LOCAL",local);monkeypatch.setattr(tool,"RECEIPT",receipt);commands=[]
    def invoked(command,**kwargs):
        commands.append(command)
        if command[0]=="rsync":shutil.copytree(run,local,dirs_exist_ok=True);return SimpleNamespace(returncode=0,stdout=b"",stderr=b"")
        return SimpleNamespace(returncode=0,stdout=json.dumps(before).encode(),stderr=b"")
    monkeypatch.setattr(tool.subprocess,"run",invoked);monkeypatch.setattr(tool.shutil,"disk_usage",lambda path:SimpleNamespace(free=20*1024**3))
    arguments=[value for name,pin in (("expected-summary-sha256",tool.digest(run/"summary.json")),("expected-declaration-sha256",tool.digest(run/"declaration.json")),
        ("authorization-sha256",tool.digest(source/"authorization.json")),("expected-tool-sha256",tool.digest(tool.__file__))) for value in ("--"+name,pin)]
    assert tool.main(arguments)==0;evidence=tool.read(receipt/"completed_pair_for_root_review.json")
    assert evidence["numeric_state_validated"] is False and evidence["simulator_control_authorized"] is False and evidence["model_promoted"] is False
    assert "--ignore-existing" in next(c for c in commands if c[0]=="rsync") and tool.inventory(local)==before["files"]
    with pytest.raises(ValueError,match="fresh destinations"):tool.main(arguments)
