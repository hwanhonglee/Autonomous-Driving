"""HH_261002 - Pure transfer/command fixtures; no SSH, data, model, CUDA, or actual dispatch."""
import ast
import hashlib
import io
import json
from pathlib import Path
import shlex
import tarfile
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import dispatch_temporal_intrinsic_curve_pair_20261002 as dispatch


@pytest.mark.parametrize("validate",[True,False])
def test_exact_private_command_environment_parent_four_assets_and_scoped_mode(validate):
    argv=shlex.split(dispatch.command("a"*64,validate=validate))
    assert argv[:9]==["env","-u","PYTHONPATH","-u","PYTHONHOME","-u","PYTHONUSERBASE","-u","PYTHONSTARTUP"]
    assert "PYTHONNOUSERSITE=1" in argv and "CUBLAS_WORKSPACE_CONFIG=:4096:8" in argv
    visibility=next(v for v in argv if v.startswith("CUDA_VISIBLE_DEVICES="))
    assert visibility=="CUDA_VISIBLE_DEVICES="+("" if validate else dispatch.training.original.GPU_UUID)
    assert str(dispatch.PYTHON) in argv and str(dispatch.SOURCE/"tools/workspace_python.py") in argv
    assert argv[argv.index("--mode")+1]==("validate-only" if validate else "fit")
    assert argv[argv.index("--parent-run")+1]==str(dispatch.PARENT)
    assert argv[argv.index("--parent-history-sha256")+1]==dispatch.PARENT_HISTORY_SHA
    assert argv[argv.index("--warm-parent-weights")+1]==str(dispatch.WARM/"model_state.npz")
    assert argv[argv.index("--backbone-parent-weights")+1]==str(dispatch.BACKBONE/"model_state.npz")
    assert argv[argv.index("--output")+1]==str(dispatch.RUN)
    assert "GPU1" not in " ".join(argv)


def test_tar_bundle_exact28_regular_sources_plus_external_authorization(tmp_path):
    sources=dispatch.training.source_snapshot();auth=tmp_path/"authorization.json";auth.write_text("original root authority")
    payload,pins=dispatch.package(sources,auth,dispatch.training.digest(auth))
    assert len(sources)==28 and len(pins)==29
    with tarfile.open(fileobj=io.BytesIO(payload),mode="r") as archive:
        members=archive.getmembers();assert len(members)==29
        for member in members:
            assert member.isfile() and not Path(member.name).is_absolute() and ".." not in Path(member.name).parts
            assert member.mode==0o600
            assert hashlib.sha256(archive.extractfile(member).read()).hexdigest()==pins[member.name]


def test_packaging_rejects_changed_root_authority(tmp_path):
    auth=tmp_path/"authorization.json";auth.write_text("original")
    with pytest.raises(Exception,match="changed while packaging"):dispatch.package(dispatch.training.source_snapshot(),auth,"0"*64)


def test_remote_create_only_script_fixed_scope_cardinality_and_no_install_or_unsafe_extract():
    code=dispatch.REMOTE_COPY;ast.parse(code)
    assert 'len(expected)==29' in code and '"authorization.json" in expected' in code
    assert 'source_worktrees/temporal_intrinsic_curve_pair_20261002_v1' in code
    assert 'member.isfile()' in code and 'path.open("xb")' in code and 'destination.exists()' in code
    assert '.is_symlink()' in code and 'member.size<=2*1024**2' in code
    assert 'extractall' not in code and 'pip' not in code and 'subprocess' not in code


def test_prepare_mode_cannot_launch_actual_fit_even_when_all_mock_preflights_pass(tmp_path):
    receipt=tmp_path/"storage/experiments/training/2026-10-02/dispatch"
    auth=tmp_path/"auth.json";auth.write_text("root authority");authsha=dispatch.training.digest(auth)
    sources=dispatch.training.source_snapshot();contract=dispatch.training.contract()
    authority=dict(actual_parent_history_sha256=dispatch.PARENT_HISTORY_SHA)
    files=dict(sources,**{"authorization.json":authsha})
    checked=dict(status="VALIDATED_TRAIN_ONLY_CPU_NO_CUDA_NO_OUTPUT",protocol=contract,source_sha256=sources,
        parent_history_sha256=dispatch.PARENT_HISTORY_SHA,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        holdout_payload_loaded=False,model_promoted=False)
    calls=[]
    def execute(argv,**kwargs):
        calls.append(argv)
        if len(calls)==1:return SimpleNamespace(returncode=0,stdout=json.dumps(dict(file_sha256=files)).encode(),stderr=b"actual transfer stderr")
        return SimpleNamespace(returncode=0,stdout=json.dumps(checked).encode(),stderr=b"actual validate stderr")
    args=["--mode","prepare","--authorization",str(auth),"--authorization-sha256",authsha,
        "--expected-runner-sha256",sources["tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py"],
        "--expected-loss-sha256",sources["learning/portable_e2e/intrinsic_spatial_path_loss_v1_20261002.py"],"--receipt",str(receipt)]
    with mock.patch.object(dispatch,"ROOT",tmp_path),mock.patch.object(dispatch.training,"validate_authorization",return_value=authority),\
        mock.patch.object(dispatch.training,"contract",return_value=contract),mock.patch.object(dispatch.training,"source_snapshot",return_value=sources),\
        mock.patch.object(dispatch,"package",return_value=(b"tar",files)),mock.patch.object(dispatch.subprocess,"run",side_effect=execute),\
        mock.patch.object(Path,"read_bytes",side_effect=lambda:b"synthetic dispatch source"),\
        mock.patch.object(dispatch.training,"digest",return_value=authsha):
        assert dispatch.main(args)==0
    assert len(calls)==2 and '--mode validate-only' in calls[1][-1] and '--mode fit' not in calls[1][-1]
    assert (receipt/"transfer_stderr.txt").read_bytes()==b"actual transfer stderr"
    assert (receipt/"remote_validation_stderr.txt").read_bytes()==b"actual validate stderr"
    assert json.loads((receipt/"summary.json").read_text())["optimizer_started"] is False


def test_actual_runner_proposed_fit_is_printed_only_not_a_subprocess_call():
    source=Path(dispatch.__file__).read_text()
    assert source.count("subprocess.run(")==2
    assert 'command(args.authorization_sha256,validate=False)' in source
    assert 'SSH+[command(args.authorization_sha256,validate=False)]' not in source
