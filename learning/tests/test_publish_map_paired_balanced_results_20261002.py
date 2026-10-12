"""HH_261002 - Synthetic archive, whole-exposure, pixel-accounting and create-only map publication tests."""

import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.e2e import publish_map_paired_balanced_results_20261002 as publisher


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False)+"\n")
    return digest(path)


def write_npz(path, **arrays):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream: np.savez_compressed(stream, **arrays)
    return digest(path)


@pytest.fixture(scope="module")
def scheduled():
    ids, crops, bits = publisher.contract.schedule()
    return ids, crops, bits, publisher.contract.schedule_proof(ids, crops, bits)


def info_for(arm, ids, crops, bits):
    actual = bits if arm == "balanced" else np.zeros_like(bits)
    flat, views = ids.ravel(), actual.ravel()
    return {"sample_sequence_sha256": hashlib.sha256(ids.tobytes()).hexdigest(),
            "crop_sequence_sha256": hashlib.sha256(crops.tobytes()).hexdigest(),
            "view_sequence_sha256": hashlib.sha256(actual.tobytes()).hexdigest(),
            "paired_FALSE_exposures_per_pair": np.bincount(flat[(flat>=729)&(views==0)]-729, minlength=480).tolist(),
            "paired_TRUE_exposures_per_pair": np.bincount(flat[(flat>=729)&(views==1)]-729, minlength=480).tolist()}


def exposure_bytes(arm, ids, crops, bits):
    actual = bits if arm == "balanced" else np.zeros_like(bits)
    return b"\n".join(json.dumps({"step": index+1, "physical_slot_indices": row.tolist(),
        "shared_crop_boxes": crops[index].tolist(), "actual_paired_TRUE_bits": actual[index].tolist()}).encode()
        for index, row in enumerate(ids))+b"\n"


@pytest.fixture(scope="module")
def full_archive(tmp_path_factory, scheduled):
    """HH_261002 - A complete synthetic20k-row fixture; never read or mutate actual run/data artifacts."""
    root = tmp_path_factory.mktemp("map_result_synthetic")
    data, run, evaluation = root/"data", root/"run", root/"evaluation"
    data.mkdir(); run.mkdir(); evaluation.mkdir()
    c = publisher.contract
    ids, crops, bits, proof = scheduled
    parent_sha = write_npz(data/"parent_weights.npz", synthetic=np.zeros(245831, np.float32))
    (data/"original_train729.npz").write_bytes(b"synthetic original immutable bytes")
    original_sha = digest(data/"original_train729.npz")
    (data/"paired_train480.npz").write_bytes(b"synthetic paired immutable bytes")
    write_npz(data/"training_schedule.npz", indices=ids, crops=crops, balanced_view_bits=bits)
    files = {name: {"sha256": digest(data/name)} for name in
             ("original_train729.npz", "paired_train480.npz", "training_schedule.npz", "parent_weights.npz")}
    common = dict(study_id=c.NAME, counts=copy.deepcopy(c.COUNTS), arms=copy.deepcopy(c.ARMS),
                  route_sha256=dict(c.ROUTES), parent_weights_sha256=parent_sha,
                  native_diagnostic_denials_preserved=True, ctrack_train_eligible=False,
                  dev_or_test_used_for_fit=False, vehicle_control_approved=False, independent_new_dev_sites=0)
    captures = {name: {"source_capture": "/synthetic/native/"+name} for name in c.ROUTES}
    review = dict(schema=c.REVIEW_SCHEMA, **common, research_fit_authorized=True,
                  render_manifest_sha256="a"*64, training_plan_sha256="b"*64,
                  optimization=c.optimization(), reviewer="synthetic source-contract test",
                  review_note="Synthetic-only review used for contract tests, not actual admission.",
                  source_reviews={name: dict(source_capture=ref["source_capture"], qa_sha256=c.QA_SHA[name],
                      route_sha256=c.ROUTES[name], reviewed_indices=[0,119,239], rgb_conditions=["false","true"],
                      semantic_target_overlay_reviewed=True, all_240_retained=True,
                      review_note="Synthetic-only full source review fixture, not actual approval.")
                      for name, ref in captures.items()})
    review_sha = write_json(data/"provenance/manual_review_and_fit_authorization.json", review)
    for name in c.ROUTES:
        for filename in ("raw_qa.json", "owner_result.json", "summary.json", "owner_plan.json"):
            write_json(data/"provenance"/name/filename, {"dataset_admitted": False, "train_eligible": False})
    provenance = {str(path.relative_to(data)): digest(path) for path in (data/"provenance").rglob("*") if path.is_file()}
    registry = dict(schema=c.REGISTRY_SCHEMA, **common, research_fit_authorized=True, optimization=c.optimization(),
                    data_files=files, common_schedule=proof, broad_registry_modified=False,
                    source_captures=captures, copied_provenance_files=provenance,
                    authorization_source_sha256=review_sha, render_manifest_sha256="a"*64,
                    training_plan_sha256="b"*64)
    paired_ids = [f"{name}:frames/{index:04d}" for name in c.ROUTES for index in range(240)]
    paired_sha = write_json(data/"paired_samples.json", [{"sample_id": value} for value in paired_ids])
    registry_sha = write_json(data/"study_train_registry.json", registry)
    manifest = dict(schema=c.DATA_SCHEMA, **common, labels=list(c.LABELS), files=files,
                    paired_sample_ids=paired_ids, paired_samples_sha256=paired_sha,
                    common_class_weights=[1.]*7, schedule_contract=proof, study_registry_sha256=registry_sha,
                    raw_model_inputs=["RGB"], depth_or_pose_model_input=False, source_receipts_modified=False)
    data_sha = write_json(data/"manifest.json", manifest)
    workspace = Path(publisher.__file__).resolve().parents[2]
    sources = {}
    for name in c.SOURCE_FILES:
        if name == "plan.md": payload = b"synthetic prospective plan\n"
        else:
            source = workspace/"tools/e2e"/name
            if not source.is_file(): source = workspace/"learning/portable_e2e"/name
            payload = source.read_bytes()
        path = data/"source"/name; path.parent.mkdir(exist_ok=True); path.write_bytes(payload)
        sources[name] = digest(path)
    # HH_261002 - The synthetic plan has its own exact pin; every imported frozen source remains real unchanged bytes.
    registry["training_plan_sha256"] = sources["plan.md"]
    review["training_plan_sha256"] = sources["plan.md"]
    registry["authorization_source_sha256"] = write_json(data/"provenance/manual_review_and_fit_authorization.json", review)
    registry["copied_provenance_files"]["provenance/manual_review_and_fit_authorization.json"] = registry["authorization_source_sha256"]
    registry_sha = write_json(data/"study_train_registry.json", registry)
    manifest["study_registry_sha256"] = registry_sha; data_sha = write_json(data/"manifest.json", manifest)
    bundle = dict(schema=c.SOURCE_SCHEMA, study_id=c.NAME, files=sources,
                  data_manifest_sha256=data_sha, study_registry_sha256=registry_sha,
                  training_plan_sha256=sources["plan.md"])
    source_sha = write_json(data/"source/bundle.json", bundle)
    for name in (*sources, "bundle.json"):
        path = run/"source"/name; path.parent.mkdir(exist_ok=True); path.write_bytes((data/"source"/name).read_bytes())
    for name in ("manifest.json", "study_train_registry.json", "paired_samples.json"):
        path = run/"inputs"/name; path.parent.mkdir(exist_ok=True); path.write_bytes((data/name).read_bytes())
    arms = {}; artifact_files = {}
    for arm in c.ARMS:
        folder = run/arm; folder.mkdir()
        folder.joinpath("weights.npz").write_bytes((data/"parent_weights.npz").read_bytes())
        folder.joinpath("checkpoint.pt").write_bytes(b"opaque synthetic checkpoint: never unpickle")
        exposures = exposure_bytes(arm, ids, crops, bits); folder.joinpath("exposures.jsonl").write_bytes(exposures)
        folder.joinpath("history.jsonl").write_text("".join(json.dumps(dict(arm=arm,step=i*100,mean_loss_100=1.,wall_seconds=float(i)))+"\n" for i in range(1,101)))
        info = dict(steps=10000, examples_seen=80000, parameter_count=245831,
                    optimizer_initial_state_entries=0, training_wall_seconds=101., initial_state_sha256="c"*64,
                    weights_sha256=digest(folder/"weights.npz"), checkpoint_sha256=digest(folder/"checkpoint.pt"),
                    full_exposure_journal_sha256=digest(folder/"exposures.jsonl"), **info_for(arm,ids,crops,bits))
        write_json(folder/"training.json", info); arms[arm] = info
        for name in ("weights.npz","checkpoint.pt"): artifact_files[arm+"/"+name] = digest(folder/name)
    splits = {}; evaluation_results = {}; eval_refs = {}
    target = np.broadcast_to((np.indices((360,640))[1] % 7).astype(np.uint8), (3,360,640)).copy()
    rgb = np.full((3,360,640,3), 38, np.uint8)
    for split in publisher.SPLITS:
        sha = write_npz(evaluation/(split+".npz"), rgb=rgb, target=target)
        splits[split] = (3, sha); eval_refs[split] = dict(file=split+".npz", count=3, sha256=sha)
        evaluation_results[split] = {}
        for arm in c.ARMS:
            prediction = target if arm == "control" else ((target.astype(np.int16)+1)%7).astype(np.uint8)
            name = arm+"/"+split+"_predictions.npz"
            artifact_files[name] = write_npz(run/name, prediction=prediction)
            measured = publisher.metrics(target, prediction)
            measured.pop("false_negative_pixels"); measured.pop("pixel_count")
            evaluation_results[split][arm] = dict(count=3, source_sha256=sha,
                known_exposed_diagnostic_not_independent=True, **measured)
    eval_manifest_sha = write_json(evaluation/"manifest.json", dict(files=eval_refs))
    artifact_files["known_evaluation.json"] = write_json(run/"known_evaluation.json", evaluation_results)
    summary = dict(status="MATCHED_BALANCED_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED", **common,
                   optimization=c.optimization(), data_manifest_sha256=data_sha,
                   study_registry_sha256=registry_sha, source_bundle_sha256=source_sha,
                   schedule_contract=proof, common_class_weights=[1.]*7, arms_result=arms,
                   known_evaluation=evaluation_results, model_promoted=False,
                   dev_used_during_optimization=False, source_and_data_unchanged=True)
    summary_sha = write_json(run/"summary.json", summary)
    write_json(run/"fit_complete.json", dict(status="BOTH_FIXED_FINAL10000_ENDPOINTS_SAVED_BEFORE_DEV",arms=arms))
    artifact_path = root/"external_artifacts.json"
    artifact_sha = write_json(artifact_path, dict(schema=publisher.ARTIFACT_SCHEMA, study_id=c.NAME,
        remote_run_root=publisher.REMOTE_RUN, remote_originals_independently_hashed=True,
        summary_sha256=summary_sha, data_manifest_sha256=data_sha, study_registry_sha256=registry_sha,
        source_bundle_sha256=source_sha, files=artifact_files))
    args = SimpleNamespace(run_root=run,data_root=data,eval_root=evaluation,output=root/"docs/fresh_results",
        caller_summary_sha256=summary_sha, expected_data_sha256=data_sha, expected_registry_sha256=registry_sha,
        expected_source_sha256=source_sha, artifact_pins_json=artifact_path, artifact_pins_sha256=artifact_sha,
        validate_only=True)
    return dict(root=root,args=args,parent_sha=parent_sha,original_sha=original_sha,
                splits=splits,eval_manifest_sha=eval_manifest_sha,summary=summary)


@pytest.fixture
def actual_contract_fixture(full_archive, monkeypatch):
    value = full_archive
    monkeypatch.setattr(publisher,"ROOT",value["root"])
    monkeypatch.setattr(publisher,"OUTPUT",value["args"].output)
    monkeypatch.setattr(publisher,"SPLITS",value["splits"])
    monkeypatch.setattr(publisher,"EVAL_MANIFEST_SHA",value["eval_manifest_sha"])
    monkeypatch.setattr(publisher.contract,"PARENT_SHA",value["parent_sha"])
    monkeypatch.setattr(publisher.contract,"ORIGINAL_SHA",value["original_sha"])
    return value


def test_full_synthetic_completed_archive_keeps_worse_classes_and_all20k_rows(actual_contract_fixture):
    args = actual_contract_fixture["args"]
    report, panels, pins = publisher.audit(args)
    assert sum(row["exposures"]["rows"] for row in report["arms"].values()) == 20000
    assert all(row["exposures"]["examples_seen"] == 80000 for row in report["arms"].values())
    assert report["evaluation"]["dev_town01"]["balanced"]["iou"] == dict.fromkeys(publisher.contract.LABELS,0.)
    assert len(panels) == 6
    assert report["formal_main_functions_approved"] == report["formal_sub_functions_approved"] == 0
    assert report["counts"]["physical_pairs"] == 480 and report["independent_new_dev_sites"] == 0
    assert report["Torch_imported"] is report["model_inference_executed"] is report["pickle_loaded"] is False
    pins.recheck()


def test_validate_only_writes_nothing(actual_contract_fixture):
    args = actual_contract_fixture["args"]
    result = publisher.publish(args)
    assert result["published"] is False and not args.output.exists()


def test_absent_target_class_keeps_null_and_false_positive_count():
    target = np.zeros((1,2,3),np.uint8); prediction = np.ones_like(target)
    result = publisher.metrics(target,prediction)
    assert result["iou"]["road"] is None and result["false_positive_pixels"]["road"] == 6
    assert result["false_negative_pixels"]["other"] == 6
    assert len(result["confusion"]) == 7 and sum(map(sum,result["confusion"])) == 6


@pytest.mark.parametrize("value",[True,1,[],"a"*63,"G"*64])
def test_external_sha_rejects_wrong_types_and_noncanonical_hex(value):
    with pytest.raises(ValueError): publisher.sha_string(value)


def test_duplicate_and_nonfinite_json_rejected():
    for value in (b'{"x":1,"x":2}',b'{"x":NaN}'):
        with pytest.raises(ValueError): publisher.strict_json(value)


@pytest.mark.parametrize("change",["missing_prediction","extra_escape","remote_not_hashed","wrong_summary"])
def test_external_remote_artifact_receipt_is_mandatory_and_bounded(actual_contract_fixture,change):
    args = actual_contract_fixture["args"]
    value = json.loads(args.artifact_pins_json.read_bytes())
    if change=="missing_prediction": value["files"].pop("control/dev_town01_predictions.npz")
    if change=="extra_escape": value["files"]["../other.npz"]="a"*64
    if change=="remote_not_hashed": value["remote_originals_independently_hashed"]=False
    if change=="wrong_summary": value["summary_sha256"]="a"*64
    with pytest.raises(ValueError):publisher.artifact_receipt(value,args.caller_summary_sha256,args.expected_data_sha256,args.expected_registry_sha256,args.expected_source_sha256)


@pytest.mark.parametrize("change",["missing_last","duplicate_step","one_crop","one_index","one_bit","bool_index"])
def test_all_actual_exposure_rows_and_order_are_required(scheduled,change):
    ids,crops,bits,_=scheduled
    rows=exposure_bytes("balanced",ids,crops,bits).splitlines()
    if change=="missing_last": rows.pop()
    else:
        row=json.loads(rows[0])
        if change=="duplicate_step":row["step"]=2
        if change=="one_crop":row["shared_crop_boxes"][0][0]+=1
        if change=="one_index":row["physical_slot_indices"][0]=(row["physical_slot_indices"][0]+1)%1209
        if change=="one_bit":row["actual_paired_TRUE_bits"][0]^=1
        if change=="bool_index":row["physical_slot_indices"][0]=True
        rows[0]=json.dumps(row).encode()
    with pytest.raises(ValueError):publisher.exposure_proof(b"\n".join(rows),info_for("balanced",ids,crops,bits),"balanced",ids,crops,bits)


def test_control_uses_only_off_even_where_balanced_on(scheduled):
    ids,crops,bits,_=scheduled
    result=publisher.exposure_proof(exposure_bytes("control",ids,crops,bits),info_for("control",ids,crops,bits),"control",ids,crops,bits)
    assert result["paired_TRUE_exposures"] == 0 and result["paired_FALSE_exposures"] > 0


def test_native2x2_canvas_preserves_rgb_and_all_prediction_pixels():
    rgb=np.full((360,640,3),43,np.uint8)
    target=(np.indices((360,640))[1]%7).astype(np.uint8)
    a=(target+1)%7;b=(target+2)%7
    image=np.asarray(publisher.canvas("dev_town01",0,rgb,target,a,b,{"control":"a"*64,"balanced":"b"*64}))
    assert image.shape==(850,1280,3)
    np.testing.assert_array_equal(image[56:416,:640],rgb)
    np.testing.assert_array_equal(image[56:416,640:],publisher.COLORS[target])
    np.testing.assert_array_equal(image[448:808,:640],publisher.COLORS[a])
    np.testing.assert_array_equal(image[448:808,640:],publisher.COLORS[b])


@pytest.mark.parametrize("class_id",[7,255])
def test_invalid_prediction_class_cannot_be_filtered_away(class_id):
    target=np.zeros((1,2,3),np.uint8)
    with pytest.raises(ValueError):publisher.metrics(target,np.full_like(target,class_id))


@pytest.mark.parametrize("failure",["shape","dtype","nonfinite","keys","parameter_count"])
def test_no_torch_numeric_model_abi_check(failure):
    parent={"x":np.zeros(245831,np.float32)};model={"x":parent["x"].copy()}
    if failure=="shape":model["x"]=np.zeros(2,np.float32)
    if failure=="dtype":model["x"]=model["x"].astype(np.float64)
    if failure=="nonfinite":model["x"][0]=np.nan
    if failure=="keys":model={"y":model["x"]}
    if failure=="parameter_count":parent={"x":np.zeros(2,np.float32)};model={"x":parent["x"].copy()}
    with pytest.raises(ValueError):publisher.numeric_weights(model,parent)


def test_output_existing_rejected_before_any_archive_read(tmp_path,monkeypatch):
    output=tmp_path/"existing";output.mkdir()
    monkeypatch.setattr(publisher,"OUTPUT",output)
    monkeypatch.setattr(publisher,"audit",lambda _:pytest.fail("no input read"))
    with pytest.raises(ValueError):publisher.publish(SimpleNamespace(output=output))


def test_output_cannot_be_inside_immutable_run(tmp_path,monkeypatch):
    output=tmp_path/"run/fresh";monkeypatch.setattr(publisher,"OUTPUT",output)
    monkeypatch.setattr(publisher,"audit",lambda _:pytest.fail("no input read"))
    with pytest.raises(ValueError,match="outside immutable"):
        publisher.publish(SimpleNamespace(output=output,run_root=tmp_path/"run",data_root=tmp_path/"data",eval_root=tmp_path/"eval"))


def test_symlink_ancestor_rejected_for_input_and_fresh_output(tmp_path):
    folder=tmp_path/"real";folder.mkdir();link=tmp_path/"link";link.symlink_to(folder,target_is_directory=True)
    with pytest.raises(ValueError):publisher.Pins().payload(link/"missing")
    with pytest.raises(ValueError):publisher.contract.canonical(link/"fresh")


def test_pinned_json_is_hashed_from_exact_parsed_bytes(tmp_path):
    path=tmp_path/"data.json";path.write_bytes(b'{"x":1}')
    pins=publisher.Pins(); assert pins.json(path,digest(path))=={"x":1}
    path.write_bytes(b'{"x":2}')
    with pytest.raises(ValueError,match="first-read"):pins.recheck()


def test_publisher_never_imports_torch_or_opens_pickle_source_ast():
    import ast
    tree=ast.parse(Path(publisher.__file__).read_bytes())
    imports=[node for node in ast.walk(tree) if isinstance(node,(ast.Import,ast.ImportFrom))]
    names=[alias.name for node in imports for alias in node.names]
    assert not any(name in ("torch","pickle") or name.startswith("torch.") for name in names)
    assert "torch.load" not in Path(publisher.__file__).read_text()


def test_plain_korean_report_does_not_hide_worse_classes(actual_contract_fixture):
    report,_,_=publisher.audit(actual_contract_fixture["args"])
    text=publisher.markdown(report,{"fixed.png":"a"*64})
    assert "메인 0/9" in text and "서브 0/30" in text and "960개 독립 장소" in text
    assert "미측정(null)" in text and all(label in text for label in publisher.contract.LABELS)
    assert "0.00%" in text and "정밀지도" in text
