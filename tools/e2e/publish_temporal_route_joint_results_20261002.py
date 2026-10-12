"""HH_261002 - Publish only the fixed6000 curvature/speed joint endpoint, with all71 gates and denied release.

Import is standard-library only. Actual publication waits for caller-pinned final
assets; intermediate checkpoints are retained and numerically checked, never
chosen as the result. NumPy is lazy and Torch pickle is never deserialized.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil

from scripts.e2e import publish_temporal_route_recovery_moving_v4_results_20261001 as previous

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.temporal_route_joint_results.20261002.v1"
NUMERIC_SCHEMA = "portable_e2e.temporal_route_joint_numeric_receipt.20261002.v1"
RUN_SCHEMA = "portable_e2e.temporal_route_joint_training.20261002.v1"
EXPORT_SCHEMA = "portable_e2e.temporal_route_joint_export_research.20261002.v1"
ARCHITECTURE_SCHEMA, MODEL_ID = previous.ARCHITECTURE_SCHEMA, previous.MODEL_ID
PARENT_CHECKPOINT_SHA = "a17544d99fd87467e4afe1c051dd598f72ab9cd0d79794f34d10291e1edce52f"
PARENT_NPZ_SHA = "0bdd747dffa6e37196e0ee6afb25fa9930d6aba6ca6f787c779ccfbeae3d4dc9"
PARENT_METADATA_SHA = "4ace621d8cf1b8d3ce269bb77aabe9e89598f3f750e18ea707a0817086b3dc2d"
PARENT_DECLARATION_SHA = "34c8a55028ea7b4cb2678a73701b16425d811fb10dd77afb5554183dd79ef6c5"
GRANDPARENT_NPZ_SHA, GRANDPARENT_METADATA_SHA = previous.PARENT_NPZ_SHA, previous.PARENT_METADATA_SHA
HELPER_FILE = "tools/e2e/publish_temporal_route_recovery_moving_v4_results_20261001.py"
HELPER_SHA = "41ddf6e0f9620ec8b4e7ba1f0b321e07fbbc0634e3d9d752a2de1f8b341ca542"
TRAINER_FILE = "tools/e2e/train_temporal_route_joint_20261002.py"
TRAINER_SHA = "971eb207d79fe6ed7e015da50a5351e9e30e13ce9631f2cf39267214379ae673"
LOADER_FILE = "learning/portable_e2e/temporal_route_joint_export_research_20261002.py"
LOADER_SHA = "3245e5ee72b6a2bca030249b3075704eb579a0331d0b0c4d6292068e7c192bab"
SOURCE_NAMES = previous.SOURCE_NAMES | {TRAINER_FILE, LOADER_FILE}
FROZEN_SOURCES = dict(previous.FROZEN_SOURCES, **{TRAINER_FILE: TRAINER_SHA, LOADER_FILE: LOADER_SHA})
TRAINABLE_PREFIXES = ("fusion.", "output_norm.", "trunk.", "curvature_head.", "speed_head.", "route_recovery_branch.")
BATCH_DOMAINS = ["recovery_noncentered_joint", "recovery_all_joint", "v4_joint", "legacy_raw_curvature_and_speed"]
RATE = dict(schedule="fixed6000_linear_warmup_cosine_two_groups", warmup_updates=300,
    shared_peak=1e-5, shared_final=1e-6, heads_peak=3e-5, heads_final=3e-6,
    gradient_clip_l2=5., weight_decay=1e-4, units="AdamW optimizer learning rate")
COHORTS, JOINT_COMPONENTS = previous.COHORTS, previous.JOINT_COMPONENTS
DOCUMENTS = tuple(name for name in previous.DOCUMENTS if name != "exports/step_003000/metadata.json") + ("exports/step_006000/metadata.json",)
require, is_sha, number, close, mean = previous.require, previous.is_sha, previous.number, previous.close, previous.mean
canonical_path, pin_payload, pin_json, recheck_pins = previous.canonical_path, previous.pin_payload, previous.pin_json, previous.recheck_pins
digest, write_new = previous.digest, previous.write_new


def trainable_names(shapes): return sorted(name for name in shapes if name.startswith(TRAINABLE_PREFIXES))


def learning_rate(step, group):
    require(type(step) is int and 1 <= step <= 6000 and group in ("shared", "heads"), "fixed6000 LR group required")
    peak, final = RATE[group+"_peak"], RATE[group+"_final"]
    if step <= 300: return peak*(step/300)
    return final + .5*(peak-final)*(1+math.cos(math.pi*(step-300)/5700))


def verify_scope(obj):
    flags = dict(training_only=True, simulation_only=True, main_training=False, model_promoted=False,
        vehicle_control_approved=False, inference_loader_authorized=False, stop_intent_trained=False,
        backbone_frozen=False, visual_encoder_frozen=True, semantic_head_parameters_frozen=True,
        semantic_outputs_preserved=False, labels_clipped=False, labels_extrapolated=False,
        legacy_decoded=False, independent_validation=False, checkpoint_selection_by_holdout=False,
        startup_recovery_fitted=False, new_signed_recovery_cohort_moving_only=True,
        parent_all_diagnostic_screens_passed=False)
    require(all(obj.get(k) is v for k,v in flags.items()) and obj.get("runtime_intent") == "explicit_cruise_GO_not_head",
        "honest partial-backbone joint scope required; no frozen97, learnedSTOP or release claim")
    counts = dict(training_episode_count=18, recovery_training_episode_count=9, replay_training_episode_count=9,
        holdout_episode_count=2, official_main_features_approved=0, official_subfeatures_approved=0,
        parent_diagnostic_pass_count=20, parent_diagnostic_screen_count=48)
    require(all(type(obj.get(k)) is int and obj[k] == v for k,v in counts.items()), "actual18TRAIN/2HOLDOUT and0/9,0/30 scope required")
    require(obj.get("fixed_recovery_driving_relative_window_seconds") == [4.,34.1]
        and all(type(v) is float for v in obj["fixed_recovery_driving_relative_window_seconds"])
        and obj.get("fixed_recovery_native_frame_offsets") == [80,682]
        and all(type(v) is int for v in obj["fixed_recovery_native_frame_offsets"]), "actual V4 rolling membership unchanged")


def verify_rate(rate):
    require(isinstance(rate,dict) and set(rate)==set(RATE)
        and all(type(rate[k]) is type(v) and rate[k]==v for k,v in RATE.items()), "fixed6000 two-group schedule required")


def endpoint_checks(before, after):
    # HH_261002 - Retain all48 original criteria and recompute the NEW11 speed gains/12 raw speed preservation gates.
    checks = previous.endpoint_checks(before,after)
    def add(name,value,threshold):
        value,threshold=number(value),number(threshold)
        checks.append(dict(name=name,actual=value,threshold=threshold,passed=value<=threshold))
    for cohort in ("recovery_train","recovery_holdout"):
        for episode,row in after[cohort]["per_episode"].items():
            add(cohort+"/"+episode+"/speed_MAE20percent_improvement",row["instantaneous_speed_mae_mps"],
                before[cohort]["per_episode"][episode]["instantaneous_speed_mae_mps"]*.8)
    for cohort in ("legacy","known599"):
        for label in ("aggregate",*sorted(after[cohort]["per_episode"])):
            old=before[cohort]["groups"]["all_eligible"] if label=="aggregate" else before[cohort]["per_episode"][label]
            new=after[cohort]["groups"]["all_eligible"] if label=="aggregate" else after[cohort]["per_episode"][label]
            add(cohort+"/"+label+"/raw_speed_MAE5percent_preservation",new["raw_spatial_preferred_speed_mae_mps"],
                old["raw_spatial_preferred_speed_mae_mps"]*1.05+1e-4)
    require(len(checks)==len({r["name"] for r in checks})==71,"all71 predeclared criteria required")
    return checks


def verify_lineage(declaration,summary,metadata,declaration_sha):
    require(declaration.get("schema")==summary.get("schema")==RUN_SCHEMA
        and metadata.get("schema")==EXPORT_SCHEMA and metadata.get("training_schema")==RUN_SCHEMA
        and metadata.get("architecture_schema")==ARCHITECTURE_SCHEMA and metadata.get("model_id")==MODEL_ID,
        "NEW joint-training/export with unchanged101 architecture required")
    for obj in (declaration,summary,metadata): verify_scope(obj)
    for obj in (declaration,metadata):
        verify_rate(obj.get("learning_rate"))
        require(obj.get("parent_checkpoint_sha256")==PARENT_CHECKPOINT_SHA and obj.get("parent_npz_sha256")==PARENT_NPZ_SHA
            and obj.get("parent_metadata_sha256")==PARENT_METADATA_SHA and obj.get("parent_declaration_sha256")==PARENT_DECLARATION_SHA
            and obj.get("batch_domains")==BATCH_DOMAINS,"exact actual V3 warmstart and declared joint domains required")
    counts=dict(optimizer_updates=6000,examples_seen=24000,recovery_examples_seen=12000,v4_examples_seen=6000,legacy_examples_seen=6000)
    require(all(type(obj.get(k)) is int and obj[k]==v for obj in (summary,metadata) for k,v in counts.items()),
        "all6000 actual updates and24000 exposures required")
    require(summary.get("status")=="FIXED6000_JOINT_COMPLETE_NOT_PROMOTED" and summary.get("wall_budget_stop") is False
        and summary.get("frozen47_tensors_byte_identical") is True and summary.get("full97_frozen_claimed") is False
        and summary.get("source_hashes_match") is True,"completed joint fit must honestly preserve only47 frozen tensors")
    meta_counts=dict(additional_optimizer_updates=6000,effective_batch_size=4,parent_optimizer_updates=3000,
        parent_known_total_optimizer_updates=6200,known_total_optimizer_updates=12200,maximum_optimizer_updates=6000)
    require(all(type(metadata.get(k)) is int and metadata[k]==v for k,v in meta_counts.items())
        and declaration.get("requested_optimizer_updates")==declaration.get("maximum_optimizer_updates")==6000
        and type(declaration.get("effective_batch_size")) is int and declaration["effective_batch_size"]==4
        and type(declaration.get("seed")) is int and declaration["seed"]==2026100201
        and declaration.get("holdout_fitted") is False and declaration.get("holdout_selected_checkpoint") is False
        and declaration.get("fair_three_seed_comparison_performed") is False
        and metadata.get("declaration_sha256")==declaration_sha,"fixed6000/fresh optimizer/no holdout selection lineage required")
    sources=declaration.get("source_sha256")
    require(isinstance(sources,dict) and set(sources)==SOURCE_NAMES and len(sources)==26
        and all(is_sha(v) for v in sources.values()) and all(sources[k]==v for k,v in FROZEN_SOURCES.items())
        and metadata.get("source_sha256")==sources,"all26 reviewed imported sources required")
    shapes=metadata["state_tensor_shapes"]; names=trainable_names(shapes); frozen=sorted(set(shapes)-set(names))
    require(len(shapes)==101 and len(names)==54 and len(frozen)==47
        and sum(math.prod(shapes[k]) for k in names)==2277827
        and metadata.get("trainable_state_names")==declaration.get("trainable_state_names")==names
        and metadata.get("frozen_state_names")==declaration.get("frozen_state_names")==frozen,
        "complete101 tensor ABI/54 learned/47 frozen partition required")
    for obj in (declaration,metadata):
        require(obj.get("trainable_state_tensor_count")==54 and obj.get("frozen_state_tensor_count")==47
            and obj.get("trainable_parameter_count")==2277827,"exact learned parameter counts required")
    require(metadata.get("model_config")==declaration.get("model_config")
        and metadata.get("recovery_config")==declaration.get("recovery_config"),"same causal101 configuration required")
    for field in ("training_dataset_fingerprint_sha256","holdout_dataset_fingerprint_sha256","frozen_state_sha256"):
        require(is_sha(metadata.get(field)) and metadata[field]==declaration.get(field),"exact declared cohort/frozen-state pins required")
    cohorts=declaration.get("cohorts",{})
    require(set(cohorts)=={"recovery_train","recovery_holdout","replay"}
        and cohorts["recovery_train"]["split"]=="train" and cohorts["recovery_train"]["sample_count"]==2718
        and cohorts["recovery_holdout"]["split"]=="val" and cohorts["recovery_holdout"]["sample_count"]==604
        and cohorts["replay"]["v4"]["statistics"]["anchor_count"]==327
        and cohorts["replay"]["legacy"]["sample_count"]==2787 and cohorts["replay"]["legacy"]["split"]=="train"
        and cohorts["replay"]["known_dev"]["sample_count"]==665 and cohorts["replay"]["known_dev"]["split"]=="val",
        "whole actual new/replay/known cohort denominators required")
    require(declaration.get("actual_dataset_available_train_anchor_count")==5832
        and declaration.get("actual_eligible_train_pool_anchor_count")==5532
        and declaration.get("maximum_fit_including_baseline_seconds")==5400
        and declaration.get("postfit_checks_separately_timed") is True,"actual membership and90min fit clock declaration required")
    checkpoints=summary["checkpoints"]
    require([r["optimizer_updates"] for r in checkpoints]==list(range(500,6001,500))
        and all(r.get("optimizer_state_retained") is True and all(is_sha(r.get(k)) for k in
            ("sha256","export_npz_sha256","export_metadata_sha256")) for r in checkpoints),"all12 fixed checkpoint receipts required")
    final=checkpoints[-1]
    require(final["sha256"]==metadata["checkpoint_sha256"] and final["export_npz_sha256"]==metadata["npz_sha256"],
        "actual final6000 export/checkpoint linkage differs")
    return final


def verify_history(history,declaration,before):
    require(len(history)==6000,"all6000 actual optimizer rows required")
    recovery,low,legacy=({r["index"]:r for r in before[k]["rows"]} for k in ("recovery_train","v4","legacy"))
    forbidden={r["token"] for k in ("recovery_holdout","known599") for r in before[k]["rows"]}
    exposure={k:Counter() for k in ("recovery","v4","legacy")}; full=Counter(); geometry=Counter()
    maps={k:Counter() for k in ("curved","straight")}; schedule=[]; same=0
    for step,row in enumerate(history,1):
        for field,value in dict(optimizer_updates=step,examples_seen=4*step,recovery_examples_seen=2*step,v4_examples_seen=step,legacy_examples_seen=step).items():
            require(type(row.get(field)) is int and row[field]==value,"actual journal update/domain counters differ")
        indices,tokens=row["indices"],row["sample_ids"]
        require(len(indices)==len(tokens)==4 and all(type(i) is int for i in indices)
            and all(i in recovery for i in indices[:2]) and indices[2]-2718 in low and indices[3]-3045 in legacy,
            "effective B4 escaped actual eligible TRAIN domain")
        wanted=[recovery[i]["token"] for i in indices[:2]]+[low[indices[2]-2718]["token"],legacy[indices[3]-3045]["token"]]
        require(tokens==wanted and not set(tokens)&forbidden,"actual token foreign/VAL/HOLDOUT/TEST or repeated-view leakage")
        exposure["recovery"].update(tokens[:2]);exposure["v4"].update([tokens[2]]);exposure["legacy"].update([tokens[3]])
        full.update([recovery[indices[1]]["episode_id"]]); old=legacy[indices[3]-3045]
        kind="curved" if step%2 else "straight";require(old["stratum"]==kind,"legacy replay must alternate true curvature strata")
        geometry.update([kind]);maps[kind].update([old["map"]]);same+=indices[0]==indices[1]
        require(set(row["per_domain_trainable_gradients"])==set(BATCH_DOMAINS),"all4 independent gradient diagnostics required")
        for value in row["per_domain_trainable_gradients"].values():
            require(value.get("all_present_branch_gradients_finite") is True
                and type(value.get("unused_branch_tensors")) is int and 0<=value["unused_branch_tensors"]<54,
                "missing/nonfinite entire trainable-domain gradient")
            number(value["gradient_l2_norm"])
        old_terms=row["legacy"]
        close(old_terms["total"],.25*number(old_terms["raw_curvature"])+number(old_terms["raw_preferred_speed"]))
        require(all(old_terms[k]==0 for k in ("intent","stop_distance","initial_acceleration","temporal_xy","temporal_speed")),
            "high-speed legacy cannot enter low-speed decoding or invented semantic tasks")
        expected=.25*(number(row["recovery0"]["total"])+number(row["recovery1"]["total"])+number(row["v4"]["total"])
            +.25*number(old_terms["raw_curvature"])+number(old_terms["raw_preferred_speed"]))
        close(row["loss"],expected)
        require(set(row["learning_rates"])=={"shared","heads"}
            and all(math.isclose(number(row["learning_rates"][g]),learning_rate(step,g),rel_tol=1e-12,abs_tol=1e-15) for g in ("shared","heads")),
            "actual fixed6000 two-group schedule differs")
        number(row["preclip_trainable_gradient_norm"]);number(row["elapsed_owned_seconds"]);schedule.append(indices)
    schedule_sha=hashlib.sha256(json.dumps(schedule,separators=(",",":")).encode()).hexdigest()
    require(schedule_sha==declaration["fixed_schedule_sha256"],"recorded schedule differs from pre-fit declaration")
    require(all(len(exposure[k])==declaration["declared_unique_source_exposure_counts"][k] for k in exposure)
        and {k:len(v) for k,v in exposure.items()}==dict(recovery=2718,v4=327,legacy=2487),"all eligible TRAIN anchors/tails must remain represented")
    require(len(full)==9 and sorted(full.values())==[666]*3+[667]*6
        and geometry==Counter(curved=3000,straight=3000)
        and sorted(maps["curved"].values())==[1500,1500]
        and sorted(maps["straight"].values())==[1000,1000,1000],"all9 profiles and balanced legacy map/curve exposure required")
    require(len(declaration["actual_profile_full302_counts"])==9 and set(declaration["actual_profile_full302_counts"].values())=={302}
        and declaration["actual_noncentered_grid_counts"] and all(type(v) is int and v>0 for v in declaration["actual_noncentered_grid_counts"].values()),
        "genuine observed noncentered strata/full302 membership required")
    return dict(actual_update_rows=6000,actual_anchor_exposures=24000,recovery_exposures=12000,v4_exposures=6000,legacy_exposures=6000,
        actual_unique_source_anchors={k:len(v) for k,v in exposure.items()},actual_full_cycle_per_episode=dict(full),
        actual_legacy_map_exposures={k:dict(v) for k,v in maps.items()},legacy_curved_exposures=3000,legacy_straight_exposures=3000,
        recorded_schedule_sha256=schedule_sha,focus_full_same_anchor_batches=same,HOLDOUT_known_VAL_or_TEST_fitted=False)


def build_report(documents,history,declaration_sha):
    declaration,summary,metadata=(documents[k] for k in ("declaration.json","summary.json","exports/step_006000/metadata.json"))
    final=verify_lineage(declaration,summary,metadata,declaration_sha);started=documents["started.json"];gpu=started["gpu"]
    require(type(gpu.get("physical_index")) is int and gpu["physical_index"]==0
        and gpu.get("uuid")=="GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5" and gpu.get("idle_before_torch") is True
        and started.get("actual_parent_all101_loaded") is True and started.get("trainable_state_tensor_count")==54
        and started.get("frozen_state_tensor_count")==47 and started.get("trainable_parameter_count")==2277827
        and started.get("frozen_state_sha256")==metadata["frozen_state_sha256"],"actual owned GPU0/all101 warmstart/54learned receipt differs")
    before,after=({k:documents[k+"_"+stage+".json"] for k in COHORTS} for stage in ("before","final"))
    for key,count,split,episodes,per_count,role in (("recovery_train",2718,"train",9,302,"TRAIN_resubstitution"),
        ("recovery_holdout",604,"val",2,302,"never_fit_actual_signed_recovery_HOLDOUT"),("v4",327,"train",1,327,"TRAIN_resubstitution")):
        previous.prior.verify_pair(before[key],after[key],count)
        for report in (before[key],after[key]): previous.verify_joint_report(report,count,split,episodes,per_count,role)
    for key,count,split,episodes in (("known599",599,"val",2),("legacy",2487,"train",8)):
        previous.prior.verify_pair(before[key],after[key],count)
        for report in (before[key],after[key]): previous.verify_raw_report(report,count,split,episodes)
    require(all(r.get("available_known_val_count")==665 and r.get("excluded_count")==66
        and r.get("role")=="previously_exposed_known_DEV599_raw_diagnostic" for r in (before["known599"],after["known599"])),
        "known599 remains already-exposed diagnostic, not independent665 evaluation")
    tokens=[{r["token"] for r in before[k]["rows"]} for k in COHORTS]
    require(all(not a&b for i,a in enumerate(tokens) for b in tokens[i+1:]),"cross-cohort source-token leakage")
    require({r["map"] for r in before["legacy"]["rows"]}=={"C_track_1_0_7","Town01","Town07"},"actual legacy TRAIN map coverage differs")
    checks=endpoint_checks(before,after);endpoint=documents["endpoint_diagnostic_screens.json"]
    require(endpoint.get("fixed_endpoint_updates")==6000 and endpoint.get("diagnostic_screen_count")==71
        and endpoint.get("retained_original_screen_count")==48 and endpoint.get("checks")==checks
        and all(type(c["passed"]) is bool for c in endpoint["checks"])
        and endpoint.get("all_diagnostic_screens_passed") is all(c["passed"] for c in checks)
        and all(endpoint.get(k) is False for k in ("independent_validation","feature_approved","vehicle_control_approved","model_promoted",
            "absolute_30kph_speed_tracking_claimed","holdout_selected_checkpoint")) and endpoint.get("fixed_final_only") is True
        and summary.get("endpoint_diagnostic_screens")==endpoint,"all71 original endpoint outcomes including failures must remain unchanged")
    journal=verify_history(history,declaration,before)
    for cohort,suffix in (("recovery_train","train"),("recovery_holdout","holdout")):
        previous.verify_response(documents["actual_signed_branch_response_"+suffix+".json"],before[cohort],after[cohort])
    joint_rows=[]
    for cohort in ("recovery_train","recovery_holdout","v4"):
        for episode,row in after[cohort]["per_episode"].items():
            joint_rows.append(dict(cohort=cohort,episode_id=episode,count=row["count"],metrics={k:dict(parent=before[cohort]["per_episode"][episode][k],
                final=row[k],delta=row[k]-before[cohort]["per_episode"][episode][k]) for k in JOINT_COMPONENTS}))
    aggregates={}
    for cohort in ("recovery_train","recovery_holdout","v4"):
        aggregates[cohort]={k:dict(parent=mean(before[cohort]["rows"],k),final=mean(after[cohort]["rows"],k)) for k in JOINT_COMPONENTS}
    raw_rows=previous.raw_comparisons(before["legacy"],after["legacy"],"legacy_TRAIN2487_raw_only")
    raw_rows+=previous.raw_comparisons(before["known599"],after["known599"],"previously_exposed_known_DEV599_not_independent")
    return dict(schema=SCHEMA,fixed_endpoint_updates=6000,official_main_function_approvals=0,official_main_function_total=9,
        official_subfunction_approvals=0,official_subfunction_total=30,model_promoted=False,vehicle_control_approved=False,
        simulator_control_authorized=False,independent_validation=False,checkpoint_selection_by_holdout=False,
        physical_trials_included=False,end_to_end_10Hz_validated=False,absolute_30kph_speed_tracking_claimed=False,
        stop_intent_trained=False,startup_recovery_fitted=False,diagnostic_pass_count=sum(c["passed"] for c in checks),
        diagnostic_gate_count=71,checks=checks,all_diagnostic_screens_passed=all(c["passed"] for c in checks),
        lineage=dict(parent_checkpoint_sha256=PARENT_CHECKPOINT_SHA,parent_npz_sha256=PARENT_NPZ_SHA,
            parent_metadata_sha256=PARENT_METADATA_SHA,parent_declaration_sha256=PARENT_DECLARATION_SHA,
            parent_diagnostic_pass_count=20,parent_diagnostic_gate_count=48,parent_known_updates=6200,additional_joint_updates=6000,
            known_total_updates=12200,final_checkpoint_sha256=metadata["checkpoint_sha256"],final_npz_sha256=metadata["npz_sha256"],
            final_metadata_sha256=final["export_metadata_sha256"],declaration_sha256=declaration_sha,
            frozen_state_sha256=metadata["frozen_state_sha256"],frozen_tensor_count=47,learned_tensor_count=54,final_tensor_count=101,
            original97_frozen_claimed=False,semantic_head_weights_frozen=True,semantic_outputs_preserved=False,
            training_sources=declaration["source_sha256"]),
        data=dict(TRAIN_episodes=18,recovery_TRAIN_episodes=9,recovery_TRAIN_anchors=2718,old_low_TRAIN_anchors=327,
            legacy_TRAIN_eligible_anchors=2487,available_TRAIN_anchors=5832,eligible_TRAIN_anchors=5532,
            HOLDOUT_episodes=2,HOLDOUT_anchors=604,HOLDOUT_fitted=False,HOLDOUT_checkpoint_selected=False,
            HOLDOUT_scope="same Town07 corridor/different measured offset-yaw-seeds; not independent unseen-map TEST",
            known_DEV_available=665,known_DEV_evaluated=599,known_DEV_is_independent=False,
            recovery_driving_window_seconds=[4.,34.1],true_future_seconds=6.4,startup_new_recovery_TRAIN=False),
        actual_journal=journal,joint_comparisons=joint_rows,joint_aggregate_comparisons=aggregates,raw_comparisons=raw_rows,
        raw_metric_definitions=dict(raw_curvature="original masked SmoothL1, not inv-m MAE",
            raw_preferred_speed="original masked SmoothL1, not m/s",raw_spatial_preferred_speed_mae_mps="physical raw-speed MAE m/s; legacy NOT decoded",
            decoded_speed="instantaneous decoded versus observed true speed; not XY chord speed",
            ADE_FDE="all64 observed future positions,6.4s horizon,in meters"),
        learning_rate=declaration["learning_rate"],actual_gpu_preflight=gpu,
        timing={k:number(summary[k]) for k in ("fit_including_baseline_seconds","postfit_endpoint_seconds","elapsed_owned_seconds")})


def verify_evidence_files(run,root,documents,*,summary_sha256,declaration_sha256,checkpoint_sha256,weights_sha256,metadata_sha256,
        parent_weights,parent_metadata,backbone_parent_weights,backbone_parent_metadata,evidence_pins=None):
    pins={} if evidence_pins is None else evidence_pins
    expected={"summary.json":summary_sha256,"declaration.json":declaration_sha256,"checkpoints/step_006000.pt":checkpoint_sha256,
        "exports/step_006000/model_state.npz":weights_sha256,"exports/step_006000/metadata.json":metadata_sha256}
    require(all(is_sha(v) for v in expected.values()),"external caller final6000 pins required")
    for name,sha in expected.items(): pin_payload(run/name,pins,expected_sha=sha)
    declaration,summary,metadata=(documents[k] for k in ("declaration.json","summary.json","exports/step_006000/metadata.json"))
    final=summary["checkpoints"][-1]
    require(final["sha256"]==checkpoint_sha256 and final["export_npz_sha256"]==weights_sha256
        and final["export_metadata_sha256"]==metadata_sha256,"external final pins versus retained receipts differ")
    for name,sha in ((HELPER_FILE,HELPER_SHA),(previous.PRIOR_FILE,previous.PRIOR_SHA)): pin_payload(root/name,pins,expected_sha=sha)
    for name,sha in declaration["source_sha256"].items():
        require(name in SOURCE_NAMES,"unexpected training source")
        pin_payload(root/name,pins,expected_sha=sha);pin_payload(run/"source"/name,pins,expected_sha=sha)
    parent=pin_json(parent_metadata,pins,expected_sha=PARENT_METADATA_SHA)
    require(parent.get("schema")==previous.EXPORT_SCHEMA and parent.get("checkpoint_sha256")==PARENT_CHECKPOINT_SHA
        and parent.get("declaration_sha256")==PARENT_DECLARATION_SHA and parent.get("optimizer_updates")==3000
        and parent.get("known_total_optimizer_updates")==6200 and parent.get("backbone_frozen") is True
        and parent.get("model_promoted") is False and parent.get("vehicle_control_approved") is False
        and parent.get("model_config")==metadata["model_config"] and parent.get("recovery_config")==metadata["recovery_config"],
        "exact unpromoted full101 V3 parent required")
    parent_arrays=previous.bounded_npz(parent_weights,PARENT_NPZ_SHA,parent["state_tensor_shapes"],parent["state_tensor_dtypes"],pins=pins)
    grand=pin_json(backbone_parent_metadata,pins,expected_sha=GRANDPARENT_METADATA_SHA)
    require(grand.get("schema")==previous.prior.EXPORT_SCHEMA and grand.get("npz_sha256")==GRANDPARENT_NPZ_SHA
        and grand.get("checkpoint_sha256")==previous.PARENT_CHECKPOINT_SHA and grand.get("optimizer_updates")==3000,
        "exact original mixed97 backbone parent required")
    grand_arrays=previous.bounded_npz(backbone_parent_weights,GRANDPARENT_NPZ_SHA,grand["state_tensor_shapes"],grand["state_tensor_dtypes"],pins=pins)
    require(len(parent_arrays)==101 and len(grand_arrays)==97 and set(parent_arrays)==set(grand_arrays)|set(previous.BRANCH_SHAPES)
        and all(parent_arrays[k].tobytes()==v.tobytes() and parent_arrays[k].shape==v.shape for k,v in grand_arrays.items()),
        "actual pre-joint V3 parent must retain all97 original backbone bytes")
    frozen=metadata["frozen_state_names"];trainable=metadata["trainable_state_names"]
    require(frozen==sorted(set(parent_arrays)-set(trainable_names(parent_arrays))) and len(frozen)==47
        and trainable==trainable_names(parent_arrays) and len(trainable)==54
        and previous.backbone_fingerprint({k:parent_arrays[k] for k in frozen})==metadata["frozen_state_sha256"],
        "actual47 original frozen input/semantic tensor fingerprint differs")
    receipts=[]
    for receipt in summary["checkpoints"]:
        step=receipt["optimizer_updates"];folder=run/"exports"/("step_%06d"%step);checkpoint=run/"checkpoints"/("step_%06d.pt"%step)
        pin_payload(checkpoint,pins,expected_sha=receipt["sha256"])
        middle=pin_json(folder/"metadata.json",pins,expected_sha=receipt["export_metadata_sha256"])
        verify_scope(middle);verify_rate(middle.get("learning_rate"))
        require(middle.get("schema")==EXPORT_SCHEMA and middle.get("training_schema")==RUN_SCHEMA
            and middle.get("source_sha256")==declaration["source_sha256"] and middle.get("declaration_sha256")==declaration_sha256
            and middle.get("checkpoint_sha256")==receipt["sha256"] and middle.get("npz_sha256")==receipt["export_npz_sha256"]
            and middle.get("parent_npz_sha256")==PARENT_NPZ_SHA and middle.get("frozen_state_sha256")==metadata["frozen_state_sha256"]
            and middle.get("trainable_state_names")==trainable and middle.get("frozen_state_names")==frozen
            and all(type(middle.get(k)) is int and middle[k]==v for k,v in dict(optimizer_updates=step,additional_optimizer_updates=step,
                examples_seen=4*step,recovery_examples_seen=2*step,v4_examples_seen=step,legacy_examples_seen=step,
                known_total_optimizer_updates=6200+step).items()),"intermediate numeric archive lineage differs; no selection")
        arrays=previous.bounded_npz(folder/"model_state.npz",receipt["export_npz_sha256"],middle["state_tensor_shapes"],middle["state_tensor_dtypes"],pins=pins)
        require(set(arrays)==set(parent_arrays) and len(arrays)==101 and all(arrays[k].shape==v.shape for k,v in parent_arrays.items())
            and all(arrays[k].tobytes()==parent_arrays[k].tobytes() for k in frozen),"one of47 frozen bytes or101 shapes changed")
        receipts.append(dict(step=step,checkpoint_sha256=receipt["sha256"],npz_sha256=receipt["export_npz_sha256"],
            metadata_sha256=receipt["export_metadata_sha256"],all101_float32_finite=True,frozen47_byte_identical=True,Torch_pickle_loaded=False))
    return dict(all12_checkpoints_and_optimizer_receipts_retained=True,all12_exports_full101_finite_frozen47_exact=True,
        original97_frozen_claimed=False,learned54_permitted_to_change=True,final_tensor_count=101,no_Torch_pickle=True,
        parent_npz_sha256=PARENT_NPZ_SHA,parent_metadata_sha256=PARENT_METADATA_SHA,
        backbone_parent_npz_sha256=GRANDPARENT_NPZ_SHA,backbone_parent_metadata_sha256=GRANDPARENT_METADATA_SHA,fixed_exports=receipts)


def numeric_receipt(run,metadata,metadata_sha,owner_id,root,parent_weights,parent_metadata,backbone_parent_weights,backbone_parent_metadata,enabled,archives):
    base=dict(schema=NUMERIC_SCHEMA,fixed_endpoint_updates=6000,checkpoint_sha256=metadata["checkpoint_sha256"],weights_sha256=metadata["npz_sha256"],
        metadata_sha256=metadata_sha,declaration_sha256=metadata["declaration_sha256"],loader_source=LOADER_FILE,loader_source_sha256=LOADER_SHA,
        numeric_validation_only=True,no_Torch_pickle=True,simulator_control_authorized=False,vehicle_control_approved=False,model_promoted=False,
        numpy_archive_validation=archives,original97_frozen_claimed=False)
    if not enabled:return dict(base,status="PASS_ARCHIVES_ONLY_REVIEWED_MODEL_LOADER_NOT_RUN",reviewed_model_loader_run=False)
    require(isinstance(owner_id,str) and owner_id.strip(),"explicit CPU-only numeric owner required")
    import importlib.util
    spec=importlib.util.spec_from_file_location("hh_joint_publish_workspace",root/"tools/workspace_python.py")
    workspace=importlib.util.module_from_spec(spec);spec.loader.exec_module(workspace);workspace.configure(root)
    from portable_e2e.temporal_route_joint_export_research_20261002 import OWNER_SCHEMA,load_route_joint_research_npz
    binding=dict(schema=OWNER_SCHEMA,mode="RESEARCH_ONLY",owner_id=owner_id,purpose="offline_cpu_numeric_validation_only",
        simulator_control_authorized=False,vehicle_control_approved=False,model_promoted=False,weights_sha256=metadata["npz_sha256"],
        metadata_sha256=metadata_sha,checkpoint_sha256=metadata["checkpoint_sha256"],declaration_sha256=metadata["declaration_sha256"],fixed_endpoint_updates=6000)
    model=load_route_joint_research_npz(run/"exports/step_006000/model_state.npz",run/"exports/step_006000/metadata.json",
        weights_sha256=metadata["npz_sha256"],metadata_sha256=metadata_sha,parent_weights=parent_weights,parent_metadata=parent_metadata,
        backbone_parent_weights=backbone_parent_weights,backbone_parent_metadata=backbone_parent_metadata,research_owner_binding=binding,source_root=root)
    require(model._research_temporal_export_metadata==metadata and model._research_recovery_simulator_control_authorized is False
        and len(model.state_dict())==101 and set(model.state_dict())==set(metadata["state_tensor_shapes"])
        and not model.training and all(not p.requires_grad for p in model.parameters()),"numeric endpoint loader altered state/control scope")
    return dict(base,status="PASS_FULL101_NUMERIC_CPU_ONLY_NOT_CONTROL",reviewed_model_loader_run=True,owner_id=owner_id,
        loader_schema=model._research_recovery_numeric_loader_schema,frozen47_byte_identical=True,learned54_explicit=True)


def markdown(report,numeric,run):
    lines=["# 주행 곡률·속도 공동 학습 — 최종 6000회", "",
        "사전 기준71개 중 **%d개 통과 / %d개 미달**입니다. 실패를 숨기거나 중간 checkpoint를 고르지 않았습니다."%(report["diagnostic_pass_count"],71-report["diagnostic_pass_count"]),"",
        "정식 주기능 **0/9**, 하위 기능 **0/30**. 이번 결과는 학습과 오프라인 평가입니다. 폐루프 주행·전체 기능·실차 사용 승인이 아닙니다.","",
        "## 무엇을 개선했나요?", "",
        "이전에는 차선 복귀 branch만 학습해 속도 오차를 줄일 수 없었습니다. 이번에는 같은101-tensor 모델에서 경로·속도·fusion/trunk54개 tensor를 함께 학습했습니다. 영상 입력부와 미학습 의미 head 등을 포함한47개 tensor는 부모와 바이트까지 같습니다.","",
        "- **MF02 경로 주행:** 직진·차선 복귀의 궤적 개선 대상입니다. 기존 회전 자료도 raw 곡률로 재학습했지만 실제 회전 성공을 입증한 것은 아닙니다.",
        "- **MF03 속도·승차감:** 실제 속도 label로 선호 속도를 학습했습니다. decoder는 그대로2.5m/s 이하이며30kph SPD01 검증이 아닙니다.",
        "- STOP/의도 head의 가중치는 동결했지만 공유 trunk 변화로 출력까지 같다는 보장은 없습니다. STOP은 아직 학습·승인하지 않았습니다.",
        "- 실제18TRAIN: 새 회복9개 +기존 저속1개 +기존 직진·회전8개. 같은 Town07 경로의 HOLDOUT2개는 학습과 checkpoint 선택에서 제외했습니다.",
        "- 고정6000 updates/24000회 노출: 회복12000 +기존 저속6000 +기존 직진·회전6000. 신규 회복은4.0~34.1초 구간이며 정지·출발 회복 학습은 아닙니다.",
        "- 기존 known DEV599는 이전부터 사용한 진단 자료입니다. 독립 TEST나 모든 맵 일반화로 부르지 않습니다.","",
        "## 전후 요약", "", "| 자료 | ADE 부모→최종(m) | FDE 부모→최종(m) | 순간 속도 MAE 부모→최종(m/s) |",
        "|---|---:|---:|---:|"]
    labels={"recovery_train":"새 회복 TRAIN2718", "recovery_holdout":"미학습 HOLDOUT604", "v4":"기존 저속 TRAIN327"}
    for cohort,label in labels.items():
        row=report["joint_aggregate_comparisons"][cohort]
        lines.append("| %s | %s |"%(label," | ".join("%.6f → %.6f"%(row[k]["parent"],row[k]["final"]) for k in JOINT_COMPONENTS[:3])))
    lines += ["", "## 미달 기준 — 전부 공개", "", "| 기준 | 최종 | 허용 상한 |", "|---|---:|---:|"]
    failures=[c for c in report["checks"] if not c["passed"]]
    if not failures:lines.append("| 미달 없음(오프라인 기준만 해당) | — | — |")
    for c in failures:lines.append("| %s | %.9f | %.9f |"%(c["name"],c["actual"],c["threshold"]))
    timing=report["timing"]
    lines += ["", "[모든71개 판정·계보](result.json), [9개 TRAIN·2개 HOLDOUT·기존327/2487/599 전체 전후 비교](comparisons.md), [원자료SHA](evidence_manifest.json), [숫자 전용 검사](numeric_loader_receipt.json).", "",
        "숫자 검사 `%s`. 12개 checkpoint는 SHA만 확인하고 `.pt`를 역직렬화하지 않았습니다. 모든 export의101개 float32 tensor 및47개 원본 바이트를 검사했습니다."%numeric["status"],"",
        "GPU 소유 %.1f초: baseline 포함 fit %.1f초, 최종 평가 %.1f초. 사전 fit 예산90분, 최종 평가는 별도입니다."%(timing["elapsed_owned_seconds"],timing["fit_including_baseline_seconds"],timing["postfit_endpoint_seconds"]),"",
        "다음은 고정 최종 가중치를 동일 조건의 실제 폐루프 직진·회복·회전 시험에 적용해 실제 성능과 기존 성능 보존을 확인하는 단계입니다. 이 도구는 추가 학습이나 simulator를 실행하지 않습니다.","", "원자료: `%s`"%run,""]
    return "\n".join(lines)


def publish(run,output,*,summary_sha256,declaration_sha256,checkpoint_sha256,weights_sha256,metadata_sha256,
        parent_weights,parent_metadata,backbone_parent_weights,backbone_parent_metadata,numeric_validation=False,owner_id=None,root=ROOT):
    run,output,root=canonical_path(run,exists=True),canonical_path(output),canonical_path(root,exists=True)
    assets=[canonical_path(p,exists=True) for p in (parent_weights,parent_metadata,backbone_parent_weights,backbone_parent_metadata)]
    parent_weights,parent_metadata,backbone_parent_weights,backbone_parent_metadata=assets
    require(run.is_dir() and root.is_dir() and not output.exists() and output.parent.is_dir(),"fresh create-only report with regular existing parents required")
    require(not output.is_relative_to(run) and not any(output.is_relative_to(p.parent) for p in assets),
        "report must remain outside immutable run and exact parent export directories")
    require(shutil.disk_usage(output.parent).free>=5*1024**3+512*1024**2,"retain5GiB+512MiB reserve; no deletion")
    pins={};documents={name:pin_json(run/name,pins) for name in DOCUMENTS}
    history=[json.loads(line,object_pairs_hook=previous.prior.unique_object,parse_constant=lambda value:(_ for _ in ()).throw(ValueError("nonfinite journal:"+value)))
        for line in pin_payload(run/"history.jsonl",pins,limit=32*1024**2).splitlines()]
    report=build_report(documents,history,declaration_sha256)
    archives=verify_evidence_files(run,root,documents,summary_sha256=summary_sha256,declaration_sha256=declaration_sha256,
        checkpoint_sha256=checkpoint_sha256,weights_sha256=weights_sha256,metadata_sha256=metadata_sha256,
        parent_weights=parent_weights,parent_metadata=parent_metadata,backbone_parent_weights=backbone_parent_weights,
        backbone_parent_metadata=backbone_parent_metadata,evidence_pins=pins)
    numeric=numeric_receipt(run,documents["exports/step_006000/metadata.json"],metadata_sha256,owner_id,root,
        parent_weights,parent_metadata,backbone_parent_weights,backbone_parent_metadata,numeric_validation,archives)
    pin_payload(Path(__file__),pins);recheck_pins(pins)
    evidence_names=[*DOCUMENTS,"history.jsonl"]
    for step in range(500,6001,500):evidence_names += ["checkpoints/step_%06d.pt"%step,"exports/step_%06d/model_state.npz"%step,"exports/step_%06d/metadata.json"%step]
    manifest=dict(schema=SCHEMA+".evidence",created_utc=datetime.now(timezone.utc).isoformat(),source_run_unmodified=True,
        first_read_bytes_pinned_and_rechecked=True,fixed_final6000_only=True,publisher_source_sha256=pins[str(Path(__file__).absolute())],
        helper_sha256=HELPER_SHA,numeric_loader_source_sha256=LOADER_SHA,
        source_files={name:dict(path=str(run/name),sha256=pins[str(run/name)]) for name in sorted(set(evidence_names))},
        parent_files={str(p):pins[str(p)] for p in assets})
    # HH_261002 - Only after all reads/pins/numeric guards succeed may a fresh report directory be created.
    output.mkdir();write_new(output/"result.json",report);write_new(output/"numeric_loader_receipt.json",numeric)
    write_new(output/"evidence_manifest.json",manifest);write_new(output/"comparisons.md",previous.comparison_markdown(report))
    write_new(output/"README.md",markdown(report,numeric,run))
    return dict(output=str(output),passed=report["diagnostic_pass_count"],total=71,official_main_approved="0/9",
        official_sub_approved="0/30",promoted=False,numeric_status=numeric["status"],physical_trials_included=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("run-root","output","parent-weights","parent-metadata","backbone-parent-weights","backbone-parent-metadata"):
        parser.add_argument("--"+name,type=Path,required=True)
    for name in ("summary","declaration","checkpoint","weights","metadata"):parser.add_argument("--"+name+"-sha256",required=True)
    parser.add_argument("--numeric-validation",action="store_true");parser.add_argument("--owner-id");args=parser.parse_args()
    print(json.dumps(publish(args.run_root,args.output,summary_sha256=args.summary_sha256,declaration_sha256=args.declaration_sha256,
        checkpoint_sha256=args.checkpoint_sha256,weights_sha256=args.weights_sha256,metadata_sha256=args.metadata_sha256,
        parent_weights=args.parent_weights,parent_metadata=args.parent_metadata,backbone_parent_weights=args.backbone_parent_weights,
        backbone_parent_metadata=args.backbone_parent_metadata,numeric_validation=args.numeric_validation,owner_id=args.owner_id),ensure_ascii=False))


if __name__=="__main__":main()
