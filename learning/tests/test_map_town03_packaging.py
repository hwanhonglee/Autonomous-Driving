"""HH_260906 - Guard Town03-only map TRAIN packaging and frozen AI v2 split bytes."""

from __future__ import annotations

import copy
from pathlib import Path

import pytest

from scripts.e2e.map_expansion_contract import AI_V2_PARENT_SHA256, TOWN03_ROUTE_SHA256, validate_expansion
from scripts.e2e.prepare_map_expansion_ai import (
    ROOT, dedup_town03, frozen_ai_v2, owned_town03_capture, preflight_town03,
)


def cross_town_fixture():
    # HH_260906 - This is a contract fixture, not fabricated evidence of CARLA or model quality.
    parent={"splits":{
        "train":{"count":497,"source_records":[{"id":i} for i in range(497)],
                 "source_hashes":{"old":"sha"},"source_summary":{"town":"Town01"},
                 "class_pixels":[10]*7},
        "dev":{"count":118,"sha256":"dev","source_summary":{"town":"Town01"}},
    }}
    routes=sorted(TOWN03_ROUTE_SHA256)
    new=copy.deepcopy(parent)
    added=[{"source_town":"Town03","route_sha256":route,"id":i}
           for route in routes for i in range(120)]
    new["splits"]["train"].update(
        count=737,source_records=copy.deepcopy(parent["splits"]["train"]["source_records"])+added,
        source_hashes={"old":"sha","new":"sha2"},class_pixels=[250]*7)
    new["parent_manifest_sha256"]=AI_V2_PARENT_SHA256
    new["expansion"]={
        "split_mode":"town_disjoint_train_v1","added_frames":240,
        "owner_exit_code":0,"collision_events":0,
        "source_bytes_unchanged_and_archived":True,"physical_capture":True,
        "dev_used_for_training":False,"parent_town":"Town01","added_town":"Town03",
        "held_out_town":"Town07","same_town_dedup_m":.25,
        "cross_town_xy_compared":False,"minimum_dev_distance_m":None,
        "minimum_old_train_distance_m":None,
        "routes":[{"town":"Town03","route_sha256":route,
                   "independent_retained_frames":120,"added_frames":120,
                   "owner_exit_code":0,"added_class_pixels":[120]*7} for route in routes],
    }
    return new,parent


def test_cross_town_contract_accepts_town_disjoint_proof():
    new,parent=cross_town_fixture()
    validate_expansion(new,parent,AI_V2_PARENT_SHA256)


@pytest.mark.parametrize("change",[
    "parent_sha","dev","old_prefix","town07_frame","fake_distance","wrong_town",
    "route_sha","right_under_120","pixel_count","record_count","no_route_contribution",
])
def test_cross_town_contract_rejects_leakage_or_incomplete_proof(change):
    new,parent=cross_town_fixture()
    proof=new["expansion"]
    if change=="parent_sha":
        new["parent_manifest_sha256"]="bad"
    elif change=="dev":
        new["splits"]["dev"]["sha256"]="changed"
    elif change=="old_prefix":
        new["splits"]["train"]["source_records"][0]["id"]=-1
    elif change=="town07_frame":
        new["splits"]["train"]["source_records"][-1]["source_town"]="Town07"
    elif change=="fake_distance":
        proof["minimum_dev_distance_m"]=999999
    elif change=="wrong_town":
        proof["routes"][0]["town"]="Town06"
    elif change=="route_sha":
        proof["routes"][0]["route_sha256"]="bad"
    elif change=="right_under_120":
        proof["routes"][1]["added_frames"]=119
    elif change=="pixel_count":
        new["splits"]["train"]["class_pixels"][0]+=1
    elif change=="record_count":
        new["splits"]["train"]["source_records"].pop()
    elif change=="no_route_contribution":
        route=proof["routes"][1]["route_sha256"]
        new["splits"]["train"]["source_records"][-1]["route_sha256"]=proof["routes"][0]["route_sha256"]
        assert route!=proof["routes"][0]["route_sha256"]
    with pytest.raises(ValueError):
        validate_expansion(new,parent,AI_V2_PARENT_SHA256)


def frame_rows(start: float, count: int = 240) -> list[dict]:
    return [{"ego_x":start+i*.3,"ego_y":0.0,"stem":f"frames/{i:04d}"}
            for i in range(count)]


def test_town03_dedup_checks_every_kept_xy_across_routes():
    first=frame_rows(0)
    second=frame_rows(0)
    # HH_260906 - A duplicate route must not claim an independent TRAIN contribution.
    with pytest.raises(ValueError,match="globally unique"):
        dedup_town03([first,second])
    second=frame_rows(100)
    selected,individual=dedup_town03([first,second])
    assert individual==[240,240]
    assert [len(rows) for rows in selected]==[240,240]
    assert selected[0][0]["stem"]==selected[1][0]["stem"]


def test_town03_dedup_requires_120_final_unique_per_route():
    first=frame_rows(0)
    # HH_260906 - 121 first-route overlaps leave 119 globally unique positions.
    second=frame_rows(0,121)+frame_rows(100,119)
    with pytest.raises(ValueError,match="globally unique"):
        dedup_town03([first,second])


DATA=ROOT/'storage/datasets/prepared/map_expanded_20260926_ai_v2'
STRAIGHT=ROOT/'storage/experiments/training/2026-09-27/map_generalization_town03_straight_p03'
RIGHT=ROOT/'storage/experiments/training/2026-09-27/map_generalization_town03_right_p03'


@pytest.mark.skipif(not all(path.is_dir() for path in (DATA,STRAIGHT,RIGHT)),
                    reason="owned Town03 capture and frozen local AI v2 arrays not present")
def test_real_owned_town03_preflight_does_not_package_or_open_town07(tmp_path: Path):
    parent=frozen_ai_v2(DATA)
    assert parent["splits"]["train"]["count"]==497
    assert parent["splits"]["dev"]["count"]==118
    ready=preflight_town03(DATA,STRAIGHT,RIGHT)
    assert [route["added_frames"] for route in ready["routes"]]==[232,184]
    assert ready["added_frames"]==416
    assert not list(tmp_path.iterdir())


@pytest.mark.skipif(not STRAIGHT.is_dir(),reason="owned Town03 capture not present")
def test_owned_town03_capture_rejects_wrong_role():
    with pytest.raises(ValueError,match="route provenance"):
        owned_town03_capture(STRAIGHT,"right")
