"""HH_261008 - Preserve V15; admit only removal of its redundant normalized past-cache-hit clone.

The frozen V1 hard DENY is never modified or called. ROOT must separately seal
the prospective plan and an external execution authority. No training, CARLA,
actuator, original-BGRA, V5 terminal-policy, or feature approval is implied.
"""
from __future__ import annotations

import argparse
import ast
from copy import deepcopy
import hashlib
import fcntl
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import resource
import signal
import stat
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.owned_recorded_ROS_shadow.20261008.v16"
HERE = Path(__file__).resolve()
NODE = ROOT / "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v16.py"
OWNER = HERE.with_name("run_owned_temporal_recorded_ros_shadow_20261008_v16.py")
PLAN_DIR = ROOT / "storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v16"
PLAN, AUTHORITY = PLAN_DIR / "ROOT_plan.json", PLAN_DIR / "ROOT_execution_authority.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v16"
EPISODE = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v1/episode"
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
BRIDGE_REPORT = ROOT / "storage/experiments/diagnostics/2026-10-07/owned_dds_info_bridge_v5/result.json"
FLOOR, SHARED, QUOTA, CLOSING = 2*1024**3, 64*1024**2, 128*1024**2, 16*1024**2
FILE_LIMIT, JSON_LIMIT, PLAN_LIMIT = 32*1024**2, 1024**2, 4*1024**2
SUBSCRIBER_PERSISTENT_MEMBERSHIP = True
CODEC = ROOT / "tools/e2e/owned_lossless_jsonl_20261008_v2.py"
CODEC_PINS = {
    "tools/e2e/owned_lossless_jsonl_20261008_v1.py":"5e859fbffa4aaebe81f8e57426cb57600a9c9272190f1e4887428991c6cccbb5",
    "learning/tests/test_owned_lossless_jsonl_20261008_v1.py":"b4c82537b3efaf4547f2c33323ee1f88508ff10bbd1cb12dcc5783d91187648f",
    "tools/e2e/owned_lossless_jsonl_20261008_v2.py":"f867fc18ff6f85038e3ea4dab8fd4b7b50bba84be8655c4db68bd6eab8eff479",
    "learning/tests/test_owned_lossless_jsonl_20261008_v2.py":"27c2b6f60947f5b1a37c6453afbea3107afc645601d3a9459b746db6201a8ba0",
}
V12_PINS = {
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v12.py":"cc77b39a9093c4e18fbfa1efae04a8dd5653e1e6ba73e3bd16b095adef394b12",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v12.py":"f3b79e68773f72f5371027faa5cf6233e51f4d8e3c5aa3366569074c4d3d2887",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v12.py":"c8953312b53b9920d4b120a3c75372e1fa10b72ba53498144cd2eb2095d998ba",
    "learning/tests/test_owned_recorded_ros_shadow_20261008_v12.py":"1700d072ca541dfa2023e970d1468f7f3388b7ca7c7d1405c4ad02bb1c8b059b",
}
_codec = None
_ledgers = {}
V14_PINS = {
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v14.py":"7cb22e4882cce95c03be6365fbca6e02287150adf1d099148c2090b7f5dbcf68",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v14.py":"c549ba48b186a6ab9d8e44505335b3c1863de07a215544b794021081f1253eb2",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v14.py":"eb3bb116972b37b01b1b737e1c5dc9007974672ac0befadc91ddfb2f16b97d8a",
    "learning/tests/test_owned_recorded_ros_shadow_20261008_v14.py":"46eb0cfbd7f3c6436de88053bb69ed9e46fb8c5c1b735eb3476f644c836c727f",
}
# HH_261008 - Preserve the exact SHA-copy candidate and its complete actual failed attempt, not a feature approval.
V15_PINS = {
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v15.py":"aa6254d87632f28b187ca12ccdec564f69bc02e2b5891a52e1e945a671d7f3b9",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v15.py":"6a1b37a045240ba4d504cfe80d7299d9965a859216f73387e5285f99d5b80dfb",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v15.py":"73a8177e736504542624a4889d0913d43d307578ad3b024d019e992a1a29eb2c",
    "learning/tests/test_owned_recorded_ros_shadow_20261008_v15.py":"1f535784553b8948b484409e9d0a118b04f2e72a335b0d6cb2f8b0b855a0e5ed",
}
V15_EVIDENCE = {
    str(ROOT/"storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v15/result.json"):"71eb507817bfda373f752fb3fa5443cf2ca9f45aea34cc532765e4eac29335dd",
    str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v15/ROOT_plan.json"):"d739a864947a855b403443e2553ca4bb7c9ff54e89db39f8bdc6764c3e425313",
    str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v15/ROOT_execution_authority.json"):"7b625f1007dc5ba0eb70316214345bdf0077514c39c091b1c0e3a5beb831df08",
}
BUILDER_PINS = {
    "learning/portable_e2e/temporal_simulation_research_v2_20261001.py":"772275dd9e452c70bd9d5364bbacb32d8bf08438ce8bf3526f7ebcf3d64884dc",
    "learning/portable_e2e/temporal_single_copy_history_research_20261001.py":"42d98271cddee6f19e01c35c29167cd9d1a8dfb1492f34f2334b74b17274d053",
}
# HH_261008 - Retain the actual failed V14 interval and whole closing seals; the membership candidate was not a feature PASS.
V14_EVIDENCE = {
    str(ROOT/"storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v14/result.json"):"acde13e028f079149d700d107c0eb392a371559ac14dd3886a18e7c8f7a3a29d",
    str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v14/ROOT_plan.json"):"849eca5259d031172107cd15d5adfc0749c9124d24fb134832115dfd6024b2d8",
    str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v14/ROOT_execution_authority.json"):"84e80f54b4fd33e4eca009556f78bba5b57fe251a4d922fce32d4b568e1e3016",
}
V13_PINS = {
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v13.py":"ce0a506195c43a6b662d32018ee09a19952526eafd73a42bad8cdd61a75f425f",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v13.py":"f71af536d15c828558560ee657d2484c8b260fcc330f4492a19aaac76c66634b",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v13.py":"48e51f8589a310eb13ba9b41abe876af8f1ab744590d3aeeebc228140a6e76b4",
    "learning/tests/test_owned_recorded_ros_shadow_20261008_v13.py":"418094458991980508a777ec647a8d8cb8e85950f229082eb513ad63910654a2",
}
# HH_261008 - Retain the actual failed V13 interval and its genuine closing source/input seals; this is not successful feature evidence.
V13_EVIDENCE = {
    str(ROOT/"storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v13/result.json"):"2b009f267a98ed089191e2fc3b735732768476d9e12482d64dd0e85a3cf273eb",
    str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v13/ROOT_plan.json"):"d2c2eb14d75b85cb7ec27a05ebde598c8598f85fb17de92e66a6481c99647174",
    str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v13/ROOT_execution_authority.json"):"0d9f61da14aa1463895fea34f31d8470288c6f84d6e052a253ca3888b2e21293",
}
FROZEN = {
    "learning/portable_e2e/temporal_recorded_ros_bundle_v1_20261005.py": "6fa21ab9447bcf73bdd2fd79b0ab72ddd94b531d3a11459334f754fdfbad7d82",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_v1_20261005.py": "e0e1b001c3684dfdf1ea41274c4c39b7f9a286d008d37d942aefbe35a3591e7e",
    "learning/portable_e2e/temporal_autoware_shadow_adapter_v1_20261005.py": "7d3cf7b6f4f131c2647921a00b62f08e4b41e30cd6b98f9583ad425267e46a0e",
}
_bundle = None
V1_PINS={
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v1.py":"7ae85b64b743e461e4723fbfb04fae429522710c84ed9b002a68a6f77ec67f4f",
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v1.py":"d38fa56ff49d947f5beb88353c650052ea5d6e98f630cf6f94afb5814bbac404",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v1.py":"d0306f5f7cb6159a4fbea1d64b484e9cd227c3e4233dc6911d5435516bfffc17",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v1.py":"d1803f2ef0b5a474b298b023e766810067eb37d73785437176fe5dcf10c30547",
}
V2_PINS={
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v2.py":"289099aab00135267a63b927341756fca96cc62f66a164c7fda4b969edbeddef",
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v2.py":"caba2e19055be8bbc191a5ff5cc17799dbaa0c707303c26019cb20fcca75066b",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v2.py":"ca92f6ed88ab5c4ae6f0929a92e06fae77ecbf6e051a880874f0dbe76e7342de",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v2.py":"1202fa1f8d3011fe28c47d901a161f0eda0a3bbc184c8b185915f71acea76fb2",
}
V3_PINS={
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v3.py":"5bbd9d07a9e8eb096caecf47531376d607dec8852b3b7fceea52f1f0858bd0ff",
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v3.py":"7babb71d007fb2636e1a6d19ac6e606ec11f4206218087d70f41f185fde30178",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v3.py":"a86012a2d7e6e63086feec2eca8d33366951b8db9005e889bd8cd48cc890e853",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v3.py":"7923aa05f74accf0a9487965f8fa02094fee3535193d80e5a6161906235ee917",
}
V4_PINS={
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v4.py":"710cab6280c06d9ad4ccf43acbe688b20e017e7cf6caff340306d460ac1d91b5",
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v4.py":"b0ff7ad6320de3f917cc8c56e4574e0e8b92cb6305cc1073e1d74bbfcd27edcc",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v4.py":"0d8eff86dca6a9a9292c82a4150e986ebac12c7dfee597edbc4d8d05756de49b",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v4.py":"211d23d56788af1ff7dac05adcb0e24e2b9a5683fdae73bd68bbc5d2e2a897ec",
}


V5_PINS={
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v5.py": "85e924da5eedb35c39da53a9b57014d8cb506cbc458c5db0a5b820f2b333efd8",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v5.py": "e8868b6614fc4d0ba4b8049812f929b761eb9c44561ec261db34b2a5d8452454",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v5.py": "b97e90ad9ede2544fbd33c2fc285a279cca7dcbba0bd0744f92926a15f362a98",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v5.py": "bdb974215f7ebd10e2d5ce032efafc160a5486a7e746b8e67dc973afab927029"
}


V6_PINS={
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v6.py": "237b901bb5359cc7cd8a30c48be9c63472f8dad688e343338a62316df61f03a7",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v6.py": "b6d7ac6e93a7f933e83c90652faa8f694668527639f94627963426c92d5206e7",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v6.py": "64af7e24b1cfb0b49bf01726fd688d2b1a98c4bd608513f650872528f084e211",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v6.py": "e1c98c6ad451f241a126506a511e06fc6e043956a0b72c9aeaaa0a7b61df74c5"
}


V7_PINS={
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v7.py": "01ab22535a512ad94b0b3c6aebf75707f874486855ca89c9162839aa2a6d3e65",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v7.py": "3f90ef3e0473f8a112e1db7dbbd3deb654dbb286f96f98cffa2a0d7a00899264",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v7.py": "b17615c09468a92d917e9c2e1ebce232db44b6fbd87347d3e82017588674fdf9",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v7.py": "4d70aaf4d9d5c415c74dbcb9ff855eb20360d6ea3b8613b667220a508c9475f0"
}
V8_PINS={
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v8.py":"59992c6ca31aba3e031aa2df9e700bc04870af9c3b65828af702461a20fe122e",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v8.py":"b58afd2374d0442b81677d69d5fd3382bc08c35dce810d6f2afb6893226cb041",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v8.py":"4333c1b5ac0a0777372df0115a28d01123c1ce963647feada734d4f09ebfe744",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v8.py":"77f97d84eb092aa551f34aa9abe8454f965930d41311b5298a9bde81cf8383e3",
}
V8_EVIDENCE={
    str(ROOT/"storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v8/result.json"):"16d598e251180a061e6d60b3e2dd88a853ab78f11f9383678cdf80978374014a",
    str(ROOT/"storage/experiments/plans/2026-10-07/temporal_recorded_ros_shadow_v8/ROOT_plan.json"):"271d28b3c52cfa5a847596dde417af30c1dea9686c41e07c702a9c78a046eaae",
}
V9_PINS={
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v9.py":"b40f2a4f30bcd5ff9ea2b9afa456cfc7aba525379b775a113a5d00b762e252d8",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v9.py":"7bbf5086d08ea0d2635c0be3783d95b2357e0127119a8aa45fa7dd6bc4978a50",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v9.py":"3dede2397248f92d2d3fe91c81dec4d474850701c67661841b9a5d8e813818bd",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v9.py":"ea9ff7e22b98f3a5584b1c322cd968117fa2033eb236cbb06769df641e8e523f",
}
V9_EVIDENCE={
    str(ROOT/"storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v9/result.json"):"c05887488287f6c217c9823a4ed06e99dbbd8b7286866737aa121836924d3062",
    str(ROOT/"storage/experiments/plans/2026-10-07/temporal_recorded_ros_shadow_v9/ROOT_plan.json"):"16f0995070d83e10fa2af8ab44cf377826cbd8c0071553de06078adc9376bc91",
}
V10_PINS={
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v10.py":"0bf76a6d711ef75e2ebd376167be6e502efe965c1b03dba42262960d8c39dfd6",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v10.py":"10dab25a3bf779766292fca6bb9dd004c9df27f7e85ba80de6ff3cc5732f19ac",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v10.py":"325c8ea9f2c49553981d04e0b56996a314a6174e88a828e987785a4f151683fa",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v10.py":"a8ed7923a0deba8f2c30bb2824f11371820c6f5d769d1690225a9741d81cb0be",
}
V10_EVIDENCE={
    str(ROOT/"storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v10/result.json"):"c1b92d82fb084ca5ce9ffb62359d5a52997a36672788b9fe237dcf58270b725e",
    str(ROOT/"storage/experiments/plans/2026-10-07/temporal_recorded_ros_shadow_v10/ROOT_plan.json"):"516af9e4323e6285a06338b3e70e14d16381c9b0b5fd09125d15d153e4d28436",
}
V11_PINS={
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v11.py":"18b679c60a6ed9ba91e09d2b431f4b7a7302e05ce4c194389438f2cd908e2b41",
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v11.py":"6d84ba0b8f3ec6875117bdcbfefd8c18a421519a26edf5baa1a27784141c7e13",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v11.py":"06c2f0e2ab7b6ff4d07bf493e9b3d15242f2e3c12f9f0e6947dcce391d5775e0",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v11.py":"904a499263604dec2f49e2fdca9468b3cb0fb4dea2ab8ef90dc51f5858f7f2a4",
}
V11_EVIDENCE={
    str(ROOT/"storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v11/result.json"):"3fbcc123a587fb3e21af763ff39b9914cc132063b04abe660b7675f9815aa37d",
    str(ROOT/"storage/experiments/plans/2026-10-07/temporal_recorded_ros_shadow_v11/ROOT_plan.json"):"f9e55834d2ecaad5537b3f946a873e26767c89987e2c546fd9ceff0c6a3e279d",
    str(ROOT/"storage/experiments/plans/2026-10-07/temporal_recorded_ros_shadow_v11/ROOT_execution_authority.json"):"665276deea1e39c3bf1f1441b9a3f860fe4ca5b50a3310fa9b4b6bc73e8176a9",
}
EXECUTOR_API_PINS={
    "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py":"2bb58dc3ed9a224c305f7f0e42ba76a44c3b893449acc1781ba55954dd6fdd16",
    "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/node.py":"24e4267dbf8306d8b25dcdfcbea9b9e33213897b846a1e5578ef4cf8ab49b445",
}


def need(value, message):
    if not value: raise ValueError(message)


def encode(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)+"\n").encode()


def digest(path):
    path = Path(path)
    need(path.is_absolute() and path.resolve(strict=True) == path and path.is_file(), "canonical regular input required")
    before = path.stat(); value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""): value.update(block)
    after = path.stat()
    need(all(getattr(after, key) == getattr(before, key) for key in ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")), "input changed while hashing")
    return value.hexdigest()


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def frozen_bundle():
    global _bundle
    if _bundle is not None: return _bundle
    for name, pin in FROZEN.items(): need(digest(ROOT/name) == pin, "frozen prototype changed")
    _bundle=load_module(ROOT/next(iter(FROZEN)), "_hh_frozen_recorded_bundle_261007")
    return _bundle


def source_pins():
    bundle = frozen_bundle()
    names = bundle.project_source_closure()
    names |= {str(p.relative_to(ROOT)) for p in (HERE, OWNER, NODE)}
    names |= {"tools/e2e/prepare_carla_common10_dataset.py", "tools/workspace_python.py", "tools/canonical_workspace.py",
              "learning/tests/test_owned_recorded_ros_shadow_20261008_v16.py",
              "tools/e2e/owned_dds_info_bridge_20261007_v1.c",
              "tools/e2e/probe_owned_dds_info_bridge_20261007_v1.py",
              "learning/tests/test_owned_dds_info_bridge_20261007_v1.py"}
    names |= {"tools/e2e/probe_owned_dds_info_bridge_20261007_v2.py", "learning/tests/test_owned_dds_info_bridge_20261007_v2.py"}
    names |= {"tools/e2e/probe_owned_dds_info_bridge_20261007_v3.py", "learning/tests/test_owned_dds_info_bridge_20261007_v3.py"}
    # HH_261007 - Seal the public matched-endpoint resolver and every reviewed probe revision, including its RMW plugin closure.
    names |= {"tools/e2e/owned_dds_info_bridge_20261007_v2.c"}
    names |= {f"{prefix}_20261007_v{version}.py" for version in (4,5) for prefix in
        ("tools/e2e/probe_owned_dds_info_bridge", "learning/tests/test_owned_dds_info_bridge")}
    for name,pin in V1_PINS.items():need(digest(ROOT/name)==pin,"failed V1 source changed")
    names |= set(V1_PINS)
    for name,pin in V2_PINS.items():need(digest(ROOT/name)==pin,"failed V2 source changed")
    names |= set(V2_PINS)
    for name,pin in V3_PINS.items():need(digest(ROOT/name)==pin,"failed V3 source changed")
    names |= set(V3_PINS)
    for name,pin in V4_PINS.items():need(digest(ROOT/name)==pin,"actual V4 source changed")
    names |= set(V4_PINS)
    # HH_261007 - Retain the source-only V5 revision while adding typed-ownership guards in a separate V6 intervention.
    for name,pin in V5_PINS.items():need(digest(ROOT/name)==pin,"source-only V5 revision changed")
    names |= set(V5_PINS)
    # HH_261007 - Preserve the failed source-only V6 patch byte-for-byte and correct its stray patch marker only in V7.
    for name,pin in V6_PINS.items():need(digest(ROOT/name)==pin,"failed V6 source changed")
    names |= set(V6_PINS)
    # HH_261007 - Consume only a separately probe-verified bridge and retain the source-only V7 revision.
    for name,pin in V7_PINS.items():need(digest(ROOT/name)==pin,"source-only V7 revision changed")
    names |= set(V7_PINS)
    # HH_261007 - Seal all actually executed V8 sources while changing only subscriber audit work placement.
    for name,pin in V8_PINS.items():need(digest(ROOT/name)==pin,"actual V8 source changed")
    names |= set(V8_PINS)
    # HH_261007 - Preserve the actually failed V9 interval while repositioning redundant full graph censuses only in V10.
    for name,pin in V9_PINS.items():need(digest(ROOT/name)==pin,"actual V9 source changed")
    names |= set(V9_PINS)
    # HH_261007 - Preserve actual V10 failures while measuring stages in a fresh protocol with identical scientific paths.
    for name,pin in V10_PINS.items():need(digest(ROOT/name)==pin,"actual V10 source changed")
    names |= set(V10_PINS)
    # HH_261008 - V11 stays immutable failed provenance; only the fresh four V12 files implement membership ownership.
    for name,pin in V11_PINS.items():need(digest(ROOT/name)==pin,"actual V11 source changed")
    names |= set(V11_PINS)
    # HH_261008 - Preserve undeployed V12 and both codec revisions as exact scientific/source provenance.
    for name,pin in {**V12_PINS,**CODEC_PINS}.items():need(digest(ROOT/name)==pin,"frozen V12/codec source changed")
    names |= set(V12_PINS) | set(CODEC_PINS)
    for name,pin in V13_PINS.items():need(digest(ROOT/name)==pin,"frozen V13 source changed")
    names |= set(V13_PINS)
    for name,pin in V14_PINS.items():need(digest(ROOT/name)==pin,"frozen V14 source changed")
    names |= set(V14_PINS)
    for name,pin in {**V15_PINS,**BUILDER_PINS}.items():need(digest(ROOT/name)==pin,"frozen V15/builder source changed")
    names |= set(V15_PINS) | set(BUILDER_PINS)
    return {name: digest(ROOT/name) for name in sorted(names)}


def bounded_json(path, pin=None, limit=JSON_LIMIT):
    path = Path(path); need(path.resolve(strict=True) == path and path.is_file() and path.stat().st_size <= limit, "bounded canonical JSON required")
    raw = path.read_bytes()
    if pin is not None: need(hashlib.sha256(raw).hexdigest() == pin, "raw JSON SHA differs")
    return frozen_bundle().decode(raw, limit)


def new_json(path, value, limit=JSON_LIMIT):
    raw = encode(value); need(len(raw) <= limit <= PLAN_LIMIT, "bounded new JSON required")
    with Path(path).open("xb") as stream: stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    return hashlib.sha256(raw).hexdigest()


def budget(startup=False, additional=0, closing=False):
    need(type(additional) is int and additional>=0,"bounded prospective physical bytes required")
    need(shutil.disk_usage(ROOT).free >= FLOOR+SHARED+(QUOTA if startup else (0 if closing else CLOSING)), "separate CPU protocol reserve denied; GPU reserve is unchanged")
    total = metadata = 0
    for directory in (OUTPUT, PLAN_DIR):
        if not directory.exists(): continue
        for path in directory.rglob("*"):
            mode = path.lstat().st_mode
            need(stat.S_ISREG(mode) or stat.S_ISDIR(mode), "special/symlink output denied")
            if stat.S_ISREG(mode):
                need(path.stat().st_size <= FILE_LIMIT, "complete raw file exceeds declared quota")
                size=path.stat().st_size;total+=size
                relative=path.relative_to(directory)
                if directory==OUTPUT and path.name in {role+".stderr" for role in ("direct","subscriber","publisher")}:
                    need(size<=1024**2,"complete owned role stderr exceeds prospective1MiB cap")
                elif directory==OUTPUT and relative.parts[0].endswith(".lossless"):
                    logical=OUTPUT/relative.parts[0].removesuffix(".lossless")
                    component=logical.name.removesuffix(".jsonl").replace(".stdout","_stdout")
                    need(component in codec().prospective_capacity()["components"],"foreign compressed journal directory denied")
                else:metadata+=size
    if not closing:need(metadata<=10*1024**2,"whole source/plan/other metadata reserve exceeded")
    need(total+additional <= QUOTA-(0 if closing else CLOSING), "all-output closing reserve exhausted")
    return total


def codec():
    global _codec
    if _codec is None:
        for name,pin in CODEC_PINS.items():need(digest(ROOT/name)==pin,"reviewed codec source changed")
        _codec=load_module(CODEC,"_hh_owned_lossless_jsonl_261008_v2")
    return _codec


def ledger_binding(path):
    # HH_261008 - Original logical call paths remain fixed; no plaintext compatibility fallback or unbounded ledger is admitted.
    path=Path(path)
    need(path.parent==OUTPUT and path.parent.resolve(strict=True)==path.parent,"exact owned logical ledger path required")
    component=path.name.removesuffix(".jsonl").replace(".stdout","_stdout")
    capacity=codec().prospective_capacity()
    need(component in capacity["components"],"unreviewed logical journal path denied")
    return path.with_name(path.name+".lossless"),capacity["components"][component]


def ledger(path):
    root,limits=ledger_binding(path)
    key=str(root)
    if key not in _ledgers:
        _ledgers[key]=codec().LosslessJSONLLedger(root,guard=lambda added:budget(additional=added),**limits)
    return _ledgers[key]


def append(path, value):
    # HH_261008 - Preserve exactly the original encode bytes and call position; compression/write/flush are synchronous.
    budget();raw=encode(value);need(len(raw)<=512*1024,"complete scalar row too large")
    return ledger(path).append(raw)


def iter_records(path):
    # HH_261008 - Every caller must exhaust this iterator; missing files mean no executed rows, never a fabricated prefix.
    root,limits=ledger_binding(path)
    if not root.exists():return
    for raw in codec().iter_rows(root,**limits):yield frozen_bundle().decode(raw,512*1024)


def count_records(path):
    return sum(1 for _ in iter_records(path))


def close_ledgers():
    # HH_261008 - Retain actual pending bytes and oversized-fragment descriptors using only the unchanged closing reserve.
    rows=[]
    for root,value in sorted(_ledgers.items()):
        errors=[]
        try:value.close()
        except BaseException as failure:errors.append(repr(failure))
        pending=value.pending_raw;pending_path=None
        if pending is not None:
            try:
                budget(additional=len(pending),closing=True)
                path=Path(root).with_name(Path(root).name+".failed.raw")
                with path.open("xb") as stream:need(stream.write(pending)==len(pending),"partial failed raw retention");stream.flush()
                pending_path=str(path)
            except BaseException as failure:errors.append("pending retention: "+repr(failure))
        verified=None
        try:
            logical=Path(root).with_name(Path(root).name.removesuffix(".lossless"))
            _,limits=ledger_binding(logical);verified=codec().verify_ledger(root,**limits)
        except BaseException as failure:errors.append("whole ledger verification: "+repr(failure))
        rows.append(dict(path=root,accepted_rows=value.count,closed=value.closed,failure=value.failure,
            pending_raw_path=pending_path,pending_raw_bytes=None if pending is None else len(pending),
            oversized_fragment_receipt=value.oversized_fragment_receipt,verification=verified,errors=errors,
            complete_prefix_is_PASS=False,failed_stream_lossless_claimed=False))
    failed=any(row["failure"] is not None or row["errors"] or row["oversized_fragment_receipt"] is not None for row in rows)
    receipt=dict(schema=SCHEMA+".codec_closing",ledgers=rows,codec_failure=failed,
        complete_failed_stream_bytes_guaranteed=False,original_ACK_clock_includes_later_journal_or_console=False)
    return receipt


def process_identity(pid):
    base = Path("/proc")/str(pid); raw = (base/"stat").read_text(); fields = raw[raw.rfind(")")+2:].split()
    return dict(pid=pid, starttime=int(fields[19]), ppid=int(fields[1]), pgid=int(fields[2]), session=int(fields[3]),
        uid=base.stat().st_uid, argv=[x.decode() for x in (base/"cmdline").read_bytes().split(b"\0") if x])


def same_generation(expected):
    actual = process_identity(expected["pid"])
    need(all(actual[k] == expected[k] for k in ("pid", "starttime", "uid", "ppid", "pgid", "session", "argv")), "owned PID generation/argv changed")
    return actual


def live_owner():
    receipt=bounded_json(OUTPUT/"owner_started.json");same_generation(receipt["identity"])
    need(os.getppid()==receipt["identity"]["pid"],"owned parent died/reparented")
    raw_fd=os.environ.get("HH_OWNED_RUNTIME_LOCK_FD","")
    need(raw_fd.isdecimal(),"exact inherited runtime lease descriptor required")
    fd=int(raw_fd);actual=os.fstat(fd);declared=receipt["lock_identity"]
    need(fd==receipt["lock_fd"] and (actual.st_dev,actual.st_ino)==(declared["dev"],declared["ino"])
        and Path("/proc/self/fd"+"/"+str(fd)).resolve(strict=True)==LOCK,"inherited lease FD/inode differs")
    fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    return receipt


def verify_project_modules(view, modules, extra=()):
    for module in (*tuple(modules), *extra):
        name, filename = getattr(module, "__name__", ""), getattr(module, "__file__", None)
        if name == "_hh_owned_dds_info_261007":
            # HH_261007 - Admit only the actual privately built, probe-verified binary, never an unsealed local extension.
            bridge = view["native_dds_bridge"]
            need(filename == bridge["module_path"] and digest(filename) == bridge["module_sha256"], "private native DDS binary/source origin differs")
            continue
        if name in ("scripts","scripts.e2e") and filename is None:
            # HH_261007 - The actual canonical bootstrap legitimately creates these TWO exact namespaces, never generic nil modules.
            expected=ROOT/"tools" if name=="scripts" else ROOT/"tools/e2e"
            need(list(module.__path__) == [str(expected)], "foreign virtual canonical namespace: "+name)
            continue
        if name.startswith(("autoware_planning_msgs", "autoware_vehicle_msgs")):
            need(type(filename) is str, "actual ROS generated module origin required")
            path=Path(filename)
            package=name.split(".")[0]
            need(path.is_relative_to(ROOT/"autonomy/install"/package) or path.is_relative_to(ROOT/"autonomy/build"/package), "foreign Autoware overlay denied")
            actual=path.resolve(strict=True)
            need(str(actual) in view["generated_ROS_sources"] and digest(actual)==view["generated_ROS_sources"][str(actual)], "generated ROS source/ABI seal differs")
            continue
        if not filename:
            need(not (name in ("portable_e2e","canonical_workspace","workspace_python") or name.startswith(("portable_e2e.","scripts."))), "unsealed virtual project module: "+name)
            continue
        path = Path(filename)
        if not path.is_absolute() or not path.is_relative_to(ROOT): continue
        relative = str(path.relative_to(ROOT))
        need(relative in view["source_sha256"] and digest(path) == view["source_sha256"][relative], "unsealed imported project source")


def authorize(plan_sha, authority_sha):
    plan = bounded_json(PLAN, plan_sha, PLAN_LIMIT)
    grant = bounded_json(AUTHORITY, authority_sha)
    need(plan["schema"] == SCHEMA and plan["fixed"] == fixed_contract(), "exact fresh recorded protocol required")
    need(grant == dict(schema=SCHEMA+".ROOT_authority", plan_path=str(PLAN), plan_sha256=plan_sha,
        source_sha256=plan["source_sha256"], recorded_CPU_ROS_only=True, training_authorized=False,
        GPU_authorized=False, CARLA_authorized=False, actuation_authorized=False, TEST_authorized=False, model_promoted=False),
        "externally SHA-pinned ROOT authority required; no self-admission flag")
    need(plan["source_sha256"] == source_pins(), "whole project source closure drift")
    for name, pin in plan["source_sha256"].items():
        need(digest(PLAN_DIR/"sources"/name) == pin, "archived/current source closure differs")
    for path, pin in plan["input_sha256"].items(): need(digest(path) == pin, "whole recorded/model/rig input changed")
    for path, pin in plan["setup_sha256"].items(): need(digest(path) == pin, "clean canonical setup source changed")
    for path, pin in plan["generated_ROS_sources"].items(): need(digest(path)==pin,"canonical generated ROS message/ABI input changed")
    need(os.environ.get("CUDA_VISIBLE_DEVICES") == "" and os.environ.get("PYTHONNOUSERSITE") == "1", "clean CPU-only environment required")
    need(os.environ.get("ROS_LOG_DIR")==str(OUTPUT/"ros_logs") and os.environ.get("OMP_NUM_THREADS")=="4"
        and os.environ.get("MKL_NUM_THREADS")=="4" and os.environ.get("OPENBLAS_NUM_THREADS")=="1", "bounded clean ROS log/CPU thread environment required")
    bundle = frozen_bundle(); bundle.verify_ros_environment(plan)
    view = deepcopy(plan); view["admitted_authority_sha256"] = authority_sha
    view["plan_sha256"] = plan_sha
    view["source_sha256"] = plan["source_sha256"]
    return view


def fixed_contract():
    return dict(camera_anchors=918, native_states=1836, direct_then_DDS_no_simultaneous_NN=True,
        ROS_DOMAIN_ID=93, ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
        CPU_threads=4, interop_threads=1, physical_GPU_used=False, policy_deadline_seconds=.1,
        TF_limits=dict(translation_m=.005, rotation_rad=.005), raw_atol=1e-6, raw_rtol=1e-6,
        whole_input_tensor_hashes=True, matching_valid_contexts_only_raw_and_trajectory=True,
        all_failure_prefill_reset_and_noncomparable_rows_retained=True, original_V5_terminal_equivalence=False,
        positive_published_trajectory_context_required_for_overall_parity=True,
        native_message_publisher_GID_required=True, private_executor_only_owned_subscriptions=True,
        publisher_gid_normalization="public_dds_matched_publication_data_for_actual_taken_handle",
        actual_raw_RMW_GID_and_publication_handle_retained=True, installed_Cyclone_RMW_plugin_sealed=True,
        installed_ROS_sources_modified=False, project_native_bridge_actuation_approved=False,
        actual_publication_enter_and_finish_rechecked_100ms=True, actual_ACK_postpublish_completion_retained=True,
        subscriber_numerical_audit_materialization="after_trajectory_path_before_ACK_wire_serialization",
        direct_numerical_audit_unchanged=True,subscriber_exactly_one_pending_audit=True,
        tensor_identity_and_inplace_version_before_after_materialization=True,
        ACK_completion_includes_deferred_audit_cost=True,model_forward_inputs_weights_policy_changed=False,
        subscriber_full_graph_census="provenance_before_use_and_every_accept_before_original_tensor_NN_and_each_output_publish",
        every_message_peer_PID_true_native_GID_type_CDR_checks_unchanged=True,
        subscriber_pre_accept_exactly_one_active_source_owned_shadow_node=True,
        destroyed_startup_probe_may_not_guard_NN=True,direct_graph_guard_and_audit_unchanged=True,
        per_message_full_graph_census_claimed=False,DDS_cryptographic_security_claimed=False,
        subscriber_local_stage_measurement_only=True,direct_numerical_and_graph_AST_unchanged=True,
        receive_stage_names=["peer","provenance_graph","publisher_gid","native_witness","CDR","CDR_hash","callback"],
        native_stage_names=["native_C_take_and_matched_endpoint_lookup","native_take_tuple_validation","native_execute_tuple_validation"],
        native_timing_exact_subscription_message_info_callback_identity=True,native_metadata_fields_modified=False,
        native_pending_bound=32,receive_pending_bound=64,native_failure_bound=8,overflow_events_retained=True,
        empty_native_takes_profiled=False,no_per_callback_disk_append=True,no_additional_NN_for_timing=True,
        ACK_completion_includes_timing_copy_compaction_and_encoding=True,missing_input_timings_padded=False,
        pure_network_latency_or_stage_sum_equals_wall_claimed=False,
        subscriber_private_executor_membership="once_per_exact_live_OwnedNode_then_public_executor_None_detach",
        normalized_history_intervention="only_past_cache_hit_clone_omitted_private_original_builder_current_and_miss_clones_final_stack_preserved",
        subscriber_persistent_membership=SUBSCRIBER_PERSISTENT_MEMBERSHIP,
        startup_probe_executor_public_shutdown_then_fresh_actual_executor=True,
        all_owned_nodes_including_publisher_outer_None_detached_on_close=True,
        foreign_executor_duplicate_membership_destroyed_node_denied=True,
        direct_publisher_spin_membership_unchanged=True,executor_membership_latency_gain_measured=False,
        logging_protocol=codec().prospective_capacity(),all_original_JSONL_and_console_bytes_losslessly_reconstructible_when_complete=True,
        compression_at_original_append_and_print_calls=True,original_ACK_clock_includes_later_journal_or_console=False,
        separate_original_console_return_witness=True,witness_binding_and_closing_serialization_excluded_from_anchor_clock=True,
        failed_oversized_generic_print_full_bytes_claimed=False,
        original_BGRA_equivalence=False, wall_10Hz_or_P3_approved=False, automatic_retry=False,
        floor_bytes=FLOOR, shared_bytes=SHARED, quota_bytes=QUOTA, closing_bytes=CLOSING,
        file_limit_bytes=FILE_LIMIT, total_deadline_seconds=1800, per_role_deadline_seconds=900,
        startup_seconds=60, receipt_seconds=5, graph_empty_settle_seconds=.5,
        ROS_LOG_DIR=str(OUTPUT/"ros_logs"), inherited_runtime_lease_required=True, Linux_parent_death_SIGTERM=True,
        runtime_lock=str(LOCK), output=str(OUTPUT), CPU_policy_changed_from_historical_V5_claim=False)


def worker_command(mode, plan_sha, authority_sha):
    return ["/usr/bin/python3", "-B", str(ROOT/"tools/workspace_python.py"), "-m", "scripts.e2e.temporal_recorded_ros_shadow_worker_20261008_v16", "--",
        "--mode", mode, "--plan-sha256", plan_sha, "--authority-sha256", authority_sha]


def pure_proxy(view):
    old = frozen_bundle()
    result = SimpleNamespace(**vars(old))
    result.verify_imported_project_modules = verify_project_modules
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("direct", "subscriber", "publisher"), required=True)
    parser.add_argument("--plan-sha256", required=True); parser.add_argument("--authority-sha256", required=True)
    args = parser.parse_args(argv)
    resource.setrlimit(resource.RLIMIT_FSIZE,(FILE_LIMIT,FILE_LIMIT));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    import ctypes
    need(ctypes.CDLL(None,use_errno=True).prctl(1,signal.SIGTERM,0,0,0)==0,"Linux parent-death guard required")
    def interrupted(number,frame):raise SystemExit("owned worker interrupted: "+str(number))
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGHUP,interrupted)
    result = dict(schema=SCHEMA+".role_result", mode=args.mode, status="FAILED_RETAINED", expected=918, completed=0, UNEXECUTED=918,
        failing_bundle_may_have_executed=True, error=None, P3_approved=False)
    try:
        view = authorize(args.plan_sha256, args.authority_sha256); budget()
        owner = live_owner()
        actual = process_identity(os.getpid())
        need(actual["argv"] == worker_command(args.mode, args.plan_sha256, args.authority_sha256)
            and actual["ppid"] == owner["identity"]["pid"] and actual["pgid"] == actual["session"] == actual["pid"], "exact owned child session/argv required")
        new_json(OUTPUT/(args.mode+"_started.json"), dict(identity=actual, plan_sha256=args.plan_sha256,
            authority_sha256=args.authority_sha256, mode=args.mode))
        # HH_261007 - Only this admitted branch imports the actual ROS/model implementation.
        implementation = load_module(NODE, "_hh_owned_recorded_ros_runtime_261007")
        result = implementation.run(args.mode, view)
    except BaseException as error:
        result["error"] = repr(error)
        path = OUTPUT/(args.mode+"_anchors.jsonl")
        try:
            result["completed"]=count_records(path);result["UNEXECUTED"]=918-result["completed"]
        except BaseException as failure:result["prefix_count_unknown_error"]=repr(failure)
    # HH_261008 - A failure before node.run returns still closes this module's own codec registry, without replacing a node-owned receipt.
    closing_path=OUTPUT/(args.mode+"_codec_closing.json")
    if not closing_path.exists():
        try:
            closing=close_ledgers();budget(additional=len(encode(closing)),closing=True);new_json(closing_path,closing)
            if closing["codec_failure"]:result["status"]="FAILED_RETAINED";result["codec_failure"]=True
        except BaseException as failure:result["codec_closing_error"]=repr(failure);result["status"]="FAILED_RETAINED"
    new_json(OUTPUT/(args.mode+"_result.json"), result)
    print(json.dumps(dict(schema=SCHEMA+".coded_console_pointer",mode=args.mode,status=result["status"],
        original_console_ledger=str(OUTPUT/(args.mode+".stdout.lossless")) if args.mode!="direct" else None,
        expected=918,completed=result["completed"],UNEXECUTED=result["UNEXECUTED"]),sort_keys=True),flush=True)
    return 0 if result["status"] == "COMPLETE_RECORDED_ROLE_NOT_FEATURE_APPROVAL" else 1


if __name__ == "__main__": raise SystemExit(main())
