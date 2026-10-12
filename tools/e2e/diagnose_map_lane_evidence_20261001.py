"""HH_261001 - Post-hoc accepted-lane evidence accounting from six frozen final numeric grids only."""

import argparse
from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / 'docs/assets/validation/2026-10-01/map_geometry_full_projection'
RUN = DOCS / 'run_v1'
SUMMARY = RUN / 'summary.json'
SUMMARY_SHA = 'b848700f12f740d1afb424187e4e5ef8fe3eb8af6819629b625da03da9397fd1'
OUTPUT = DOCS / 'lane_evidence_diagnosis_v1'
TEST = ROOT / 'learning/tests/test_map_lane_evidence_diagnosis_20261001.py'
ROUTES = ('ctrack_left', 'ctrack_straight')
ARMS = ('control', 'geometry')
LABELS = ('road', 'sidewalk', 'lane_marking', 'traffic_light_fixture', 'traffic_sign', 'pole')
LANE = 2
EXPECTED_SUPPORT = {'ctrack_left': 1375, 'ctrack_straight': 1172}
FLOOR, LIMIT = 5 * 1024**3, 1024**2
SOURCES = {
    ROOT / 'learning/portable_e2e/mapping.py': '264f9dda759f6986bde9c614e5cf3ae21680a22ae58d855db11564ceb3b3b867',
    ROOT / 'learning/portable_e2e/mapping_projection.py': 'c42ab7d20454475f544b8073d6a23a104a0a2bb1cc2cbb7be092fa3e871cbbee',
    ROOT / 'tools/e2e/project_map_geometry_full_20261001.py': '079f163a95d130e3aa0a1ac4ed209242ace86e85009bcec5070d14f3b5e33c3f',
    ROOT / 'tools/e2e/project_map_photometric_fixed_20261001.py': 'a229abee7fc4a62bfe37e738ffdf212174b28c3a70f2e4e5d9fade47ee9a0bf3',
}
GROUPS = ('a_no_learned_observation', 'b_observed_without_accepted_lane_evidence',
          'c_accepted_lane_evidence_loses', 'd_lane_dominant')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    require(path.is_file() and not path.is_symlink(), 'regular frozen file required: ' + str(path))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def object_sha(value):
    return hashlib.sha256(encoded(value)).hexdigest()


@dataclass(frozen=True)
class Cell:
    order: tuple
    frames: tuple
    confidence: tuple

    @property
    def dominant(self):
        # HH_261001 - Python max keeps the first insertion-order label on an exact confidence-sum tie.
        return max(self.order, key=lambda index: self.confidence[index])


def unpack(arrays):
    require(set(arrays) == {'cell_xy', 'label_order', 'observed_frames', 'confidence_sum'}, 'exact numeric grid keys required')
    xy, order, counts, sums = (arrays[k] for k in ('cell_xy', 'label_order', 'observed_frames', 'confidence_sum'))
    n = len(xy)
    require(n <= 200000 and xy.shape == (n, 2) and xy.dtype == np.int64
        and order.shape == counts.shape == sums.shape == (n, 6)
        and order.dtype == np.int8 and counts.dtype == np.int16 and sums.dtype == np.float64
        and np.all((order >= -1) & (order < 6)) and np.all((counts >= 0) & (counts <= 240))
        and np.isfinite(sums).all() and np.all((sums >= 0) & (sums <= counts)), 'fixed full240 numeric grid ABI differs')
    result, last = {}, None
    for key, row_order, number, weight in zip(xy, order, counts, sums):
        key = tuple(map(int, key))
        entries = tuple(int(i) for i in row_order if i >= 0)
        require((last is None or key > last) and entries and len(entries) == len(set(entries))
            and set(entries) == set(np.nonzero(number)[0]) and np.all(weight[number == 0] == 0)
            and np.all(weight[number > 0] > 0), 'ordered unique cells, label insertion order or evidence positivity differs')
        result[key] = Cell(entries, tuple(map(int, number)), tuple(map(float, weight)))
        last = key
    return result


def snapshot(grid, meta):
    require(meta['frame_id'] == 'carla_ue:C_track_1_0_7' and meta['source'] in ('oracle_sensor', 'learned_prediction')
        and math.isfinite(meta['timestamp_s']) and meta['unobserved_is_unknown'] is True
        and meta['hd_map_approved'] is False and meta['cells'] == len(grid), 'final grid metadata differs')
    return dict(schema='portable_e2e.semantic_evidence.v1', frame_id=meta['frame_id'], source=meta['source'],
        timestamp_s=meta['timestamp_s'], resolution_m=.2, unobserved_is_unknown=True, hd_map_approved=False,
        cells=[dict(ix=key[0], iy=key[1], labels={LABELS[i]: dict(observed_frames=value.frames[i],
            confidence_sum=value.confidence[i]) for i in value.order}) for key, value in grid.items()],
        current_dynamic=[])


def membership(keys):
    keys = sorted(keys)
    return dict(count=len(keys), sorted_cell_xy_sha256=object_sha(keys))


def partition(oracle, learned):
    """HH_261001 - Evidence means accepted projected per-frame cell/class votes, not raw image probability."""
    lane_cells = {key for key, row in oracle.items() if row.dominant == LANE}
    groups = {name: set() for name in GROUPS}
    for key in lane_cells:
        value = learned.get(key)
        if value is None:
            group = GROUPS[0]
        elif value.frames[LANE] == 0:
            require(value.confidence[LANE] == 0, 'zero lane frames must have zero lane sum')
            group = GROUPS[1]
        elif value.dominant != LANE:
            group = GROUPS[2]
        else:
            group = GROUPS[3]
        groups[group].add(key)
    require(set().union(*groups.values()) == lane_cells
        and sum(map(len, groups.values())) == len(lane_cells), 'four-way partition is not exhaustive and exclusive')
    b, c, d = (groups[name] for name in GROUPS[1:])
    lane_ever = {key for key, row in learned.items() if row.frames[LANE] > 0}
    lane_wins = {key for key, row in learned.items() if row.dominant == LANE}
    fp_ever, fp_current = lane_ever - lane_cells, lane_wins - lane_cells
    no_lane = groups[GROUPS[0]] | b
    missed = lane_cells - d
    tied_losses = {key for key in c if learned[key].confidence[LANE] == learned[key].confidence[learned[key].dominant]}
    nonlane_oracle_labels = Counter('unknown_no_oracle_cell' if key not in oracle else LABELS[oracle[key].dominant]
                                  for key in fp_ever)
    oracle_secondary = {key for key in fp_ever if key in oracle and oracle[key].frames[LANE] > 0}
    return dict(oracle_dominant_lane_cells=len(lane_cells),
        four_way={name: dict(membership(keys), denominator=len(lane_cells),
                            fraction=len(keys) / len(lane_cells) if lane_cells else None) for name, keys in groups.items()},
        counts_sum=sum(map(len, groups.values())),
        accepted_lane_evidence_missing=dict(numerator=len(no_lane), denominator=len(lane_cells),
            fraction=len(no_lane) / len(lane_cells) if lane_cells else None,
            among_current_lane_misses=dict(numerator=len(no_lane), denominator=len(missed),
                fraction=len(no_lane) / len(missed) if missed else None)),
        accepted_lane_evidence_losing_to=dict(Counter(LABELS[learned[key].dominant] for key in c)),
        observed_without_lane_dominant_labels=dict(Counter(LABELS[learned[key].dominant] for key in b)),
        exact_confidence_sum_tied_lane_losses=membership(tied_losses),
        any_accepted_lane_presence_set_diagnostic=dict(
            this_is_not_a_changed_fusion_or_threshold_experiment=True,
            current_dominant=dict(lane_cells=len(lane_wins), true_positive_cells=len(d),
                false_positive_cells=len(fp_current),
                precision_vs_oracle_dominant=len(d) / len(lane_wins) if lane_wins else None),
            if_every_lane_evidence_cell_were_labeled_lane=dict(lane_cells=len(lane_ever),
                true_positive_cells=len(lane_ever & lane_cells), false_positive_cells=len(fp_ever),
                precision_vs_oracle_dominant=len(lane_ever & lane_cells) / len(lane_ever) if lane_ever else None,
                additional_true_positive_cells=len(c), additional_false_positive_cells=len(fp_ever - fp_current)),
            false_positive_oracle_dominant_labels=dict(nonlane_oracle_labels),
            false_positive_cells_with_secondary_oracle_lane_evidence=len(oracle_secondary),
            false_positive_cells_without_any_oracle_lane_evidence=len(fp_ever - oracle_secondary),
            interpretation='False positives are relative to oracle-dominant 2D cell labels, not proof of nonexistent physical markings.'))


def analyze():
    require(sha(SUMMARY) == SUMMARY_SHA, 'caller-released completed projection summary differs')
    summary = json.loads(SUMMARY.read_text())
    require(summary['status'] == 'FULL_RECORDING_ORACLE_ASSISTED_ACCUMULATION_DIAGNOSTIC'
        and summary['source_and_inputs_unchanged'] is True and summary['completed_frame_count'] == 480
        and summary['policy']['frames_per_route'] == 240 and summary['policy']['minimum_confidence'] == .5
        and summary['policy']['stride'] == 3 and summary['policy']['depth_strict_range_m'] == [.1, 40]
        and summary['policy']['resolution_m'] == .2 and summary['policy']['ground_height_filter'] is None
        and summary['policy']['oracle_depth'] is True and summary['policy']['oracle_pose'] is True
        and summary['policy']['label_table'] == {str(i + 1): name for i, name in enumerate(LABELS)},
        'fixed full-recording projection contract differs')
    ledger = {str(SUMMARY): SUMMARY_SHA}
    for path, expected in SOURCES.items():
        require(summary['sources'][str(path)] == expected == sha(path), 'unchanged projector/fusion semantics required')
        ledger[str(path)] = expected
    result = {}
    for route in ROUTES:
        grids = {}
        route_summary = summary['scenarios'][route]
        for arm in ('oracle',) + ARMS:
            meta = route_summary['grids'][arm]
            file = f'{route}_{arm}_grid.npz'
            require(meta['file'] == file, 'fixed final-grid filename required')
            path = RUN / file
            expected = summary['artifact_sha256'][file]
            require(sha(path) == expected, 'frozen final-grid bytes differ')
            ledger[str(path)] = expected
            with np.load(path, allow_pickle=False) as archive:
                arrays = {key: archive[key].copy() for key in archive.files}
            grids[arm] = unpack(arrays)
            require(object_sha(snapshot(grids[arm], meta)) == meta['snapshot_sha256'], 'restored final-grid snapshot digest differs')
            require(meta['source'] == ('oracle_sensor' if arm == 'oracle' else 'learned_prediction'), 'grid source role differs')
        result[route] = {}
        for arm in ARMS:
            item = partition(grids['oracle'], grids[arm])
            existing = route_summary['comparisons'][arm]['classes']['lane_marking']
            current = item['any_accepted_lane_presence_set_diagnostic']['current_dominant']
            require(item['oracle_dominant_lane_cells'] == EXPECTED_SUPPORT[route] == existing['oracle_cells']
                and current['true_positive_cells'] == existing['true_positive_cells']
                and current['false_positive_cells'] == existing['false_positive_cells']
                and current['lane_cells'] == existing['learned_cells'], 'independent lane denominators/TP/FP differ from completed report')
            result[route][arm] = item
    require(all(sha(Path(path)) == expected for path, expected in ledger.items()), 'source/evidence changed during diagnosis')
    return dict(schema='portable_e2e.map_lane_accepted_evidence_diagnosis.v1',
        status='POST_HOC_FROZEN_GRID_ACCOUNTING_ONLY', input_sha256=ledger, results=result,
        source_semantics=dict(dominant='max accumulated confidence_sum; exact ties retain first label insertion order from label_order',
            frame_aggregation='maximum positive confidence per frame/cell/class; one frame vote per class, then sums across frames',
            observed_frames_is_not_the_dominant_key=True, tie_uses_class_id=False,
            observed_scope='accepted projected labels only: argmax class, confidence>=0.5, stride3, strict0.1<depth<40m,0.2m grid'),
        interpretation='Missing accepted lane evidence is the largest accounting category; final grids cannot identify the upstream raw-image, confidence, sampling or projection cause.',
        limitations=['No causal proof; zero accepted lane evidence does not prove a raw image perception miss or zero network lane probability.',
            'Depth and pose are oracle simulator inputs; no ground-height filter was used and 2D cells can mix surfaces over time.',
            'Any-lane-presence set counts illustrate rare-label promotion risk, not a new fusion run, threshold recommendation or accuracy guarantee.',
            'Oracle-dominant non-lane cells can contain secondary oracle lane evidence; FP counts are relative to this fixed reference.',
            'Original LEFT owner FAIL and unavailable original live replay remain unchanged; this is exposed external diagnostic evidence, not TRAIN eligibility.'],
        loaded_final_numeric_grids=6, neural_forwards=0, training_steps=0,
        changed_threshold=False, changed_fusion=False, gpu_used=False, simulator_launched=False, hd_map_approved=False)


def findings(result):
    lines = ['# HH_261001 — 차선 셀 증거 누락/누적 투표 분해', '',
        '완료된 최종 수치 격자6개만 읽은 사후 진단입니다. 새 추론·GPU·시뮬레이터·임계값/융합 변경은 없습니다.', '',
        '| 경로 / 모델 | 기준 차선 셀 | (a) 학습 관측 없음 | (b) 관측 있으나 채택된 차선 증거 없음 | (c) 차선 증거 있으나 우세 라벨에 패함 | (d) 차선 우세 |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    for route in ROUTES:
        for arm in ARMS:
            row = result['results'][route][arm]
            values = [row['four_way'][name]['count'] for name in GROUPS]
            lines.append('| ' + route + ' / ' + arm + ' | ' + str(row['oracle_dominant_lane_cells']) + ' | ' + ' | '.join(map(str, values)) + ' |')
    lines += ['', '각 행의(a)+(b)+(c)+(d)는 분모와 정확히 같습니다. 가장 큰 항목은(b)입니다. 현재 차선 누락 중(a)+(b)의 비율:', '']
    for route in ROUTES:
        for arm in ARMS:
            row = result['results'][route][arm]['accepted_lane_evidence_missing']['among_current_lane_misses']
            lines.append(f"- {route} / {arm}: {row['numerator']}/{row['denominator']} = {row['fraction']:.2%}.")
    lines += ['', '따라서 **이미 채택된 차선 증거가 투표에서 밀리는 현상보다, 해당 기준 셀에 채택된 차선 증거가 없는 경우가 대부분**입니다. '
        '이는 원본 영상의 인식 실패를 확정하지 않습니다. confidence≥0.5, stride3, 깊이0.1~40m의 엄격한 범위와0.2m 양자화가 모두 개입합니다. '
        '최종 격자만으로 분류·낮은 확신·샘플링·투영 위치 중 원인을 분리할 수 없습니다.', '',
        '우세 라벨은 관측 프레임 수가 아닌 confidence_sum의 최댓값입니다. 정확한 동률이면 label_order에 저장된 최초 삽입 라벨이 이깁니다. '
        '이 네 비교에서 차선이 패한 confidence_sum 동률은 모두0개입니다.', '',
        '## 희귀 라벨을 한 번이라도 관측하면 승격하는 경우의 위험', '',
        '격자를 바꾸지 않고 존재 집합만 센 가정입니다. 기준은 동일한 oracle 우세 라벨이며 물리적 차선의 절대 진실이 아닙니다.', '',
        '| 경로 / 모델 | 현재 TP / FP | 차선 증거 있는 셀 전부를 차선으로 부를 때 TP / FP | 추가 TP / 추가 FP | FP 중 oracle 보조 차선 증거 있음 / 전혀 없음 |',
        '| --- | ---: | ---: | ---: | ---: |']
    for route in ROUTES:
        for arm in ARMS:
            row = result['results'][route][arm]['any_accepted_lane_presence_set_diagnostic']
            a, b = row['current_dominant'], row['if_every_lane_evidence_cell_were_labeled_lane']
            lines.append(f"| {route} / {arm} | {a['true_positive_cells']} / {a['false_positive_cells']} | "
                f"{b['true_positive_cells']} / {b['false_positive_cells']} | {b['additional_true_positive_cells']} / {b['additional_false_positive_cells']} | "
                f"{row['false_positive_cells_with_secondary_oracle_lane_evidence']} / {row['false_positive_cells_without_any_oracle_lane_evidence']} |")
    lines += ['', '단순 희귀 라벨 승격은 위 표처럼 추가 TP보다 많은 기준 FP를 늘릴 수 있습니다. 일부 FP 셀에는 oracle 보조 차선 증거도 있으므로 '
        '모두 허구 차선이라고 단정할 수는 없습니다. 이 수치로 새 임계값·융합 정책을 선택하거나 지도 승인을 주장하지 않습니다.', '',
        '입력 요약·6개NPZ·원래 융합/투영 소스 SHA256, 복원된 snapshot 일치, 각 범주 셀 좌표 집합 해시와 분자/분모는 result.json에 있습니다. '
        'LEFT의 원래 owner FAIL 및 원래 live replay 불가는 그대로 보존됩니다.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--write', action='store_true', help='write new separate diagnosis; default only checks and prints counts')
    args = parser.parse_args()
    result = analyze()
    if args.write:
        require(not OUTPUT.exists() and not OUTPUT.is_symlink(), 'fresh separate post-hoc output required')
        require(shutil.disk_usage(ROOT).free >= FLOOR + LIMIT, 'preserve5GiB floor plus1MiB diagnostic reserve')
        result['diagnostic_source_sha256'] = {str(Path(__file__)): sha(Path(__file__)), str(TEST): sha(TEST)}
        raw = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + '\n'
        text = findings(result)
        require(len(raw.encode()) + len(text.encode()) + Path(__file__).stat().st_size + TEST.stat().st_size < LIMIT,
                'new diagnosis exceeds bounded1MiB budget')
        OUTPUT.mkdir()
        for name, value in (('result.json', raw), ('DIAGNOSIS.md', text)):
            with (OUTPUT / name).open('x') as stream:
                stream.write(value)
        require(shutil.disk_usage(ROOT).free >= FLOOR, '5GiB free-space floor changed during write')
    print(json.dumps(dict(status=result['status'], counts={route: {arm: {
        name: result['results'][route][arm]['four_way'][name]['count'] for name in GROUPS}
        for arm in ARMS} for route in ROUTES}, output=str(OUTPUT) if args.write else None), indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
