"""HH_261001 - Plot frozen offline records; preserve historical native JPEG bytes without inference."""

import argparse
import hashlib
import html
import json
import math
from pathlib import Path
import shutil

from PIL import Image, ImageDraw, ImageFont, PngImagePlugin

ROOT = Path(__file__).resolve().parents[2]
REPLAY = ROOT / 'docs/assets/validation/2026-10-01/warmup_supervision_ab/directional_gradient_fixed149_v1'
TRIAL = ROOT / 'storage/experiments/validation/2026-10-01/warmup_supervision_W1_town07_long_v1'
EPISODE = TRIAL / 'episode.partial'
OUTPUT = REPLAY.with_name('directional_gradient_visual_evidence_v1')
TEST = ROOT / 'learning/tests/test_plot_directional_fixed149_evidence_20261001.py'
FONT = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
CASES = (3, 117)
CAMERAS = ('CAM_FRONT', 'CAM_BACK', 'CAM_FRONT_LEFT', 'CAM_BACK_LEFT', 'CAM_FRONT_RIGHT', 'CAM_BACK_RIGHT')
MAX_BYTES, FLOOR = 4 * 1024**2, 5 * 1024**3
COLORS = {'A': '#1769aa', 'B': '#d35e00'}
PINS = {
    REPLAY / 'records.jsonl': '6d8d6a405d6b67df9731fde7a940ba20152147fe6db2c2a2a0ff5f33ec6d5ec5',
    REPLAY / 'summary.json': '84a6e83414d96cfdc52d78260195b0fd3d44711d5eb7c1dfde013303d55f9268',
    EPISODE / 'camera_frames.jsonl': 'f68ed8c93edaaa6dffa22fc2f577a3157d820901a9de95a759568ac0a77c2269',
    EPISODE / 'states.jsonl': 'c3d92ae3ec97ab2914236093275d950356afb04f81475ac4a91e463764397058',
    EPISODE / 'learned_control.jsonl': '3c50214fa5da16898221a80ce6208cce89612060c6618174eaa69a02c9895105',
    EPISODE / 'manifest.json': '0a6359f9f4223051fe51c65c51f25922d066bd1df08765e5c2fcf41eff371ce5',
    TRIAL / 'sampled_exact_runtime_inputs/index.jsonl': '951fd7b4f79e6da8e12e5f4af5c82ebcba5b1e1e6dc067e568c23af2d62db8bb',
    ROOT / 'learning/portable_e2e/runtime_contract.py': '38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3',
    FONT: '690243adfefe0ce154b547db6205794bd30ac4277275179517a90994f4980648',
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    require(path.is_file() and not path.is_symlink(), 'missing/unsafe evidence file: ' + str(path))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def jsha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def arrays(records):
    require(len(records) == 149 and [r['sequence'] for r in records] == list(range(149)), 'full fixed149 records required')
    overview = {'sequence': list(range(149))}
    for arm in ('A', 'B'):
        overview[arm] = dict(preview_mps=[r[arm]['raw_controller_preview_speed_mps'] for r in records],
            hold_rule_proposal=[r[arm]['tracker_proposal']['desired_speed_mps'] <= .05 for r in records])
    cases = {}
    for seq in CASES:
        row = records[seq]
        require(row['A_bit_exact_to_historical_W1'] and row['new_A_or_B_control_ACK'] is False,
                'explanatory case is not exact historical-A/offline-only evidence')
        case = dict(sequence=seq, frame=row['historical_W1_inference_frame'], timestamp_ns=row['timestamp_ns'],
            time_s=[(i + 1) * .1 for i in range(64)], historical_progress_m=row['historical_W1_progress_m'])
        for arm in ('A', 'B'):
            value = row[arm]
            require(len(value['selected_xy_m']) == len(value['selected_speed_mps']) == 64
                and value['raw_controller_preview_speed_mps'] == value['selected_speed_mps'][2]
                and value['counterfactual_control_acknowledged'] is False, 'fixed64/0.3s/offline arrays differ')
            require(all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in value['selected_xy_m'])
                and all(math.isfinite(v) and 0 <= v <= 8.5 for v in value['selected_speed_mps']), 'finite bounded plot values required')
            case[arm] = dict(xy_m=value['selected_xy_m'], speed_mps=value['selected_speed_mps'],
                preview_mps=value['raw_controller_preview_speed_mps'], raw_argmax=value['raw_argmax'],
                tracker_proposal=value['tracker_proposal'])
        cases[str(seq)] = case
    return dict(overview=overview, cases=cases, timestep_s=.1,
        xy_axes=dict(forward_min_m=-40, forward_max_m=40, right_min_m=-40, right_max_m=40,
                     equal_meter_scale=True, transformation='plot right=-ego_y, plot forward=ego_x'),
        speed_axes=dict(time_min_s=0, time_max_s=6.4, speed_min_mps=0, speed_max_mps=8.5))


def associate(row, camera_rows, states, index, control_rows):
    frame, stamp = row['historical_W1_inference_frame'], row['timestamp_ns']
    candidates = [r for r in camera_rows if r['frame'] == frame]
    state_matches = [r for r in states if r['frame'] == frame]
    require(len(candidates) == len(state_matches) == 1, 'exact unique native frame association unavailable; no nearest-frame fallback')
    camera, state = candidates[0], state_matches[0]
    require(round(camera['timestamp'] * 1e9) == round(state['timestamp'] * 1e9) == stamp
        and tuple(camera['camera_order']) == CAMERAS and set(camera['images']) == set(CAMERAS)
        and camera['timestamp_span_sec'] == 0
        and all(round(camera['source_timestamps'][cam] * 1e9) == stamp for cam in CAMERAS)
        and state['route_progress_m'] == row['historical_W1_progress_m'], 'native image/state timestamps or camera order differ')
    entry = index[row['sequence']]
    require(entry['sequence'] == row['sequence'] and entry['tensor_file_sha256'] == row['tensor_file_sha256']
        and entry['tensor_file'] == row['tensor_file'] and entry['timing']['now_ns'] == stamp
        and entry['timing']['camera_stamps_ns'] == [stamp] * 6, 'exact archived input/frame association differs')
    controls = [r for r in control_rows if r.get('inference_frame') == frame
        and not r.get('held_between_camera_frames') and r.get('raw_output') is not None]
    require(len(controls) == 1 and controls[0]['source_checkpoint_sha256'] == entry['source_checkpoint_sha256']
        and jsha(controls[0]['raw_output']) == row['A']['raw_output_sha256'] == entry['raw_output_sha256'],
        'historical W1 raw-output/frame binding differs')
    return dict(frame=frame, timestamp_ns=stamp, camera_record=camera, state_record_sha256=jsha(state),
        historical_control_record_sha256=jsha(controls[0]), archive_index_record_sha256=jsha(entry),
        tensor_file=row['tensor_file'], tensor_file_sha256=row['tensor_file_sha256'],
        source_camera_frame_record_sha256=jsha(camera), exact_association=True)


def load():
    require(all(sha(path) == value for path, value in PINS.items()), 'frozen source evidence changed')
    records = rows(REPLAY / 'records.jsonl')
    data = arrays(records)
    index = rows(TRIAL / 'sampled_exact_runtime_inputs/index.jsonl')
    camera_rows, states, controls = (rows(EPISODE / name) for name in
        ('camera_frames.jsonl', 'states.jsonl', 'learned_control.jsonl'))
    associations, image_sources = {}, {}
    for seq in CASES:
        associations[str(seq)] = associate(records[seq], camera_rows, states, index, controls)
        relative_tensor = Path(records[seq]['tensor_file'])
        require(relative_tensor.name == str(relative_tensor), 'unsafe tensor reference')
        tensor = TRIAL / 'sampled_exact_runtime_inputs' / relative_tensor
        require(sha(tensor) == records[seq]['tensor_file_sha256'], 'actual recorded input bytes differ')
        image_sources[str(tensor)] = sha(tensor)
        for camera, relative in associations[str(seq)]['camera_record']['images'].items():
            path = EPISODE / relative
            require(not Path(relative).is_absolute() and '..' not in Path(relative).parts
                and path.resolve().is_relative_to(EPISODE.resolve()), 'unsafe native camera reference')
            image_sources[str(path)] = sha(path)
            with Image.open(path) as image:
                require(image.format == 'JPEG' and image.size == (640, 360), 'actual native640x360 JPEG required; no resize fallback')
    return records, data, associations, image_sources


def canvas(title):
    image = Image.new('RGB', (1200, 740), 'white')
    draw = ImageDraw.Draw(image)
    draw.text((24, 16), title, fill='#152536', font=ImageFont.truetype(str(FONT), 23))
    draw.text((24, 52), 'OFFLINE on historical W1 inputs | No new A/B driving, sensor capture or control ACK',
              fill='#9a2727', font=ImageFont.truetype(str(FONT), 16))
    return image, draw


def text(draw, position, value, color='#253747', size=15):
    draw.text(position, value, fill=color, font=ImageFont.truetype(str(FONT), size))


def transform(rect, bounds, point):
    x, y, width, height = rect
    xmin, xmax, ymin, ymax = bounds
    px, py = point
    require(xmin <= px <= xmax and ymin <= py <= ymax, 'plot point outside fixed axes; do not clip')
    return x + (px - xmin) / (xmax - xmin) * width, y + height - (py - ymin) / (ymax - ymin) * height


def axes(draw, rect, bounds, xticks, yticks):
    x, y, width, height = rect
    draw.rectangle((x, y, x + width, y + height), outline='#536777', width=1)
    for value in xticks:
        px, _ = transform(rect, bounds, (value, bounds[2]))
        draw.line((px, y, px, y + height), fill='#e3e8ed')
        text(draw, (px - 13, y + height + 8), f'{value:g}', size=13)
    for value in yticks:
        _, py = transform(rect, bounds, (bounds[0], value))
        draw.line((x, py, x + width, py), fill='#e3e8ed')
        text(draw, (x - 38, py - 8), f'{value:g}', size=13)


def series(draw, rect, bounds, points, color, joined=True):
    pixels = [transform(rect, bounds, point) for point in points]
    if joined and len(pixels) > 1:
        draw.line(pixels, fill=color, width=3)
    for px, py in pixels:
        draw.ellipse((px - 2, py - 2, px + 2, py + 2), outline=color, width=1)
    return {'source_points': points, 'pixel_points': pixels}


def overview(data):
    image, draw = canvas('All 149 fixed archived inputs: preview speeds and hold-rule proposals')
    rect, bounds = (80, 135, 1065, 345), (0, 148, 0, 8.5)
    axes(draw, rect, bounds, (0, 25, 50, 75, 100, 125, 148), (0, 2, 4, 6, 8))
    text(draw, (80, 100), 'Raw selected speed at future t=0.3 s (m/s); uncapped, not actual measured speed')
    evidence = {}
    for arm in ('A', 'B'):
        values = data['overview'][arm]
        points = list(zip(data['overview']['sequence'], values['preview_mps']))
        evidence[arm] = series(draw, rect, bounds, points, COLORS[arm], joined=False)
        hold_y = 550 if arm == 'A' else 590
        count = sum(values['hold_rule_proposal'])
        text(draw, (15, hold_y - 10), arm, COLORS[arm], size=18)
        for seq, hold in zip(data['overview']['sequence'], values['hold_rule_proposal']):
            px, _ = transform(rect, bounds, (seq, 0))
            draw.ellipse((px - 3, hold_y - 3, px + 3, hold_y + 3),
                         fill=COLORS[arm] if hold else 'white', outline=COLORS[arm])
        text(draw, (80, hold_y + 11), f'{arm}: {count}/149 hold-rule proposals (filled); hollow = no hold rule', COLORS[arm])
    for seq in CASES:
        px, _ = transform(rect, bounds, (seq, 0))
        draw.line((px, 125, px, 480), fill='#808080')
        text(draw, (max(80, px - 35), 78), f'seq {seq}', size=13)
    text(draw, (80, 510), 'Historical sparse input sequence (0..148), not continuous B elapsed time')
    text(draw, (80, 658), 'Hold rule: desired speed <=0.05 m/s; separate from other braking proposals. A blue / B orange.')
    text(draw, (80, 688), 'No outcome filtering, interpolation or smoothing. Both endpoints pass selected runtime gates149/149.')
    return image, evidence


def case_plot(data, seq):
    case = data['cases'][str(seq)]
    label = 'regression' if seq == 3 else 'partial preview recovery at historical stall'
    image, draw = canvas(f'Post-hoc explanatory case {seq}: {label}')
    xy_rect, xy_bounds = (65, 140, 480, 480), (-40, 40, -40, 40)
    speed_rect, speed_bounds = (650, 190, 480, 355), (0, 6.4, 0, 8.5)
    axes(draw, xy_rect, xy_bounds, (-40, -20, 0, 20, 40), (-40, -20, 0, 20, 40))
    axes(draw, speed_rect, speed_bounds, (0, 1, 2, 3, 4, 5, 6.4), (0, 2, 4, 6, 8))
    text(draw, (65, 106), 'Selected64 XY: ego at origin; equal metre scale')
    text(draw, (170, 650), 'Right = -ego Y (m); forward = ego X (up)')
    text(draw, (650, 157), 'Selected64 speeds (m/s), t=0.1..6.4 s')
    text(draw, (800, 580), 'Future prediction time (s)')
    origin = transform(xy_rect, xy_bounds, (0, 0))
    draw.line((origin[0]-7, origin[1], origin[0]+7, origin[1]), fill='black', width=2)
    draw.line((origin[0], origin[1]-7, origin[0], origin[1]+7), fill='black', width=2)
    evidence = {}
    for arm in ('A', 'B'):
        value = case[arm]
        evidence[arm] = dict(
            xy=series(draw, xy_rect, xy_bounds, [(-p[1], p[0]) for p in value['xy_m']], COLORS[arm]),
            speed=series(draw, speed_rect, speed_bounds, list(zip(case['time_s'], value['speed_mps'])), COLORS[arm]))
        y = 608 if arm == 'A' else 632
        text(draw, (650, y), f"{arm} candidate {value['raw_argmax']}; preview={value['preview_mps']:.6f} m/s", COLORS[arm])
    preview_x, _ = transform(speed_rect, speed_bounds, (.3, 0))
    draw.line((preview_x, 190, preview_x, 545), fill='#777777')
    text(draw, (650, 116), 'Vertical marker: 0.3 s; original controller cap: 2.5 m/s')
    text(draw, (65, 700), f"Historical W1 frame {case['frame']}; progress {case['historical_progress_m']:.6f} m. Identical axes in both cases.")
    return image, evidence


def save_plot(name, image, evidence, logical_arrays):
    # HH_261001 - Persist the exact arrays used by the renderer and bind the PNG to their digest.
    metadata = PngImagePlugin.PngInfo()
    metadata.add_text('SourceRecordsSHA256', PINS[REPLAY / 'records.jsonl'])
    metadata.add_text('LogicalPlotArraysSHA256', jsha(logical_arrays))
    metadata.add_text('RenderedPointsSHA256', jsha(evidence))
    path = OUTPUT / name
    with path.open('xb') as stream:
        image.save(stream, format='PNG', pnginfo=metadata, optimize=True)
    with Image.open(path) as check:
        require(check.info['LogicalPlotArraysSHA256'] == jsha(logical_arrays)
            and check.info['RenderedPointsSHA256'] == jsha(evidence), 'PNG numeric-array identity differs')
    return dict(file=name, sha256=sha(path), logical_arrays_sha256=jsha(logical_arrays),
                rendered_points_sha256=jsha(evidence), rendered_points=evidence)


def gallery(case, images):
    seq, frame = case['sequence'], case['frame']
    cards = ''.join(f'<figure><figcaption>{html.escape(row["camera"])} / W1 frame {frame} / 원본640×360</figcaption>'
        f'<img src="{html.escape(row["file"])}" width="640" height="360" alt="역사적 W1 {html.escape(row["camera"])} 프레임 {frame}"></figure>'
        for row in images)
    return ('<!doctype html><html lang="ko"><meta charset="utf-8"><title>W1 원본 카메라 — 오프라인 설명</title>'
        '<style>body{font-family:sans-serif;margin:20px} .native{display:grid;grid-template-columns:repeat(3,640px);gap:12px;overflow:visible}'
        'figure{margin:0}img{display:block;width:640px;height:360px;max-width:none;max-height:none}.scroll{overflow-x:auto}</style>'
        f'<h1>사후 설명 사례 {seq} — 역사적 W1 카메라</h1><p>프레임 {frame}. A/B 모두 이 W1 입력을 오프라인으로 비교했습니다. '
        'B 실주행 영상이나 새 제어 ACK가 아닙니다. 카메라12장 중 이 사례의6장은 JPEG 바이트 그대로이며 합성·크롭·리사이즈하지 않았습니다.</p>'
        f'<p><a href="case_{seq:03d}.png">선택64 궤적·속도 비교</a> · <a href="README.md">해석과 한계</a></p><div class="scroll"><div class="native">{cards}</div></div></html>')


def readme():
    return '''# HH_261001 — 방향성 기울기 A/B 오프라인 설명

기존 W1 실주행에서 저장한 고정149개 입력에 대한 **오프라인 예측 비교**입니다. 새 추론·GPU·시뮬레이터 실행은 없으며, B 실주행·센서 촬영·제어 ACK나 주행 개선의 증거가 아닙니다. 아래 두 사례는 결과를 본 뒤 고른 **사후 설명 사례**입니다.

## 전체149개

![전체149개 속도 미리보기와 홀드 제안](overview149.png)

위: 0.3초 후 예측 속도이며 실제 차량 속도가 아닙니다. 아래: 채운 점은 원래 제어기의 홀드 조건(목표 속도≤0.05m/s), 빈 점은 비해당입니다. 정확히0인 미리보기는 A40→B6개, 홀드 조건은40→8개입니다. 149개를 모두 표시하며 보간·평활화·결과 필터링을 하지 않았습니다. 홀드가 아닌 과속 감속 등 다른 브레이크 제안은 별도입니다.

## 사례3: 유지한 회귀

![사례3 선택64점 궤적과 속도](case_003.png)

후보5→1, 미리보기0.572454→0m/s. 오프라인 제안은 스로틀0.400718→브레이크0.3으로 바뀝니다. **개선되지 않은 사례도 그대로 보존했습니다.** 원본 영상은 역사적 W1 프레임1400, 시간8.0905960635초입니다.

[프레임1400의 원본6카메라 보기 — 각640×360, 리사이즈 없음](case_003_native.html)

## 사례117: 정체 입력에서 미리보기 일부 회복

![사례117 선택64점 궤적과 속도](case_117.png)

후보1을 유지하면서 미리보기0→0.201293m/s, 오프라인 제안은 브레이크0.3→스로틀0.140905로 바뀝니다. 차량이 실제로 출발했다는 뜻은 아닙니다. 이 사례와 나머지5개 유사 사례는 역사적 W1의 동일 정체 위치(진행200.827611m)를 공유하므로 **독립적인6회 회복이 아닙니다.** 원본 프레임3680, 시간122.0905977623초입니다.

[프레임3680의 원본6카메라 보기 — 각640×360, 리사이즈 없음](case_117_native.html)

두 궤적 그림은 A/B와 두 사례 모두 같은 축 범위·미터 비율을 사용합니다. 원점은 자차이며 위쪽이 전방X, 오른쪽이−Y입니다. 속도곡선은 원래64점(0.1~6.4초)을 모두 표시하며 물리적 차량 궤적이 아닙니다. 기본 런타임 궤적 게이트 통과는 차체/스윕 안전 여유 통과와 다릅니다.

## 원본과 수치 검증

`evidence.json`에 고정 재생 레코드·요약·카메라/상태/제어 저널·프레임 연결·입력 아카이브·원본/복사 JPEG·도구/테스트 해시를 기록했습니다. 두 사례 모두 추론 프레임, 상태 프레임, 카메라6개 타임스탬프와 입력 아카이브 시각이 정확히 일치해야 하며 가까운 프레임으로 대체하지 않습니다. 원본 JPEG12장은 복사 후 SHA256이 같음을 확인했습니다. 그래프는 영상과 합성하지 않았고 JPEG 디코딩 결과를 다시 인코딩하지 않았습니다.

`plot_arrays.json`의 배열은 고정 레코드와 항목별 동일함을 검증했고 PNG 메타데이터에 배열 해시를 넣었습니다. 모든 그림의 실제 사용 점과 픽셀 좌표도 `evidence.json`에 남겼습니다. 이 작업은 고정 레코드를 그렸을 뿐 신경망 추론을 실행하지 않았습니다.

두 모델은 기존 우회전 DEV 절대 지표3개를 모두 실패합니다. 이 자료는 학습 데이터 승인·모델 승격·새 주행 성공 주장이 아닙니다. 새 산출물과 도구/테스트 합계는4MiB 미만, 저장공간5GiB 하한을 유지합니다.
'''


def total_bytes():
    return sum(p.stat().st_size for p in OUTPUT.rglob('*') if p.is_file()) + Path(__file__).stat().st_size + TEST.stat().st_size


def verify_rendered_points(data, item):
    """HH_261001 - Independently reconstruct each plotted coordinate from the exact frozen numeric arrays."""
    def mapped(points, rect, bounds):
        return dict(source_points=points, pixel_points=[transform(rect, bounds, p) for p in points])
    if item['file'] == 'overview149.png':
        logical = data['overview']
        expected = {arm: mapped(list(zip(logical['sequence'], logical[arm]['preview_mps'])),
                    (80, 135, 1065, 345), (0, 148, 0, 8.5)) for arm in ('A', 'B')}
    else:
        require(item['file'] in ('case_003.png', 'case_117.png'), 'undeclared explanatory plot')
        seq = str(int(item['file'][5:8]))
        logical = data['cases'][seq]
        expected = {arm: dict(
            xy=mapped([(-p[1], p[0]) for p in logical[arm]['xy_m']], (65, 140, 480, 480), (-40, 40, -40, 40)),
            speed=mapped(list(zip(logical['time_s'], logical[arm]['speed_mps'])), (650, 190, 480, 355), (0, 6.4, 0, 8.5)))
            for arm in ('A', 'B')}
    require(jsha(logical) == item['logical_arrays_sha256']
        and jsha(expected) == item['rendered_points_sha256'] == jsha(item['rendered_points']),
        'rendered numerical coordinates differ from frozen records')


def verify():
    records, expected, associations, image_sources = load()
    evidence = json.loads((OUTPUT / 'evidence.json').read_text())
    actual = json.loads((OUTPUT / 'plot_arrays.json').read_text())
    require(actual == expected and evidence['plot_arrays_sha256'] == sha(OUTPUT / 'plot_arrays.json'), 'plotted arrays differ from frozen records')
    require(evidence['frame_associations'] == associations, 'saved frame association differs')
    for path, expected_sha in evidence['source_sha256'].items():
        require(sha(Path(path)) == expected_sha, 'source bytes changed after rendering')
    for item in evidence['plots']:
        verify_rendered_points(actual, item)
        require(sha(OUTPUT / item['file']) == item['sha256'], 'rendered PNG bytes changed')
        with Image.open(OUTPUT / item['file']) as image:
            require(image.info['LogicalPlotArraysSHA256'] == item['logical_arrays_sha256']
                and image.info['RenderedPointsSHA256'] == jsha(item['rendered_points']), 'rendered array metadata differs')
    for item in evidence['native_images']:
        require(sha(OUTPUT / item['file']) == item['source_sha256'] == image_sources[item['source']],
                'native JPEG was modified, reencoded or resized')
    require(total_bytes() < MAX_BYTES and shutil.disk_usage(ROOT).free >= FLOOR, 'output budget or5GiB floor violated')
    return dict(status='VERIFIED_OFFLINE_RECORD_PLOTS_ONLY', fixed_inputs=149, explanatory_cases=list(CASES),
        native_images=12, native_pixels_unchanged=True, no_neural_inference=True, simulator_launches=0,
        new_control_ACK=0, total_new_bytes=total_bytes(), free_bytes=shutil.disk_usage(ROOT).free)


def generate():
    require(not OUTPUT.exists() and not OUTPUT.is_symlink(), 'fresh visual-evidence output required')
    require(shutil.disk_usage(ROOT).free >= FLOOR + MAX_BYTES, 'preserve5GiB floor plus bounded4MiB generation reserve')
    records, data, associations, image_sources = load()
    source = {str(path): value for path, value in PINS.items()}
    source.update(image_sources)
    source.update({str(Path(__file__)): sha(Path(__file__)), str(TEST): sha(TEST)})
    OUTPUT.mkdir()
    write(OUTPUT / 'plot_arrays.json', data)
    require(json.loads((OUTPUT / 'plot_arrays.json').read_text()) == arrays(records), 'numeric plot arrays not exact')
    image, points = overview(data)
    plots = [save_plot('overview149.png', image, points, data['overview'])]
    native = []
    for seq in CASES:
        image, points = case_plot(data, seq)
        plots.append(save_plot(f'case_{seq:03d}.png', image, points, data['cases'][str(seq)]))
        case_images = []
        (OUTPUT / f'case_{seq:03d}_native').mkdir()
        for camera in CAMERAS:
            src = EPISODE / associations[str(seq)]['camera_record']['images'][camera]
            dest = OUTPUT / f'case_{seq:03d}_native' / (camera + '.jpg')
            with src.open('rb') as inp, dest.open('xb') as out:
                shutil.copyfileobj(inp, out)
            require(sha(src) == sha(dest), 'native byte-preserving camera copy failed')
            item = dict(sequence=seq, camera=camera, source=str(src), source_sha256=sha(src),
                file=str(dest.relative_to(OUTPUT)), size=[640, 360], resized=False, reencoded=False)
            native.append(item)
            case_images.append(item)
        with (OUTPUT / f'case_{seq:03d}_native.html').open('x') as stream:
            stream.write(gallery(data['cases'][str(seq)], case_images))
    with (OUTPUT / 'README.md').open('x') as stream:
        stream.write(readme())
    evidence = dict(schema='portable_e2e.directional_fixed149_visual_evidence.v1',
        source_sha256=source, plot_arrays_sha256=sha(OUTPUT / 'plot_arrays.json'), frame_associations=associations,
        native_images=native, plots=plots, case_selection='POST_HOC_EXPLANATORY_FIXED_SEQUENCES_3_AND_117',
        neural_forwards=0, simulator_launches=0, new_control_ACK=0, model_promoted=False,
        camera_images_are='historical W1 only, shared inputs to offline A/B, never B live images')
    write(OUTPUT / 'evidence.json', evidence)
    return verify()


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--mode', choices=('generate', 'verify'), required=True)
    args = parser.parse_args()
    print(json.dumps(generate() if args.mode == 'generate' else verify(), indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
