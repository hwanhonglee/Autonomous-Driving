"""HH_261001 - Raw and inset body diagnostics cannot turn intrusion into positive clearance."""

from shapely.geometry import box
from scripts.e2e.audit_simulation_research_body_lane_20261001 import region_measurement, candidate_bound_rows


def test_interior_margin_and_outside_failure():
    region = box(0, 0, 10, 10)
    clear = region_measurement(box(2, 3, 4, 5), region)
    assert clear['outside_m2'] == 0 and clear['minimum_boundary_clearance_m'] == 2
    bad = region_measurement(box(-1, 3, 4, 5), region)
    assert bad['outside_m2'] == 2 and not bad['area_gate_clear']
    assert bad['minimum_boundary_clearance_m'] is None


def test_raw_pass_does_not_hide_inset_failure():
    shape = box(.05, 2, 1, 3)
    raw = box(0, 0, 10, 10)
    assert region_measurement(shape, raw)['area_gate_clear']
    assert not region_measurement(shape, raw.buffer(-.1))['area_gate_clear']


def test_boundary_contact_has_zero_clearance_not_positive():
    result = region_measurement(box(0, 2, 1, 3), box(0, 0, 10, 10))
    assert result['area_gate_clear'] and result['strictly_covered']
    assert result['minimum_boundary_clearance_m'] == 0


def test_candidate_math_retains_all_twelve_and_deduplicates_held_command():
    xy = [[[.2 * (i + 1), 0.] for i in range(64)] for _ in range(12)]
    output = dict(candidate_xy=xy, candidate_speed=[[2.] * 64 for _ in range(12)], candidate_logits=[0.] * 12)
    command = dict(raw_output=output, inference_frame=100, source_checkpoint_sha256='fixed', selected={'candidate_index': 0})
    state = dict(frame=100, timestamp=5., x=0., y=0., z=0., yaw=0., vx=2., vy=0.)
    route = dict(route=[dict(x=0., y=0.), dict(x=100., y=0.)], route_length_m=100.)
    rows, summary = candidate_bound_rows([command, command], [state], route, 'fixed')
    assert len(rows) == 12 and summary['all_candidates']['clear_count'] == 12
    assert summary['selected_candidates']['candidate_count'] == 1
    output['candidate_speed'][1][0] = 'NaN'
    rows, summary = candidate_bound_rows([command], [state], route, 'fixed')
    assert len(rows) == 12 and summary['all_candidates']['unassessed_nonfinite_count'] == 1
    assert rows[1]['diagnostic'] is None and not rows[1]['all_nine_mathematical_checks_clear']
