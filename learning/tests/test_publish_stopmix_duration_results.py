"""HH_260906 - Keep publication complete, denominator-aware and separate from vehicle approval."""
from pathlib import Path
import json

import pytest

from scripts.e2e import publish_stopmix_duration_results as publisher


def test_unknown_group_is_not_zero():
    assert publisher.value(None) == '미관측'
    assert publisher.value(0.) == '0.0000'


def test_decision_count_preserves_empty_group():
    assert publisher.decision_count({'sample_count': 0, 'stops': 0}, 'stops') == '미관측'
    assert publisher.decision_count({'sample_count': 0, 'stop_selection_count': None}, 'stops') == '미관측'
    assert publisher.decision_count({'sample_count': 208, 'stops': 57}, 'stops') == '57/208'


@pytest.mark.parametrize('total,count', [(0, 1), (3, -1), (3, 4), (True, 0), (3, False)])
def test_decision_count_rejects_impossible_denominator(total, count):
    with pytest.raises(ValueError, match='observed denominator'):
        publisher.decision_count({'sample_count': total, 'stops': count}, 'stops')


def test_decision_row_separates_family_and_zero_speed():
    row = publisher.decision_row('val / 10 / seed_20260903 / B', {
        'target_motion_groups': {
            'continuing_motion': dict(sample_count=208, selected_terminal_exact_zero_count=57,
                selected_terminal_at_or_below_0p1_count=58, stop_selection_count=21),
            'stationary_hold': dict(sample_count=0, stop_selection_count=None)},
        'all_samples': dict(sample_count=337, selected_reacceleration_after_exact_future_zero_count=2)})
    assert row.endswith('| 57/208 | 58/208 | 21/208 | 2/337 | 미관측 |')


def test_fixed_indices_cover_entire_declared_val():
    assert publisher.INDICES == (0, 31, 61, 92, 122, 153, 183, 214, 244, 275, 305, 336)


def test_table_retains_all_seeds_and_signed_regression():
    runs = {}
    for run in publisher.curves.RUNS:
        points = []
        for epoch, value in ((None, 2.), (10, 1.), (20, 3.)):
            points.append(dict(epoch=epoch, metrics={key: value for key in publisher.curves.KEYS}))
        runs[run] = dict(points=points)
    table = publisher.metric_table(dict(runs=runs))
    assert table.count('+1.0000') == 6
    for run in publisher.curves.RUNS:
        assert run in table


def test_partial_verification_never_creates_public_output(tmp_path):
    receipt = tmp_path / 'verification.json'
    receipt.write_text(json.dumps(dict(status='PARTIAL', campaign_completion_verified=False)))
    output = tmp_path / 'publication'
    with pytest.raises(ValueError, match='final independent'):
        publisher.publish(tmp_path / 'results', tmp_path / 'analysis', receipt, output)
    assert not output.exists()


def test_no_overwrite(tmp_path):
    with pytest.raises(ValueError, match='fresh publication'):
        publisher.publish(tmp_path / 'results', tmp_path / 'analysis', tmp_path / 'verify.json', tmp_path)


def test_no_publication_inside_evidence(tmp_path):
    with pytest.raises(ValueError, match='disjoint'):
        publisher.publish(tmp_path, tmp_path / 'analysis', tmp_path / 'verify.json', tmp_path / 'public')


def test_post_verification_behavior_mutation_is_rejected(tmp_path):
    source = tmp_path / 'behavior/summary.json'
    source.parent.mkdir()
    source.write_text('{"splits":{"val":{"sample_count":337}}}')
    inventory = dict(remote_file_sha256={'behavior/summary.json': publisher.curves.sha(source)})
    publisher.inventory_pin(source, tmp_path, inventory)
    source.write_text('{"splits":{"val":{"sample_count":1}}}')
    with pytest.raises(ValueError, match='verified remote inventory'):
        publisher.inventory_pin(source, tmp_path, inventory)
