"""HH_260906 - Never waive new failures, extra skips, collection errors or partial runs."""
import importlib.util
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('hh_asset_retirement', ROOT/'tools/retire_asset_alias.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_historical_failures_are_explicit_and_not_called_pass():
    result = module.evaluate_suite('FAILED old::test_case\n1 failed, 8330 passed, 8 skipped', 1, ['old::test_case'])
    assert result['status'] == 'NO_NEW_FAILURES'
    assert result['preexisting_failures'] == ['old::test_case']


@pytest.mark.parametrize('text,code', [
    ('FAILED new::test_case\n1 failed, 8330 passed, 8 skipped', 1),
    ('ERROR old::test_case\n8330 passed, 1 error', 1),
    ('FAILED old::test_case\n1 failed, 40 passed', 1),
    ('8330 passed, 9 skipped', 0),
    ('8330 passed', 2),
    ('8330 passed', 1),
    ('FAILED old::test_case\n1 failed, 8330 passed', 0),
])
def test_incomplete_or_regressed_suite_is_rejected(text, code):
    with pytest.raises(RuntimeError):
        module.evaluate_suite(text, code, ['old::test_case'])


def test_all_green_suite_is_distinct():
    assert module.evaluate_suite('8350 passed, 8 skipped', 0, [])['status'] == 'PASS'
