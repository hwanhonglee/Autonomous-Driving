"""HH_261007 - Test CPU environment admission before any ROS/Torch import or real report write."""
import sys

import pytest

from scripts.e2e import check_nontraining_cpu_ros_environment_20261007_v1 as subject


def clean(monkeypatch):
    monkeypatch.setattr(sys, 'executable', '/usr/bin/python3')
    for key, value in dict(CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE='1', ROS_DOMAIN_ID='93',
                           ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                           AMENT_PREFIX_PATH='', PYTHONPATH='').items():
        monkeypatch.setenv(key, value)
    calls = []
    def deny(name):
        calls.append(name)
        raise AssertionError('import should not occur before rejection')
    monkeypatch.setattr(subject.importlib, 'import_module', deny)
    return calls


@pytest.mark.parametrize('key,value', [
    ('CUDA_VISIBLE_DEVICES', '0'), ('PYTHONNOUSERSITE', '0'), ('ROS_DOMAIN_ID', '10'),
    ('ROS_LOCALHOST_ONLY', '0'), ('RMW_IMPLEMENTATION', 'unknown'),
    ('AMENT_PREFIX_PATH', '/home/a/carla-autoware-universe/autoware/install'),
    ('PYTHONPATH', '/home/a/carla-autoware-universe/autoware/install/example'),
])
def test_unsafe_environment_rejected_before_import(monkeypatch, key, value):
    calls = clean(monkeypatch)
    monkeypatch.setenv(key, value)
    with pytest.raises(RuntimeError):
        subject.check()
    assert calls == []


def test_default_command_does_not_write(tmp_path, monkeypatch):
    monkeypatch.setattr(subject, 'OUTPUT', tmp_path / 'absent')
    with pytest.raises(RuntimeError):
        subject.main([])
    assert not subject.OUTPUT.exists()


def test_existing_report_refused(tmp_path, monkeypatch):
    original = tmp_path / 'output'
    original.mkdir()
    (original / 'result.json').write_bytes(b'original report')
    monkeypatch.setattr(subject, 'OUTPUT', original)
    with pytest.raises(RuntimeError):
        subject.main(['--write-report'])
    assert (original / 'result.json').read_bytes() == b'original report'
