"""HH_261001 - Guard backend diagnosis against input edits or tolerance acceptance."""

import numpy as np
import pytest
from scripts.e2e.diagnose_warmup_cpu_replay_20261001 import (
    ENV_KEYS, MODES, error_metrics, profile_environment, tensorize, write_new)
from scripts.e2e.capture_exact_simulation_inputs_20261001 import array_digest


def test_startup_environments_are_independent(monkeypatch):
    for name in ENV_KEYS:
        monkeypatch.setenv(name, '99')
    expected = {'unset': (None, None), 'omp2': ('2', None), 'openblas2': (None, '2'),
                'live2': ('2', '2'), 'live2_mkldnn_off': ('2', '2')}
    for name, values in expected.items():
        env = profile_environment(name)
        assert (env.get('OMP_NUM_THREADS'), env.get('OPENBLAS_NUM_THREADS')) == values
        assert env.get('MKL_NUM_THREADS') is None
        assert env['CUDA_VISIBLE_DEVICES'] == ''


@pytest.mark.parametrize('mode', MODES)
def test_storage_modes_preserve_values_and_contiguous_stride(mode):
    arrays = {'x': np.arange(24, dtype=np.float32).reshape(1, 6, 4),
              'mask': np.asarray([[True, False]], dtype=np.bool_)}
    for key, value in tensorize(arrays, mode).items():
        assert array_digest(value.numpy()) == array_digest(arrays[key])
        assert value.is_contiguous()
        assert (value.data_ptr() == arrays[key].ctypes.data) == (mode == 'numpy')


def test_one_ulp_is_reported_not_accepted():
    expected = np.asarray([-1., 0., 1.], dtype=np.float32)
    actual = np.nextafter(expected, np.float32(np.inf))
    result = error_metrics({'x': actual}, {'x': expected})['x']
    assert result['max_float32_ulp_distance'] == 1
    assert result['unequal_elements'] == 3
    assert 'accepted' not in result


def test_new_receipts_cannot_overwrite(tmp_path):
    path = tmp_path / 'receipt.json'
    write_new(path, {'status': 'ORIGINAL_FAILURE'})
    with pytest.raises(FileExistsError):
        write_new(path, {'status': 'PASS'})
    assert 'ORIGINAL_FAILURE' in path.read_text()
