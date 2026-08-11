# Copyright 2026 Hwanhong Lee
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# HH_260811 - Regress NovAtel-first failover hysteresis and fix validity checks.

import math

import pytest

from autoware_gnss_failover_selector.selector_logic import FailoverSelector
from autoware_gnss_failover_selector.selector_logic import is_fix_valid
from autoware_gnss_failover_selector.selector_logic import SelectorConfig
from autoware_gnss_failover_selector.selector_logic import Source


def config() -> SelectorConfig:
    return SelectorConfig(
        primary_timeout_sec=1.0,
        fallback_timeout_sec=5.0,
        primary_failure_hold_sec=0.5,
        fallback_activation_hold_sec=0.5,
        primary_recovery_hold_sec=2.0,
        fallback_failure_hold_sec=0.5,
    )


def test_primary_is_selected_immediately_when_valid() -> None:
    selector = FailoverSelector(config(), now=0.0)
    selector.observe(Source.PRIMARY, True, 0.0)
    assert selector.evaluate(0.0) == Source.PRIMARY


def test_stale_primary_changes_to_stable_fallback_after_holds() -> None:
    selector = FailoverSelector(config(), now=0.0)
    selector.observe(Source.PRIMARY, True, 0.0)
    selector.evaluate(0.0)
    selector.observe(Source.FALLBACK, True, 0.1)
    selector.evaluate(0.1)

    assert selector.evaluate(1.49) == Source.PRIMARY
    assert selector.evaluate(1.50) == Source.FALLBACK


def test_primary_recovery_requires_continuous_hold() -> None:
    selector = FailoverSelector(config(), now=0.0)
    selector.observe(Source.FALLBACK, True, 0.0)
    assert selector.evaluate(0.5) == Source.FALLBACK

    selector.observe(Source.PRIMARY, True, 1.0)
    selector.observe(Source.PRIMARY, True, 1.8)
    selector.observe(Source.PRIMARY, True, 2.6)
    assert selector.evaluate(2.9) == Source.FALLBACK
    selector.observe(Source.PRIMARY, True, 3.0)
    assert selector.evaluate(3.0) == Source.PRIMARY


def test_invalid_primary_resets_recovery_hold() -> None:
    selector = FailoverSelector(config(), now=0.0)
    selector.observe(Source.FALLBACK, True, 0.0)
    selector.evaluate(0.5)
    selector.observe(Source.PRIMARY, True, 1.0)
    selector.evaluate(1.0)
    selector.observe(Source.PRIMARY, False, 2.0)
    selector.evaluate(2.0)
    selector.observe(Source.PRIMARY, True, 2.1)
    selector.observe(Source.PRIMARY, True, 3.0)
    selector.observe(Source.PRIMARY, True, 3.9)
    assert selector.evaluate(4.0) == Source.FALLBACK
    selector.observe(Source.PRIMARY, True, 4.11)
    assert selector.evaluate(4.11) == Source.PRIMARY


def test_both_failed_selects_none_and_does_not_bypass_recovery() -> None:
    selector = FailoverSelector(config(), now=0.0)
    selector.observe(Source.PRIMARY, True, 0.0)
    selector.evaluate(0.0)
    assert selector.evaluate(1.5) == Source.NONE

    selector.observe(Source.PRIMARY, True, 1.6)
    selector.observe(Source.PRIMARY, True, 2.5)
    selector.observe(Source.PRIMARY, True, 3.4)
    assert selector.evaluate(3.5) == Source.NONE
    selector.observe(Source.PRIMARY, True, 3.6)
    assert selector.evaluate(3.6) == Source.PRIMARY


def test_fix_validity_rejects_no_fix_nonfinite_and_out_of_range() -> None:
    kwargs = {
        "status": 0,
        "latitude": 37.0,
        "longitude": 127.0,
        "altitude": 30.0,
        "covariance_type": 2,
        "covariance": [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.2],
    }
    assert is_fix_valid(**kwargs)
    assert not is_fix_valid(**{**kwargs, "status": -1})
    assert not is_fix_valid(**{**kwargs, "latitude": math.nan})
    assert not is_fix_valid(**{**kwargs, "longitude": 181.0})


def test_fix_covariance_gates_are_fail_closed() -> None:
    kwargs = {
        "status": 2,
        "latitude": 37.0,
        "longitude": 127.0,
        "altitude": 30.0,
        "covariance_type": 0,
        "covariance": [0.0] * 9,
    }
    assert is_fix_valid(**kwargs)
    assert not is_fix_valid(**kwargs, require_known_covariance=True)
    assert not is_fix_valid(**kwargs, max_horizontal_variance_m2=1.0)

    known = {**kwargs, "covariance_type": 2, "covariance": [0.0] * 9}
    known["covariance"][0] = 4.0
    known["covariance"][4] = 3.0
    assert not is_fix_valid(**known, max_horizontal_variance_m2=1.0)


def test_configuration_and_time_validation() -> None:
    with pytest.raises(ValueError):
        SelectorConfig(primary_timeout_sec=0.0)
    selector = FailoverSelector(config(), now=1.0)
    with pytest.raises(ValueError):
        selector.evaluate(0.9)
