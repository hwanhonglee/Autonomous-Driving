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

"""Unit tests for fail-closed NovAtel HEADING2 readiness validation."""

import math

import pytest

from autoware_gnss_failover_selector.orientation_logic import (
    decode_inspvax_alignment,
)
from autoware_gnss_failover_selector.orientation_logic import Heading2Config
from autoware_gnss_failover_selector.orientation_logic import (
    is_heading2_orientation_valid,
)
from autoware_gnss_failover_selector.orientation_logic import OrientationSource


def valid_heading2_fields() -> dict:
    return {
        "solution_status": 0,
        "position_type": 50,
        "baseline_length_m": 1.2,
        "heading_deg": 20.0,
        "pitch_deg": -1.0,
        "heading_stdev_deg": 0.2,
        "pitch_stdev_deg": 0.4,
        "config": Heading2Config(),
    }


# HH_260811 - Require computed NARROW_INT HEADING2 with bounded uncertainty.
def test_heading2_requires_fixed_finite_bounded_solution() -> None:
    fields = valid_heading2_fields()
    assert is_heading2_orientation_valid(**fields)
    assert is_heading2_orientation_valid(**{**fields, "baseline_length_m": -1.0})
    assert not is_heading2_orientation_valid(**{**fields, "solution_status": 1})
    assert not is_heading2_orientation_valid(**{**fields, "position_type": 34})
    assert not is_heading2_orientation_valid(**{**fields, "heading_deg": 360.0})
    assert not is_heading2_orientation_valid(
        **{**fields, "heading_stdev_deg": 6.0}
    )
    assert not is_heading2_orientation_valid(
        **{**fields, "baseline_length_m": math.nan}
    )
    assert not is_heading2_orientation_valid(
        **{**fields, "baseline_length_m": -2.0}
    )


def test_heading2_configuration_rejects_unsafe_limits() -> None:
    with pytest.raises(ValueError):
        Heading2Config(allowed_position_types=())
    with pytest.raises(ValueError):
        Heading2Config(max_heading_rmse_deg=0.0)


# HH_260811 - Keep HEADING2 outside the set of direct orientation output sources.
def test_only_complete_inspvax_can_be_an_orientation_output_source() -> None:
    assert set(OrientationSource) == {
        OrientationSource.NONE,
        OrientationSource.INSPVAX,
    }


# HH_260811 - Decode dual-antenna alignment and active ALIGN aiding from INSPVAX.
def test_inspvax_extended_status_reports_dual_alignment_evidence() -> None:
    ext_status = (3 << 26) | 0x10
    assert decode_inspvax_alignment(ext_status) == ("dual_antenna", True)
    assert decode_inspvax_alignment(2 << 26) == ("kinematic", False)
    with pytest.raises(ValueError):
        decode_inspvax_alignment(-1)
