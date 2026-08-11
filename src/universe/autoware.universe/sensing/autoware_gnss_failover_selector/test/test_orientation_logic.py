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

# HH_260811 - Regress fail-closed NovAtel INS orientation conversion and validity gates.

import math

import pytest

from autoware_gnss_failover_selector.orientation_logic import (
    is_inspvax_orientation_valid,
)
from autoware_gnss_failover_selector.orientation_logic import OrientationConfig
from autoware_gnss_failover_selector.orientation_logic import (
    quaternion_from_novatel_attitude,
)
from autoware_gnss_failover_selector.orientation_logic import rmse_radians


def valid_fields() -> dict:
    return {
        "ins_status": 3,
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "azimuth_deg": 90.0,
        "roll_stdev_deg": 0.1,
        "pitch_stdev_deg": 0.2,
        "azimuth_stdev_deg": 0.3,
        "config": OrientationConfig(),
    }


def test_waiting_azimuth_and_large_yaw_rmse_fail_closed() -> None:
    fields = valid_fields()
    assert not is_inspvax_orientation_valid(**{**fields, "ins_status": 10})
    assert not is_inspvax_orientation_valid(
        **{**fields, "azimuth_stdev_deg": 180.0}
    )


def test_only_finite_bounded_solution_good_attitude_is_accepted() -> None:
    fields = valid_fields()
    assert is_inspvax_orientation_valid(**fields)
    assert not is_inspvax_orientation_valid(**{**fields, "roll_deg": math.nan})
    assert not is_inspvax_orientation_valid(**{**fields, "pitch_deg": 91.0})
    assert not is_inspvax_orientation_valid(**{**fields, "roll_stdev_deg": -0.1})


def test_novatel_north_azimuth_converts_to_ros_enu_yaw() -> None:
    x, y, z, w = quaternion_from_novatel_attitude(0.0, 0.0, 0.0)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))

    x, y, z, w = quaternion_from_novatel_attitude(0.0, 0.0, 90.0)
    assert (x, y, z, w) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quaternion_is_normalized_and_rmse_converts_to_radians() -> None:
    quaternion = quaternion_from_novatel_attitude(10.0, -5.0, 280.0)
    assert sum(value * value for value in quaternion) == pytest.approx(1.0)
    assert rmse_radians(180.0, 90.0, 45.0) == pytest.approx(
        (math.pi, math.pi / 2.0, math.pi / 4.0)
    )


def test_orientation_configuration_rejects_unsafe_limits() -> None:
    with pytest.raises(ValueError):
        OrientationConfig(max_yaw_rmse_deg=0.0)
