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

"""Pure checks and coordinate conversion for NovAtel OEM7 orientation logs."""

# HH_260811 - Isolate fail-closed INS and dual-heading validation from ROS runtime state.

from dataclasses import dataclass
from enum import Enum
import math
from typing import Tuple


INS_SOLUTION_GOOD = 3
SOLUTION_COMPUTED = 0
NARROW_INT = 50
ALIGN_UPDATE_MASK = 0x00000010
ALIGNMENT_INDICATION_SHIFT = 26
ALIGNMENT_INDICATION_MASK = 0x7
ALIGNMENT_INDICATION_NAMES = {
    0: "incomplete",
    1: "static",
    2: "kinematic",
    3: "dual_antenna",
    4: "user_command",
    5: "nvm_seed",
}


# HH_260811 - Identify the active fail-closed orientation source for diagnostics.
class OrientationSource(str, Enum):
    """Sources from which a complete vehicle orientation can be constructed."""

    NONE = "none"
    INSPVAX = "inspvax"


@dataclass(frozen=True)
class OrientationConfig:
    """Validity bounds for an INSPVAX orientation solution."""

    required_ins_status: int = INS_SOLUTION_GOOD
    max_yaw_rmse_deg: float = 5.0
    max_roll_pitch_rmse_deg: float = 10.0

    def __post_init__(self) -> None:
        if self.required_ins_status < 0:
            raise ValueError("required_ins_status must be nonnegative")
        limits = (self.max_yaw_rmse_deg, self.max_roll_pitch_rmse_deg)
        if not all(math.isfinite(value) and value > 0.0 for value in limits):
            raise ValueError("Orientation RMSE limits must be finite and positive")


# HH_260811 - Gate dual-antenna HEADING2 on a computed integer-fixed solution.
@dataclass(frozen=True)
class Heading2Config:
    """Validity bounds for an OEM7 dual-antenna HEADING2 solution."""

    required_solution_status: int = SOLUTION_COMPUTED
    allowed_position_types: Tuple[int, ...] = (NARROW_INT,)
    max_heading_rmse_deg: float = 5.0
    max_pitch_rmse_deg: float = 10.0

    def __post_init__(self) -> None:
        if self.required_solution_status < 0:
            raise ValueError("required_solution_status must be nonnegative")
        if not self.allowed_position_types or any(
            value < 0 for value in self.allowed_position_types
        ):
            raise ValueError("allowed_position_types must contain nonnegative values")
        limits = (self.max_heading_rmse_deg, self.max_pitch_rmse_deg)
        if not all(math.isfinite(value) and value > 0.0 for value in limits):
            raise ValueError("HEADING2 RMSE limits must be finite and positive")


def is_inspvax_orientation_valid(
    *,
    ins_status: int,
    roll_deg: float,
    pitch_deg: float,
    azimuth_deg: float,
    roll_stdev_deg: float,
    pitch_stdev_deg: float,
    azimuth_stdev_deg: float,
    config: OrientationConfig,
) -> bool:
    """Accept only a converged, finite INS orientation below RMSE limits."""
    if ins_status != config.required_ins_status:
        return False
    values = (
        roll_deg,
        pitch_deg,
        azimuth_deg,
        roll_stdev_deg,
        pitch_stdev_deg,
        azimuth_stdev_deg,
    )
    if not all(math.isfinite(value) for value in values):
        return False
    if not -180.0 <= roll_deg <= 180.0:
        return False
    if not -90.0 <= pitch_deg <= 90.0:
        return False
    if not 0.0 <= azimuth_deg <= 360.0:
        return False
    if min(roll_stdev_deg, pitch_stdev_deg, azimuth_stdev_deg) < 0.0:
        return False
    if azimuth_stdev_deg > config.max_yaw_rmse_deg:
        return False
    if max(roll_stdev_deg, pitch_stdev_deg) > config.max_roll_pitch_rmse_deg:
        return False
    return True


# HH_260811 - Validate HEADING2 as dual-antenna readiness evidence, not body attitude.
def is_heading2_orientation_valid(
    *,
    solution_status: int,
    position_type: int,
    baseline_length_m: float,
    heading_deg: float,
    pitch_deg: float,
    heading_stdev_deg: float,
    pitch_stdev_deg: float,
    config: Heading2Config,
) -> bool:
    """Accept only finite, computed, integer-fixed dual-antenna heading data."""
    if solution_status != config.required_solution_status:
        return False
    if position_type not in config.allowed_position_types:
        return False
    values = (
        baseline_length_m,
        heading_deg,
        pitch_deg,
        heading_stdev_deg,
        pitch_stdev_deg,
    )
    if not all(math.isfinite(value) for value in values):
        return False
    # OEM7 documents -1 m as a valid model-dependent HEADING2 baseline value.
    if baseline_length_m < 0.0 and not math.isclose(baseline_length_m, -1.0):
        return False
    if not 0.0 <= heading_deg < 360.0:
        return False
    if not -90.0 <= pitch_deg <= 90.0:
        return False
    if min(heading_stdev_deg, pitch_stdev_deg) < 0.0:
        return False
    if heading_stdev_deg > config.max_heading_rmse_deg:
        return False
    if pitch_stdev_deg > config.max_pitch_rmse_deg:
        return False
    return True


# HH_260811 - Decode OEM7 INSPVAX extended status as non-authoritative source evidence.
def decode_inspvax_alignment(ext_solution_status: int) -> Tuple[str, bool]:
    """Return completed alignment indication and current ALIGN-update usage."""
    if ext_solution_status < 0 or ext_solution_status > 0xFFFFFFFF:
        raise ValueError("ext_solution_status must be an unsigned 32-bit value")
    indication = (
        ext_solution_status >> ALIGNMENT_INDICATION_SHIFT
    ) & ALIGNMENT_INDICATION_MASK
    name = ALIGNMENT_INDICATION_NAMES.get(indication, f"unknown_{indication}")
    align_update_active = bool(ext_solution_status & ALIGN_UPDATE_MASK)
    return name, align_update_active


def quaternion_from_novatel_attitude(
    roll_deg: float, pitch_deg: float, azimuth_deg: float
) -> Tuple[float, float, float, float]:
    """Convert NovAtel roll/pitch/azimuth degrees to a normalized ROS ENU quaternion."""
    roll = math.radians(roll_deg)
    pitch = math.radians(-pitch_deg)
    yaw = math.radians(90.0 - azimuth_deg)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("Cannot normalize INSPVAX orientation quaternion")
    return x / norm, y / norm, z / norm, w / norm


def rmse_radians(
    roll_stdev_deg: float, pitch_stdev_deg: float, azimuth_stdev_deg: float
) -> Tuple[float, float, float]:
    """Convert NovAtel one-sigma attitude standard deviations to radians."""
    return tuple(
        math.radians(value)
        for value in (roll_stdev_deg, pitch_stdev_deg, azimuth_stdev_deg)
    )
