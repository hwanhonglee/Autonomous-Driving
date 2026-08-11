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

"""Pure state machine and data checks used by the GNSS selector node."""

# HH_260811 - Keep source-health and hysteresis decisions independent of ROS runtime state.

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional
from typing import Sequence


class Source(str, Enum):
    """Selectable GNSS sources."""

    NONE = "none"
    PRIMARY = "primary"
    FALLBACK = "fallback"


@dataclass(frozen=True)
class SelectorConfig:
    """Timing configuration for :class:`FailoverSelector`."""

    primary_timeout_sec: float = 1.0
    fallback_timeout_sec: float = 2.0
    primary_failure_hold_sec: float = 1.0
    fallback_activation_hold_sec: float = 1.0
    primary_recovery_hold_sec: float = 5.0
    fallback_failure_hold_sec: float = 1.0
    primary_initial_hold_sec: float = 0.0

    def __post_init__(self) -> None:
        positive = (self.primary_timeout_sec, self.fallback_timeout_sec)
        nonnegative = (
            self.primary_failure_hold_sec,
            self.fallback_activation_hold_sec,
            self.primary_recovery_hold_sec,
            self.fallback_failure_hold_sec,
            self.primary_initial_hold_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("GNSS source timeouts must be finite and greater than zero")
        if not all(math.isfinite(value) and value >= 0.0 for value in nonnegative):
            raise ValueError("GNSS selector hold times must be finite and nonnegative")


@dataclass
class SourceObservation:
    """Arrival-time based health state for one source."""

    created_at: float
    last_arrival: Optional[float] = None
    last_valid: bool = False
    healthy_since: Optional[float] = None

    def is_healthy(self, now: float, timeout_sec: float) -> bool:
        """Return true only for a valid and non-stale last observation."""
        return bool(
            self.last_valid
            and self.last_arrival is not None
            and now - self.last_arrival <= timeout_sec
        )

    def observe(self, valid: bool, now: float, timeout_sec: float) -> None:
        """Record a message arrival and reset continuous-health time on gaps."""
        previously_healthy = self.is_healthy(now, timeout_sec)
        if valid:
            if not previously_healthy:
                self.healthy_since = now
        else:
            self.healthy_since = None
        self.last_arrival = now
        self.last_valid = valid

    def healthy_duration(self, now: float, timeout_sec: float) -> float:
        """Return duration of uninterrupted valid, fresh input."""
        if not self.is_healthy(now, timeout_sec) or self.healthy_since is None:
            return 0.0
        return max(0.0, now - self.healthy_since)

    def unhealthy_duration(self, now: float, timeout_sec: float) -> float:
        """Return duration for which this source has been invalid or stale."""
        if self.is_healthy(now, timeout_sec):
            return 0.0
        if self.last_arrival is None:
            unhealthy_since = self.created_at
        elif self.last_valid:
            unhealthy_since = self.last_arrival + timeout_sec
        else:
            unhealthy_since = self.last_arrival
        return max(0.0, now - unhealthy_since)

    def age(self, now: float) -> Optional[float]:
        """Return seconds since the last message, or ``None`` before first input."""
        if self.last_arrival is None:
            return None
        return max(0.0, now - self.last_arrival)


class FailoverSelector:
    """Primary-first selector with failover and recovery hysteresis."""

    def __init__(self, config: SelectorConfig, now: float = 0.0) -> None:
        self.config = config
        self.primary = SourceObservation(created_at=now)
        self.fallback = SourceObservation(created_at=now)
        self.selected = Source.NONE
        self._primary_recovery_required = False
        self._last_now = now

    def _check_time(self, now: float) -> None:
        if not math.isfinite(now) or now < self._last_now:
            raise ValueError("Selector time must be finite and monotonic")
        self._last_now = now

    def observe(self, source: Source, valid: bool, now: float) -> None:
        """Record one source observation using a monotonic arrival timestamp."""
        self._check_time(now)
        if source == Source.PRIMARY:
            self.primary.observe(valid, now, self.config.primary_timeout_sec)
        elif source == Source.FALLBACK:
            self.fallback.observe(valid, now, self.config.fallback_timeout_sec)
        else:
            raise ValueError("Cannot observe Source.NONE")

    def evaluate(self, now: float) -> Source:
        """Evaluate source health, update selection, and return the selection."""
        self._check_time(now)
        primary_healthy = self.primary.is_healthy(
            now, self.config.primary_timeout_sec
        )
        fallback_healthy = self.fallback.is_healthy(
            now, self.config.fallback_timeout_sec
        )
        primary_good_for = self.primary.healthy_duration(
            now, self.config.primary_timeout_sec
        )
        fallback_good_for = self.fallback.healthy_duration(
            now, self.config.fallback_timeout_sec
        )
        primary_bad_for = self.primary.unhealthy_duration(
            now, self.config.primary_timeout_sec
        )
        fallback_bad_for = self.fallback.unhealthy_duration(
            now, self.config.fallback_timeout_sec
        )

        next_source = self.selected
        if self.selected == Source.NONE:
            primary_hold = (
                self.config.primary_recovery_hold_sec
                if self._primary_recovery_required
                else self.config.primary_initial_hold_sec
            )
            if primary_healthy and primary_good_for >= primary_hold:
                next_source = Source.PRIMARY
            elif (
                not primary_healthy
                and primary_bad_for >= self.config.primary_failure_hold_sec
                and fallback_healthy
                and fallback_good_for >= self.config.fallback_activation_hold_sec
            ):
                next_source = Source.FALLBACK
        elif self.selected == Source.PRIMARY:
            if (
                not primary_healthy
                and primary_bad_for >= self.config.primary_failure_hold_sec
            ):
                next_source = (
                    Source.FALLBACK
                    if fallback_healthy
                    and fallback_good_for
                    >= self.config.fallback_activation_hold_sec
                    else Source.NONE
                )
        elif self.selected == Source.FALLBACK:
            if (
                primary_healthy
                and primary_good_for >= self.config.primary_recovery_hold_sec
            ):
                next_source = Source.PRIMARY
            elif (
                not fallback_healthy
                and fallback_bad_for >= self.config.fallback_failure_hold_sec
            ):
                next_source = Source.NONE

        if self.selected == Source.PRIMARY and next_source != Source.PRIMARY:
            self._primary_recovery_required = True
        if next_source == Source.PRIMARY:
            self._primary_recovery_required = False
        self.selected = next_source
        return self.selected


def is_fix_valid(
    *,
    status: int,
    latitude: float,
    longitude: float,
    altitude: float,
    covariance_type: int,
    covariance: Sequence[float],
    minimum_status: int = 0,
    require_known_covariance: bool = False,
    max_horizontal_variance_m2: Optional[float] = None,
) -> bool:
    """Validate the common safety-relevant fields of a NavSatFix message."""
    if status < minimum_status:
        return False
    if not all(math.isfinite(value) for value in (latitude, longitude, altitude)):
        return False
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        return False
    if len(covariance) != 9:
        return False

    covariance_known = covariance_type > 0
    if require_known_covariance and not covariance_known:
        return False
    if covariance_known:
        diagonal = (covariance[0], covariance[4], covariance[8])
        if not all(math.isfinite(value) and value >= 0.0 for value in diagonal):
            return False

    if max_horizontal_variance_m2 is not None:
        if not covariance_known:
            return False
        if not math.isfinite(max_horizontal_variance_m2) or max_horizontal_variance_m2 < 0.0:
            return False
        if max(covariance[0], covariance[4]) > max_horizontal_variance_m2:
            return False
    return True
