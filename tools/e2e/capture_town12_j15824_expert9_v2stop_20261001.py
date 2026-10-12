#!/usr/bin/env python3
"""HH_261001 - Explicit combined 9 km/h and comfortable_v2 correction, not a causal speed A/B."""

from contextlib import contextmanager

from scripts.e2e import capture_town12_j15824_spawn_lift_v2_20261001 as inherited


TARGET_SPEED_KMH = 9.0
MAX_DURATION_SEC = 180.0
GOAL_STOP_PROFILE = "comfortable_v2"


def revised_arguments(arguments):
    result = list(arguments)
    for option, value in (("--target-speed-kmh", "9"), ("--max-duration-sec", "180"),
                          ("--goal-stop-profile", GOAL_STOP_PROFILE)):
        if result.count(option) != 1:
            raise ValueError(f"fixed parent argument must appear once: {option}")
        result[result.index(option) + 1] = value
    return result


@contextmanager
def configured_variant():
    lift = inherited.inherited
    previous_arguments, previous_speed = lift.ORIGINAL_ARGUMENTS, lift.base.TARGET_SPEED_KMH
    try:
        lift.ORIGINAL_ARGUMENTS = lambda output, route: revised_arguments(previous_arguments(output, route))
        lift.base.TARGET_SPEED_KMH = TARGET_SPEED_KMH
        yield
    finally:
        lift.ORIGINAL_ARGUMENTS, lift.base.TARGET_SPEED_KMH = previous_arguments, previous_speed


def main(argv=None):
    with configured_variant():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
