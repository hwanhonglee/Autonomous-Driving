"""HH_261001 - Research-only past-camera velocity derivative in the unchanged snapshot reference convention."""

import math


def rotate(vector, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c*vector[0]-s*vector[1], s*vector[0]+c*vector[1])


def center_world_velocity(state, wheelbase_m):
    """HH_261001 - Undo the collector's rear-axle vy shift before matching its center-acceleration convention."""
    center_body = (state["vx"], state["vy"] + wheelbase_m*.5*state["yaw_rate"])
    return rotate(center_body, state["yaw"])


def camera_acceleration(current, previous, wheelbase_m):
    """HH_261001 - Only current/previous observed camera states; no future labels, clipping, or zero fallback."""
    if not math.isfinite(wheelbase_m) or not 1.5 <= wheelbase_m <= 4.5:
        raise ValueError("verified collector reference wheelbase required")
    required = ("timestamp", "yaw", "vx", "vy", "yaw_rate", "ax", "ay")
    if any(key not in current or not math.isfinite(current[key]) for key in required):
        raise ValueError("finite current observed pose/velocity/acceleration required")
    if previous is None or "yaw" not in previous:
        return dict(ax=current["ax"], ay=current["ay"], transformed=False,
                    reason="no_previous_camera_pose_preserve_original", dt_s=None)
    if any(key not in previous or not math.isfinite(previous[key]) for key in required):
        raise ValueError("finite previous observed camera state required")
    dt = current["timestamp"]-previous["timestamp"]
    if not .09 <= dt <= .11:
        raise ValueError("causal adjacent10Hz camera interval required; do not bridge missing/future frames")
    now, before = center_world_velocity(current,wheelbase_m), center_world_velocity(previous,wheelbase_m)
    acceleration_world = ((now[0]-before[0])/dt, (now[1]-before[1])/dt)
    ax,ay = rotate(acceleration_world,-current["yaw"])
    return dict(ax=ax,ay=ay,transformed=True,reason="previous_camera_world_velocity_difference_current_body",dt_s=dt)
