"""HH_261001 - A correct native derivative can alias at camera frequency without a frame/sign bug."""

import pytest

from scripts.e2e.audit_ah_acceleration_sampling_20261001 import native_records, project


def test_projection_is_carla_yaw_into_forward_axis():
    assert project([2., 0., 0.], 0.) == 2.
    assert project([0., -2., 0.], -90.) == pytest.approx(2.)


def test_native_alternation_becomes_wrong_sign_camera_sample_but_positive_causal_mean():
    velocities = [0., .15, .14, .29, .28]
    states = []
    for i, v in enumerate(velocities):
        a = 0. if i == 0 else (v-velocities[i-1])/.05
        states.append(dict(frame=i, timestamp=i*.05, capture_phase="driving", vx=v, ax=a,
            world_velocity_carla=[v,0.,0.], world_acceleration_carla=[a,0.,0.],
            actor_snapshot_transform_carla=dict(yaw=0.), current_control=dict(throttle=.5,brake=0.),
            control_transport=dict(frame_binding_pass=True)))
    records = native_records(states,[dict(frame=0),dict(frame=2),dict(frame=4)])
    camera = [row for row in records if row["camera_frame"]]
    assert all(row["recorded_ax_mps2"] == pytest.approx(-.2) for row in camera)
    assert all(row["causal_two_native_mean_ax_mps2"] == pytest.approx(1.4) for row in camera)
    assert all(row["camera_interval_projected_world_dv_dt_mps2"] == pytest.approx(1.4) for row in camera)
