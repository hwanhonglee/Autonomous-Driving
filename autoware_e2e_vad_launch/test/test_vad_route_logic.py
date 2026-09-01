#!/usr/bin/env python3

import copy
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

from nav_msgs.msg import Odometry

SCRIPT_DIRECTORY = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIRECTORY))

from vad_route_logic import (  # noqa: E402
    RouteOffsetProfile,
    RoutePlan,
    RoutePoint,
    constrain_trajectory_points_to_route,
    endpoint_fixed_whittaker,
    limit_trajectory_speed_for_acceleration,
    limit_trajectory_speed_for_curvature,
    limit_trajectory_speed_for_route_curvature,
    limit_trajectory_speed_recovery,
    resample_trajectory_points,
    route_alignment_blocked,
    smooth_trajectory_geometry,
    stabilize_trajectory_lateral_offsets,
    trajectory_arc_lengths,
    trajectory_planar_curvatures,
    trajectory_sample_distances,
    zero_velocity_distance_for_goal,
)
from vad_route_manager import VadRouteManager  # noqa: E402


def make_trajectory_point(x, y, time_sec, speed, quaternion=(0.0, 0.0, 0.0, 1.0)):
    seconds = int(time_sec)
    nanoseconds = int(round((time_sec - seconds) * 1.0e9))
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=SimpleNamespace(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            ),
        ),
        time_from_start=SimpleNamespace(sec=seconds, nanosec=nanoseconds),
        longitudinal_velocity_mps=speed,
        lateral_velocity_mps=0.0,
        acceleration_mps2=0.0,
        heading_rate_rps=0.0,
        front_wheel_angle_rad=0.0,
        rear_wheel_angle_rad=0.0,
    )


class RoutePlanTest(unittest.TestCase):
    def setUp(self):
        self.plan = RoutePlan(
            {"scenario": "left"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(5.0, 0.0, 0.0, 0.0, 5.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 0, "LEFT"),
                RoutePoint(10.0, 5.0, 0.0, math.pi / 2.0, 15.0, 0, "LEFT"),
                RoutePoint(10.0, 10.0, 0.0, math.pi / 2.0, 20.0, 3, "LANEFOLLOW"),
            ],
        )

    def test_projection_uses_route_arc_length(self):
        projection = self.plan.project(7.0, 1.0)
        self.assertAlmostEqual(projection.progress_m, 7.0)
        self.assertAlmostEqual(projection.cross_track_error_m, 1.0)

    def test_route_manager_faults_on_negative_signed_deviation(self):
        class SignedProjectionRoute:
            def project(self, *_args):
                return SimpleNamespace(progress_m=1.0, cross_track_error_m=-4.0)

            def remaining(self, _progress_m):
                return 9.0

            def command_at(self, *_args):
                return 3

        faults = []
        manager = SimpleNamespace(
            route=SignedProjectionRoute(),
            progress_m=0.0,
            route_projection_backtrack_m=3.0,
            route_projection_forward_m=80.0,
            maneuver_lookahead_m=3.0,
            maneuver_exit_lookahead_m=2.5,
            max_route_deviation_m=3.5,
            actual_path=SimpleNamespace(
                header=SimpleNamespace(stamp=None), poses=[]
            ),
            actual_path_pub=SimpleNamespace(publish=lambda _message: None),
            _set_fault=faults.append,
            _update_goal_state=lambda: None,
        )
        odometry = Odometry()

        VadRouteManager._on_odometry(manager, odometry)

        self.assertEqual(faults, ["route_deviation:abs(-4.00m)>3.50m"])

    def test_projection_does_not_regress(self):
        projection = self.plan.project(4.0, 0.0, previous_progress_m=8.0)
        self.assertAlmostEqual(projection.progress_m, 8.0)

    def test_command_looks_ahead_and_returns_to_lane_follow(self):
        self.assertEqual(self.plan.command_at(1.0, 5.0), 3)
        self.assertEqual(self.plan.command_at(3.0, 8.0), 0)
        self.assertEqual(self.plan.command_at(12.0, 2.0), 0)
        self.assertEqual(self.plan.command_at(19.0, 2.0), 3)

    def test_command_can_return_to_lane_follow_before_maneuver_end(self):
        self.assertEqual(self.plan.command_at(14.0, 2.0, exit_lookahead_m=6.0), 3)
        self.assertEqual(self.plan.command_at(14.0, 2.0, exit_lookahead_m=2.0), 0)

    def test_directional_command_is_not_masked_by_straight(self):
        plan = RoutePlan(
            {"scenario": "left"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(5.0, 0.0, 0.0, 0.0, 5.0, 2, "STRAIGHT"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 0, "LEFT"),
                RoutePoint(10.0, 5.0, 0.0, math.pi / 2.0, 15.0, 0, "LEFT"),
            ],
        )

        self.assertEqual(plan.command_at(0.0, 12.0), 0)
        self.assertEqual(plan.command_at(6.0, 6.0), 0)
        self.assertEqual(plan.command_at(0.0, 7.0), 2)

    def test_remaining_is_clamped(self):
        self.assertEqual(self.plan.remaining(7.5), 12.5)
        self.assertEqual(self.plan.remaining(25.0), 0.0)

    def test_sample_at_interpolates_route_pose(self):
        sample = self.plan.sample_at(2.5)
        self.assertAlmostEqual(sample.x, 2.5)
        self.assertAlmostEqual(sample.y, 0.0)
        self.assertAlmostEqual(sample.yaw, 0.0)

    def test_load_appends_exact_goal_and_extends_cumulative_distance(self):
        payload = self._route_payload()
        payload["goal_ros_pose"] = {
            "x": 13.0,
            "y": 4.0,
            "z": 0.3,
            "yaw": 1.25,
        }

        plan = self._load_route_payload(payload)

        self.assertEqual(len(plan.points), 3)
        self.assertEqual(
            (plan.goal.x, plan.goal.y, plan.goal.z, plan.goal.yaw),
            (13.0, 4.0, 0.3, 1.25),
        )
        self.assertAlmostEqual(plan.goal.distance_m, 15.0)
        self.assertAlmostEqual(plan.length_m, 15.0)
        self.assertAlmostEqual(plan.metadata["route_length_m"], 15.0)
        self.assertEqual(plan.goal.vad_command, plan.points[-2].vad_command)
        self.assertEqual(plan.goal.road_option, plan.points[-2].road_option)
        self.assertEqual(plan.metadata["runtime_goal_z_m"], 0.3)
        self.assertEqual(
            plan.metadata["runtime_goal_z_policy"], "legacy_goal_ros_pose_z"
        )

    def test_load_does_not_duplicate_goal_within_endpoint_epsilon(self):
        payload = self._route_payload()
        payload["goal_ros_pose"] = {
            "x": 10.0005,
            "y": 0.0,
            "z": 0.3,
            "yaw": 0.25,
        }

        plan = self._load_route_payload(payload)

        self.assertEqual(len(plan.points), 2)
        self.assertEqual(
            (plan.goal.x, plan.goal.y, plan.goal.z, plan.goal.yaw),
            (10.0005, 0.0, 0.3, 0.25),
        )
        self.assertAlmostEqual(plan.length_m, 10.0)

    def test_load_never_injects_spawn_clearance_z_into_runtime_route(self):
        payload = self._route_payload()
        payload["route"][-1]["z"] = 1.25
        original_goal_ros_pose = {
            "x": 10.0,
            "y": 0.0,
            "z": 1.75,
            "yaw": 0.25,
        }
        payload["goal_ros_pose"] = {**original_goal_ros_pose, "z": 1.25}
        payload["goal_carla_transform"] = {
            "x": 10.0,
            "y": 0.0,
            "z": 1.25,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -math.degrees(0.25),
        }
        payload["goal_endpoint_provenance"] = {
            "endpoint_source": "spawn_points",
            "endpoint_index": 7,
            "original_goal_carla_transform": {
                **payload["goal_carla_transform"],
                "z": 1.75,
            },
            "original_goal_ros_pose": original_goal_ros_pose,
            "terminal_z_normalization": {
                "policy": "last_road_waypoint_z",
                "original_endpoint_z_m": 1.75,
                "last_road_waypoint_z_m": 1.25,
                "runtime_goal_z_m": 1.25,
                "serialized_terminal_z_m": 1.25,
                "applied_offset_m": -0.5,
            },
        }

        plan = self._load_route_payload(payload)

        self.assertEqual(len(plan.points), 2)
        self.assertEqual(
            (plan.goal.x, plan.goal.y, plan.goal.z, plan.goal.yaw),
            (10.0, 0.0, 1.25, 0.25),
        )
        self.assertEqual(plan.metadata["goal_ros_pose"]["z"], 1.25)
        self.assertEqual(plan.metadata["runtime_goal_z_m"], 1.25)
        self.assertEqual(
            plan.metadata["runtime_goal_z_policy"], "last_road_waypoint_z"
        )

    def test_load_rejects_drifted_road_z_provenance(self):
        payload = self._route_payload()
        payload["goal_ros_pose"] = {
            "x": 10.0,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
        }
        payload["goal_carla_transform"] = {
            "x": 10.0,
            "y": 0.0,
            "z": 0.0,
        }
        payload["goal_endpoint_provenance"] = {
            "endpoint_source": "spawn_points",
            "endpoint_index": 1,
            "original_goal_carla_transform": {"z": 0.5},
            "original_goal_ros_pose": {"z": 0.5},
            "terminal_z_normalization": {
                "policy": "last_road_waypoint_z",
                "original_endpoint_z_m": 0.5,
                "last_road_waypoint_z_m": 0.0,
                "runtime_goal_z_m": 0.0,
                "serialized_terminal_z_m": 0.25,
                "applied_offset_m": -0.5,
            },
        }

        with self.assertRaisesRegex(ValueError, "serialized_terminal_z_m"):
            self._load_route_payload(payload)

    def test_load_without_goal_metadata_keeps_legacy_endpoint(self):
        plan = self._load_route_payload(self._route_payload())

        self.assertEqual(len(plan.points), 2)
        self.assertEqual((plan.goal.x, plan.goal.y), (10.0, 0.0))
        self.assertEqual(plan.metadata["route_length_m"], 10.0)

    def test_load_rejects_malformed_goal_metadata(self):
        malformed_goals = (
            [],
            {"y": 0.0, "z": 0.0, "yaw": 0.0},
            {"x": "bad", "y": 0.0, "z": 0.0, "yaw": 0.0},
            {"x": 10.0, "y": 0.0, "z": 0.0, "yaw": math.inf},
        )
        for goal_pose in malformed_goals:
            with self.subTest(goal_pose=goal_pose):
                payload = self._route_payload()
                payload["goal_ros_pose"] = goal_pose
                with self.assertRaisesRegex(ValueError, "goal_ros_pose"):
                    self._load_route_payload(payload)

    def test_load_requires_base_link_route_coordinates(self):
        payload = self._route_payload()
        payload["coordinate_reference"] = "vehicle_center"

        with self.assertRaisesRegex(ValueError, "base_link"):
            self._load_route_payload(payload)

    @staticmethod
    def _route_payload():
        return {
            "schema_version": 1,
            "route_length_m": 10.0,
            "route": [
                {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "yaw": 0.0,
                    "distance_m": 0.0,
                    "vad_command": 3,
                    "road_option": "LANEFOLLOW",
                },
                {
                    "x": 10.0,
                    "y": 0.0,
                    "z": 0.0,
                    "yaw": 0.0,
                    "distance_m": 10.0,
                    "vad_command": 0,
                    "road_option": "LEFT",
                },
            ],
        }

    @staticmethod
    def _load_route_payload(payload):
        with tempfile.TemporaryDirectory() as directory:
            route_file = Path(directory) / "route.json"
            route_file.write_text(json.dumps(payload), encoding="utf-8")
            return RoutePlan.load(route_file)

    def test_standard_route_gate_requires_explicit_alignment(self):
        self.assertTrue(route_alignment_blocked(True, None))
        self.assertTrue(route_alignment_blocked(True, False))
        self.assertFalse(route_alignment_blocked(True, True))
        self.assertFalse(route_alignment_blocked(False, None))

    def test_standard_route_gate_fails_closed_on_stale_heartbeat(self):
        self.assertFalse(route_alignment_blocked(True, True, 2.5, 2.5))
        self.assertTrue(route_alignment_blocked(True, True, 2.5001, 2.5))
        self.assertTrue(route_alignment_blocked(True, True, -0.1, 2.5))
        self.assertTrue(route_alignment_blocked(True, True, None, 2.5))
        self.assertFalse(route_alignment_blocked(False, True, 99.0, 2.5))

    def test_emergency_stop_without_candidate_is_anchored_to_current_ego(self):
        yaw = 1.2
        odometry = Odometry()
        odometry.pose.pose.position.x = 100.0
        odometry.pose.pose.position.y = -20.0
        odometry.pose.pose.position.z = 0.5
        odometry.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odometry.pose.pose.orientation.w = math.cos(yaw * 0.5)
        manager = SimpleNamespace(odom=odometry)

        trajectory = VadRouteManager._emergency_stop_trajectory(manager, None)

        self.assertEqual(trajectory.header.frame_id, "map")
        self.assertEqual(len(trajectory.points), 3)
        first, second, third = trajectory.points
        self.assertAlmostEqual(first.pose.position.x, 100.0)
        self.assertAlmostEqual(first.pose.position.y, -20.0)
        self.assertAlmostEqual(
            math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            ),
            0.5,
        )
        self.assertAlmostEqual(first.pose.orientation.z, math.sin(yaw * 0.5))
        self.assertEqual(second.time_from_start.nanosec, 100_000_000)
        self.assertAlmostEqual(
            math.hypot(
                third.pose.position.x - second.pose.position.x,
                third.pose.position.y - second.pose.position.y,
            ),
            0.5,
        )
        self.assertAlmostEqual(third.pose.orientation.z, math.sin((yaw + 0.001) * 0.5))
        self.assertEqual(third.time_from_start.nanosec, 200_000_000)


class TrajectoryResamplingTest(unittest.TestCase):
    def test_route_corridor_clamps_candidate_and_recomputes_heading(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(0.0, 1.0, 0.0, 1.0),
            make_trajectory_point(2.0, 1.0, 1.0, 1.0),
        ]
        correction = constrain_trajectory_points_to_route(
            points, route, progress_m=0.0, corridor_half_width_m=0.5
        )
        self.assertAlmostEqual(correction, 0.5)
        self.assertEqual([point.pose.position.y for point in points], [0.5, 0.5])
        self.assertAlmostEqual(points[0].pose.orientation.z, 0.0)
        self.assertAlmostEqual(points[0].pose.orientation.w, 1.0)

    def test_route_corridor_supports_asymmetric_lateral_bounds(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(0.0, -0.4, 0.0, 1.0),
            make_trajectory_point(1.0, -0.4, 1.0, 1.0),
            make_trajectory_point(2.0, 0.4, 2.0, 1.0),
        ]

        correction = constrain_trajectory_points_to_route(
            points,
            route,
            progress_m=0.0,
            corridor_half_width_m=0.5,
            lateral_offset_min_m=-0.2,
            lateral_offset_max_m=0.5,
        )

        self.assertAlmostEqual(correction, 0.2)
        self.assertEqual(
            [round(point.pose.position.y, 6) for point in points],
            [-0.2, -0.2, 0.4],
        )

    def test_route_corridor_quintic_entry_preserves_point_zero(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(float(index), 0.9, float(index), 1.0)
            for index in range(7)
        ]
        initial_pose = copy.deepcopy(points[0].pose)

        constrain_trajectory_points_to_route(
            points,
            route,
            progress_m=0.0,
            corridor_half_width_m=0.5,
            lateral_offset_min_m=-0.5,
            lateral_offset_max_m=0.2,
            entry_distance_m=5.0,
        )

        self.assertEqual(points[0].pose.position, initial_pose.position)
        offsets = [point.pose.position.y for point in points]
        self.assertTrue(
            all(second <= first + 1.0e-9 for first, second in zip(offsets, offsets[1:]))
        )
        self.assertAlmostEqual(offsets[5], 0.2)
        self.assertAlmostEqual(offsets[6], 0.2)

    def test_zero_entry_distance_is_legacy_equivalent(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        legacy = [
            make_trajectory_point(float(index), 0.9, float(index), 1.0)
            for index in range(4)
        ]
        explicit_zero = copy.deepcopy(legacy)

        legacy_correction = constrain_trajectory_points_to_route(
            legacy, route, 0.0, 0.5, lateral_offset_max_m=0.2
        )
        zero_correction = constrain_trajectory_points_to_route(
            explicit_zero,
            route,
            0.0,
            0.5,
            lateral_offset_max_m=0.2,
            entry_distance_m=0.0,
        )

        self.assertEqual(legacy, explicit_zero)
        self.assertEqual(legacy_correction, zero_correction)

    def test_route_corridor_entry_ramp_rejects_soft_mode(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(2.0, 0.0, 0.0, 0.0, 2.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(0.0, 0.8, 0.0, 1.0),
            make_trajectory_point(1.0, 0.8, 1.0, 1.0),
        ]
        with self.assertRaisesRegex(ValueError, "entry ramp requires hard mode"):
            constrain_trajectory_points_to_route(
                points, route, 0.0, 0.5, mode="soft", entry_distance_m=5.0
            )

    def test_route_corridor_rejects_lateral_bounds_outside_radial_corridor(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(2.0, 0.0, 0.0, 0.0, 2.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(0.0, 0.0, 0.0, 1.0),
            make_trajectory_point(1.0, 0.0, 1.0, 1.0),
        ]
        with self.assertRaisesRegex(ValueError, "lateral bounds"):
            constrain_trajectory_points_to_route(
                points,
                route,
                0.0,
                0.5,
                lateral_offset_min_m=-0.6,
            )

    def test_route_corridor_truncates_points_beyond_goal_without_overlap(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(9.0 + index * 0.5, 0.0, index * 0.1, 1.0)
            for index in range(21)
        ]
        correction = constrain_trajectory_points_to_route(
            points, route, progress_m=9.0, corridor_half_width_m=0.5
        )

        self.assertAlmostEqual(correction, 0.0)
        self.assertEqual([point.pose.position.x for point in points], [9.0, 9.5, 10.0])
        self.assertTrue(
            all(
                math.hypot(
                    second.pose.position.x - first.pose.position.x,
                    second.pose.position.y - first.pose.position.y,
                )
                > 1.0e-4
                for first, second in zip(points, points[1:])
            )
        )

    def test_route_corridor_preserves_heading_when_geometry_is_unchanged(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        yaw = math.pi / 2.0
        quaternion = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        points = [
            make_trajectory_point(0.0, 0.0, 0.0, 1.0, quaternion),
            make_trajectory_point(1.0, 0.0, 1.0, 1.0, quaternion),
        ]
        correction = constrain_trajectory_points_to_route(
            points, route, progress_m=0.0, corridor_half_width_m=0.5
        )

        self.assertAlmostEqual(correction, 0.0)
        self.assertAlmostEqual(points[0].pose.orientation.z, quaternion[2])
        self.assertAlmostEqual(points[0].pose.orientation.w, quaternion[3])

    def test_soft_route_corridor_saturates_without_a_hard_boundary(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(0.0, 0.1, 0.0, 1.0),
            make_trajectory_point(1.0, 0.5, 1.0, 1.0),
            make_trajectory_point(2.0, 1.0, 2.0, 1.0),
        ]
        expected_positions = []
        expected_corrections = []
        for point, distance in zip(points, trajectory_arc_lengths(points)):
            reference = route.sample_at(distance)
            dx = point.pose.position.x - reference.x
            dy = point.pose.position.y - reference.y
            error = math.hypot(dx, dy)
            constrained = 0.5 * math.tanh(error / 0.5)
            scale = constrained / error
            expected_positions.append(
                (reference.x + dx * scale, reference.y + dy * scale)
            )
            expected_corrections.append(error - constrained)

        correction = constrain_trajectory_points_to_route(
            points,
            route,
            progress_m=0.0,
            corridor_half_width_m=0.5,
            mode="soft",
        )

        for point, expected_position in zip(points, expected_positions):
            self.assertAlmostEqual(point.pose.position.x, expected_position[0])
            self.assertAlmostEqual(point.pose.position.y, expected_position[1])
        self.assertAlmostEqual(correction, max(expected_corrections))

    def test_route_corridor_rejects_unknown_mode(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(2.0, 0.0, 0.0, 0.0, 2.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(0.0, 0.0, 0.0, 1.0),
            make_trajectory_point(1.0, 0.0, 1.0, 1.0),
        ]
        with self.assertRaisesRegex(ValueError, "unsupported route corridor mode"):
            constrain_trajectory_points_to_route(points, route, 0.0, 0.5, mode="bad")

    def test_temporal_offset_filter_anchors_first_point_and_blends_overlap(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(float(index), -0.4, float(index), 1.0)
            for index in range(3)
        ]
        previous = RouteOffsetProfile(
            progress_m=(0.0, 1.0, 2.0),
            lateral_offset_m=(0.4, 0.4, 0.4),
        )

        profile, correction = stabilize_trajectory_lateral_offsets(
            points,
            route,
            progress_m=0.0,
            previous_profile=previous,
            current_gain=0.5,
            anchor_distance_m=1.0,
            corridor_half_width_m=0.5,
        )

        self.assertAlmostEqual(points[0].pose.position.y, -0.4)
        self.assertGreater(points[1].pose.position.y, -0.4)
        self.assertLess(points[1].pose.position.y, 0.4)
        self.assertGreater(points[2].pose.position.y, points[1].pose.position.y)
        self.assertEqual(profile.progress_m, (0.0, 1.0, 2.0))
        self.assertGreater(correction, 0.0)

    def test_temporal_filter_preserves_corridor_entry_envelope(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(float(index), 0.9, float(index), 1.0)
            for index in range(7)
        ]
        constrain_trajectory_points_to_route(
            points,
            route,
            progress_m=0.0,
            corridor_half_width_m=0.5,
            lateral_offset_min_m=-0.5,
            lateral_offset_max_m=0.2,
            entry_distance_m=5.0,
        )
        point_zero = copy.deepcopy(points[0].pose.position)
        previous = RouteOffsetProfile(
            progress_m=tuple(float(index) for index in range(7)),
            lateral_offset_m=tuple(-0.4 for _ in range(7)),
        )

        stabilize_trajectory_lateral_offsets(
            points,
            route,
            progress_m=0.0,
            previous_profile=previous,
            current_gain=0.75,
            anchor_distance_m=2.0,
            corridor_half_width_m=0.5,
            lateral_offset_min_m=-0.5,
            lateral_offset_max_m=0.2,
            entry_distance_m=5.0,
        )

        self.assertEqual(points[0].pose.position, point_zero)
        distances = trajectory_arc_lengths(points)
        self.assertTrue(
            all(
                -0.5 - 1.0e-9 <= point.pose.position.y <= 0.2 + 1.0e-9
                for point, distance in zip(points, distances)
                if distance >= 5.0
            )
        )

    def test_temporal_filter_outlier_gate_skips_small_overlapping_update(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(float(index), -0.1, float(index), 1.0)
            for index in range(3)
        ]
        previous = RouteOffsetProfile(
            progress_m=(0.0, 1.0, 2.0),
            lateral_offset_m=(0.05, 0.05, 0.05),
        )

        profile, correction = stabilize_trajectory_lateral_offsets(
            points,
            route,
            progress_m=0.0,
            previous_profile=previous,
            current_gain=0.5,
            anchor_distance_m=1.0,
            corridor_half_width_m=0.5,
            activation_threshold_m=0.2,
        )

        self.assertEqual(correction, 0.0)
        self.assertTrue(
            all(abs(point.pose.position.y + 0.1) <= 1.0e-9 for point in points)
        )
        self.assertEqual(profile.lateral_offset_m, (-0.1, -0.1, -0.1))

    def test_temporal_filter_outlier_gate_blends_large_overlapping_update(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(10.0, 0.0, 0.0, 0.0, 10.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(float(index), -0.4, float(index), 1.0)
            for index in range(3)
        ]
        previous = RouteOffsetProfile(
            progress_m=(0.0, 1.0, 2.0),
            lateral_offset_m=(0.4, 0.4, 0.4),
        )

        _, correction = stabilize_trajectory_lateral_offsets(
            points,
            route,
            progress_m=0.0,
            previous_profile=previous,
            current_gain=0.5,
            anchor_distance_m=1.0,
            corridor_half_width_m=0.5,
            activation_threshold_m=0.2,
        )

        self.assertGreater(correction, 0.0)
        self.assertAlmostEqual(points[0].pose.position.y, -0.4)
        self.assertGreater(points[2].pose.position.y, -0.4)

        with self.assertRaisesRegex(ValueError, "activation threshold"):
            stabilize_trajectory_lateral_offsets(
                points,
                route,
                progress_m=0.0,
                previous_profile=previous,
                current_gain=0.5,
                anchor_distance_m=1.0,
                corridor_half_width_m=0.5,
                activation_threshold_m=math.nan,
            )

    def test_temporal_filter_default_bounds_keep_legacy_radial_projection(self):
        route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(5.0, 0.0, 0.0, 0.0, 5.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(index + 0.4, 0.8, float(index), 1.0)
            for index in range(3)
        ]

        stabilize_trajectory_lateral_offsets(
            points,
            route,
            progress_m=0.0,
            previous_profile=None,
            current_gain=0.75,
            anchor_distance_m=2.0,
            corridor_half_width_m=0.5,
        )

        scale = 0.5 / math.hypot(0.4, 0.8)
        for index, point in enumerate(points):
            self.assertAlmostEqual(point.pose.position.x, index + 0.4 * scale)
            self.assertAlmostEqual(point.pose.position.y, 0.8 * scale)

    def test_curvature_speed_cap_limits_lateral_acceleration(self):
        radius = 1.0
        points = [
            make_trajectory_point(
                radius * math.cos(angle),
                radius * math.sin(angle),
                float(index),
                3.0,
            )
            for index, angle in enumerate(
                (0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0, math.pi)
            )
        ]

        reduction = limit_trajectory_speed_for_curvature(
            points,
            maximum_lateral_acceleration_mps2=1.0,
            preview_distance_m=3.0,
            comfortable_deceleration_mps2=1.2,
        )

        self.assertGreater(reduction, 1.9)
        self.assertTrue(
            all(point.longitudinal_velocity_mps <= 1.0 + 1.0e-6 for point in points)
        )

    def test_acceleration_cap_prevents_curve_exit_speed_jump(self):
        points = [
            make_trajectory_point(float(index) * 0.5, 0.0, index * 0.1, 8.5)
            for index in range(6)
        ]

        reduction = limit_trajectory_speed_for_acceleration(
            points,
            maximum_acceleration_mps2=2.0,
            initial_speed_mps=3.0,
            initial_distance_m=0.5,
        )

        self.assertGreater(reduction, 4.0)
        previous_speed = 3.0
        for point in points:
            self.assertLessEqual(
                point.longitudinal_velocity_mps**2 - previous_speed**2,
                2.0 * 2.0 * 0.5 + 1.0e-9,
            )
            previous_speed = point.longitudinal_velocity_mps

    def test_acceleration_cap_rejects_invalid_contract(self):
        point = make_trajectory_point(0.0, 0.0, 0.0, 1.0)
        with self.assertRaisesRegex(ValueError, "must be positive"):
            limit_trajectory_speed_for_acceleration([point], 0.0, 0.0, 0.5)
        with self.assertRaisesRegex(ValueError, "must be non-negative"):
            limit_trajectory_speed_for_acceleration([point], 1.0, -1.0, 0.5)

    def test_speed_recovery_cap_leaves_first_point_and_limits_curve_exit(self):
        points = [
            make_trajectory_point(float(index), 0.0, index * 0.1, speed)
            for index, speed in enumerate((2.0, 8.333, 8.333, 8.333))
        ]

        reduction = limit_trajectory_speed_recovery(points, 1.5)

        self.assertGreater(reduction, 4.0)
        self.assertEqual(points[0].longitudinal_velocity_mps, 2.0)
        for first, second in zip(points, points[1:]):
            self.assertLessEqual(
                second.longitudinal_velocity_mps**2
                - first.longitudinal_velocity_mps**2,
                2.0 * 1.5 * 1.0 + 1.0e-9,
            )

    def test_route_curvature_cap_sees_turn_beyond_candidate_horizon(self):
        route = RoutePlan(
            {"scenario": "left"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(15.0, 0.0, 0.0, 0.0, 15.0, 3, "LANEFOLLOW"),
                RoutePoint(16.0, 0.0, 0.0, 0.0, 16.0, 0, "LEFT"),
                RoutePoint(16.0, 1.0, 0.0, math.pi / 2.0, 17.0, 0, "LEFT"),
                RoutePoint(16.0, 10.0, 0.0, math.pi / 2.0, 26.0, 3, "LANEFOLLOW"),
            ],
        )
        points = [
            make_trajectory_point(float(index), 0.0, index * 0.1, 8.333)
            for index in range(4)
        ]

        reduction = limit_trajectory_speed_for_route_curvature(
            points,
            route,
            progress_m=0.0,
            maximum_lateral_acceleration_mps2=1.2,
            lookahead_distance_m=20.0,
            comfortable_deceleration_mps2=2.0,
        )

        self.assertGreater(reduction, 0.1)
        self.assertLess(points[0].longitudinal_velocity_mps, 8.333)

    def test_recalculated_acceleration_uses_asymmetric_speed_profile_limits(self):
        points = [
            make_trajectory_point(0.0, 0.0, 0.0, 1.0),
            make_trajectory_point(0.5, 0.0, 0.1, 3.0),
            make_trajectory_point(1.0, 0.0, 0.2, 0.0),
        ]
        manager = SimpleNamespace(
            maximum_longitudinal_acceleration_mps2=1.5,
            comfortable_deceleration_mps2=2.0,
        )

        VadRouteManager._recalculate_acceleration(
            manager, SimpleNamespace(points=points)
        )

        self.assertEqual(points[0].acceleration_mps2, 1.5)
        self.assertEqual(points[1].acceleration_mps2, -2.0)
        self.assertEqual(points[2].acceleration_mps2, 0.0)

    def test_sample_distances_include_exact_stop_and_end(self):
        self.assertEqual(
            trajectory_sample_distances(2.0, 0.5, extra_distances=(1.2, 1.0)),
            [0.0, 0.5, 1.0, 1.2, 1.5, 2.0],
        )

    def test_near_grid_stop_replaces_grid_value(self):
        stop_distance = 1.0000005
        samples = trajectory_sample_distances(
            2.0, 0.5, extra_distances=(stop_distance,)
        )
        self.assertIn(stop_distance, samples)
        self.assertNotIn(1.0, samples)

    def test_controller_stop_offset_places_zero_point_past_tolerance(self):
        self.assertAlmostEqual(
            zero_velocity_distance_for_goal(1.47, 1.0, 0.49), 0.96
        )

    def test_resampling_interpolates_fields_and_normalizes_quaternion(self):
        points = [
            make_trajectory_point(0.0, 0.0, 0.0, 2.0),
            make_trajectory_point(
                2.0,
                0.0,
                2.0,
                4.0,
                quaternion=(0.0, 0.0, 1.0, 0.0),
            ),
        ]
        result = resample_trajectory_points(points, 0.5, extra_distances=(1.2,))

        self.assertEqual([point.pose.position.x for point in result], [0.0, 0.5, 1.0, 1.2, 1.5, 2.0])
        middle = result[2]
        self.assertAlmostEqual(middle.longitudinal_velocity_mps, 3.0)
        self.assertEqual(middle.time_from_start.sec, 1)
        self.assertEqual(middle.time_from_start.nanosec, 0)
        orientation = middle.pose.orientation
        self.assertAlmostEqual(
            orientation.x**2 + orientation.y**2 + orientation.z**2 + orientation.w**2,
            1.0,
        )

    def test_resampling_uses_shortest_quaternion_sign(self):
        points = [
            make_trajectory_point(0.0, 0.0, 0.0, 1.0),
            make_trajectory_point(
                1.0,
                0.0,
                1.0,
                1.0,
                quaternion=(0.0, 0.0, 0.0, -1.0),
            ),
        ]
        result = resample_trajectory_points(points, 0.5)
        self.assertAlmostEqual(result[1].pose.orientation.w, 1.0)

    def test_resampling_normalizes_endpoints(self):
        points = [
            make_trajectory_point(
                0.0, 0.0, 0.0, 1.0, quaternion=(0.0, 0.0, 0.0, 0.5)
            ),
            make_trajectory_point(
                1.0, 0.0, 1.0, 1.0, quaternion=(0.0, 0.0, 0.0, 1.5)
            ),
        ]
        result = resample_trajectory_points(points, 0.5)
        self.assertAlmostEqual(result[0].pose.orientation.w, 1.0)
        self.assertAlmostEqual(result[-1].pose.orientation.w, 1.0)

    def test_resampling_rejects_zero_length_path(self):
        points = [
            make_trajectory_point(0.0, 0.0, 0.0, 0.0),
            make_trajectory_point(0.0, 0.0, 1.0, 0.0),
        ]
        with self.assertRaisesRegex(ValueError, "no planar extent"):
            resample_trajectory_points(points, 0.5)


class TrajectoryGeometrySmoothingTest(unittest.TestCase):
    def setUp(self):
        self.route = RoutePlan(
            {"scenario": "straight"},
            [
                RoutePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3, "LANEFOLLOW"),
                RoutePoint(20.0, 0.0, 0.0, 0.0, 20.0, 3, "LANEFOLLOW"),
            ],
        )

    @staticmethod
    def _zigzag_points():
        return [
            make_trajectory_point(float(index), y, float(index), 1.0 + index)
            for index, y in enumerate((0.0, 0.30, -0.30, 0.30, 0.0))
        ]

    def _smooth(
        self,
        points,
        strength=100.0,
        interval_m=0.25,
        max_deviation_m=5.0,
        corridor_half_width_m=5.0,
        stop_distance_m=None,
    ):
        return smooth_trajectory_geometry(
            points,
            self.route,
            progress_m=0.0,
            strength=strength,
            interval_m=interval_m,
            max_deviation_m=max_deviation_m,
            corridor_half_width_m=corridor_half_width_m,
            stop_distance_m=stop_distance_m,
        )

    def test_zero_strength_is_exact_no_op_even_for_degenerate_input(self):
        points = [make_trajectory_point(1.0, 2.0, 0.0, 3.0)]
        original = copy.deepcopy(points)

        stop_distance = 0.75
        result = self._smooth(
            points,
            strength=0.0,
            interval_m=math.nan,
            max_deviation_m=math.nan,
            stop_distance_m=stop_distance,
        )

        self.assertEqual(result.correction_m, 0.0)
        self.assertEqual(result.stop_distance_m, stop_distance)
        self.assertEqual(points, original)

    def test_endpoints_and_exact_stop_distance_point_are_fixed(self):
        points = self._zigzag_points()
        source_distances = trajectory_arc_lengths(points)
        stop_distance = source_distances[2]
        fixed_positions = (
            (points[0].pose.position.x, points[0].pose.position.y),
            (points[2].pose.position.x, points[2].pose.position.y),
            (points[-1].pose.position.x, points[-1].pose.position.y),
        )

        result = self._smooth(points, stop_distance_m=stop_distance)

        stop_index = min(
            range(len(points)),
            key=lambda index: math.hypot(
                points[index].pose.position.x - fixed_positions[1][0],
                points[index].pose.position.y - fixed_positions[1][1],
            ),
        )
        actual_positions = (
            (points[0].pose.position.x, points[0].pose.position.y),
            (
                points[stop_index].pose.position.x,
                points[stop_index].pose.position.y,
            ),
            (points[-1].pose.position.x, points[-1].pose.position.y),
        )
        for actual, expected in zip(actual_positions, fixed_positions):
            self.assertAlmostEqual(actual[0], expected[0], places=8)
            self.assertAlmostEqual(actual[1], expected[1], places=8)
        self.assertAlmostEqual(
            result.stop_distance_m,
            trajectory_arc_lengths(points)[stop_index],
            places=9,
        )

    def test_velocity_profile_starts_zero_speed_at_fixed_stop_anchor(self):
        points = self._zigzag_points()
        stop_position = (
            points[2].pose.position.x,
            points[2].pose.position.y,
        )
        original_stop_distance = trajectory_arc_lengths(points)[2]

        result = self._smooth(
            points, stop_distance_m=original_stop_distance
        )
        stop_index = min(
            range(len(points)),
            key=lambda index: math.hypot(
                points[index].pose.position.x - stop_position[0],
                points[index].pose.position.y - stop_position[1],
            ),
        )
        manager = SimpleNamespace(
            comfortable_deceleration_mps2=1.2,
            maximum_speed_mps=10.0,
            longitudinal_velocity_source="vad_prediction",
            nominal_cruise_speed_mps=0.0,
        )
        smoothed_distances = trajectory_arc_lengths(points)
        legacy_zero_index = next(
            index
            for index, distance in enumerate(smoothed_distances)
            if distance >= original_stop_distance - 1.0e-3
        )

        VadRouteManager._apply_velocity_profile(
            manager, points, result.stop_distance_m
        )

        zero_indices = [
            index
            for index, point in enumerate(points)
            if point.longitudinal_velocity_mps == 0.0
        ]
        self.assertNotEqual(legacy_zero_index, stop_index)
        self.assertEqual(zero_indices[0], stop_index)
        self.assertGreater(points[stop_index - 1].longitudinal_velocity_mps, 0.0)
        self.assertAlmostEqual(
            trajectory_arc_lengths(points)[stop_index],
            result.stop_distance_m,
            places=9,
        )
        self.assertNotAlmostEqual(
            result.stop_distance_m, original_stop_distance, places=3
        )
        self.assertGreater(
            abs(result.stop_distance_m - original_stop_distance), 0.15
        )

    def test_explicit_nominal_overlay_sets_cruise_and_preserves_hard_stop(self):
        manager = SimpleNamespace(
            comfortable_deceleration_mps2=2.0,
            maximum_speed_mps=8.333333333333334,
            longitudinal_velocity_source="explicit_simulation_nominal",
            nominal_cruise_speed_mps=8.333333333333334,
        )
        cruise = [
            make_trajectory_point(float(index), 0.0, index * 0.1, 2.5)
            for index in range(4)
        ]
        VadRouteManager._apply_velocity_profile(manager, cruise, 100.0)
        self.assertTrue(
            all(
                point.longitudinal_velocity_mps == 8.333333333333334
                for point in cruise
            )
        )

        hard_stop = [
            make_trajectory_point(float(index), 0.0, index * 0.1, speed)
            for index, speed in enumerate((2.5, 2.5, 0.0, 2.5))
        ]
        VadRouteManager._apply_velocity_profile(manager, hard_stop, 100.0)
        self.assertEqual(hard_stop[2].longitudinal_velocity_mps, 0.0)
        self.assertEqual(hard_stop[3].longitudinal_velocity_mps, 0.0)
        self.assertLess(hard_stop[1].longitudinal_velocity_mps, 8.333333333333334)

    def test_straight_geometry_remains_straight(self):
        points = [
            make_trajectory_point(float(index), 0.2, float(index), 2.0)
            for index in range(5)
        ]

        result = self._smooth(points)

        self.assertLess(result.correction_m, 1.0e-10)
        self.assertTrue(
            all(abs(point.pose.position.y - 0.2) < 1.0e-10 for point in points)
        )
        self.assertTrue(
            all(abs(point.pose.orientation.z) < 1.0e-10 for point in points)
        )

    def test_smoothing_reduces_curvature_roughness(self):
        points = self._zigzag_points()
        baseline = resample_trajectory_points(points, 0.25)
        baseline_roughness = sum(
            abs(value) for value in trajectory_planar_curvatures(baseline)
        )

        result = self._smooth(points)
        smoothed_roughness = sum(
            abs(value) for value in trajectory_planar_curvatures(points)
        )

        self.assertGreater(result.correction_m, 0.0)
        self.assertLess(smoothed_roughness, baseline_roughness * 0.5)

    def test_smoothed_points_remain_inside_route_corridor(self):
        points = [
            make_trajectory_point(float(index), y, float(index), 2.0)
            for index, y in enumerate((0.45, -0.45, 0.45, -0.45, 0.45))
        ]

        self._smooth(
            points,
            strength=300.0,
            max_deviation_m=1.0,
            corridor_half_width_m=0.5,
        )

        self.assertLessEqual(
            max(abs(point.pose.position.y) for point in points), 0.5 + 1.0e-9
        )

    def test_smoothing_preserves_fixed_point_inside_entry_envelope(self):
        points = [
            make_trajectory_point(float(index), offset, float(index), 2.0)
            for index, offset in enumerate((0.9, 0.75, 0.45, 0.25, 0.2, 0.2, 0.2))
        ]
        first_position = copy.deepcopy(points[0].pose.position)

        result = smooth_trajectory_geometry(
            points,
            self.route,
            progress_m=0.0,
            strength=10.0,
            interval_m=0.25,
            max_deviation_m=0.2,
            corridor_half_width_m=0.5,
            lateral_offset_min_m=-0.5,
            lateral_offset_max_m=0.2,
            entry_distance_m=5.0,
        )

        self.assertGreaterEqual(result.correction_m, 0.0)
        self.assertEqual(points[0].pose.position, first_position)
        distances = trajectory_arc_lengths(points)
        self.assertTrue(
            all(
                point.pose.position.y <= 0.2 + 1.0e-6
                for point, distance in zip(points, distances)
                if distance >= 5.0
            )
        )

    def test_fixed_endpoint_or_stop_outside_corridor_is_transactional(self):
        cases = (
            (
                "endpoint",
                (0.6, 0.1, 0.0, 0.0, 0.0),
                None,
            ),
            (
                "stop",
                (0.0, 0.1, 0.6, 0.1, 0.0),
                2.0 * math.hypot(1.0, 0.1) + math.hypot(1.0, 0.5),
            ),
        )
        for label, offsets, stop_distance_m in cases:
            with self.subTest(label=label):
                points = [
                    make_trajectory_point(float(index), offset, float(index), 2.0)
                    for index, offset in enumerate(offsets)
                ]
                if label == "stop":
                    stop_distance_m = trajectory_arc_lengths(points)[2]
                original = copy.deepcopy(points)

                with self.assertRaisesRegex(ValueError, "fixed point .* outside corridor"):
                    self._smooth(
                        points,
                        strength=10.0,
                        max_deviation_m=1.0,
                        corridor_half_width_m=0.5,
                        stop_distance_m=stop_distance_m,
                    )

                self.assertEqual(points, original)

    def test_enabled_pass_preserves_interpolated_non_pose_fields(self):
        points = self._zigzag_points()
        stop_distance = trajectory_arc_lengths(points)[2]
        expected = resample_trajectory_points(
            points, 0.25, extra_distances=(stop_distance,)
        )

        self._smooth(points, stop_distance_m=stop_distance)

        self.assertEqual(len(points), len(expected))
        for actual, reference in zip(points, expected):
            self.assertEqual(actual.time_from_start, reference.time_from_start)
            for field in (
                "longitudinal_velocity_mps",
                "lateral_velocity_mps",
                "acceleration_mps2",
                "heading_rate_rps",
                "front_wheel_angle_rad",
                "rear_wheel_angle_rad",
            ):
                self.assertEqual(getattr(actual, field), getattr(reference, field))

    def test_deviation_guard_is_transactional(self):
        points = self._zigzag_points()
        original = copy.deepcopy(points)

        with self.assertRaisesRegex(ValueError, "exceeds .* guard"):
            self._smooth(points, max_deviation_m=0.001)

        self.assertEqual(points, original)

    def test_manager_rejection_warns_and_fails_open_without_fault(self):
        points = self._zigzag_points()
        original = copy.deepcopy(points)
        warnings = []
        manager = SimpleNamespace(
            trajectory_geometry_smoothing_strength=100.0,
            trajectory_geometry_smoothing_interval_m=0.25,
            trajectory_geometry_smoothing_max_deviation_m=0.001,
            route=self.route,
            progress_m=0.0,
            route_corridor_half_width_m=5.0,
            route_corridor_mode="hard",
            route_corridor_entry_distance_m=0.0,
            get_logger=lambda: SimpleNamespace(warning=warnings.append),
        )

        result = VadRouteManager._apply_geometry_smoothing(
            manager,
            points,
            stop_distance_m=2.0,
            lateral_offset_min_m=-5.0,
            lateral_offset_max_m=5.0,
        )

        self.assertEqual(result.correction_m, 0.0)
        self.assertEqual(result.stop_distance_m, 2.0)
        self.assertEqual(points, original)
        self.assertEqual(len(warnings), 1)
        self.assertIn("keeping conditioned VAD geometry", warnings[0])
        self.assertFalse(hasattr(manager, "fault"))

    def test_enabled_pass_rejects_degenerate_and_invalid_inputs(self):
        with self.assertRaisesRegex(ValueError, "at least three"):
            self._smooth(self._zigzag_points()[:2])
        overlapping = [
            make_trajectory_point(0.0, 0.0, float(index), 1.0)
            for index in range(3)
        ]
        with self.assertRaisesRegex(ValueError, "no planar extent"):
            self._smooth(overlapping)
        with self.assertRaisesRegex(ValueError, "finite and non-negative"):
            self._smooth(self._zigzag_points(), strength=math.nan)
        with self.assertRaisesRegex(ValueError, "anchor index"):
            endpoint_fixed_whittaker(
                [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
                1.0,
                fixed_indices=(3,),
            )

    def test_manager_maps_turn_corridor_bounds_by_command(self):
        corridor = 0.5
        inward = 0.2
        left_outward = 0.5
        right_outward = 0.3

        self.assertEqual(
            VadRouteManager._lateral_corridor_bounds(
                0, corridor, inward, left_outward, right_outward
            ),
            (-left_outward, inward),
        )
        self.assertEqual(
            VadRouteManager._lateral_corridor_bounds(
                1, corridor, inward, left_outward, right_outward
            ),
            (-inward, right_outward),
        )
        for command in (2, 3, 4, 5):
            with self.subTest(command=command):
                self.assertEqual(
                    VadRouteManager._lateral_corridor_bounds(
                        command,
                        corridor,
                        inward,
                        left_outward,
                        right_outward,
                    ),
                    (-corridor, corridor),
                )

    def test_manager_resolves_optional_turn_corridor_overrides(self):
        resolve = VadRouteManager._resolve_turn_outward_corridor_width
        self.assertEqual(resolve(0.5, 0.0, "left"), 0.5)
        self.assertEqual(resolve(0.5, 0.3, "right"), 0.3)
        for override in (-0.1, math.nan, 0.6):
            with self.subTest(override=override):
                with self.assertRaises(RuntimeError):
                    resolve(0.5, override, "test_override")

    def test_manager_resolves_directional_filter_gains(self):
        resolve = VadRouteManager._resolve_trajectory_lateral_filter_gain
        self.assertEqual(resolve(1.0, 0.0, "left"), 1.0)
        self.assertEqual(resolve(1.0, 0.75, "right"), 0.75)
        for override in (-0.1, math.nan, 1.1):
            with self.subTest(override=override):
                with self.assertRaises(RuntimeError):
                    resolve(1.0, override, "test_override")

        manager = SimpleNamespace(
            command=0,
            trajectory_lateral_filter_gain=1.0,
            left_turn_trajectory_lateral_filter_gain=0.8,
            right_turn_trajectory_lateral_filter_gain=0.7,
        )
        gain_for_command = VadRouteManager._trajectory_lateral_filter_gain_for_command
        self.assertEqual(gain_for_command(manager), 0.8)
        manager.command = 1
        self.assertEqual(gain_for_command(manager), 0.7)
        manager.command = 3
        self.assertEqual(gain_for_command(manager), 1.0)

    def test_minimal_and_full_launch_propagate_opt_in_parameters(self):
        launch_directory = Path(__file__).resolve().parents[1] / "launch"
        expected_defaults = {
            "left_turn_outward_corridor_half_width_m": "0.0",
            "left_turn_trajectory_lateral_filter_gain": "0.0",
            "right_turn_outward_corridor_half_width_m": "0.0",
            "right_turn_trajectory_lateral_filter_gain": "0.0",
            "route_corridor_entry_distance_m": "0.0",
            "trajectory_lateral_filter_activation_threshold_m": "0.0",
            "turn_outward_corridor_half_width_m": "0.5",
            "trajectory_geometry_smoothing_strength": "0.0",
            "trajectory_geometry_smoothing_interval_m": "0.25",
            "trajectory_geometry_smoothing_max_deviation_m": "0.10",
            "longitudinal_velocity_source": "vad_prediction",
            "nominal_cruise_speed_mps": "0.0",
            "route_curvature_lookahead_m": "0.0",
        }
        for launch_name in ("carla_vad.launch.xml", "carla_vad_full.launch.xml"):
            with self.subTest(launch_name=launch_name):
                root = ET.parse(launch_directory / launch_name).getroot()
                arguments = {
                    element.attrib["name"]: element.attrib.get("default")
                    for element in root.findall("arg")
                }
                parameters = {
                    element.attrib["name"]: element.attrib.get("value")
                    for element in root.findall(".//param")
                    if "name" in element.attrib
                }
                for name, default in expected_defaults.items():
                    self.assertEqual(arguments.get(name), default)
                    self.assertEqual(parameters.get(name), f"$(var {name})")


if __name__ == "__main__":
    unittest.main()
