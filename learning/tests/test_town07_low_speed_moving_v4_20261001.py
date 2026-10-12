"""HH_261001 - Synthetic V4 closure, frame-reference, and denominator checks."""

import hashlib
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.e2e import audit_town07_low_speed_moving_v4_20261001 as qa
from scripts.e2e import town07_low_speed_moving_v4_task_20261001 as task
from scripts.e2e import town07_moving_external_sources_v4_20261001 as sources


class V4SyntheticChecks(unittest.TestCase):
    def test_fresh_identity_rebinding_restores_frozen_task(self):
        original = {name: getattr(task.original, name) for name in task.PATCHED}
        with task.activated():
            self.assertEqual(task.original.RUN, task.RUN)
            self.assertEqual(task.original.PLAN, task.PLAN)
            self.assertEqual(task.original.SLOT, task.SLOT)
            self.assertIn('tools/e2e/carla_physics_substeps.py', task.original.SOURCE_PATHS)
        self.assertEqual({name: getattr(task.original, name) for name in original}, original)

    def test_actor_center_speed_uses_yaw_rate_offset_without_relaxing_tolerance(self):
        vx, center_vy, yaw_rate = 2.0, .13, .04
        state = dict(vx=vx, vy=center_vy - 1.425 * yaw_rate,
                     yaw_rate=yaw_rate, world_velocity_carla=[vx, center_vy, 0.])
        speed = qa.center_speed_mps(state, 2.85)
        self.assertEqual(speed, math.hypot(vx, center_vy))
        self.assertGreater(abs(speed - math.hypot(state['vx'], state['vy'])), 1e-9)
        state['world_velocity_carla'][1] += 1e-6
        with self.assertRaises(ValueError):
            qa.center_speed_mps(state, 2.85)

    def test_geometry_future_rows_are_not_mistaken_for_summary(self):
        cameras = [dict(frame=1), dict(frame=3)]
        summary = dict(camera_anchor_count=2)
        rows = [dict(frame=1), dict(frame=3)]
        def provider(*_args):
            return summary, rows
        actual_rows, actual_summary = qa.geometry_rows_first([], cameras, {}, [], 'slot', provider)
        self.assertEqual(actual_rows, rows)
        self.assertEqual(actual_summary, summary)

    def test_declared_unused_source_is_archived_before_owner_plan(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            run = root / 'fresh_v4'
            run.mkdir()
            relative = 'tools/e2e/carla_physics_substeps.py'
            current = root / relative
            current.parent.mkdir(parents=True)
            current.write_bytes(b'fixed inherited physics source')
            digest = hashlib.sha256(current.read_bytes()).hexdigest()
            addendum = dict(source_sha256={relative: digest})
            owner_plan = dict(source_sha256={})
            with patch.object(task, 'ROOT', root), patch.object(task, 'RUN', run):
                sources.archive_declared_sources(run, owner_plan, addendum)
                self.assertEqual(owner_plan['source_sha256'][relative], digest)
                sources.verify_owner_archive(run, owner_plan, addendum)
                (run / 'source' / relative).write_bytes(b'changed')
                with self.assertRaises(ValueError):
                    sources.verify_owner_archive(run, owner_plan, addendum)

    def test_full_fixed_denominator_fails_closed(self):
        numerical = dict(status=qa.CLEAR, selected_window_count=326)
        with patch.object(qa, 'numerical_audit', return_value=(numerical, {})), \
             patch.object(qa, 'source_audit', return_value=(dict(status='PASS'), {})):
            summary, ledgers = qa.audit(task.RUN, 'plan', 'addendum', 'owner', 'manifest')
        self.assertEqual(summary['status'], 'BLOCKED_NECESSARY_CHECKS')
        self.assertEqual(summary['selected_window_count'], 326)
        self.assertEqual(ledgers, {})

    def test_frozen_auditor_is_restored_after_adapted_call(self):
        old_plan = qa.frozen.PLAN_SHA
        old_control = qa.frozen.control_audit
        old_geometry = qa.frozen.geometry.geometry_futures
        def inspect_call(*_args):
            self.assertEqual(qa.frozen.PLAN_SHA, 'synthetic-plan')
            self.assertIs(qa.frozen.control_audit, qa.control_audit)
            self.assertIs(qa.frozen.geometry.geometry_futures, qa.geometry_rows_first)
            self.assertEqual(task.original.RUN, task.RUN)
            return dict(status='SYNTHETIC'), {}
        with patch.object(task, 'verify_plan', return_value={}), \
             patch.object(qa.frozen, 'audit', side_effect=inspect_call):
            qa.numerical_audit(task.RUN, 'synthetic-plan', 'owner', 'manifest')
        self.assertEqual(qa.frozen.PLAN_SHA, old_plan)
        self.assertIs(qa.frozen.control_audit, old_control)
        self.assertIs(qa.frozen.geometry.geometry_futures, old_geometry)


if __name__ == '__main__':
    unittest.main()
