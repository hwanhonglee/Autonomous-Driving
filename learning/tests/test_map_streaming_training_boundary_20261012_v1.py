"""HH_261012 - Exercise bounded lifecycle chronology with standard-library mocks only.

No real TRAIN archive, NumPy/Torch/Pillow provider, NN, optimizer, GPU or SSH is
used. Callback phase labels and hashes below are fictional fixture metadata.
"""
from copy import deepcopy
import hashlib
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from portable_e2e import map_streaming_training_boundary_20261012_v1 as boundary


def row(step):
    return dict(step=step, physical_slot_indices=list(range(8)),
        shared_crop_boxes=[[0, 0, 360, 640] for _ in range(8)],
        actual_paired_TRUE_bits=[0] * 8, RGB_sha256="a" * 64, target_sha256="b" * 64)


class Reader:
    def __init__(self, failure=None):
        self.closes, self.reads, self.failure, self.closing_errors = 0, 0, failure, []

    def read(self, slot):
        if self.closes:
            raise ValueError("outer reader was closed early")
        self.reads += 1
        return slot, slot

    def close(self):
        self.closes += 1
        if self.failure:
            raise ValueError(self.failure)


class Adapter:
    def __init__(self, rows, reader, crop, *, mode, checkpoint, harness):
        self.rows, self.reader, self.mode, self.checkpoint, self.harness = rows, reader, mode, checkpoint, harness
        self.cursor, self.pending, self.produced, self.acked, self.closed = 0, None, 0, 0, False
        self.aborted, self.close_calls = False, 0

    def next_batch(self):
        step = self.rows[self.cursor]["step"]
        self.pending = step
        for slot in self.rows[self.cursor]["physical_slot_indices"]:
            self.reader.read(slot)
        self.produced += 1
        self.harness.maximum_pending = max(self.harness.maximum_pending, 1)
        if self.harness.fault == "produced_order":
            step += 1
        return SimpleNamespace(step=step, receipt=dict(step=step, mode=self.mode,
            selected_crop_boxes=deepcopy(self.rows[self.cursor]["shared_crop_boxes"])))

    def acknowledge(self, step, *, consumer_receipt):
        if self.harness.fault == "ACK":
            raise ValueError("ACK denied")
        if step != self.pending:
            raise ValueError("pending chronology differs")
        self.cursor += 1
        self.acked += 1
        self.pending = None
        result = dict(step=step, consumer_receipt=deepcopy(consumer_receipt),
            native_and_host_buffers_rehashed=True, consumer_execution_authenticated=False)
        if self.harness.fault == "ACK_receipt":
            result["native_and_host_buffers_rehashed"] = 1
        return result

    def abort(self, error, *, step=None):
        self.aborted = True

    def close(self):
        self.close_calls += 1
        self.closed = True
        self.reader.close()
        if self.harness.fault == "adapter_close":
            raise ValueError("adapter close denied")
        return dict(closed=True, PASS_batches=self.acked,
            FAIL_batches=int(self.aborted and self.pending is not None),
            UNEXECUTED_batches=len(self.rows) - self.acked - int(self.aborted and self.pending is not None),
            pending_step=self.pending, cleanup_errors=[], acknowledgements=[dict(step=v["step"]) for v in self.rows[:self.acked]])


class Journal:
    def __init__(self, harness):
        self.harness, self.records, self.closed, self.close_calls = harness, [], False, 0

    def append(self, record):
        event = record["event"]
        if self.harness.fault == "update_journal" and event == "OPTIMIZER_RETURNED_NOT_ACKED":
            raise ValueError("update write denied")
        if self.harness.fault == "ACK_journal" and event == "SERIAL_ACK_RETURNED":
            raise ValueError("ACK write denied")
        self.records.append(deepcopy(record))
        self.harness.maximum_journal_rows = max(self.harness.maximum_journal_rows, len(self.records))
        pin = boundary.metadata_sha(record)
        result = dict(written=True, flushed=True, step=record["step"], record_sha256=pin)
        if self.harness.fault == "not_flushed":
            result["flushed"] = False
        if self.harness.fault == "bool_coercion":
            result["written"] = 1
        if self.harness.fault == "journal_pin":
            result["record_sha256"] = "c" * 64
        if self.harness.fault == "journal_mutation":
            record["mode"] = "mutated"
        if self.harness.fault == "input_mutation" and event == "OPTIMIZER_RETURNED_NOT_ACKED":
            self.harness.input_token += 1
        return result

    def close(self):
        self.closed = True
        self.close_calls += 1
        if self.harness.fault == "journal_close":
            raise ValueError("journal close denied")
        return dict(closed=True, rows_written=len(self.records), rows_flushed=len(self.records), closing_errors=[])


class Harness:
    def __init__(self, fault=None):
        self.fault, self.last_adapter, self.last_journal = fault, None, None
        self.maximum_chunk_rows = self.maximum_journal_rows = self.maximum_pending = 0
        self.adapter_calls, self.journal_calls, self.callback_calls, self.input_token = 0, 0, 0, 0
        self.owner = None

    def adapter(self, rows, reader, crop, *, mode, checkpoint):
        self.adapter_calls += 1
        self.maximum_chunk_rows = max(self.maximum_chunk_rows, len(rows))
        self.last_adapter = Adapter(rows, reader, crop, mode=mode, checkpoint=checkpoint, harness=self)
        return self.last_adapter

    def journal(self, ordinal):
        self.journal_calls += 1
        self.last_journal = Journal(self)
        return self.last_journal

    def build(self, source, *, baseline, mode, checkpoint):
        if self.fault == "build":
            raise ValueError("CPU build denied")
        return SimpleNamespace(step=source.step, mode=mode)

    def verify(self, bundle, *, checkpoint):
        if self.fault == "verify":
            raise ValueError("CPU verify denied")
        return dict(step=bundle.step, mode=bundle.mode, token=self.input_token,
            consumer_receipt=dict(consumer="fictional_CPU_consumer", accepted=True,
                                 host_RGB_sha256="a" * 64, host_target_sha256="b" * 64))

    def consume(self, bundle, mark):
        self.callback_calls += 1
        for phase in boundary.PHASES:
            if self.fault == "NN" and phase == "FORWARD_CHECKED":
                raise ValueError("forward failed")
            if self.fault == "optimizer" and phase == "OPTIMIZER_RETURNED":
                raise ValueError("optimizer raised after a possible partial write")
            if self.fault == "phase_order" and phase == "LOSS_CHECKED":
                mark("BACKWARD_CHECKED")
            mark(phase)
            if self.fault == "reentrant" and phase == "LOSS_CHECKED":
                self.owner.run()
            if self.fault == "callback_close" and phase == "LOSS_CHECKED":
                self.owner.close()
        result = dict(step=bundle.step, optimizer_updates=bundle.step,
                      fictional_fixture=True, loss=0.5, gradient_norm=1.0)
        if self.fault == "update_order":
            result["optimizer_updates"] += 1
        if self.fault == "oversized_update":
            result["text"] = "x" * 4096
        if self.fault == "nan":
            result["loss"] = float("nan")
        return result

    def make(self, n=17, rows=None, reader=None, mode=boundary.MODES[0], checkpoint=lambda: None):
        self.reader = reader if reader is not None else Reader()
        self.owner = boundary.StreamingTrainingBoundary(
            rows if rows is not None else (row(step) for step in range(1, n + 1)), self.reader,
            lambda *args: None, mode=mode, declared_steps=n, consume=self.consume,
            journal_factory=self.journal, checkpoint=checkpoint, adapter_factory=self.adapter,
            build=self.build, verify=self.verify)
        return self.owner


class BoundaryTests(unittest.TestCase):
    def test_chunk_edge_cases_and_real_reader_closed_once(self):
        for n in (1, 7, 8, 9, 16, 17):
            for mode in boundary.MODES:
                with self.subTest(n=n, mode=mode):
                    h = Harness(); owner = h.make(n=n, mode=mode); result = owner.run()
                    self.assertEqual(result["status"], "COMPLETED_BOUNDARY_CALLBACKS_NOT_MODEL_APPROVAL")
                    self.assertEqual((result["PASS_batches"], result["FAIL_batches"], result["UNEXECUTED_batches"]), (n, 0, 0))
                    self.assertEqual(result["counts"]["chunks_closed"], (n + 7) // 8)
                    self.assertEqual(result["counts"]["ACK_returned"], n)
                    self.assertEqual(result["counts"]["optimizer_returned"], n)
                    self.assertEqual(result["counts"]["journal_rows_written"], 2 * n)
                    self.assertEqual(h.reader.closes, 1)
                    self.assertEqual(h.reader.reads, n * 8)
                    self.assertEqual(owner.close(), result)
                    self.assertEqual(h.reader.closes, 1)

    def test_full20k_scalar_only_bounded_rows_ack_history_and_journal_buffers(self):
        h = Harness(); owner = h.make(n=20000); result = owner.run()
        self.assertEqual(result["PASS_batches"], 20000)
        self.assertEqual(result["counts"]["chunks_opened"], 2500)
        self.assertEqual(result["counts"]["maximum_buffered_rows"], 8)
        self.assertEqual((h.maximum_chunk_rows, h.maximum_pending, h.maximum_journal_rows), (8, 1, 16))
        self.assertEqual(result["current_buffered_rows"], 0)
        self.assertEqual(len(result["last_closed_chunk"]["acknowledgements"]), 8)
        self.assertEqual(h.reader.closes, 1)
        self.assertLess(len(boundary.encoded(result)), 16 * 1024)

    def test_NN_failure_has_no_optimizer_or_ACK_and_stops_after_first_callback(self):
        h = Harness("NN"); result = h.make(n=17).run()
        self.assertEqual(result["status"], "FAILED_RETAINED_NO_RETRY")
        self.assertEqual((result["PASS_batches"], result["FAIL_batches"], result["UNEXECUTED_batches"]), (0, 1, 16))
        self.assertEqual(result["counts"]["optimizer_attempted"], 0)
        self.assertEqual(result["counts"]["ACK_returned"], 0)
        self.assertEqual(h.callback_calls, 1)
        self.assertEqual(h.reader.closes, 1)

    def test_optimizer_exception_preserves_unknown_partial_state_no_retry(self):
        h = Harness("optimizer"); result = h.make(n=9).run()
        self.assertEqual(result["optimizer_partial_state"], "UNKNOWN_PARTIAL_NO_RETRY")
        self.assertEqual(result["counts"]["optimizer_attempted"], 1)
        self.assertEqual(result["counts"]["optimizer_returned"], 0)
        self.assertEqual(result["counts"]["ACK_returned"], 0)
        self.assertEqual(result["failed_tail"]["step"], 1)

    def test_returned_optimizer_is_retained_when_ACK_fails(self):
        h = Harness("ACK"); result = h.make(n=9).run()
        self.assertEqual(result["counts"]["optimizer_returned"], 1)
        self.assertEqual(result["counts"]["ACK_returned"], 0)
        self.assertEqual(result["counts"]["journal_rows_written"], 1)
        self.assertEqual(result["failure"]["phase"], "SERIAL_ACK")
        self.assertEqual(result["failed_tail"]["update_receipt"]["optimizer_updates"], 1)

    def test_selected_crop_native_receipt_and_failed_tail_are_reconstructable(self):
        h = Harness("ACK"); result = h.make(n=9).run()
        stored = h.last_journal.records[0]["source_receipt"]
        self.assertEqual(stored["selected_crop_boxes"], [[0, 0, 360, 640]] * 8)
        self.assertEqual(result["failed_tail"]["source_receipt"], stored)
        self.assertEqual(stored["step"], 1)

    def test_returned_ACK_is_not_erased_when_ACK_journal_write_fails(self):
        h = Harness("ACK_journal"); result = h.make(n=9).run()
        self.assertEqual(result["counts"]["optimizer_returned"], 1)
        self.assertEqual(result["counts"]["ACK_returned"], 1)
        self.assertEqual(result["counts"]["fully_journaled_steps"], 0)
        self.assertEqual(result["failure"]["phase"], "ACK_JOURNAL")
        self.assertEqual(result["failed_tail"]["serial_ACK"]["step"], 1)

    def test_update_journal_failure_retains_returned_optimizer_without_ACK(self):
        h = Harness("update_journal"); result = h.make(n=9).run()
        self.assertEqual(result["counts"]["optimizer_returned"], 1)
        self.assertEqual(result["counts"]["ACK_returned"], 0)
        self.assertEqual(result["counts"]["journal_rows_written"], 0)

    def test_input_reverification_after_journal_denies_changed_buffer(self):
        h = Harness("input_mutation"); result = h.make(n=9).run()
        self.assertEqual(result["failure"]["phase"], "REVERIFY_BEFORE_ACK")
        self.assertEqual(result["counts"]["optimizer_returned"], 1)
        self.assertEqual(result["counts"]["ACK_returned"], 0)

    def test_order_metadata_flush_phase_and_receipt_failures_are_retained(self):
        faults = ("produced_order", "build", "verify", "phase_order", "update_order", "not_flushed",
                  "journal_pin", "journal_mutation", "oversized_update", "nan", "bool_coercion", "ACK_receipt")
        for fault in faults:
            with self.subTest(fault=fault):
                h = Harness(fault); result = h.make(n=9).run()
                self.assertEqual(result["status"], "FAILED_RETAINED_NO_RETRY")
                self.assertEqual(result["PASS_batches"], 0)
                self.assertEqual(result["FAIL_batches"], 1)
                self.assertEqual(h.reader.closes, 1)
                self.assertEqual(h.adapter_calls, 1)

    def test_chunk_close_failure_keeps_completed_ACKs_without_virtual_failed_batch(self):
        for fault in ("journal_close", "adapter_close"):
            with self.subTest(fault=fault):
                h = Harness(fault); result = h.make(n=9).run()
                self.assertEqual(result["status"], "FAILED_RETAINED_NO_RETRY")
                self.assertEqual(result["counts"]["ACK_returned"], 8)
                self.assertEqual((result["PASS_batches"], result["FAIL_batches"], result["UNEXECUTED_batches"]), (8, 0, 1))
                self.assertEqual(h.reader.closes, 1)

    def test_short_stream_failure_has_no_fabricated_future_rows(self):
        h = Harness(); result = h.make(n=9, rows=iter([row(1)])).run()
        self.assertEqual(result["failure"]["phase"], "READ_DECLARATION_CHUNK")
        self.assertEqual(result["failure"]["step"], 2)
        self.assertEqual(result["counts"]["produced"], 0)
        self.assertEqual(result["counts"]["rows_examined"], 1)
        self.assertEqual((result["PASS_batches"], result["FAIL_batches"], result["UNEXECUTED_batches"]), (0, 1, 8))

    def test_extra_stream_row_is_outside_declared_denominator(self):
        h = Harness(); result = h.make(n=1, rows=iter([row(1), row(2)])).run()
        self.assertEqual(result["failure"]["phase"], "VERIFY_STREAM_END")
        self.assertIsNone(result["failure"]["step"])
        self.assertEqual((result["PASS_batches"], result["FAIL_batches"], result["UNEXECUTED_batches"]), (1, 0, 0))

    def test_duplicate_bool_or_reordered_declaration_steps_denied(self):
        for bad in (1, True, 3):
            with self.subTest(step=bad):
                second = row(2); second["step"] = bad
                h = Harness(); result = h.make(n=2, rows=iter([row(1), second])).run()
                self.assertEqual(result["failure"]["step"], 2)
                self.assertEqual(h.adapter_calls, 0)
                self.assertEqual(result["counts"]["produced"], 0)

    def test_FALSE_only_exact_integer_and_crop_bounds_before_adapter(self):
        faults = ("TRUE", "bool_FALSE", "slot", "crop", "hash")
        for fault in faults:
            with self.subTest(fault=fault):
                bad = row(1)
                if fault == "TRUE": bad["actual_paired_TRUE_bits"][0] = 1
                elif fault == "bool_FALSE": bad["actual_paired_TRUE_bits"][0] = False
                elif fault == "slot": bad["physical_slot_indices"][0] = 1209
                elif fault == "crop": bad["shared_crop_boxes"][0] = [0, 0, 361, 640]
                else: bad["RGB_sha256"] = "x" * 64
                h = Harness(); result = h.make(n=1, rows=iter([bad])).run()
                self.assertEqual(h.adapter_calls, 0)
                self.assertEqual(result["status"], "FAILED_RETAINED_NO_RETRY")

    def test_reentry_close_from_callback_and_repeat_run_denied(self):
        for fault in ("reentrant", "callback_close"):
            with self.subTest(fault=fault):
                h = Harness(fault); owner = h.make(n=9); result = owner.run()
                self.assertEqual(result["status"], "FAILED_RETAINED_NO_RETRY")
                with self.assertRaises(ValueError): owner.run()
                self.assertEqual(owner.close(), result)

    def test_owner_thread_and_PID_denials_do_not_close_reader(self):
        h = Harness(); owner = h.make(n=1)
        failures = []
        def foreign():
            try: owner.run()
            except ValueError as error: failures.append(error)
        thread = threading.Thread(target=foreign); thread.start(); thread.join()
        self.assertEqual(len(failures), 1)
        with patch.object(boundary.os, "getpid", return_value=os.getpid() + 1):
            with self.assertRaises(ValueError): owner.close()
        self.assertEqual(h.reader.closes, 0)
        owner.close()

    def test_outer_close_failure_and_observation_preserved(self):
        h = Harness(); reader = Reader("outer close denied"); result = h.make(n=1, reader=reader).run()
        self.assertEqual(result["status"], "FAILED_RETAINED_NO_RETRY")
        self.assertEqual(result["PASS_batches"], 1)
        self.assertEqual(result["FAIL_batches"], 0)
        self.assertEqual(result["cleanup_errors"][0]["scope"], "outer_reader_close")
        self.assertEqual(reader.closes, 1)
        h.owner.close(); self.assertEqual(reader.closes, 1)

    def test_outer_reader_closing_observations_fail_without_extra_failed_batch(self):
        reader = Reader(); reader.closing_errors = [dict(scope="fd", type="ValueError", message="seal differs")]
        h = Harness(); result = h.make(n=1, reader=reader).run()
        self.assertEqual(result["status"], "FAILED_RETAINED_NO_RETRY")
        self.assertEqual(result["PASS_batches"], 1)
        self.assertEqual(result["FAIL_batches"], 0)
        self.assertEqual(result["cleanup_errors"][0]["scope"], "outer_reader_closing_observations")

    def test_invalid_step_count_and_full_list_are_denied_before_owning_reader(self):
        for count in (0, True, 20001):
            with self.subTest(count=count):
                h = Harness(); reader = Reader()
                with self.assertRaises(ValueError): h.make(n=count, reader=reader)
                self.assertEqual(reader.closes, 0)
        h = Harness()
        with self.assertRaises(ValueError): h.make(n=1, rows=[row(1)])

    def test_close_before_run_preserves_all_unexecuted_and_no_callbacks(self):
        h = Harness(); owner = h.make(n=9); result = owner.close()
        self.assertEqual(result["status"], "CLOSED_NOT_EXECUTED")
        self.assertEqual(result["UNEXECUTED_batches"], 9)
        self.assertEqual(h.callback_calls, 0)
        with self.assertRaises(ValueError): owner.run()

    def test_checkpoint_after_optimizer_return_preserves_commit_observation(self):
        h = Harness()
        def checkpoint():
            if h.owner is not None and h.owner._phase == "OPTIMIZER_RETURNED":
                raise ValueError("deadline after optimizer returned")
        result = h.make(n=9, checkpoint=checkpoint).run()
        self.assertEqual(result["counts"]["optimizer_returned"], 1)
        self.assertEqual(result["counts"]["ACK_returned"], 0)
        self.assertEqual(result["failure"]["phase"], "OPTIMIZER_RETURNED")

    def test_metadata_foreign_copy_hooks_and_oversized_values_denied(self):
        class Foreign:
            def __deepcopy__(self, memo): raise AssertionError("foreign copy hook must not execute")
        with self.assertRaises(ValueError): boundary.encoded(dict(value=Foreign()))
        with self.assertRaises(ValueError): boundary.encoded(dict(value="x" * (boundary.MAX_METADATA_BYTES + 1)))
        with self.assertRaises(ValueError): boundary.encoded(dict(value=1 << 129))

    def test_nonowning_facade_repeat_close_and_reads_after_close(self):
        reader = Reader(); facade = boundary.NonOwningReader(reader, lambda: None)
        self.assertEqual(facade.read(3), (3, 3))
        facade.close(); facade.close()
        self.assertEqual(facade.close_calls, 1)
        self.assertEqual(reader.closes, 0)
        with self.assertRaises(ValueError): facade.read(3)

    def test_module_import_does_not_import_numeric_providers(self):
        path = Path(boundary.__file__).resolve()
        script = ("import importlib.util, sys; "
                  f"s=importlib.util.spec_from_file_location('boundary_probe', {str(path)!r}); "
                  "m=importlib.util.module_from_spec(s); s.loader.exec_module(m); "
                  "assert not any(n in sys.modules for n in ('numpy','torch','PIL')); "
                  "assert m.SCOPE['NN_trainer_implemented'] is False")
        completed = subprocess.run([sys.executable, "-I", "-B", "-c", script],
            stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=20)
        self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == "__main__":
    unittest.main()
