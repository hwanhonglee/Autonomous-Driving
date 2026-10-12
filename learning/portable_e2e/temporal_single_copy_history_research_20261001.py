"""HH_261001 - Remove redundant temporary history copies, preserving every causal input and hash guard."""

from __future__ import annotations

from collections import OrderedDict

import torch

from .model import ModelConfig
from .runtime import build_runtime_tensors
from .temporal_cached_encoder_v2 import CameraFrameIdentityV2, TemporalCacheWindowV2
from .temporal_dataset_v2 import TIME_OFFSETS_NS, _relative_pose
from .temporal_headroom_camera_research_v2_20261001 import HeadroomOwnedCameraTemporalSimulationResearchPolicyV2
from .temporal_simulation_research_v2_20261001 import _require


SCHEMA = "portable_e2e.single_copy_normalized_camera_history.20261001.v1"


class _SingleCopyBuildMixin:
    """HH_261001 - torch.stack owns its result; retain separate cache clones but omit pre-stack clones of hits."""

    def build_inputs(self, frame, *, past_frames=(), now_ns, ego_stamp_ns, route_stamp_ns):
        self._validate_frame(frame)
        _require(type(now_ns) is int and type(ego_stamp_ns) is int and type(route_stamp_ns) is int,
                 "integer current/ego/route stamps required")
        stamps = [frame.timestamp_ns, frame.pose_timestamp_ns, ego_stamp_ns, route_stamp_ns]
        _require(all(0 <= now_ns - stamp <= 150_000_000 for stamp in stamps)
                 and max(stamps) - min(stamps) <= 50_000_000, "stale, future or skewed current inputs")
        _require(isinstance(past_frames, (list, tuple)) and len(past_frames) <= 9,
                 "bounded actual past frame list required")
        previous = -1
        for past in past_frames:
            self._validate_frame(past)
            _require(previous < past.timestamp_ns < frame.timestamp_ns,
                     "past frames must increase strictly without duplicates or future/current copies")
            _require(past.inputs.calibration == frame.inputs.calibration,
                     "temporal camera rig calibration changed inside history")
            previous = past.timestamp_ns
        actual = tuple(past_frames) + (frame,)
        # HH_261001 - Preserve full source admission: immutable ownership or fresh mutable-camera content hashing.
        hashes = {item.timestamp_ns: tuple(self._camera_digest(camera) for camera in item.inputs.camera_frames)
                  for item in actual}
        current = build_runtime_tensors(frame.inputs, ModelConfig())
        tensors = {name: value for name, value in current.items() if name != "images"}
        groups, valid, poses, offsets, identities = [], [], [], [], []
        for offset in TIME_OFFSETS_NS:
            target = frame.timestamp_ns + offset
            found = [item for item in actual if abs(item.timestamp_ns - target) <= 3_000]
            _require(len(found) <= 1, "ambiguous real temporal slot timestamps")
            item = found[0] if found else None
            if item is None:
                groups.append(torch.zeros_like(current["images"][0]))
                valid.append(False); poses.append((0., 0., 0.)); offsets.append(offset / 1.e9); identities.append(None)
                continue
            camera_hashes = hashes[item.timestamp_ns]
            key = item.timestamp_ns, camera_hashes
            if item is frame:
                image = current["images"][0]
                self._images[key] = image.detach().clone()
            elif key in self._images:
                # HH_261001 - No external alias escapes: torch.stack below copies this private read-only use.
                image = self._images[key]
            else:
                image = build_runtime_tensors(item.inputs, ModelConfig())["images"][0]
                self._images[key] = image.detach().clone()
            groups.append(image)
            valid.append(True)
            poses.append(_relative_pose(frame, item))
            offsets.append((item.timestamp_ns - frame.timestamp_ns) / 1.e9)
            identities.append(CameraFrameIdentityV2(item.timestamp_ns, camera_hashes))
        self._images = OrderedDict((key, value) for key, value in self._images.items()
            if frame.timestamp_ns - 800_003_000 <= key[0] <= frame.timestamp_ns)
        while len(self._images) > 9:
            self._images.popitem(last=False)
        tensors.update(images=torch.stack(groups)[None], time_valid=torch.tensor(valid, dtype=torch.bool)[None],
                       relative_pose=torch.tensor(poses, dtype=torch.float32)[None],
                       time_offsets_s=torch.tensor(offsets, dtype=torch.float32)[None])
        window = TemporalCacheWindowV2(*self.identity, frame.timestamp_ns, tuple(identities))
        return tensors, window


# HH_261001 - Insert only the builder immediately beneath the immutable-owner method; its full audits still wrap us.
from .temporal_owned_camera_history_v2_20261001 import OwnedCameraTemporalSimulationResearchPolicyV2
from .temporal_simulation_research_v2_20261001 import TemporalSimulationResearchPolicyV2


class _SingleCopyBase(_SingleCopyBuildMixin, TemporalSimulationResearchPolicyV2):
    pass


class SingleCopyOwnedHeadroomPolicyV2(HeadroomOwnedCameraTemporalSimulationResearchPolicyV2,
                                     OwnedCameraTemporalSimulationResearchPolicyV2, _SingleCopyBase):
    """HH_261001 - MRO retains immutable source admission and normalized full-tensor SHA, original PI and 100ms."""

