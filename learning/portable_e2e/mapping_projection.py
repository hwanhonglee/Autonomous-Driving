"""HH_260906 - Project undistorted CARLA BGRA oracle frames in UE map coordinates.

Depth is optical-axis depth; UE camera coordinates are forward/right/up.
The label table is supplied from the installed CARLA enum, not a web table.
"""
import math
import numpy as np
from .mapping import Observation, STATIC_LABELS


def project_frame(depth_bgra, semantic_bgra, world_from_camera, *, frame_id,
                  timestamp_s, fov_deg, label_table, stride=3, max_depth_m=40):
    """Return sparse oracle evidence; no inferred boundaries or lane connectivity."""
    depth = np.asarray(depth_bgra)
    semantic = np.asarray(semantic_bgra)
    matrix = np.asarray(world_from_camera, dtype=float)
    if depth.ndim != 3 or depth.shape[-1] != 4 or semantic.shape != depth.shape:
        raise ValueError("co-registered HxWx4 images required")
    if depth.dtype != np.uint8 or semantic.dtype != np.uint8:
        raise ValueError("raw uint8 BGRA required")
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("finite homogeneous transform required")
    if not (np.allclose(matrix[3], [0, 0, 0, 1])
            and np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
            and np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5)):
        raise ValueError("rigid transform required")
    if not math.isfinite(fov_deg) or not 0 < fov_deg < 180:
        raise ValueError("invalid horizontal field of view")
    if isinstance(stride, bool) or not isinstance(stride, int) or stride < 1:
        raise ValueError("positive integer stride required")
    if not math.isfinite(max_depth_m) or not 0 < max_depth_m < 1000:
        raise ValueError("bounded depth range required")
    h, w, _ = depth.shape
    if h < 2 or w < 2:
        raise ValueError("image dimensions too small")
    d = depth[::stride, ::stride].astype(np.float64)
    meters = (d[..., 2] + d[..., 1] * 256 + d[..., 0] * 65536) * (1000 / 16777215)
    tags = semantic[::stride, ::stride, 2]
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    valid = (meters > .1) & (meters < max_depth_m) & np.isin(tags, list(label_table))
    z = meters[valid]
    f = w / (2 * math.tan(math.radians(fov_deg) / 2))
    xyz = np.stack([z, (u[valid] - w / 2) * z / f,
                    -(v[valid] - h / 2) * z / f, np.ones_like(z)])
    world = (matrix @ xyz).T
    points = tuple((float(p[0]), float(p[1]), label_table[int(tag)], 1.0)
                   for p, tag in zip(world, tags[valid]))
    return Observation(timestamp_s, frame_id, "oracle_sensor", points)


def project_predicted_frame(depth_m, predicted_classes, confidence, world_from_camera, *,
                            frame_id, timestamp_s, fov_deg, label_table, stride=3,
                            max_depth_m=40.0, min_confidence=0.5):
    """HH_260929 - Project learned labels using separately supplied metric depth and pose.

    The function never accepts a CARLA semantic image. The caller must disclose
    whether depth and pose are oracle simulator signals or deployable sensors.
    A valid projection is map plumbing, not lane topology or HD-map approval.
    """
    depth = np.asarray(depth_m)
    prediction = np.asarray(predicted_classes)
    certainty = np.asarray(confidence)
    matrix = np.asarray(world_from_camera, dtype=float)
    if depth.ndim != 2 or depth.shape != prediction.shape or depth.shape != certainty.shape:
        raise ValueError("co-registered HxW depth, prediction and confidence required")
    if depth.shape[0] < 2 or depth.shape[1] < 2:
        raise ValueError("image dimensions too small")
    if not np.issubdtype(depth.dtype, np.floating) or not np.issubdtype(prediction.dtype, np.integer):
        raise ValueError("floating metric depth and integer class image required")
    if not np.issubdtype(certainty.dtype, np.floating) or not np.isfinite(certainty).all():
        raise ValueError("finite floating confidence required")
    if np.any((certainty < 0) | (certainty > 1)):
        raise ValueError("confidence must be in [0,1]")
    if not label_table or any(not isinstance(key, int) or key < 0 or
                              label not in STATIC_LABELS for key, label in label_table.items()):
        raise ValueError("explicit static class label table required")
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not (
            np.allclose(matrix[3], [0, 0, 0, 1]) and
            np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5) and
            np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5)):
        raise ValueError("rigid transform required")
    if not math.isfinite(fov_deg) or not 0 < fov_deg < 180:
        raise ValueError("invalid horizontal field of view")
    if isinstance(stride, bool) or not isinstance(stride, int) or stride < 1:
        raise ValueError("positive integer stride required")
    if not math.isfinite(max_depth_m) or not 0 < max_depth_m < 1000:
        raise ValueError("bounded depth range required")
    if not math.isfinite(min_confidence) or not 0 <= min_confidence <= 1:
        raise ValueError("confidence threshold must be in [0,1]")
    h, w = depth.shape
    z = depth[::stride, ::stride]
    tag = prediction[::stride, ::stride]
    score = certainty[::stride, ::stride]
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    valid = (np.isfinite(z) & (z > .1) & (z < max_depth_m) &
             (score >= min_confidence) & np.isin(tag, list(label_table)))
    selected_depth = z[valid].astype(np.float64)
    f = w / (2 * math.tan(math.radians(fov_deg) / 2))
    xyz = np.stack([selected_depth,
                    (u[valid] - w / 2) * selected_depth / f,
                    -(v[valid] - h / 2) * selected_depth / f,
                    np.ones_like(selected_depth)])
    world = (matrix @ xyz).T
    points = tuple((float(position[0]), float(position[1]),
                    label_table[int(class_id)], float(probability))
                   for position, class_id, probability in
                   zip(world, tag[valid], score[valid]))
    return Observation(timestamp_s, frame_id, "learned_prediction", points)
