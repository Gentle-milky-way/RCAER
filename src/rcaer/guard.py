"""Task-preserving, local guarded anchor projection (paper Section III-B).

This is a conservative reference geometric strategy, not recovered original code.
It requires a calibrated per-pixel 3-D point cloud and target-specific mask.
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from .config import Bounds
from .descriptor import valid_depth
from .models import Anchor, GuardResult, Observation, Pose


def check_observation(obs: Observation) -> None:
    if obs.depth_m.ndim != 2:
        raise ValueError("depth must be a 2-D image")
    h, w = obs.depth_m.shape
    if obs.task_mask.shape != (h, w) or obs.points_base_m.shape != (h, w, 3):
        raise ValueError("depth, target mask and aligned xyz dimensions must match")
    if obs.depth_valid_mask is not None and obs.depth_valid_mask.shape != (h, w):
        raise ValueError("depth_valid_mask dimensions do not match")
    if not math.isfinite(obs.table_z_m):
        raise ValueError("table height must be finite")


def project_anchor(anchor: Anchor, obs: Observation, bounds: Bounds) -> GuardResult:
    check_observation(obs)
    h, w = obs.depth_m.shape
    u0, v0 = anchor.pixel_uv
    us, vs = anchor.semantic_pixel_uv
    if any(not (0 <= u < w and 0 <= v < h) for u, v in ((u0, v0), (us, vs))):
        return GuardResult(None, False, ("pixel_out_of_frame",))
    if not obs.task_mask[vs, us]:
        return GuardResult(None, False, ("semantic_target_not_in_mask",))
    if not np.all(np.isfinite(anchor.pose.xyz())):
        return GuardResult(None, False, ("invalid_selected_pose",))
    # Semantic target must be in same task-mask connected region as any candidate.
    from scipy.ndimage import label

    labels, _ = label(obs.task_mask.astype(bool))
    target_component = labels == labels[vs, us]
    dmask = distance_transform_edt(target_component)
    valid = valid_depth(obs, *bounds.z_m)
    xyz = obs.points_base_m
    yy, xx = np.indices((h, w))
    center_dist = np.hypot(xx - u0, yy - v0)
    semantic_dist = np.hypot(xx - us, yy - vs)
    xyz_finite = np.isfinite(xyz).all(axis=2)
    lateral = np.linalg.norm(xyz[:, :, :2] - anchor.pose.xyz()[:2], axis=2)
    is_candidate = (
        target_component & valid & xyz_finite
        & (center_dist <= bounds.max_projection_px)
        & (semantic_dist <= bounds.max_semantic_radius_px)
        & (lateral <= bounds.max_semantic_lateral_m)
        & (lateral <= bounds.max_pregrasp_lateral_m)
    )
    if not np.any(is_candidate):
        return GuardResult(None, False, ("no_semantically_valid_local_projection",))
    # Prefer stable object interior, but minimize unnecessary re-anchoring.
    score = np.minimum(dmask, 30.0) - 0.20 * center_dist
    score[~is_candidate] = -np.inf
    v, u = map(int, np.unravel_index(int(np.argmax(score)), score.shape))
    p = xyz[v, u]
    # Conservative height correction. Object clearance is separately gated.
    new_z = max(float(p[2]), obs.table_z_m + bounds.min_grasp_clearance_m)
    new_anchor = Anchor(Pose(float(p[0]), float(p[1]), new_z,
                             anchor.pose.qx, anchor.pose.qy,
                             anchor.pose.qz, anchor.pose.qw),
                        (u, v), anchor.semantic_pixel_uv, anchor.object_id,
                        anchor.approach_direction_base)
    shift = float(np.linalg.norm(p[:2] - anchor.pose.xyz()[:2]))
    return GuardResult(new_anchor, True, (), float(center_dist[v, u]), shift,
                       max(0.0, new_z - float(p[2])))
