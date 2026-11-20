"""Seven implemented cues, paper Table I and Eq. (2)-(3)."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from .models import CueVector, Margins, Material, Measurements, Observation


def clip01(x: float) -> float:
    if not math.isfinite(x):
        raise ValueError("non-finite cue input")
    return max(0.0, min(1.0, float(x)))


def ramp(x: float, lo: float, hi: float) -> float:
    if hi <= lo:
        raise ValueError("invalid ramp interval")
    return clip01((x - lo) / (hi - lo))


def ramp_down(x: float, lo: float, hi: float) -> float:
    return 1.0 - ramp(x, lo, hi)


def material_cue(material: Material) -> float:
    if material.reflective not in ("none", "medium", "high"):
        raise ValueError("reflective must be none/medium/high")
    wr = {"none": 0.0, "medium": 0.20, "high": 0.30}[material.reflective]
    wt = 0.25 if (material.transparent or material.glass) else 0.0
    wd = 0.15 if material.deformable else 0.0
    wthin = 0.10 if material.thin_walled else 0.0
    u = material.annotation_uncertainty
    if not 0.0 <= u <= 1.0:
        raise ValueError("annotation uncertainty must be in [0,1]")
    return clip01(0.20 + wr + wt + wd + wthin + 0.20 * u)


def reachability_cue(m: Margins) -> float:
    values = (m.workspace, m.tilt, m.lateral, m.rotation)
    if any(not math.isfinite(v) or not 0 <= v <= 1 for v in values):
        raise ValueError("normalized margins must each be in [0,1]")
    return clip01(1.0 - min(values))


def valid_depth(obs: Observation, z_min: float, z_max: float) -> np.ndarray:
    d = obs.depth_m
    # Workspace z bounds refer to calibrated robot-base frame, not camera depth.
    base_z = obs.points_base_m[:, :, 2]
    valid = np.isfinite(d) & (d > 0) & np.isfinite(base_z) & (base_z >= z_min) & (base_z <= z_max)
    if obs.depth_valid_mask is not None:
        valid &= obs.depth_valid_mask.astype(bool)
    return valid


def depth_patch_stats(
    obs: Observation, uv: tuple[int, int], radius_px: int, z_min: float, z_max: float
) -> tuple[float, float]:
    u, v = uv
    h, w = obs.depth_m.shape
    if not (0 <= u < w and 0 <= v < h):
        raise ValueError("projected anchor pixel outside image")
    # Circle, not rectangular corner pixels, around the anchor.
    ys, xs = np.ogrid[max(0, v-radius_px):min(h, v+radius_px+1),
                       max(0, u-radius_px):min(w, u+radius_px+1)]
    disk = (xs-u)**2 + (ys-v)**2 <= radius_px**2
    d = obs.depth_m[max(0,v-radius_px):min(h,v+radius_px+1),
                    max(0,u-radius_px):min(w,u+radius_px+1)]
    mask = valid_depth(obs, z_min, z_max)[max(0,v-radius_px):min(h,v+radius_px+1),
                                          max(0,u-radius_px):min(w,u+radius_px+1)]
    n = int(disk.sum())
    is_valid = mask & disk
    count = int(is_valid.sum())
    if not n or not count:
        return 1.0, 0.0
    return 1.0 - count/n, float(np.std(d[is_valid]))


def edge_distance_px(task_mask: np.ndarray, uv: tuple[int, int]) -> float:
    u, v = uv
    if not task_mask[v, u]:
        return 0.0
    return max(0.0, float(distance_transform_edt(task_mask.astype(bool))[v, u]) - 1.0)


def build_cues(
    obs: Observation, uv: tuple[int, int], m: Measurements, material: Material,
    grasp_clearance_m: float, radius_px: int, z_min: float, z_max: float,
) -> CueVector:
    rinvalid, depth_std_m = depth_patch_stats(obs, uv, radius_px, z_min, z_max)
    zD = max(ramp(rinvalid, 0.10, 0.45), ramp(depth_std_m, 0.006, 0.025))
    zH = ramp_down(m.object_height_m, 0.075, 0.140)
    zT = max(ramp_down(grasp_clearance_m, 0.028, 0.060),
             ramp_down(m.object_table_clearance_m, 0.010, 0.035))
    zE = ramp_down(edge_distance_px(obs.task_mask, uv), 0.0, 30.0)
    zM = material_cue(material)
    zA = reachability_cue(m.margins)
    zC = max(ramp_down(m.gripper_opening_margin_m, 0.004, 0.018),
             ramp_down(m.normalized_bilateral_contact_area, 0.35, 0.75))
    return CueVector(zD, zH, zT, zE, zM, zA, zC)


def aggregate_risk(cues: CueVector) -> float:
    values = list(cues.as_dict().values())
    if any(not 0 <= x <= 1 or not math.isfinite(x) for x in values):
        raise ValueError("all seven cues must be finite and in [0,1]")
    return max(values)
