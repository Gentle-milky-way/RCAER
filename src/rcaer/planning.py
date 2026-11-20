"""Independent abort gate and geometric waypoint INTENTS.

Not an IK, collision, realtime servo, or robot-driver implementation.
"""
import math

from .config import Bounds, Profile, Settings
from .models import Anchor, Measurements, Observation, Pose, Waypoint


def hard_gate(anchor: Anchor, obs: Observation, m: Measurements, b: Bounds) -> tuple[str, ...]:
    reasons = []
    p = anchor.pose
    q = (p.qx, p.qy, p.qz, p.qw)
    if not all(math.isfinite(v) for v in q) or abs(math.sqrt(sum(v*v for v in q))-1.0) > 1e-3:
        reasons.append("invalid_orientation_quaternion")
    direction = anchor.approach_direction_base
    if (len(direction) != 3 or not all(math.isfinite(v) for v in direction)
            or abs(math.sqrt(sum(v*v for v in direction))-1.0) > 1e-3):
        reasons.append("invalid_approach_direction")
    if not all(math.isfinite(x) for x in (p.x, p.y, p.z)) or not (
        b.x_m[0] <= p.x <= b.x_m[1]
        and b.y_m[0] <= p.y <= b.y_m[1]
        and b.z_m[0] <= p.z <= b.z_m[1]
    ):
        reasons.append("workspace")
    if not math.isfinite(obs.table_z_m):
        reasons.append("invalid_table_plane")
    grasp_clearance = p.z - obs.table_z_m
    if grasp_clearance < b.min_grasp_clearance_m or m.object_table_clearance_m < b.min_object_clearance_m:
        reasons.append("table_clearance")
    if m.gripper_table_clearance_m is not None and m.gripper_table_clearance_m < b.min_grasp_clearance_m:
        reasons.append("measured_gripper_table_clearance")
    if m.grasp_height_margin_m is not None and m.grasp_height_margin_m < b.min_grasp_height_margin_m:
        reasons.append("grasp_height_margin")
    if not math.isfinite(m.tilt_deg) or m.tilt_deg > b.max_tilt_deg or m.tilt_deg < 0:
        reasons.append("tilt")
    if not math.isfinite(m.rotation_to_target_deg) or not 0 <= m.rotation_to_target_deg <= b.max_rotation_deg:
        reasons.append("rotation")
    if not math.isfinite(m.pregrasp_lateral_offset_m) or not 0 <= m.pregrasp_lateral_offset_m <= b.max_pregrasp_lateral_m:
        reasons.append("pregrasp_lateral")
    if not m.reachability_ok:
        reasons.append("reachability")
    if not m.plan_feasible or not math.isfinite(m.motion_plan_time_s) or not math.isfinite(m.motion_plan_budget_s) or m.motion_plan_time_s < 0 or m.motion_plan_budget_s <= 0 or m.motion_plan_time_s > m.motion_plan_budget_s:
        reasons.append("motion_planning")
    return tuple(reasons)


def generate_waypoint_intents(
    anchor: Anchor, profile: Profile, cfg: Settings, obs: Observation
) -> list[Waypoint]:
    b = cfg.bounds
    p = anchor.pose
    nominal_v = cfg.nominal_speed_m_s
    if not math.isfinite(nominal_v) or nominal_v <= 0:
        raise ValueError("positive nominal speed required")
    dx, dy, dz = anchor.approach_direction_base
    pre = Pose(p.x - dx * profile.offset_m,
               p.y - dy * profile.offset_m,
               p.z - dz * profile.offset_m, p.qx, p.qy, p.qz, p.qw)
    lift = p.z + min(cfg.lift_m, b.max_lift_m)
    if not (b.x_m[0] <= pre.x <= b.x_m[1]
            and b.y_m[0] <= pre.y <= b.y_m[1]
            and b.z_m[0] <= pre.z <= b.z_m[1]
            and lift <= b.z_m[1]):
        raise ValueError("waypoint_outside_workspace")
    # Simple Cartesian intent placeholders; final collision / orientation / IK
    # checking MUST precede execution by an actual motion planner.
    v = nominal_v * profile.velocity_scale
    initial = pre
    lift_pose = Pose(p.x, p.y, lift, p.qx, p.qy, p.qz, p.qw)
    return [
        Waypoint("initial", initial, v, "open"),
        Waypoint("pre_approach", initial, v),
        Waypoint("guarded_approach", p, v),
        Waypoint("contact_probe", p, v, "hold", profile.probe_m),
        Waypoint("close", p, 0.0, "close"),
        Waypoint("lift", lift_pose, v),
        Waypoint("retreat", initial, v),
    ]
