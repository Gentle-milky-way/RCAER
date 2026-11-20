"""Typed inputs and outputs for the post-selection execution layer."""
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(frozen=True)
class Pose:
    """Position in calibrated robot-base frame; orientation is opaque to this layer.

    The downstream controller must provide calibrated orientation and its own
    collision-safe IK/trajectory validation. This reference only plans positions.
    """

    x: float
    y: float
    z: float
    # 6-DoF pose: xyzw quaternion. The reference layer carries orientation
    # through unchanged; actual orientation control belongs to the planner.
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def xyz(self) -> np.ndarray:
        return np.array((self.x, self.y, self.z), dtype=float)


@dataclass(frozen=True)
class Anchor:
    pose: Pose
    pixel_uv: tuple[int, int]  # (u, v); target mask contains ONLY the selected task part
    semantic_pixel_uv: tuple[int, int]
    object_id: str
    # Unit vector toward contact in base coordinates; vertical default for demos.
    approach_direction_base: tuple[float, float, float] = (0.0, 0.0, -1.0)


@dataclass(frozen=True)
class Material:
    reflective: str = "none"  # none | medium | high
    transparent: bool = False
    glass: bool = False
    deformable: bool = False
    thin_walled: bool = False
    annotation_uncertainty: float = 0.0


@dataclass(frozen=True)
class Margins:
    """Planner-supplied normalized safety margins in [0,1].

    The paper does not prescribe their exact normalization. The caller must
    calculate them from calibrated workspace/tilt/offset/rotation constraints.
    """

    workspace: float
    tilt: float
    lateral: float
    rotation: float


@dataclass(frozen=True)
class Measurements:
    object_height_m: float
    object_table_clearance_m: float
    gripper_opening_margin_m: float
    normalized_bilateral_contact_area: float
    margins: Margins
    gripper_table_clearance_m: float | None = None
    grasp_height_margin_m: float | None = None
    tilt_deg: float = 0.0
    rotation_to_target_deg: float = 0.0
    pregrasp_lateral_offset_m: float = 0.0
    plan_feasible: bool = False
    # From narrow IK/collision/trajectory planning only; frontend excluded.
    motion_plan_time_s: float = 0.0
    motion_plan_budget_s: float = 2.0
    # Caller supplies any additional hard reachability/recovery veto.
    reachability_ok: bool = False


@dataclass
class Observation:
    """Calibrated aligned depth (meters), xyz base-frame (meters), and task mask.

    `depth_valid_mask` may include depth-to-table consistency and sensor validity.
    If absent, only finite, non-zero, workspace-range tests can be applied.
    """

    depth_m: np.ndarray  # (H,W)
    points_base_m: np.ndarray  # (H,W,3)
    task_mask: np.ndarray  # (H,W), selected object/part ONLY
    table_z_m: float
    depth_valid_mask: np.ndarray | None = None


@dataclass
class Packet:
    trial_id: str
    scene: str
    instruction: str
    anchor: Anchor
    observation: Observation
    measurements: Measurements
    material: Material = field(default_factory=Material)
    packet_id: str = ""


@dataclass(frozen=True)
class GuardResult:
    anchor: Anchor | None
    success: bool
    reasons: tuple[str, ...]
    projection_px: float = 0.0
    lateral_shift_m: float = 0.0
    table_lift_m: float = 0.0


@dataclass(frozen=True)
class CueVector:
    zD: float
    zH: float
    zT: float
    zE: float
    zM: float
    zA: float
    zC: float

    def as_dict(self) -> dict[str, float]:
        return {k: getattr(self, k) for k in ("zD", "zH", "zT", "zE", "zM", "zA", "zC")}


@dataclass(frozen=True)
class Waypoint:
    phase: str
    position: Pose
    velocity_m_s: float
    gripper_action: str = "hold"
    probe_amplitude_m: float = 0.0
    # No IK, compliance setpoint, collision check, or actuator command is emitted.


@dataclass
class Decision:
    trial_id: str
    packet_id: str
    object_id: str
    scene: str
    instruction: str
    selected_anchor: Anchor
    guarded: GuardResult
    cues: CueVector | None
    risk: float | None
    tier: str | None
    profile: Any | None
    aborted: bool
    abort_reasons: tuple[str, ...]
    waypoints: list[Waypoint]
    codebook_version: str
    mode: str = "M3"
    notes: str = "Dry-run geometric waypoints only; not hardware-safe commands"

    def audit_dict(self) -> dict[str, Any]:
        """JSON-compatible decision trace; outcome/timing fields can be added later."""
        from dataclasses import asdict

        data = asdict(self)
        data["outcome"] = {"task_valid": None, "physical_success": None,
                           "primary_failure": None, "secondary_failure": None}
        data["overhead"] = {"planning_time_s": None, "execution_time_s": None,
                            "runtime_total_s": None, "retry_count": None}
        data["gate"] = {"abort": self.aborted, "reasons": list(self.abort_reasons),
                        "protective_stop": None, "recovery": None}
        data["waypoint_count"] = len(self.waypoints)
        return data
