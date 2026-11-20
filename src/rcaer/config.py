"""Paper-aligned constants (Tables I-III); NOT universal safe robot limits."""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Bounds:
    x_m: tuple[float, float] = (0.1786, 0.7029)
    y_m: tuple[float, float] = (-0.3268, 0.5419)
    z_m: tuple[float, float] = (0.0414, 0.7200)
    min_grasp_clearance_m: float = 0.028
    min_object_clearance_m: float = 0.010
    min_grasp_height_margin_m: float = 0.075
    max_tilt_deg: float = 14.0
    max_pregrasp_lateral_m: float = 0.035
    max_rotation_deg: float = 110.0
    max_semantic_radius_px: float = 180.0
    max_projection_px: float = 30.0
    max_semantic_lateral_m: float = 0.030
    max_lift_m: float = 0.160
    approach_offset_m: tuple[float, float] = (0.08, 0.10)
    velocity_scale: tuple[float, float] = (0.45, 0.80)
    probe_amplitude_m: tuple[float, float] = (0.0, 0.004)


@dataclass(frozen=True)
class Settings:
    bounds: Bounds = field(default_factory=Bounds)
    low_to_mid: float = 0.50
    mid_to_high: float = 0.70
    patch_radius_px: int = 30
    nominal_speed_m_s: float = 0.1  # EXAMPLE, not a paper-specified value
    lift_m: float = 0.12
    # Paper describes plane-inconsistent depth, but does not publish a tolerance.
    # Use upstream validity mask for the real calibration; keep this disabled here.
    plane_consistency_tolerance_m: float | None = None
    codebook_version: str = "paper-table-I-III-v1"


@dataclass(frozen=True)
class Profile:
    tier: str
    velocity_scale: float
    probe_m: float
    offset_m: float
    compliance: str
    stiffness_translation_n_m: float
    stiffness_rotation_nm_rad: float
    damping_translation_ns_m: float
    damping_rotation_nms_rad: float
    grip_force_n: float
    grip_speed_mm_s: float


PROFILES = {
    "low": Profile("low", 0.80, 0.000, 0.08, "nominal", 2000, 100, 75, 3.5, 40, 150),
    "mid": Profile("mid", 0.60, 0.002, 0.09, "medium-soft", 1000, 60, 53, 2.7, 25, 100),
    "high": Profile("high", 0.45, 0.004, 0.10, "soft", 500, 30, 37, 1.9, 15, 50),
}
