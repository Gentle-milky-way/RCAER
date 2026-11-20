"""Deterministic max-risk router, bounded profile projection and replay."""
from dataclasses import replace

from .config import Bounds, PROFILES, Profile, Settings
from .descriptor import aggregate_risk, clip01
from .models import CueVector


def select_tier(risk: float, low_to_mid: float = 0.50, mid_to_high: float = 0.70) -> str:
    if not (0.0 < low_to_mid < mid_to_high < 1.0):
        raise ValueError("require 0 < low_to_mid < mid_to_high < 1")
    x = clip01(risk)
    return "low" if x < low_to_mid else "mid" if x < mid_to_high else "high"


def bound_profile(profile: Profile, b: Bounds) -> Profile:
    def sat(value: float, limits: tuple[float, float]) -> float:
        return min(max(value, limits[0]), limits[1])

    return replace(profile,
                   velocity_scale=sat(profile.velocity_scale, b.velocity_scale),
                   offset_m=sat(profile.offset_m, b.approach_offset_m),
                   probe_m=sat(profile.probe_m, b.probe_amplitude_m))


def route(cues: CueVector, cfg: Settings = Settings()) -> tuple[float, str, Profile]:
    risk = aggregate_risk(cues)
    tier = select_tier(risk, cfg.low_to_mid, cfg.mid_to_high)
    return risk, tier, bound_profile(PROFILES[tier], cfg.bounds)


def replay(cues: CueVector, *, remove_cue: str | None = None,
           replacement_material: float | None = None,
           cfg: Settings = Settings()) -> dict:
    """Counterfactual ROUTE ONLY. A physical success counterfactual is impossible."""
    values = cues.as_dict()
    if remove_cue is not None:
        if remove_cue not in values:
            raise ValueError("unknown cue")
        values[remove_cue] = 0.0
    if replacement_material is not None:
        values["zM"] = clip01(replacement_material)
    r, tier, profile = route(CueVector(**values), cfg)
    return {"risk": r, "tier": tier, "profile": profile,
            "counterfactual_outcome": "NOT_ESTIMABLE_FROM_ROUTER_REPLAY"}
