"""RCAER: reference implementation of risk-conditioned auditable execution routing."""
from .config import Bounds, Profile, Settings
from .descriptor import aggregate_risk, build_cues, material_cue
from .engine import append_audit, audit_to_json, decide
from .models import Anchor, CueVector, Margins, Material, Measurements, Observation, Packet, Pose
from .router import replay, route, select_tier

__all__ = ["Anchor", "Bounds", "CueVector", "Margins", "Material", "Measurements",
           "Observation", "Packet", "Pose", "Profile", "Settings", "aggregate_risk",
           "append_audit", "audit_to_json", "build_cues", "decide", "material_cue",
           "replay", "route", "select_tier"]
