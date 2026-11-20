"""Orchestrates the post-selection risk-conditioned execution decision."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any


from .config import PROFILES, Settings
from .descriptor import build_cues
from .guard import check_observation, project_anchor
from .models import Decision, GuardResult, Packet
from .planning import generate_waypoint_intents, hard_gate
from .router import bound_profile, route


def _abort(packet: Packet, cfg: Settings, guard: GuardResult,
           reasons: tuple[str, ...], cues=None, risk=None, tier=None, profile=None,
           mode: str = "M3") -> Decision:
    return Decision(packet.trial_id, packet.packet_id, packet.anchor.object_id,
                    packet.scene, packet.instruction, packet.anchor, guard,
                    cues, risk, tier, profile, True, reasons, [], cfg.codebook_version, mode=mode)


def decide(packet: Packet, cfg: Settings = Settings(), mode: str = "M3") -> Decision:
    """Fail-closed decision for M1 (raw/fixed), M2 (guarded/fixed), M3 (guarded/routed)."""
    if mode not in ("M1", "M2", "M3"):
        raise ValueError("mode must be M1, M2, or M3")
    try:
        check_observation(packet.observation)
        guard = (GuardResult(packet.anchor, True, ("raw_anchor_unmodified",))
                 if mode == "M1" else project_anchor(packet.anchor, packet.observation, cfg.bounds))
    except (ValueError, IndexError, TypeError, FloatingPointError) as exc:
        guard = GuardResult(None, False, ("invalid_observation",))
        return _abort(packet, cfg, guard, (f"invalid_observation:{type(exc).__name__}",), mode=mode)
    if not guard.success or guard.anchor is None:
        return _abort(packet, cfg, guard, guard.reasons, mode=mode)
    m = packet.measurements
    obs = packet.observation
    gate_reasons = hard_gate(guard.anchor, obs, m, cfg.bounds)
    if gate_reasons:
        return _abort(packet, cfg, guard, gate_reasons, mode=mode)
    try:
        grasp_clearance = guard.anchor.pose.z - obs.table_z_m
        if mode == "M3":
            cues = build_cues(obs, guard.anchor.pixel_uv, m, packet.material,
                              grasp_clearance, cfg.patch_radius_px,
                              *cfg.bounds.z_m)
            risk, tier, profile = route(cues, cfg)
        else:
            cues, risk, tier = None, None, "low"
            profile = bound_profile(PROFILES["low"], cfg.bounds)
        waypoints = generate_waypoint_intents(guard.anchor, profile, cfg, obs)
    except (ValueError, TypeError, OverflowError, FloatingPointError) as exc:
        return _abort(packet, cfg, guard, (f"invalid_descriptor_or_waypoints:{type(exc).__name__}",), mode=mode)
    return Decision(packet.trial_id, packet.packet_id, packet.anchor.object_id,
                    packet.scene, packet.instruction, packet.anchor, guard,
                    cues, risk, tier, profile, False, (), waypoints,
                    cfg.codebook_version, mode=mode)


def audit_to_json(decision: Decision, indent: int = 2) -> str:
    return json.dumps(decision.audit_dict(), indent=indent, ensure_ascii=False,
                      allow_nan=False)


def append_audit(path: str | Path, decision: Decision,
                 outcome: dict[str, Any] | None = None,
                 overhead: dict[str, Any] | None = None) -> None:
    """Append a JSONL audit. Never fabricate outcome; supplied result is external."""
    record = decision.audit_dict()
    if outcome is not None:
        unknown = set(outcome) - set(record["outcome"])
        if unknown:
            raise ValueError(f"unsupported outcome fields: {unknown}")
        record["outcome"].update(outcome)
    if overhead is not None:
        unknown = set(overhead) - set(record["overhead"])
        if unknown:
            raise ValueError(f"unsupported overhead fields: {unknown}")
        record["overhead"].update(overhead)
    line = json.dumps(record, ensure_ascii=False, allow_nan=False)
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("a", encoding="utf-8") as f:
        f.write(line + "\n")
