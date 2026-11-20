from dataclasses import replace
import json

import numpy as np

from rcaer import Pose, append_audit, audit_to_json, decide
from rcaer.guard import project_anchor


def test_dry_run_end_to_end(packet):
    out = decide(packet)
    assert not out.aborted, out.abort_reasons
    assert out.guarded.success
    assert out.tier == "low"
    assert out.profile.grip_force_n == 40
    assert [p.phase for p in out.waypoints] == ["initial", "pre_approach",
           "guarded_approach", "contact_probe", "close", "lift", "retreat"]
    assert "outcome" in json.loads(audit_to_json(out))


def test_guard_moves_edge_inward(packet):
    packet.anchor = replace(packet.anchor, pixel_uv=(80,50),
                            pose=Pose(.415,.1,.30))
    guarded = project_anchor(packet.anchor, packet.observation,
                             __import__("rcaer").Bounds())
    assert guarded.success
    assert guarded.anchor.pixel_uv[0] < 80
    assert guarded.projection_px <= 30
    assert guarded.lateral_shift_m <= 0.030


def test_semantic_target_is_preserved(packet):
    packet.anchor = replace(packet.anchor, semantic_pixel_uv=(0,0))
    out = decide(packet)
    assert out.aborted
    assert "semantic_target_not_in_mask" in out.abort_reasons


def test_no_valid_depth_fails_closed(packet):
    packet.observation.depth_m[:] = np.nan
    out = decide(packet)
    assert out.aborted
    assert "no_semantically_valid_local_projection" in out.abort_reasons


def test_table_gate_after_projection(packet):
    packet.measurements = replace(packet.measurements, object_table_clearance_m=.009)
    out = decide(packet)
    assert out.aborted
    assert "table_clearance" in out.abort_reasons
    assert not out.waypoints


def test_reachability_and_motion_planning_gates(packet):
    packet.measurements = replace(packet.measurements, reachability_ok=False,
                                  motion_plan_time_s=2.01)
    out = decide(packet)
    assert "reachability" in out.abort_reasons
    assert "motion_planning" in out.abort_reasons


def test_workspace_gate(packet):
    packet.observation.points_base_m[:, :, 0] = .9
    packet.anchor = replace(packet.anchor, pose=Pose(.9,.1,.3))
    out = decide(packet)
    assert out.aborted
    assert "workspace" in out.abort_reasons


def test_waypoint_ceiling_gate(packet):
    packet.observation.points_base_m[:,:,2] = .71
    packet.anchor = replace(packet.anchor, pose=Pose(.4,.1,.71))
    out = decide(packet)
    assert out.aborted
    assert "invalid_descriptor_or_waypoints:ValueError" in out.abort_reasons


def test_audit_never_fabricates_outcomes(packet, tmp_path):
    result = decide(packet)
    p = tmp_path / "records.jsonl"
    append_audit(p, result)
    row = json.loads(p.read_text().strip())
    assert row["outcome"]["physical_success"] is None
    assert row["codebook_version"] == "paper-table-I-III-v1"
    append_audit(p, result, outcome={"physical_success": True})
    assert len(p.read_text().splitlines()) == 2


def test_original_study_modes_share_fixed_profile(packet):
    m1 = decide(packet, mode="M1")
    m2 = decide(packet, mode="M2")
    m3 = decide(packet, mode="M3")
    assert all(not x.aborted for x in (m1, m2, m3))
    assert m1.guarded.anchor == packet.anchor
    assert m1.guarded.reasons == ("raw_anchor_unmodified",)
    assert m2.guarded.anchor == m3.guarded.anchor
    assert m1.profile == m2.profile
    assert m1.cues is None and m2.cues is None and m3.cues is not None
    assert m3.mode == "M3"

def test_missing_planner_validation_fails_closed(packet):
    packet.measurements = replace(packet.measurements, plan_feasible=False, reachability_ok=False)
    out = decide(packet)
    assert out.aborted
    assert "motion_planning" in out.abort_reasons
    assert "reachability" in out.abort_reasons


def test_approach_axis_and_orientation_passthrough(packet):
    packet.anchor = replace(packet.anchor,
                            pose=Pose(.4,.1,.3,0,0,0,1),
                            approach_direction_base=(0.0,0.0,-1.0))
    result = decide(packet)
    assert not result.aborted
    assert result.waypoints[0].position.z > result.waypoints[2].position.z
    assert result.waypoints[2].position.qw == 1.0

def test_bad_orientation_fails_closed(packet):
    packet.anchor = replace(packet.anchor,pose=Pose(.4,.1,.3,0,0,0,0))
    result = decide(packet)
    assert result.aborted
    assert "invalid_orientation_quaternion" in result.abort_reasons
