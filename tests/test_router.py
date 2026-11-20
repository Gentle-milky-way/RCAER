from dataclasses import replace

import pytest

from rcaer import Bounds, CueVector, Material, aggregate_risk, material_cue, replay, route, select_tier
from rcaer.config import PROFILES
from rcaer.router import bound_profile


def test_tier_thresholds():
    assert select_tier(0.4999) == "low"
    assert select_tier(0.50) == "mid"
    assert select_tier(0.699999) == "mid"
    assert select_tier(0.70) == "high"


def test_table_ii_profiles():
    assert PROFILES["low"].grip_force_n == 40
    assert PROFILES["mid"].probe_m == 0.002
    assert PROFILES["high"].velocity_scale == 0.45
    assert PROFILES["high"].damping_rotation_nms_rad == 1.9


def test_clip_is_defensive_not_a_new_policy():
    p = replace(PROFILES["mid"], probe_m=0.1, offset_m=0.15,
                velocity_scale=0.1)
    bounded = bound_profile(p, Bounds())
    assert bounded.probe_m == 0.004
    assert bounded.offset_m == 0.10
    assert bounded.velocity_scale == 0.45
    assert bound_profile(PROFILES["mid"], Bounds()) == PROFILES["mid"]


def test_material_prior_and_validation():
    assert material_cue(Material()) == pytest.approx(0.20)
    assert material_cue(Material(reflective="high", transparent=True,
                                 deformable=True, thin_walled=True,
                                 annotation_uncertainty=1)) == 1.0
    with pytest.raises(ValueError):
        material_cue(Material(annotation_uncertainty=1.1))


def test_conservative_max_and_loo_replay():
    cues = CueVector(0.2, 0.24, 0.35, 0.11, 0.45, 0.21, 0.61)
    assert aggregate_risk(cues) == pytest.approx(0.61)
    original = route(cues)
    assert original[1] == "mid"
    changed = replay(cues, remove_cue="zC")
    assert changed["tier"] == "low"
    assert changed["counterfactual_outcome"] == "NOT_ESTIMABLE_FROM_ROUTER_REPLAY"
    assert replay(cues, replacement_material=0.8)["tier"] == "high"


def test_unbounded_input_aborts_not_silent_clipping():
    with pytest.raises(ValueError):
        aggregate_risk(CueVector(1.2, 0, 0, 0, 0, 0, 0))
    with pytest.raises(ValueError):
        select_tier(float("nan"))
