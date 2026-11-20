from dataclasses import replace

import numpy as np
import pytest

from rcaer import Margins
from rcaer.descriptor import (
    build_cues, depth_patch_stats, ramp, ramp_down, reachability_cue,
)


def test_ramps():
    assert ramp(0.10, 0.10, 0.45) == 0
    assert ramp(0.45, 0.10, 0.45) == 1
    assert ramp_down(0.10, 0.10, 0.45) == 1
    assert ramp_down(0.45, 0.10, 0.45) == 0


def test_depth_invalid_and_missing(packet):
    obs = packet.observation
    f, sd = depth_patch_stats(obs, (50, 50), 30, 0.0414, 0.72)
    assert f == 0 and sd == pytest.approx(0)
    obs.depth_m[:] = np.nan
    f, sd = depth_patch_stats(obs, (50, 50), 30, 0.0414, 0.72)
    assert f == 1 and sd == 0


def test_cue_uses_exact_paper_ranges(packet):
    packet.measurements = replace(packet.measurements, object_height_m=0.075,
                                  gripper_opening_margin_m=0.004,
                                  normalized_bilateral_contact_area=0.35,
                                  object_table_clearance_m=0.010)
    cues = build_cues(packet.observation, (50, 50), packet.measurements,
                      packet.material, 0.028, 30, 0.0414, 0.72)
    assert cues.zH == 1
    assert cues.zT == 1
    assert cues.zC == 1
    assert cues.zD == 0


def test_reachability_requires_normalized_margins():
    assert reachability_cue(Margins(.8,.7,.9,.5)) == pytest.approx(.5)
    with pytest.raises(ValueError):
        reachability_cue(Margins(.8,-.1,.9,.5))
