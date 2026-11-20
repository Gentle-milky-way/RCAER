"""Synthetic dry run, NOT a replication of published robot experiments."""

import numpy as np

from rcaer import (
    Anchor, Margins, Material, Measurements, Observation, Packet, Pose,
    audit_to_json, decide,
)


def make_demo_packet() -> Packet:
    h, w = 101, 101
    depth = np.full((h, w), 0.45, dtype=float)
    # Synthetically missing pixels provoke depth quality risk.
    depth[35:65, 35:65] = np.nan
    xyz = np.zeros((h, w, 3), dtype=float)
    yy, xx = np.indices((h, w))
    xyz[:, :, 0] = 0.42 + (xx - 50) * 0.0005
    xyz[:, :, 1] = 0.10 + (yy - 50) * 0.0005
    xyz[:, :, 2] = 0.29
    mask = (xx-50)**2 + (yy-50)**2 <= 38**2
    return Packet(
        trial_id="SYNTHETIC-0001", packet_id="D-001", scene="simple",
        instruction="pick the selected demo object",
        anchor=Anchor(Pose(0.42, 0.10, 0.29), (50, 50), (50, 50), "SYNTHETIC-O01"),
        observation=Observation(depth, xyz, mask, table_z_m=0.16),
        measurements=Measurements(
            object_height_m=0.125, object_table_clearance_m=0.031,
            gripper_opening_margin_m=0.014,
            normalized_bilateral_contact_area=0.60,
            margins=Margins(0.9, 0.8, 0.8, 0.9),
            tilt_deg=6, rotation_to_target_deg=20,
            gripper_table_clearance_m=0.13, grasp_height_margin_m=0.13,
            motion_plan_time_s=0.1, plan_feasible=True, reachability_ok=True,
        ),
        material=Material(transparent=True),
    )


if __name__ == "__main__":
    packet = make_demo_packet()
    decision = decide(packet)
    print(audit_to_json(decision))
