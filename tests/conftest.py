import numpy as np
import pytest

from rcaer import Anchor, Margins, Material, Measurements, Observation, Packet, Pose


@pytest.fixture
def packet():
    yy, xx = np.indices((101, 101))
    depth = np.full((101, 101), 0.40)
    xyz = np.zeros((101, 101, 3))
    xyz[:, :, 0] = 0.4 + (xx - 50)*0.0005
    xyz[:, :, 1] = 0.1 + (yy - 50)*0.0005
    xyz[:, :, 2] = 0.30
    mask = (xx-50)**2 + (yy-50)**2 < 35**2
    return Packet("test-trial", "simple", "pick test object",
                  Anchor(Pose(0.4,0.1,0.3), (50,50), (50,50), "O01"),
                  Observation(depth, xyz, mask, 0.16),
                  Measurements(object_height_m=0.15, object_table_clearance_m=0.04,
                               gripper_opening_margin_m=0.020,
                               normalized_bilateral_contact_area=0.80,
                               margins=Margins(0.95,0.95,0.95,0.95),
                               gripper_table_clearance_m=0.14,
                               grasp_height_margin_m=0.14,
                               motion_plan_time_s=0.15, plan_feasible=True, reachability_ok=True),
                  material=Material())
